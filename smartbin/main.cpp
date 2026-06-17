#include "camera.hpp"
#include "config.hpp"
#include "detector_hsv.hpp"
#include "hardware_drive.hpp"
#include "motor_translation.hpp"
#include "operation_interface.hpp"

#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>

// TODO: Do we need all of these imports? (remove unused ones)
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

/**
 * Tasks:
 *   Vision Detection      -> periodic
 *   Trajectory Prediction -> sporadic
 *   Motor Control         -> periodic
 *   Operation Interface   -> sporadic
 */

// Shared variables for inter-task communication
static std::atomic<bool> g_running{true}; // process lifetime (Ctrl-C / "q")
static std::atomic<bool> g_active{true}; // start/stop flag: Operation Interface -> Motor Control

// Signal handler
static void onSignal(int) { g_running = false; }

// P_obj - written by Vision Detection, read by Trajectory Prediction.
// "seq" lets the prediction task detect a fresh sample (its release event).
static struct {
    pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER; // TODO: ?
    pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
    DetectionResult det{};
    uint64_t seq = 0;
} global_object_position;

// P_target - written by Trajectory Prediction, read by Motor Control.
static struct {
    pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER; // TODO: ?
    MotorCommand cmd{};
} g_pTarget;

// TODO: Do we still need this? (remove?)
// Debug frame - written by Vision Detection, shown by Operation Interface.
static struct {
    pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
    cv::Mat frame;
    uint64_t seq = 0;
} g_debug;

// TODO:?
struct AppContext {
    Camera *cam;
    HsvDetector *detector;
    MotorTranslation *motor;
    bool hwOk;
};

// TODO: ?
/**
 * SCHED_DEADLINE plumbing - glibc has no wrapper for sched_setattr(2).
 */
struct SchedAttr {
    uint32_t size;
    uint32_t sched_policy;
    uint64_t sched_flags;
    int32_t sched_nice;
    uint32_t sched_priority;
    uint64_t sched_runtime;
    uint64_t sched_deadline;
    uint64_t sched_period;
};

// TODO: ?
#ifndef SCHED_DEADLINE
#define SCHED_DEADLINE 6
#endif
#ifndef SCHED_FLAG_RESET_ON_FORK
#define SCHED_FLAG_RESET_ON_FORK 0x01
#endif

// TODO: ?
static void setDeadlineSched(const char *name, long long runtimeNs, long long deadlineNs, long long periodNs) {
    SchedAttr attr{};
    attr.size = sizeof(attr);
    attr.sched_policy = SCHED_DEADLINE;
    // The kernel refuses clone() from a SCHED_DEADLINE task (EAGAIN), so
    // helper threads spawned inside our tasks (TBB/OpenCV workers, LCCV)
    // would fail to start. RESET_ON_FORK makes them plain SCHED_OTHER.
    attr.sched_flags = SCHED_FLAG_RESET_ON_FORK;
    attr.sched_runtime = static_cast<uint64_t>(runtimeNs);
    attr.sched_deadline = static_cast<uint64_t>(deadlineNs);
    attr.sched_period = static_cast<uint64_t>(periodNs);
    if (syscall(SYS_sched_setattr, 0, &attr, 0) != 0)
        std::cerr << "[" << name << "] SCHED_DEADLINE unavailable (" << std::strerror(errno)
                  << ") - running with the default policy. Run as root on a "
                     "PREEMPT_RT kernel for real-time guarantees.\n";
}

// TODO: ?
static void timespecAddNs(timespec &t, long long ns) {
    t.tv_nsec += ns;
    while (t.tv_nsec >= 1'000'000'000L) {
        t.tv_nsec -= 1'000'000'000L;
        ++t.tv_sec;
    }
}

// TODO: ?
static MotorCommand stopCommand() {
    MotorCommand cmd{};
    cmd.stop = true;
    return cmd;
}

/**
 * Task: Vision Detection (periodic)
 * The blocking readFrame() is the release point: the camera delivers frames
 * at FRAME_FPS, so the task is periodic with T = TASK_VISION_PERIOD_NS.
 */
static void *visionTask(void *arg) {
    auto &ctx = *static_cast<AppContext *>(arg);

    // Request SCHED_DEADLINE for the vision task
    setDeadlineSched("vision", Config::TASK_VISION_RUNTIME_NS, Config::TASK_VISION_DEADLINE_NS, Config::TASK_VISION_PERIOD_NS);

    cv::Mat frame;
    while (g_running) {
        if (!ctx.cam->readFrame(frame)) {
            std::cerr << "[Vision] Failed to read frame - retrying...\n";
            timespec retry{0, 30'000'000}; // Retry after 30 ms (camera frame period)
            nanosleep(&retry, nullptr);
            continue;
        }

        // Obtain detection result for this frame
        DetectionResult det = ctx.detector->detect(frame);

        // Write detection result to shared variable
        pthread_mutex_lock(&global_object_position.mtx);
        global_object_position.det = det;
        ++global_object_position.seq;
        pthread_cond_signal(&global_object_position.cond); // release Trajectory Prediction
        pthread_mutex_unlock(&global_object_position.mtx);

        // Debug display (only when enabled in config)
        if (Config::SHOW_WINDOW) {
            pthread_mutex_lock(&g_debug.mtx);
            ++g_debug.seq;
            pthread_mutex_unlock(&g_debug.mtx);
        }
    }

    // Wake the prediction task so it can observe g_running and exit.
    // TODO: ?
    pthread_mutex_lock(&global_object_position.mtx);
    pthread_cond_broadcast(&global_object_position.cond);
    pthread_mutex_unlock(&global_object_position.mtx);
    return nullptr;
}

/**
 * Task: Trajectory Prediction (sporadic)
 * Released by a new P_obj sample; minimum inter-arrival time is the camera
 * period, and the CBS budget bounds its CPU use (sporadic-server behaviour).
 */
static void *predictionTask(void *arg) {
    auto &ctx = *static_cast<AppContext *>(arg); // TODO: ?

    // TODO: ? sporadic, not EDF
    setDeadlineSched("prediction", Config::TASK_PRED_RUNTIME_NS, Config::TASK_PRED_DEADLINE_NS, Config::TASK_PRED_PERIOD_NS);

    uint64_t lastSeq = 0;
    while (g_running) {
        pthread_mutex_lock(&global_object_position.mtx);
        while (global_object_position.seq == lastSeq && g_running)
            // Task sleeps until a new detection result is available (vision task signals global_object_position.cond)
            pthread_cond_wait(&global_object_position.cond, &global_object_position.mtx);
        DetectionResult det = global_object_position.det; // copy the detection result while holding the lock
        lastSeq = global_object_position.seq;
        pthread_mutex_unlock(&global_object_position.mtx);

        // Early exit if the process is shutting down
        if (!g_running)
            break;

        // Compute motor commands based on the detection result
        MotorCommand cmd = ctx.motor->compute(det.detected, det.centroid, det.boundingBox);

        // Write motor commands to shared variable
        pthread_mutex_lock(&g_pTarget.mtx);
        g_pTarget.cmd = cmd;
        pthread_mutex_unlock(&g_pTarget.mtx);
    }
    return nullptr;
}

/**
 * Task: Motor Control (periodic, 50 Hz)
 * Reads the latest P_target and writes PWM duty cycles to the hat over I2C.
 * Honours the start/stop flag from the Operation Interface.
 */
static void *motorTask(void *arg) {
    auto &ctx = *static_cast<AppContext *>(arg); // TODO: ?

    // Request SCHED_DEADLINE for the motor control task
    setDeadlineSched("motor", Config::TASK_MOTOR_RUNTIME_NS, Config::TASK_MOTOR_DEADLINE_NS, Config::TASK_MOTOR_PERIOD_NS);

    // Record current time for periodic wake-ups
    timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (g_running) {
        // Read the latest motor command from the prediction task
        MotorCommand cmd;
        pthread_mutex_lock(&g_pTarget.mtx);
        cmd = g_pTarget.cmd;
        pthread_mutex_unlock(&g_pTarget.mtx);

        if (!g_active)
            cmd = stopCommand();

        // Apply motor command to hardware (if initialized successfully)
        if (ctx.hwOk)
            hardwareApply(cmd);

        // Sleep until the next period
        timespecAddNs(next, Config::TASK_MOTOR_PERIOD_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }

    if (ctx.hwOk)
        hardwareApply(stopCommand()); // never leave the wheels spinning
    return nullptr;
}

// TODO: ?
/**
 * Task: Operation Interface (low-rate)
 * Two control surfaces, both non-blocking so the task keeps its period:
 *   - stdin: 's' toggles start/stop, 'q' quits.
 *   - WiFi remote: an ESP8266 sends button presses over TCP (see
 *     operation_interface.{hpp,cpp}); START toggles start/stop, QUIT quits.
 * With SHOW_WINDOW it also displays the latest debug frame (all highgui calls
 * stay on this one thread).
 */
static void *opInterfaceTask(void *arg) {
    auto &ctx = *static_cast<AppContext *>(arg);
    (void)ctx;

    // Request SCHED_DEADLINE for the operation interface task
    setDeadlineSched("opIface", Config::TASK_UI_RUNTIME_NS, Config::TASK_UI_DEADLINE_NS, Config::TASK_UI_PERIOD_NS);

    // WiFi remote (ESP8266). If the socket fails to open we log and carry on
    // with stdin-only control rather than aborting the whole task.
    OperationInterface remote(Config::OPIF_PORT);
    remote.start();

    auto toggleActive = [] {
        g_active = !g_active;
        std::cout << (g_active ? "[OpIface] started\n" : "[OpIface] stopped\n");
    };

    timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (g_running) {
        // Non-blocking console input
        pollfd pfd{STDIN_FILENO, POLLIN, 0};
        if (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
            char buf[64];
            ssize_t n = read(STDIN_FILENO, buf, sizeof buf);
            for (ssize_t i = 0; i < n; ++i) {
                if (buf[i] == 'q')
                    g_running = false;
                if (buf[i] == 's')
                    toggleActive();
            }
        }

        // Non-blocking WiFi remote input
        OperationInterface::Events ev = remote.poll();
        if (ev.toggleActive)
            toggleActive();
        if (ev.quit)
            g_running = false;

        // Sleep until the next period
        timespecAddNs(next, Config::TASK_UI_PERIOD_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }
    return nullptr;
}

int main(int argc, char *argv[]) {
    // Handle SIGINT and SIGTERM for graceful shutdown (set g_running = false)
    std::signal(SIGINT, onSignal); // on Ctrl+C
    std::signal(SIGTERM, onSignal); // on kill command

    // Check if we want to compile for desktop or rpi
    int deviceIndex = (argc > 1) ? std::stoi(argv[1]) : Config::DEVICE_INDEX;

    // Initialize camera, detector and motor
    Camera cam(deviceIndex, Config::CAM_VIDEO_WIDTH, Config::CAM_VIDEO_HEIGHT, Config::CAM_FPS);
    auto detector = HsvDetector(); // TODO: Select algorithm to use for detection
    MotorTranslation motor(Config::CAM_VIDEO_WIDTH, Config::CAM_VIDEO_HEIGHT);

    // Check camera errors
    if (!cam.open()) {
        std::cerr << "Could not open camera. Exiting.\n";
        return 1;
    }

    // Initialize hardware drive
    bool hwOk = hardwareInit();

    // without locking rather than crash.
    rlimit memlock{RLIM_INFINITY, RLIM_INFINITY}; // TODO:?
    if (setrlimit(RLIMIT_MEMLOCK, &memlock) != 0) {
        std::cerr << "setrlimit(RLIMIT_MEMLOCK) failed (" << std::strerror(errno) << ") - skipping mlockall; page faults may add latency.\n";
    }

    // Lock all memory regions to avoid page-fault latency in the RT tasks
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) { // TODO:?
        std::cerr << "mlockall failed (" << std::strerror(errno) << ") - page faults may add latency.\n";
    }

    g_pTarget.cmd = stopCommand(); // TODO:?

    AppContext ctx{&cam, &detector, &motor, hwOk}; // TODO:?

    std::cout << "Trashcan tracker running. 's' = start/stop, 'q' or Ctrl-C = quit.\n";

    // TODO: Add checks to verify errors?
    pthread_t tVision, tPrediction, tMotor, tOpIface;
    pthread_create(&tVision, nullptr, visionTask, &ctx); // computer vision task
    pthread_create(&tPrediction, nullptr, predictionTask, &ctx); // prediction task
    pthread_create(&tMotor, nullptr, motorTask, &ctx); // motor task
    pthread_create(&tOpIface, nullptr, opInterfaceTask, &ctx); // operation interface task

    pthread_join(tVision, nullptr);
    pthread_join(tPrediction, nullptr);
    pthread_join(tMotor, nullptr);
    pthread_join(tOpIface, nullptr);

    std::cout << "\nShutting down.\n";
    hardwareShutdown();
    cam.release();
    cv::destroyAllWindows();
    return 0;
}
