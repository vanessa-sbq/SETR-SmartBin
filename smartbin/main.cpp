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
 *   Vision Detection      -> Periodic
 *   Trajectory Prediction -> Sporadic
 *   Motor Control         -> Periodic
 *   Operation Interface   -> Periodic
 */

// Shared variables for inter-task communication
static std::atomic<bool> global_program_running{true}; // Process lifetime
static std::atomic<bool> global_robot_active{true}; // Start/stop flag: Operation Interface -> Motor Control

// Signal handler
static void onSignal(int) { global_program_running = false; }

// Written by Vision Detection, read by Trajectory Prediction.
// Since the Trajectory Prediction is supposed to be sporadic we need the condition for this to be possible in Linux because it does not have a specific sporadic server.
static struct {
    pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
    DetectionResult det{};
    uint64_t seq = 0; // "seq" lets the prediction task detect a fresh sample (also its release event).
} global_object_position;

// Written by Trajectory Prediction, read by Motor Control.
static struct {
    pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
    MotorCommand cmd{};
} global_prediction_target;

/*
    Global app context.
    Needed because pthread_create only allows a single argument of type void* to be used in each task and our
    tasks need more than one parameter. For example, the vision task requires both the camera and the detector.
*/
struct AppContext {
    Camera *cam;
    HsvDetector *detector;
    MotorTranslation *motor;
    bool hwOk;
};

#ifndef SCHED_DEADLINE
#define SCHED_DEADLINE 6
#endif
#ifndef SCHED_FLAG_RESET_ON_FORK
#define SCHED_FLAG_RESET_ON_FORK 0x01
#endif

/*
    Helper function. Helps set deadline schedule. It also uses SCHED_FLAG_RESET_ON_FORK to allow library code to fork and run
    as normal best-effort work. This does not compromise the schedulability of EDF tasks because if one becomes active we will just
    remove the CPU from the one that called the blocking library code and instead we will run the task with the current
    highest priority.
*/
static void setDeadlineSched(const char *name, long long runtimeNs, long long deadlineNs, long long periodNs) {
    struct sched_attr attr;
    attr.size = sizeof(attr);
    attr.sched_policy = SCHED_DEADLINE;
    attr.sched_nice = 0;
    attr.sched_priority = 0;
    // The kernel refuses clone() or fork() from a SCHED_DEADLINE task (EAGAIN), so OpenCV or LCCV would fail to start.
    attr.sched_flags = SCHED_FLAG_RESET_ON_FORK;
    attr.sched_runtime = static_cast<uint64_t>(runtimeNs);
    attr.sched_deadline = static_cast<uint64_t>(deadlineNs);
    attr.sched_period = static_cast<uint64_t>(periodNs);
    attr.sched_util_min = 0;
    attr.sched_util_max = 0;
    if (syscall(SYS_sched_setattr, 0, &attr, 0) != 0)
        std::cerr << "[" << name << "] SCHED_DEADLINE unavailable (" << std::strerror(errno)
                  << ") - running with the default policy. Run as root on a "
                     "PREEMPT_RT kernel for real-time guarantees.\n";
}

/*
    Helper function.
    When a task finishes early and we need to wait we call this function to wait for a specific (ns) number of nanoseconds.
*/
static void timespecAddNs(timespec &t, long long ns) {
    t.tv_nsec += ns;
    while (t.tv_nsec >= 1'000'000'000L) {
        t.tv_nsec -= 1'000'000'000L;
        ++t.tv_sec;
    }
}

/*
    Helper function.
    Just sets the command for the motor to stop.
*/
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
    while (global_program_running) {
        if (!ctx.cam->readFrame(frame)) {
            std::cerr << "[Vision] Failed to read frame, retrying...\n";
            timespec next;
            clock_gettime(CLOCK_MONOTONIC, &next);
            timespecAddNs(next, 30'000'000);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
            continue;
        }

        // Obtain detection result for this frame
        DetectionResult det = ctx.detector->detect(frame);

        // Write detection result to shared variable
        pthread_mutex_lock(&global_object_position.mtx);
        global_object_position.det = det;
        ++global_object_position.seq;
        pthread_cond_signal(&global_object_position.cond); // Release Trajectory Prediction
        pthread_mutex_unlock(&global_object_position.mtx);
    }

    // Wake the prediction task so it can observe global_program_running and exit.
    pthread_mutex_lock(&global_object_position.mtx);
    pthread_cond_signal(&global_object_position.cond);
    pthread_mutex_unlock(&global_object_position.mtx);
    return nullptr;
}

/**
 * Task: Trajectory Prediction (sporadic)
 * Released by a new global_object_position; minimum inter-arrival time is the camera
 * period, and the CBS budget bounds its CPU use (sporadic-server behaviour).
 */
static void *predictionTask(void *arg) {
    auto &ctx = *static_cast<AppContext *>(arg);

    // Request SCHED_DEADLINE for the motor control task
    setDeadlineSched("prediction", Config::TASK_PRED_RUNTIME_NS, Config::TASK_PRED_DEADLINE_NS, Config::TASK_PRED_PERIOD_NS);

    uint64_t lastSeq = 0;
    while (global_program_running) {
        pthread_mutex_lock(&global_object_position.mtx);
        while (global_object_position.seq == lastSeq && global_program_running)
            // Task sleeps until a new detection result is available (vision task signals global_object_position.cond)
            pthread_cond_wait(&global_object_position.cond, &global_object_position.mtx);
        DetectionResult det = global_object_position.det; // Copy the detection result while holding the lock
        lastSeq = global_object_position.seq;
        pthread_mutex_unlock(&global_object_position.mtx);

        // Early exit if the process is shutting down
        if (!global_program_running)
            break;

        // Compute motor commands based on the detection result
        MotorCommand cmd = ctx.motor->compute(det.detected, det.centroid, det.boundingBox);

        // Write motor commands to shared variable
        pthread_mutex_lock(&global_prediction_target.mtx);
        global_prediction_target.cmd = cmd;
        pthread_mutex_unlock(&global_prediction_target.mtx);
    }
    return nullptr;
}

/**
 * Task: Motor Control (periodic, 50 Hz)
 * Reads the latest P_target and writes PWM duty cycles to the hat over I2C.
 * Uses the start/stop flag from the Operation Interface.
 */
static void *motorTask(void *arg) {
    auto &ctx = *static_cast<AppContext *>(arg);

    // Request SCHED_DEADLINE for the motor control task
    setDeadlineSched("motor", Config::TASK_MOTOR_RUNTIME_NS, Config::TASK_MOTOR_DEADLINE_NS, Config::TASK_MOTOR_PERIOD_NS);

    // Record current time for periodic wake-ups
    timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (global_program_running) {
        // Read the latest motor command from the prediction task
        MotorCommand cmd;
        pthread_mutex_lock(&global_prediction_target.mtx);
        cmd = global_prediction_target.cmd;
        pthread_mutex_unlock(&global_prediction_target.mtx);

        if (!global_robot_active)
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


/**
 * Task: Operation Interface
 * WiFi remote (non-blocking): an esp8266 sends button presses over TCP.
 * The START button toggles start/stop.
 */
static void *opInterfaceTask(void *arg) {
    // Request SCHED_DEADLINE for the operation interface task
    setDeadlineSched("opIface", Config::TASK_UI_RUNTIME_NS, Config::TASK_UI_DEADLINE_NS, Config::TASK_UI_PERIOD_NS);

    // WiFi remote (esp8266). If the socket fails to open we log and carry on.
    OperationInterface remote(Config::OPIF_PORT);
    remote.start();

    auto toggleActive = [] {
        global_robot_active = !global_robot_active;
        std::cout << (global_robot_active ? "[OpIface] started\n" : "[OpIface] stopped\n");
    };

    timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (global_program_running) {
        // Non-blocking WiFi remote input
        OperationInterface::Events ev = remote.poll();
        if (ev.toggleActive)
            toggleActive();
        if (ev.quit)
            global_program_running = false;

        // Sleep until the next period
        timespecAddNs(next, Config::TASK_UI_PERIOD_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }
    return nullptr;
}

int main() {
    // Handle SIGINT and SIGTERM for graceful shutdown (set global_program_running = false)
    std::signal(SIGINT, onSignal); // on Ctrl+C
    std::signal(SIGTERM, onSignal); // on kill command

    // Initialize camera, detector and motor
    Camera cam(Config::DEVICE_INDEX, Config::CAM_VIDEO_WIDTH, Config::CAM_VIDEO_HEIGHT, Config::CAM_FPS);
    HsvDetector detector = HsvDetector();
    MotorTranslation motor(Config::CAM_VIDEO_WIDTH, Config::CAM_VIDEO_HEIGHT);

    // Check camera errors
    if (!cam.open()) {
        std::cerr << "Could not open camera. Exiting.\n";
        return 1;
    }

    // Initialize hardware drive
    bool hwOk = hardwareInit();

    rlimit memlock{RLIM_INFINITY, RLIM_INFINITY};
    if (setrlimit(RLIMIT_MEMLOCK, &memlock) != 0) {
        std::cerr << "setrlimit(RLIMIT_MEMLOCK) failed (" << std::strerror(errno) << ") - skipping mlockall; page faults may add latency.\n";
    }

    // Lock all memory regions to avoid page-fault latency in the RT tasks
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "mlockall failed (" << std::strerror(errno) << ") - page faults may add latency.\n";
    }

    // Initialze the motor command to prevent the robot from moving.
    global_prediction_target.cmd = stopCommand();

    // Store the app context and save all the initialized hardware.
    AppContext ctx{&cam, &detector, &motor, hwOk};

    std::cout << "SmartBin running.\n";

    pthread_t tVision, tPrediction, tMotor, tOpIface;
    pthread_create(&tVision, nullptr, visionTask, &ctx); // computer vision task
    pthread_create(&tPrediction, nullptr, predictionTask, &ctx); // prediction task
    pthread_create(&tMotor, nullptr, motorTask, &ctx); // motor task
    pthread_create(&tOpIface, nullptr, opInterfaceTask, &ctx); // operation interface task

    pthread_join(tVision, nullptr);
    pthread_join(tPrediction, nullptr);
    pthread_join(tMotor, nullptr);
    pthread_join(tOpIface, nullptr);

    std::cout << "\nShutting down. Goodbye :(\n";
    hardwareShutdown();
    cam.release();
    cv::destroyAllWindows();
    return 0;
}
