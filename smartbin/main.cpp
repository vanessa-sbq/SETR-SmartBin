#include "camera.hpp"
#include "detector.hpp"
#include "motor_translation.hpp"
#include "config.hpp"
#include "hardware_drive.hpp"

#include <iostream>
#include <csignal>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>

#include <pthread.h>
#include <poll.h>
#include <sched.h>
#include <time.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/syscall.h>

// ---------------------------------------------------------------------------
// Task set:
//   Vision Detection      periodic, paced by the camera (30 fps)
//   Trajectory Prediction sporadic, released by a detection event (mit = T_cam)
//   Motor Control         periodic, 50 Hz
//   Operation Interface   low-rate, start/stop flag + debug display
//
// Inter-task communication is via shared variables guarded by POSIX
// pthread_mutex: P_obj (vision -> prediction) and P_target (prediction ->
// motor). Every thread asks the kernel for SCHED_DEADLINE (EDF); for the
// sporadic prediction task the CBS budget acts as a sporadic server,
// bounding its CPU usage while keeping it responsive to detection events.
// ---------------------------------------------------------------------------

static std::atomic<bool> g_running{true};  // process lifetime (Ctrl-C / 'q')
static std::atomic<bool> g_active{true};   // start/stop flag: Operation Interface -> Motor Control
static std::atomic<bool> g_moving{false};  // bin in motion: Trajectory Prediction -> Vision Detection

static void onSignal(int) { g_running = false; }

// P_obj - written by Vision Detection, read by Trajectory Prediction.
// `seq` lets the prediction task detect a fresh sample (its release event).
static struct {
    pthread_mutex_t mtx  = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t  cond = PTHREAD_COND_INITIALIZER;
    DetectionResult det{};
    uint64_t        seq = 0;
} g_pObj;

// P_target - written by Trajectory Prediction, read by Motor Control.
static struct {
    pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
    MotorCommand    cmd{};
} g_pTarget;

// Debug frame - written by Vision Detection, shown by Operation Interface.
static struct {
    pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
    cv::Mat         frame;
    uint64_t        seq = 0;
} g_debug;

struct AppContext {
    Camera*           cam;
    IDetector*        detector;
    MotorTranslation* motor;
    bool              hwOk;
};

// ---------------------------------------------------------------------------
// SCHED_DEADLINE plumbing - glibc has no wrapper for sched_setattr(2).
// ---------------------------------------------------------------------------
struct SchedAttr {
    uint32_t size;
    uint32_t sched_policy;
    uint64_t sched_flags;
    int32_t  sched_nice;
    uint32_t sched_priority;
    uint64_t sched_runtime;
    uint64_t sched_deadline;
    uint64_t sched_period;
};

#ifndef SCHED_DEADLINE
#define SCHED_DEADLINE 6
#endif
#ifndef SCHED_FLAG_RESET_ON_FORK
#define SCHED_FLAG_RESET_ON_FORK 0x01
#endif

static void setDeadlineSched(const char* name, long long runtimeNs,
                             long long deadlineNs, long long periodNs) {
    SchedAttr attr{};
    attr.size           = sizeof(attr);
    attr.sched_policy   = SCHED_DEADLINE;
    // The kernel refuses clone() from a SCHED_DEADLINE task (EAGAIN), so
    // helper threads spawned inside our tasks (TBB/OpenCV workers, LCCV)
    // would fail to start. RESET_ON_FORK makes them plain SCHED_OTHER.
    attr.sched_flags    = SCHED_FLAG_RESET_ON_FORK;
    attr.sched_runtime  = static_cast<uint64_t>(runtimeNs);
    attr.sched_deadline = static_cast<uint64_t>(deadlineNs);
    attr.sched_period   = static_cast<uint64_t>(periodNs);
    if (syscall(SYS_sched_setattr, 0, &attr, 0) != 0)
        std::cerr << "[" << name << "] SCHED_DEADLINE unavailable ("
                  << std::strerror(errno)
                  << ") - running with the default policy. Run as root on a "
                     "PREEMPT_RT kernel for real-time guarantees.\n";
}

static void timespecAddNs(timespec& t, long long ns) {
    t.tv_nsec += ns;
    while (t.tv_nsec >= 1'000'000'000L) {
        t.tv_nsec -= 1'000'000'000L;
        ++t.tv_sec;
    }
}

static MotorCommand stopCommand() {
    MotorCommand cmd{};
    cmd.stop = true;
    return cmd;
}

// ---------------------------------------------------------------------------
// Task: Vision Detection (periodic)
// The blocking readFrame() is the release point: the camera delivers frames
// at FRAME_FPS, so the task is periodic with T = TASK_VISION_PERIOD_NS.
// ---------------------------------------------------------------------------
static void* visionTask(void* arg) {
    auto& ctx = *static_cast<AppContext*>(arg);
    setDeadlineSched("vision", Config::TASK_VISION_RUNTIME_NS,
                     Config::TASK_VISION_DEADLINE_NS,
                     Config::TASK_VISION_PERIOD_NS);

    cv::Mat frame;
    while (g_running) {
        if (!ctx.cam->readFrame(frame)) {
            std::cerr << "[Vision] Failed to read frame - retrying...\n";
            timespec retry{0, 30'000'000};
            nanosleep(&retry, nullptr);
            continue;
        }

        // Pass the motion flag so the detector can manage learning rate +
        // cooldown while the bin repositions.
        DetectionResult det = ctx.detector->detect(frame, g_moving.load());

        pthread_mutex_lock(&g_pObj.mtx);
        g_pObj.det = det;
        ++g_pObj.seq;
        pthread_cond_signal(&g_pObj.cond);   // release Trajectory Prediction
        pthread_mutex_unlock(&g_pObj.mtx);

        if (Config::SHOW_WINDOW) {
            cv::Mat dbg = ctx.detector->drawDebug(frame, det);
            pthread_mutex_lock(&g_debug.mtx);
            g_debug.frame = dbg;             // fresh Mat each pass: safe handoff
            ++g_debug.seq;
            pthread_mutex_unlock(&g_debug.mtx);
        }
    }

    // Wake the prediction task so it can observe g_running and exit.
    pthread_mutex_lock(&g_pObj.mtx);
    pthread_cond_broadcast(&g_pObj.cond);
    pthread_mutex_unlock(&g_pObj.mtx);
    return nullptr;
}

// ---------------------------------------------------------------------------
// Task: Trajectory Prediction (sporadic)
// Released by a new P_obj sample; minimum inter-arrival time is the camera
// period, and the CBS budget bounds its CPU use (sporadic-server behaviour).
// ---------------------------------------------------------------------------
static void* predictionTask(void* arg) {
    auto& ctx = *static_cast<AppContext*>(arg);
    setDeadlineSched("prediction", Config::TASK_PRED_RUNTIME_NS,
                     Config::TASK_PRED_DEADLINE_NS,
                     Config::TASK_PRED_PERIOD_NS);

    uint64_t lastSeq = 0;
    while (g_running) {
        pthread_mutex_lock(&g_pObj.mtx);
        while (g_pObj.seq == lastSeq && g_running)
            pthread_cond_wait(&g_pObj.cond, &g_pObj.mtx);
        DetectionResult det = g_pObj.det;
        lastSeq = g_pObj.seq;
        pthread_mutex_unlock(&g_pObj.mtx);

        if (!g_running) break;

        MotorCommand cmd = ctx.motor->compute(det.detected, det.centroid, det.boundingBox);
        g_moving = ctx.motor->isMoving();

        pthread_mutex_lock(&g_pTarget.mtx);
        g_pTarget.cmd = cmd;
        pthread_mutex_unlock(&g_pTarget.mtx);
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// Task: Motor Control (periodic, 50 Hz)
// Reads the latest P_target and writes PWM duty cycles to the hat over I2C.
// Honours the start/stop flag from the Operation Interface.
// ---------------------------------------------------------------------------
static void* motorTask(void* arg) {
    auto& ctx = *static_cast<AppContext*>(arg);
    setDeadlineSched("motor", Config::TASK_MOTOR_RUNTIME_NS,
                     Config::TASK_MOTOR_DEADLINE_NS,
                     Config::TASK_MOTOR_PERIOD_NS);

    timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (g_running) {
        MotorCommand cmd;
        pthread_mutex_lock(&g_pTarget.mtx);
        cmd = g_pTarget.cmd;
        pthread_mutex_unlock(&g_pTarget.mtx);

        if (!g_active)
            cmd = stopCommand();

        if (ctx.hwOk)
            hardwareApply(cmd);

        timespecAddNs(next, Config::TASK_MOTOR_PERIOD_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }

    if (ctx.hwOk)
        hardwareApply(stopCommand());        // never leave the wheels spinning
    return nullptr;
}

// ---------------------------------------------------------------------------
// Task: Operation Interface (low-rate)
// stdin: 's' toggles start/stop, 'q' quits. With SHOW_WINDOW it also displays
// the latest debug frame (all highgui calls stay on this one thread).
// ---------------------------------------------------------------------------
static void* opInterfaceTask(void* arg) {
    auto& ctx = *static_cast<AppContext*>(arg);
    (void)ctx;
    setDeadlineSched("opIface", Config::TASK_UI_RUNTIME_NS,
                     Config::TASK_UI_DEADLINE_NS,
                     Config::TASK_UI_PERIOD_NS);

    if (Config::SHOW_WINDOW)
        cv::namedWindow("Trashcan Tracker", cv::WINDOW_AUTOSIZE);

    uint64_t shownSeq = 0;
    timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (g_running) {
        // Non-blocking console input
        pollfd pfd{STDIN_FILENO, POLLIN, 0};
        if (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
            char buf[64];
            ssize_t n = read(STDIN_FILENO, buf, sizeof buf);
            for (ssize_t i = 0; i < n; ++i) {
                if (buf[i] == 'q') g_running = false;
                if (buf[i] == 's') {
                    g_active = !g_active;
                    std::cout << (g_active ? "[OpIface] started\n"
                                           : "[OpIface] stopped\n");
                }
            }
        }

        if (Config::SHOW_WINDOW) {
            cv::Mat dbg;
            uint64_t seq;
            pthread_mutex_lock(&g_debug.mtx);
            seq = g_debug.seq;
            if (seq != shownSeq && !g_debug.frame.empty())
                dbg = g_debug.frame;
            pthread_mutex_unlock(&g_debug.mtx);

            if (!dbg.empty()) {
                shownSeq = seq;

                cv::Point centre(Config::FRAME_W / 2, Config::FRAME_H / 2);
                cv::drawMarker(dbg, centre, {255, 255, 0}, cv::MARKER_CROSS, 24, 1);

                pthread_mutex_lock(&g_pTarget.mtx);
                float distM = g_pTarget.cmd.distanceM;
                bool  stop  = g_pTarget.cmd.stop;
                pthread_mutex_unlock(&g_pTarget.mtx);

                if (!stop && distM > 0.f) {
                    std::string distStr = "Dist: " +
                        std::to_string(static_cast<int>(distM * 100)) + " cm";
                    cv::putText(dbg, distStr, {Config::FRAME_W - 110, 18},
                                cv::FONT_HERSHEY_SIMPLEX, 0.45, {0, 255, 255}, 1);
                }

                cv::putText(dbg, "Frame: " + std::to_string(seq),
                            {5, Config::FRAME_H - 8},
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, {200, 200, 200}, 1);

                cv::imshow("Trashcan Tracker", dbg);
            }
            if (cv::waitKey(1) == 'q')
                g_running = false;
        }

        timespecAddNs(next, Config::TASK_UI_PERIOD_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }
    return nullptr;
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    // Check if we want to compile for desktop or rpi
    int deviceIndex = (argc > 1) ? std::stoi(argv[1]) : Config::DEVICE_INDEX;

    // Initialize camera, detector and motor
    Camera cam(deviceIndex, Config::FRAME_W, Config::FRAME_H, Config::FRAME_FPS);
    auto detector = makeDetector(DetectorKind::Kalman); // TODO: Select algorithm to use for detection
    MotorTranslation motor(Config::FRAME_W, Config::FRAME_H);

    // Check camera errors
    if (!cam.open()) {
        std::cerr << "Could not open camera. Exiting.\n";
        return 1;
    }

    // Initialize hardware drive
    bool hwOk = hardwareInit();

    // without locking rather than crash.
    rlimit memlock{RLIM_INFINITY, RLIM_INFINITY};
    if (setrlimit(RLIMIT_MEMLOCK, &memlock) != 0)
        std::cerr << "setrlimit(RLIMIT_MEMLOCK) failed (" << std::strerror(errno) << ") - skipping mlockall; page faults may add latency.\n";

    // Lock all memory regions to avoid page-fault latency in the RT tasks
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) // TODO:?
        std::cerr << "mlockall failed (" << std::strerror(errno) << ") - page faults may add latency.\n";

    g_pTarget.cmd = stopCommand(); // TODO:?

    AppContext ctx{&cam, detector.get(), &motor, hwOk}; // TODO:?

    std::cout << "Trashcan tracker running. 's' = start/stop, 'q' or Ctrl-C = quit.\n";

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
