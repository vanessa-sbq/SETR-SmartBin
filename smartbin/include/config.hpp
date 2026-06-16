#pragma once

// Central configuration (edit this file only).
// All subsystems (main, motor, detector, camera) read their defaults from here.

namespace Config {
    // Camera
    constexpr int   DEVICE_INDEX = 0;       // 0 = first camera; override via argv
    constexpr int   FRAME_W      = 640;     // 16:9 width  — lower = faster MOG2
    constexpr int   FRAME_H      = 480;     // 16:9 height — quarter pixels vs 640×360
    constexpr int   FRAME_FPS    = 30;

    // Lens FOV (degrees)
    constexpr float H_FOV_DEG    = 102.f;   // horizontal FOV — Pi Camera Module 3 Wide
    constexpr float V_FOV_DEG    = 76.5f;   // = H_FOV * (3/4) for 4:3 sensor (640×480)

    // Detector
    constexpr int   DET_MIN_AREA        = 80;   // px²
    constexpr int   DET_ERODE_ITER      = 1;
    constexpr int   DET_DILATE_ITER     = 2;

    // Learned yellow colour model — Hue+Saturation Gaussian gate.
    // Replaces the fixed HSV inRange box: a pixel is "yellow" when its
    // Mahalanobis² distance to (mean, covariance) in (H,S) space is below the
    // gate. Defaults below are seeded from the legacy hue band (12..42, mid 27)
    // and saturation band (80..255, mid 167); refit at runtime with
    // HsvDetector::trainColorModel() / YellowColorModel::load().
    constexpr float DET_COLOR_MEAN_H = 27.f;   // mean Hue        (OpenCV 8-bit, 0..179)
    constexpr float DET_COLOR_MEAN_S = 167.f;  // mean Saturation (0..255)
    constexpr float DET_COLOR_VAR_H  = 56.f;   // Hue variance        (~±15 → σ≈7.5)
    constexpr float DET_COLOR_VAR_S  = 1870.f; // Saturation variance (~±87 → σ≈43)
    constexpr float DET_COLOR_GATE   = 9.0f;   // Mahalanobis² cutoff (~3σ)
    constexpr int   DET_COLOR_MIN_V  = 60;     // reject near-black pixels (unstable hue)

    constexpr float DET_MIN_CIRCULARITY = 0.4f; // discard non-round blobs
    constexpr float DET_MAX_DIST_M      = 2.0f; // ignore detections beyond this

    // Kalman trajectory detector
    // Frame indices are relative to the first detection of an object (= frame 0).
    constexpr int   KAL_INIT_FRAME    = 10;   // seed trajectory (frames 0..N)
    constexpr int   KAL_MID_FRAME     = 5;    // intermediate point used at init
    constexpr int   KAL_CORRECT_FRAME = 15;   // refine trajectory here
    constexpr int   KAL_LOST_FRAMES   = 15;   // missing longer than this -> new object
    constexpr float KAL_LOOKAHEAD     = 8.f;  // frames to lead the predicted intercept
    constexpr float KAL_PROCESS_NOISE = 1e-2f;// model trust (lower = trust motion model)
    constexpr float KAL_MEAS_NOISE    = 1e-1f;// measurement trust (lower = trust camera)

    // Motor/P-controller
    constexpr float MOT_OBJECT_HEIGHT_M  = 0.25f; // real size of tracked object (m) (tennis ball=0.067, bottle=0.22)
    constexpr float MOT_KP_X             = 0.5f;  // gain: robot_x (m) → forward vel (m/s)
    constexpr float MOT_KP_Y             = 0.5f;  // gain: robot_y (m) → lateral vel (m/s)
    constexpr float MOT_MAX_V            = 0.3f;  // m/s velocity cap
    constexpr float MOT_EMA_ALPHA        = 0.4f;  // centroid/distance smoothing factor

    // Mecanum chassis (OSOYOO FlexiRover 2024007500)
    constexpr float MECH_WHEEL_RADIUS    = 0.040f;  // m  (80 mm diameter)
    constexpr float MECH_LX              = 0.0475f; // m  half wheelbase
    constexpr float MECH_LY              = 0.103f;  // m  half track width
    constexpr float MECH_MAX_WHEEL_SPD   = 1.0f;    // normalized -1..1
    constexpr float MECH_CAM_OFFSET_Y    = 0.13f;   // m  camera Y offset from robot centre

    // Display
    constexpr bool  SHOW_WINDOW = false; // keep false on Pi (imshow is expensive)

    // Real-time tasks — SCHED_DEADLINE parameters in nanoseconds.
    // Kernel requires runtime (C) <= deadline (D) <= period (T).
    // Vision Detection: periodic, paced by the camera (30 fps)
    constexpr long long TASK_VISION_RUNTIME_NS  =  20'000'000;
    constexpr long long TASK_VISION_DEADLINE_NS =  33'333'333;
    constexpr long long TASK_VISION_PERIOD_NS   =  33'333'333;

    // Trajectory Prediction: sporadic, released by a detection event;
    // minimum inter-arrival time = camera period. D < T: a prediction
    // delivered close to the next frame is useless.
    constexpr long long TASK_PRED_RUNTIME_NS    =   3'000'000;
    constexpr long long TASK_PRED_DEADLINE_NS   =  10'000'000;
    constexpr long long TASK_PRED_PERIOD_NS     =  33'333'333;

    // Motor Control: periodic 50 Hz control loop. D < T so urgent commands
    // (e.g. stop) land early in the cycle.
    constexpr long long TASK_MOTOR_RUNTIME_NS   =   2'000'000;
    constexpr long long TASK_MOTOR_DEADLINE_NS  =   5'000'000;
    constexpr long long TASK_MOTOR_PERIOD_NS    =  20'000'000;

    // Operation Interface - ESP8266 WiFi remote (see wifi_raspberry/wifi.cpp).
    // Button names are matched case-insensitively against the CSV the remote
    // sends. Port matches the reference server.
    constexpr int         OPIF_PORT       = 6767;    // TCP listen port
    constexpr const char* OPIF_BTN_TOGGLE = "START"; // toggles the start/stop flag
    constexpr const char* OPIF_BTN_QUIT   = "QUIT";  // requests shutdown

    // Operation Interface: low-rate housekeeping (start/stop, debug display)
    constexpr long long TASK_UI_RUNTIME_NS      =   5'000'000;
    constexpr long long TASK_UI_DEADLINE_NS     = 100'000'000;
    constexpr long long TASK_UI_PERIOD_NS       = 100'000'000;

} // namespace Config