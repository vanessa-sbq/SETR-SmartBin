#pragma once

// Central configuration (edit this file only).
// All subsystems (main, motor, detector, camera) read their defaults from here.

namespace Config {
// Camera
constexpr int DEVICE_INDEX = 0; // Index. Raspberry may have more than one camera attached to it.
constexpr int CAM_VIDEO_WIDTH = 640;
constexpr int CAM_VIDEO_HEIGHT = 480;
constexpr int CAM_FPS = 30;
constexpr float CAM_H_FOV_DEG = 102.f; // horizontal FOV — Pi Camera Module 3 Wide

// Detector
constexpr float OBJECT_DIAMETER_M = 0.25f; // Object size
constexpr int DET_MIN_AREA = 80; // px²
constexpr int DET_ERODE_ITER = 1;
constexpr int DET_DILATE_ITER = 2;

// HSV
constexpr float DET_COLOR_MEAN_HUE = 27.f;
constexpr float DET_COLOR_MEAN_SATURATION = 167.f;
constexpr float DET_COLOR_VARIANCE_HUE = 56.f;
constexpr float DET_COLOR_VARIANCE_SATURATION = 1870.f;
constexpr float DET_COLOR_THREASHOLD = 9.0f;
constexpr int DET_COLOR_MIN_HUE_VALUE = 60; // Helps reject near-black pixels
constexpr float DET_MIN_CIRCULARITY = 0.4f; // discard non-round blobs
constexpr float DET_MAX_DIST_M = 2.0f; // ignore detections beyond this

// Motor / Trash Values
constexpr float MOT_KP_X = 0.5f; // gain: robot_x (m) → forward vel (m/s)
constexpr float MOT_KP_Y = 0.5f; // gain: robot_y (m) → lateral vel (m/s)
constexpr float MECH_CAM_OFFSET_Y = 0.13f; // m  camera Y offset from robot centre

// Display
constexpr bool SHOW_WINDOW = false;

// Real-time tasks — SCHED_DEADLINE parameters in nanoseconds.
// Kernel requires runtime (C) <= deadline (D) <= period (T).
// Vision Detection: periodic, paced by the camera (30 fps)
constexpr long long TASK_VISION_RUNTIME_NS = 20'000'000;
constexpr long long TASK_VISION_DEADLINE_NS = 33'333'333;
constexpr long long TASK_VISION_PERIOD_NS = 33'333'333;

// Trajectory Prediction: sporadic, released by a detection event;
// minimum inter-arrival time = camera period. D < T: a prediction
// delivered close to the next frame is useless.
constexpr long long TASK_PRED_RUNTIME_NS = 3'000'000;
constexpr long long TASK_PRED_DEADLINE_NS = 10'000'000;
constexpr long long TASK_PRED_PERIOD_NS = 33'333'333;

// Motor Control: periodic 50 Hz control loop. D < T so urgent commands
// (e.g. stop) land early in the cycle.
constexpr long long TASK_MOTOR_RUNTIME_NS = 2'000'000;
constexpr long long TASK_MOTOR_DEADLINE_NS = 5'000'000;
constexpr long long TASK_MOTOR_PERIOD_NS = 20'000'000;

// Operation Interface - ESP8266 WiFi remote (see wifi_raspberry/wifi.cpp).
// Button names are matched case-insensitively against the CSV the remote
// sends. Port matches the reference server.
constexpr int OPIF_PORT = 6767; // TCP listen port
constexpr const char *OPIF_BTN_TOGGLE = "START"; // toggles the start/stop flag
constexpr const char *OPIF_BTN_QUIT = "QUIT"; // requests shutdown

// Operation Interface: low-rate housekeeping (start/stop, debug display)
constexpr long long TASK_UI_RUNTIME_NS = 5'000'000;
constexpr long long TASK_UI_DEADLINE_NS = 100'000'000;
constexpr long long TASK_UI_PERIOD_NS = 100'000'000;

} // namespace Config