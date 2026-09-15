#pragma once

namespace Config {
// Camera
constexpr int DEVICE_INDEX = 0;
constexpr int CAM_VIDEO_WIDTH = 1280;
constexpr int CAM_VIDEO_HEIGHT = 960;
constexpr int CAM_FPS = 30;
constexpr float CAMERA_HEIGHT_FOV_DEGREES = 102.f;

// Detector
constexpr float OBJECT_DIAMETER_M = 0.15f; // Object size
constexpr int DETECTOR_MIN_AREA = 80; // px^2
constexpr int DETECTOR_ERODE_ITER = 1;
constexpr int DETECTOR_DILATE_ITER = 2;

// HSV
constexpr float DET_COLOR_MEAN_HUE = 27.f;
constexpr float DET_COLOR_MEAN_SATURATION = 167.f;
constexpr float DET_COLOR_VARIANCE_HUE = 56.f;
constexpr float DET_COLOR_VARIANCE_SATURATION = 1870.f;
constexpr float DET_COLOR_THREASHOLD = 9.0f;
constexpr int DETECTOR_COLOR_MIN_HUE_VALUE = 60; // Helps reject near-black pixels
constexpr float DETECTOR_MIN_CIRCULARITY = 0.4f; // Discard non-round blobs
constexpr float DETECTOR_MAX_DIST_M = 2.0f; // Ignore detections beyond this

// Motor / Trash Values
constexpr float MOT_KP_X = 0.5f; // Gain: robot_x (m) -> forward vel (m/s)
constexpr float MOT_KP_Y = 0.5f; // Gain: robot_y (m) -> lateral vel (m/s)
constexpr float CAM_OFFSET_Y = 0.13f; // Camera Y offset from robot centre (meters)

// Operation Interface
// Button names are matched case-insensitively against the CSV that the esp8266 sends.
constexpr int OPIF_PORT = 6767;
constexpr const char *OPIF_BTN_TOGGLE = "START"; // toggles the start/stop flag
constexpr const char *OPIF_BTN_QUIT = "B"; // requests shutdown

// Display
constexpr bool SHOW_WINDOW = false;

// Real-time tasks: SCHED_DEADLINE parameters in nanoseconds.
// Kernel requires runtime (C) <= deadline (D) <= period (T).

// Vision Detection: periodic, paced by the camera (30 fps)
constexpr long long TASK_VISION_RUNTIME_NS = 30'000'000; // 30ms
constexpr long long TASK_VISION_DEADLINE_NS = 50'000'000; // 50ms
constexpr long long TASK_VISION_PERIOD_NS = 50'000'000; // 50ms

// Trajectory Prediction: sporadic, released by a detection event.
// minimum inter-arrival time = camera period. D < T: a prediction
// delivered close to the next frame is useless.
constexpr long long TASK_PRED_RUNTIME_NS = 3'000'000; // 3ms
constexpr long long TASK_PRED_DEADLINE_NS = 10'000'000; // 10ms
constexpr long long TASK_PRED_PERIOD_NS = 33'333'333; // 33ms

// Motor Control: periodic 50 Hz control loop. D < T so urgent commands
// (e.g. stop) land early in the cycle.
constexpr long long TASK_MOTOR_RUNTIME_NS = 5'000'000; // 5ms
constexpr long long TASK_MOTOR_DEADLINE_NS = 20'000'000; // 20ms
constexpr long long TASK_MOTOR_PERIOD_NS = 20'000'000; // 20ms

// Operation Interface task
constexpr long long TASK_UI_RUNTIME_NS = 5'000'000; // 5ms
constexpr long long TASK_UI_DEADLINE_NS = 100'000'000; // 100ms
constexpr long long TASK_UI_PERIOD_NS = 100'000'000; // 100ms

}