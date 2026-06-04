#pragma once

// ═════════════════════════════════════════════════════════════════════════════
// Central configuration — edit this file only.
// All subsystems (main, motor, detector, camera) read their defaults from here.
// ═════════════════════════════════════════════════════════════════════════════

namespace Config {

    // ── Camera ────────────────────────────────────────────────────────────────
    constexpr int   DEVICE_INDEX = 0;       // 0 = first camera; override via argv
    constexpr int   FRAME_W      = 640;     // 16:9 width  — lower = faster MOG2
    constexpr int   FRAME_H      = 480;     // 16:9 height — quarter pixels vs 640×360
    constexpr int   FRAME_FPS    = 30;

    // ── Lens FOV (degrees) ───────────────────────────────────────────────────
    constexpr float H_FOV_DEG    = 102.f;   // horizontal FOV — Pi Camera Module 3 Wide
    constexpr float V_FOV_DEG    = 76.5f;   // = H_FOV * (3/4) for 4:3 sensor (640×480)

    // ── Detector ─────────────────────────────────────────────────────────────
    constexpr int   DET_MIN_AREA        = 80;   // px²
    constexpr int   DET_ERODE_ITER      = 1;
    constexpr int   DET_DILATE_ITER     = 2;

    // HSV color filter (orange/yellow ball)
    constexpr int   DET_HSV_LO_H        = 12;
    constexpr int   DET_HSV_LO_S        = 80;
    constexpr int   DET_HSV_LO_V        = 80;
    constexpr int   DET_HSV_HI_H        = 42;
    constexpr int   DET_HSV_HI_S        = 255;
    constexpr int   DET_HSV_HI_V        = 255;

    constexpr float DET_MIN_CIRCULARITY = 0.4f; // discard non-round blobs
    constexpr float DET_MAX_DIST_M      = 2.0f; // ignore detections beyond this

    // ── Motor / P-controller ─────────────────────────────────────────────────
    constexpr float MOT_OBJECT_HEIGHT_M  = 0.25f; // real size of tracked object (m)
                                                   // tennis ball=0.067, bottle=0.22
    constexpr float MOT_KP_X             = 0.5f;  // gain: robot_x (m) → forward vel (m/s)
    constexpr float MOT_KP_Y             = 0.5f;  // gain: robot_y (m) → lateral vel (m/s)
    constexpr float MOT_MAX_V            = 0.3f;  // m/s velocity cap
    constexpr float MOT_EMA_ALPHA        = 0.4f;  // centroid/distance smoothing factor

    // ── Mecanum chassis (OSOYOO FlexiRover 2024007500) ───────────────────────
    constexpr float MECH_WHEEL_RADIUS    = 0.040f;  // m  (80 mm diameter)
    constexpr float MECH_LX              = 0.0475f; // m  half wheelbase
    constexpr float MECH_LY              = 0.103f;  // m  half track width
    constexpr float MECH_MAX_WHEEL_SPD   = 1.0f;    // normalized -1..1
    constexpr float MECH_CAM_OFFSET_Y    = 0.13f;   // m  camera Y offset from robot centre

    // ── Display ───────────────────────────────────────────────────────────────
    constexpr bool  SHOW_WINDOW = true;   // keep false on Pi — imshow is expensive

} // namespace Config