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
    constexpr float H_FOV_DEG    = 78.f;    // horizontal FOV of your phone camera
    constexpr float V_FOV_DEG    = 43.9f;   // = H_FOV * (9/16) for 16:9 sensor

    // ── Detector ─────────────────────────────────────────────────────────────
    constexpr int   DET_MIN_AREA        = 400;  // px² — lower to match smaller resolution
    constexpr int   DET_HISTORY         = 60;   // shorter history = faster background re-learn
    constexpr float DET_VAR_THRESHOLD   = 40.f; // MOG2 sensitivity — lower = more sensitive
    constexpr int   DET_ERODE_ITER      = 1;
    constexpr int   DET_DILATE_ITER     = 2;

    // How many frames to skip detection after the can stops moving.
    // Gives MOG2 time to re-learn the new background before trusting detections.
    constexpr int   DET_COOLDOWN_FRAMES = 3;

    // MOG2 learning rate while the can is moving (fast re-learn of new background).
    // -1 = automatic (used when stationary).
    constexpr float DET_MOVING_LEARN_RATE = 0.5f;

    // ── Motor ─────────────────────────────────────────────────────────────────
    constexpr float MOT_DEAD_ZONE_X      = 15.f;  // px — scale down with resolution
    constexpr float MOT_DEAD_ZONE_Y      = 15.f;
    constexpr float MOT_MAX_VEL_X        = 0.1f;  // m/s — max strafe speed
    constexpr float MOT_MAX_VEL_Y        = 0.1f;  // m/s — max fwd/back speed
    constexpr float MOT_OBJECT_HEIGHT_M  = 0.15f; // real height of tracked object (m)
                                                   // tennis ball=0.067, bottle=0.22

    // ── Display ───────────────────────────────────────────────────────────────
    constexpr bool  SHOW_WINDOW = true;   // keep false on Pi — imshow is expensive

} // namespace Config