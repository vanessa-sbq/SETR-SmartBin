#pragma once
#include <opencv2/core/types.hpp>
#include "config.hpp"

// Holonomic (omnidirectional) motor command.
// All velocity values are in m/s.
//   velX: strafe  — negative = left,    positive = right
//   velY: forward — negative = retreat, positive = advance
// Diagonal movement happens naturally when both are nonzero.
struct MotorCommand {
    float velX;         // strafe      (m/s)
    float velY;         // fwd/back    (m/s)
    float distanceM;    // estimated object distance (m), 0 if unknown
    bool  stop;
};

struct MotorConfig {
    float deadZoneX         = Config::MOT_DEAD_ZONE_X;
    float deadZoneY         = Config::MOT_DEAD_ZONE_Y;
    float maxVelX           = Config::MOT_MAX_VEL_X;
    float maxVelY           = Config::MOT_MAX_VEL_Y;
    float hFovDeg           = Config::H_FOV_DEG;
    float vFovDeg           = Config::V_FOV_DEG;
    float objectRealHeightM = Config::MOT_OBJECT_HEIGHT_M;
};

class MotorController {
public:
    using Config = MotorConfig;

    explicit MotorController(int frameWidth, int frameHeight, Config cfg = Config{});

    MotorCommand compute(bool detected, cv::Point2f centroid, int bboxHeight);

    // True when the last computed command has non-zero velocity.
    // Used by main to tell the Detector whether the can is moving.
    bool isMoving() const { return m_moving; }

    static void printCommand(const MotorCommand& cmd);

private:
    int    m_fw, m_fh;
    Config m_cfg;
    float  m_focalLengthPx;
    bool   m_moving = false;

    float clamp(float v, float lo, float hi) const;
};