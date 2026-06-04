#pragma once
#include <chrono>
#include <opencv2/core/types.hpp>
#include "config.hpp"
#include "motor_command.hpp"

struct MotorConfig {
    float objectSizeCm  = Config::MOT_OBJECT_HEIGHT_M * 100.f;
    float kpX           = Config::MOT_KP_X;
    float kpY           = Config::MOT_KP_Y;
    float maxV          = Config::MOT_MAX_V;
    float emaAlpha      = Config::MOT_EMA_ALPHA;
    float hFovDeg       = Config::H_FOV_DEG;
    float wheelRadius   = Config::MECH_WHEEL_RADIUS;
    float lx            = Config::MECH_LX;
    float ly            = Config::MECH_LY;
    float maxWheelSpeed = Config::MECH_MAX_WHEEL_SPD;
    float camOffsetY    = Config::MECH_CAM_OFFSET_Y;
};

class MotorTranslation {
public:
    using Config = MotorConfig;

    explicit MotorTranslation(int frameWidth, int frameHeight, Config cfg = Config{});

    MotorCommand compute(bool detected, cv::Point2f centroid, cv::Rect bbox);

    bool isMoving() const { return m_moving; }

    static void printCommand(const MotorCommand& cmd);

private:
    int    m_fw, m_fh;
    Config m_cfg;
    float  m_focalLengthPx;
    bool   m_moving = false;

    float m_smoothCx   = -1.f;   // -1 = uninitialized
    float m_smoothCy   = -1.f;
    float m_smoothDist = 0.f;
    float m_prevCx     = 0.f;
    float m_prevCy     = 0.f;
    float m_prevDist   = 0.f;
    bool  m_hasPrev    = false;
    std::chrono::steady_clock::time_point m_prevTime;

    float clamp(float v, float lo, float hi) const;
};
