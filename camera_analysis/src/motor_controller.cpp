#include "motor_controller.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

MotorController::MotorController(int frameWidth, int frameHeight, Config cfg)
    : m_fw(frameWidth), m_fh(frameHeight), m_cfg(cfg)
{
    float vFovRad   = m_cfg.vFovDeg * 3.14159265f / 180.f;
    m_focalLengthPx = (static_cast<float>(m_fh) / 2.f) / std::tan(vFovRad / 2.f);
}

static float estimateDistance(float realHeightM, float focalPx, int bboxHeightPx) {
    if (bboxHeightPx <= 0) return 0.f;
    return (realHeightM * focalPx) / static_cast<float>(bboxHeightPx);
}

MotorCommand MotorController::compute(bool detected, cv::Point2f centroid, int bboxHeight) {
    MotorCommand cmd{0.f, 0.f, 0.f, true};

    if (!detected) {
        m_moving = false;
        printCommand(cmd);
        return cmd;
    }

    cmd.stop = false;

    float distM = estimateDistance(m_cfg.objectRealHeightM, m_focalLengthPx, bboxHeight);
    cmd.distanceM = distM;

    if (distM <= 0.05f) {
        m_moving = false;
        printCommand(cmd);
        return cmd;
    }

    const float cx = static_cast<float>(m_fw) / 2.f;
    const float cy = static_cast<float>(m_fh) / 2.f;

    float errX_px = centroid.x - cx;
    if (std::abs(errX_px) > m_cfg.deadZoneX) {
        float errX_m = errX_px * distM / m_focalLengthPx;
        cmd.velX = clamp(errX_m, -m_cfg.maxVelX, m_cfg.maxVelX);
    }

    float errY_px = centroid.y - cy;
    if (std::abs(errY_px) > m_cfg.deadZoneY) {
        float errY_m = errY_px * distM / m_focalLengthPx;
        cmd.velY = clamp(errY_m, -m_cfg.maxVelY, m_cfg.maxVelY);
    }

    m_moving = (cmd.velX != 0.f || cmd.velY != 0.f);
    printCommand(cmd);
    return cmd;
}

void MotorController::printCommand(const MotorCommand& cmd) {
    if (cmd.stop) {
        std::cout << "[MOTOR] STOP\n";
        return;
    }

    const float THRESHOLD = 0.01f;
    float vx = std::abs(cmd.velX) >= THRESHOLD ? cmd.velX : 0.f;
    float vy = std::abs(cmd.velY) >= THRESHOLD ? cmd.velY : 0.f;

    std::string dir;
    if      (vy >  0.f) dir += "FWD ";
    else if (vy <  0.f) dir += "BWD ";
    if      (vx >  0.f) dir += "RIGHT";
    else if (vx <  0.f) dir += "LEFT ";
    if (dir.empty())    dir  = "HOLD";

    bool moving = (vx != 0.f || vy != 0.f);
    int  angle  = 0;
    if (moving) {
        float a = std::atan2(vy, vx) * 180.f / 3.14159265f;
        if (a < 0.f)    a += 360.f;
        if (a >= 360.f) a  = 0.f;
        angle = static_cast<int>(std::round(a));
    }

    std::cout << std::fixed << std::setprecision(3)
              << "[MOTOR] velX=" << std::setw(7) << cmd.velX << " m/s"
              << "  velY=" << std::setw(7) << cmd.velY << " m/s"
              << "  dist=" << std::setw(5) << cmd.distanceM << " m"
              << "  (" << dir << ")";

    if (moving)
        std::cout << "  angle=" << std::setw(3) << angle << "deg";
    else
        std::cout << "  angle= N/A";

    std::cout << "\n";
}

float MotorController::clamp(float v, float lo, float hi) const {
    return v < lo ? lo : (v > hi ? hi : v);
}