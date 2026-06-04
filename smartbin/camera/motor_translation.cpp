#include "motor_translation.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>

static void mecanumIK(float vx, float vy, float omega,
                      float wheelRadius, float lx, float ly, float maxWheelSpd,
                      float& fl, float& fr, float& rl, float& rr)
{
    float k = lx + ly;
    fl = (vx + vy - omega * k) / wheelRadius;
    fr = (vx - vy + omega * k) / wheelRadius;
    rl = (vx - vy - omega * k) / wheelRadius;
    rr = (vx + vy + omega * k) / wheelRadius;

    float maxW = std::max({std::abs(fl), std::abs(fr), std::abs(rl), std::abs(rr), 1e-9f});
    float limit = maxWheelSpd / wheelRadius;
    if (maxW > limit) {
        float scale = limit / maxW;
        fl *= scale; fr *= scale; rl *= scale; rr *= scale;
    }

    float norm = wheelRadius / maxWheelSpd;
    fl *= norm; fr *= norm; rl *= norm; rr *= norm;
}

MotorTranslation::MotorTranslation(int frameWidth, int frameHeight, Config cfg)
    : m_fw(frameWidth), m_fh(frameHeight), m_cfg(cfg)
{
    float hFovRad   = m_cfg.hFovDeg * 3.14159265f / 180.f;
    m_focalLengthPx = (static_cast<float>(m_fw) / 2.f) / std::tan(hFovRad / 2.f);
}

MotorCommand MotorTranslation::compute(bool detected, cv::Point2f centroid, cv::Rect bbox) {
    MotorCommand cmd{};
    cmd.stop = true;

    if (!detected) {
        m_moving   = false;
        m_smoothCx = -1.f;
        m_hasPrev  = false;
        printCommand(cmd);
        return cmd;
    }

    cmd.stop = false;

    // ── EMA smoothing ─────────────────────────────────────────────────────────
    float cx      = centroid.x;
    float cy      = centroid.y;
    float pixSize = static_cast<float>(std::max(bbox.width, bbox.height));
    float dist    = (pixSize > 0.f)
                    ? (m_cfg.objectSizeCm * m_focalLengthPx) / pixSize
                    : 0.f;   // cm

    if (m_smoothCx < 0.f) {
        m_smoothCx = cx;  m_smoothCy = cy;  m_smoothDist = dist;
    } else {
        const float a = m_cfg.emaAlpha;
        m_smoothCx   = a * cx   + (1.f - a) * m_smoothCx;
        m_smoothCy   = a * cy   + (1.f - a) * m_smoothCy;
        if (dist > 0.f)
            m_smoothDist = a * dist + (1.f - a) * m_smoothDist;
    }

    cmd.distanceM = m_smoothDist / 100.f;

    // ── Robot-frame position ──────────────────────────────────────────────────
    // Camera looks up: optical axis = Z (height above floor).
    // Image vertical  → robot X axis (front/back).
    // Image horizontal → robot Y axis (left/right), camera 13 cm off centre.
    float cmPerPx = m_smoothDist / m_focalLengthPx;
    float imgCx   = static_cast<float>(m_fw) / 2.f;
    float imgCy   = static_cast<float>(m_fh) / 2.f;
    float robotX  = (imgCy - m_smoothCy) * cmPerPx / 100.f;
    float robotY  = -(m_smoothCx - imgCx) * cmPerPx / 100.f + m_cfg.camOffsetY;

    // ── Ball velocity (pixel-delta / Δt) ──────────────────────────────────────
    auto now = std::chrono::steady_clock::now();
    if (m_hasPrev && m_prevDist > 0.f) {
        float dt = std::chrono::duration<float>(now - m_prevTime).count();
        if (dt > 0.f) {
            cmd.ballVx = -(m_smoothCy - m_prevCy) * cmPerPx / 100.f / dt;
            cmd.ballVy = -(m_smoothCx - m_prevCx) * cmPerPx / 100.f / dt;
            cmd.ballVz = (m_smoothDist - m_prevDist) / 100.f / dt;
        }
    }
    m_prevCx   = m_smoothCx;
    m_prevCy   = m_smoothCy;
    m_prevDist = m_smoothDist;
    m_prevTime = now;
    m_hasPrev  = true;

    // ── P-controller ──────────────────────────────────────────────────────────
    float pVy  = clamp( m_cfg.kpX * robotX, -m_cfg.maxV, m_cfg.maxV);
    float pVx  = -clamp(-m_cfg.kpY * robotY, -m_cfg.maxV, m_cfg.maxV);

    cmd.velX = pVx;
    cmd.velY = pVy;

    // ── Mecanum IK ────────────────────────────────────────────────────────────
    mecanumIK(pVx, pVy, 0.f,
              1.0, m_cfg.lx, m_cfg.ly, m_cfg.maxWheelSpeed,
              cmd.wheelFL, cmd.wheelFR, cmd.wheelRL, cmd.wheelRR);

    m_moving = (cmd.wheelFL != 0.f || cmd.wheelFR != 0.f ||
                cmd.wheelRL != 0.f || cmd.wheelRR != 0.f);
    printCommand(cmd);
    return cmd;
}

void MotorTranslation::printCommand(const MotorCommand& cmd) {
    if (cmd.stop) {
        std::cout << "[MOTOR] STOP\n";
        return;
    }
    std::cout << std::fixed << std::setprecision(3)
              << "[MOTOR] dist=" << std::setw(6) << cmd.distanceM << "m"
              << "  velX=" << std::setw(7) << cmd.velX
              << "  velY=" << std::setw(7) << cmd.velY;

    if (cmd.ballVx != 0.f || cmd.ballVy != 0.f || cmd.ballVz != 0.f) {
        std::cout << std::setprecision(2)
                  << "  ball vx=" << std::setw(6) << cmd.ballVx
                  << " vy=" << std::setw(6) << cmd.ballVy
                  << " vz=" << std::setw(6) << cmd.ballVz;
    }

    std::cout << std::setprecision(2)
              << "  | FL=" << std::setw(5) << cmd.wheelFL
              << " FR=" << std::setw(5) << cmd.wheelFR
              << " RL=" << std::setw(5) << cmd.wheelRL
              << " RR=" << std::setw(5) << cmd.wheelRR
              << "\n";
}

float MotorTranslation::clamp(float v, float lo, float hi) const {
    return v < lo ? lo : (v > hi ? hi : v);
}
