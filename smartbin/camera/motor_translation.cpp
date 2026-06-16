#include "motor_translation.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

/**
 * Mecanum Inverse Kinematics (translate robot velocities into wheel speeds)
 */
static void mecanumIK(float vx, float vy, float& fl, float& fr, float& rl, float& rr) {
    fl = vx + vy; // front left wheel speed
    fr = vx - vy; // front right wheel speed
    rl = vx - vy; // rear left wheel speed
    rr = vx + vy; // rear right wheel speed
}

/**
 * MotorTranslation constructor
 */
MotorTranslation::MotorTranslation(int frameWidth, int frameHeight, Config cfg) : m_fw(frameWidth), m_fh(frameHeight), m_cfg(cfg){
    float hFovRad   = m_cfg.hFovDeg * 3.14159265f / 180.f;
    m_focalLengthPx = (static_cast<float>(m_fw) / 2.f) / std::tan(hFovRad / 2.f);
}

/**
 * Compute motor commands based on object detection results
 */
MotorCommand MotorTranslation::compute(bool detected, cv::Point2f centroid, cv::Rect bbox) {
    MotorCommand cmd{};
    cmd.stop = true;

    // Early exit if no object detected
    if (!detected) {
        printCommand(cmd);
        return cmd;
    }

    cmd.stop = false;

    // Estimate distance to object using the pinhole model:
    // distance = (real_size * focal_length) / pixel_size
    // Pinhole Model background: HediVision, "Pinhole Camera Model" https://hedivision.github.io/Pinhole.html
    float pixSize = static_cast<float>(std::max(bbox.width, bbox.height));
    float dist    = (pixSize > 0.f) ? (m_cfg.objectSizeCm * m_focalLengthPx) / pixSize : 0.f; // cm
    cmd.distanceM = dist / 100.f;
    
    // Robot-frame position 
    // Camera looks up: optical axis = Z (height above floor).
    // Image vertical -> robot X axis (front/back).
    // Image horizontal -> robot Y axis (left/right), camera is camOffsetY off centre.
    float cmPerPx = dist / m_focalLengthPx;
    float imgCx   = static_cast<float>(m_fw) / 2.f; // image center x in pixels
    float imgCy   = static_cast<float>(m_fh) / 2.f; // image center y in pixels
    float cx      = centroid.x; // detected object center x in pixels
    float cy      = centroid.y; // detected object center y in pixels
    float robotX  = (imgCy - cy) * cmPerPx / 100.f + m_cfg.camOffsetY; // how far front/back the object is relative to the robot center in m (positive = in front)
    float robotY  = -(cx - imgCx) * cmPerPx / 100.f; // How far left/right the object is relative to the robot center in m (positive = left)

    // Robot velocity calculation
    float pVy = m_cfg.kpX * robotX; // proportional control for forward/backward motion
    float pVx = m_cfg.kpY * robotY; // proportional control for left/right motion
    cmd.velX = pVx;
    cmd.velY = pVy;

    // Mecanum Inverse Kinematics
    mecanumIK(pVx, pVy, cmd.wheelFL, cmd.wheelFR, cmd.wheelRL, cmd.wheelRR);

    printCommand(cmd); // TODO: Remove? (DEBUG)
    return cmd;
}

/**
 * Debug function to see motor commands
 */
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
