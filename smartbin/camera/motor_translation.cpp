#include "motor_translation.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>

/**
 * Mecanum Inverse Kinematics (translate robot velocities into wheel speeds)
 */
static void mecanumIK(float vx, float vy, float &fl, float &fr, float &rl, float &rr) {
    fl = vx + vy; // Front left wheel speed
    fr = vx - vy; // Front right wheel speed
    rl = vx - vy; // Rear left wheel speed
    rr = vx + vy; // Rear right wheel speed
}

/**
 * MotorTranslation constructor
 */
MotorTranslation::MotorTranslation(int frameWidth, int frameHeight, Config cfg) : motor_frame_width(frameWidth), motor_frame_height(frameHeight), motor_config(cfg) {
    float hFovRad = motor_config.hFovDeg * 3.14159265f / 180.f;
    motor_focalLengthPx = (static_cast<float>(motor_frame_width) / 2.f) / std::tan(hFovRad / 2.f);
}

/**
 * Compute motor commands based on object detection results
 */
MotorCommand MotorTranslation::compute(bool detected, cv::Point2f centroid, cv::Rect bbox) {
    MotorCommand cmd{};
    cmd.stop = true;

    // Early exit if no object detected
    if (!detected) {
        return cmd;
    }

    cmd.stop = false;

    // Estimate distance to object
    float pixSize = static_cast<float>(std::max(bbox.width, bbox.height));
    float dist = (pixSize > 0.f) ? (motor_config.objectSizeCm * motor_focalLengthPx) / pixSize : 0.f; // cm
    cmd.distanceM = dist / 100.f;

    // Robot-frame position
    // Camera looks up: optical axis = Z
    // Image vertical -> robot X axis (front/back)
    // Image horizontal -> robot Y axis (left/right), camera is camOffsetY off centre.
    float cmPerPx = dist / motor_focalLengthPx;
    float imgCx = static_cast<float>(motor_frame_width) / 2.f; // Image center x in pixels
    float imgCy = static_cast<float>(motor_frame_height) / 2.f; // Image center y in pixels
    float cx = centroid.x; // Detected object center x in pixels
    float cy = centroid.y; // Detected object center y in pixels
    float robotX = (imgCy - cy) * cmPerPx / 100.f + motor_config.camOffsetY; // How far front/back the object is relative to the robot center in meters
    float robotY = -(cx - imgCx) * cmPerPx / 100.f; // How far left/right the object is relative to the robot center in meters

    // Robot velocity calculation
    float pVy = motor_config.kpX * robotX; // Proportional control for forward/backward motion
    float pVx = motor_config.kpY * robotY; // Proportional control for left/right motion
    cmd.velX = pVx;
    cmd.velY = pVy;

    // Mecanum Inverse Kinematics
    mecanumIK(pVx, pVy, cmd.wheelFL, cmd.wheelFR, cmd.wheelRL, cmd.wheelRR);

    return cmd;
}

