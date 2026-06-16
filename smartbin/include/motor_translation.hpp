#pragma once
#include <chrono>
#include <opencv2/core/types.hpp>
#include "config.hpp"
#include "motor_command.hpp"

struct MotorConfig {
    float objectSizeCm  = Config::MOT_OBJECT_HEIGHT_M * 100.f; // real-world object size in cm 
    float kpX           = Config::MOT_KP_X; // proportional gain for forward/backward motion
    float kpY           = Config::MOT_KP_Y; // proportional gain for left/right motion
    float hFovDeg       = Config::H_FOV_DEG; // horizontal field of view of the camera in degrees
    float camOffsetY    = Config::MECH_CAM_OFFSET_Y; // camera offset from robot center in m (positive = forward)
};

class MotorTranslation {
public:
    using Config = MotorConfig;
    explicit MotorTranslation(int frameWidth, int frameHeight, Config cfg = Config{});
    MotorCommand compute(bool detected, cv::Point2f centroid, cv::Rect bbox);
    bool isMoving() const { return m_moving; }
    static void printCommand(const MotorCommand& cmd);

private:
    int    m_fw, m_fh; // frame width and height in pixels
    Config m_cfg; // configuration parameters
    float  m_focalLengthPx; // focal length in pixels, derived from frame width and horizontal FOV
    bool   m_moving = false; // whether the bin is currently moving
};
