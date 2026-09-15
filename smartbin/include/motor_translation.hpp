#pragma once
#include "config.hpp"
#include "motor_command.hpp"
#include <chrono>
#include <opencv2/core/types.hpp>

struct MotorConfig {
    float objectSizeCm = Config::OBJECT_DIAMETER_M * 100.f; // Real-world object size in cm
    float kpX = Config::MOT_KP_X; // Proportional gain for forward/backward motion
    float kpY = Config::MOT_KP_Y; // Proportional gain for left/right motion
    float hFovDeg = Config::CAMERA_HEIGHT_FOV_DEGREES; // Horizontal field of view of the camera in degrees
    float camOffsetY = Config::CAM_OFFSET_Y; // Camera offset from robot center in meters (positive = forward)
};

class MotorTranslation {
  public:
    using Config = MotorConfig;
    explicit MotorTranslation(int frameWidth, int frameHeight, Config cfg = Config{});
    MotorCommand compute(bool detected, cv::Point2f centroid, cv::Rect bbox);

  private:
    int motor_frame_width, motor_frame_height; // Frame width and height in pixels
    Config motor_config; // Configuration parameters
    float motor_focalLengthPx; // Focal length in pixels, derived from frame width and horizontal FOV
};
