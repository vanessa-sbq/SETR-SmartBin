#pragma once
#include "config.hpp"
#include "motor_command.hpp"
#include <chrono>
#include <opencv2/core/types.hpp>

struct MotorConfig {
    float objectSizeCm = Config::OBJECT_DIAMETER_M * 100.f; // real-world object size in cm
    float kpX = Config::MOT_KP_X; // proportional gain for forward/backward motion
    float kpY = Config::MOT_KP_Y; // proportional gain for left/right motion
    float hFovDeg = Config::CAM_H_FOV_DEG; // horizontal field of view of the camera in degrees
    float camOffsetY = Config::MECH_CAM_OFFSET_Y; // camera offset from robot center in m (positive = forward)
};

class MotorTranslation {
  public:
    using Config = MotorConfig;
    explicit MotorTranslation(int frameWidth, int frameHeight, Config cfg = Config{});
    MotorCommand compute(bool detected, cv::Point2f centroid, cv::Rect bbox);

  private:
    int motor_frame_width, motor_frame_height; // frame width and height in pixels
    Config motor_config; // configuration parameters
    float motor_focalLengthPx; // focal length in pixels, derived from frame width and horizontal FOV
};
