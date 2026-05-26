#pragma once
#include <opencv2/opencv.hpp>
#include "config.hpp"

struct DetectionResult {
    bool        detected;
    cv::Point2f centroid;
    float       area;
    cv::Rect    boundingBox;
};

struct DetectorConfig {
    int   history          = Config::DET_HISTORY;
    float varThreshold     = Config::DET_VAR_THRESHOLD;
    int   minArea          = Config::DET_MIN_AREA;
    int   dilateIterations = Config::DET_DILATE_ITER;
    int   erodeIterations  = Config::DET_ERODE_ITER;
    int   cooldownFrames   = Config::DET_COOLDOWN_FRAMES;
    float movingLearnRate  = Config::DET_MOVING_LEARN_RATE;
};

class Detector {
public:
    using Config = DetectorConfig;

    explicit Detector(Config cfg = Config{});

    // isMoving: true when the can is currently executing a motor command.
    // While moving:  MOG2 runs with a high learning rate to absorb background
    //                shift, but detect() always returns detected=false.
    // After stopping: detection is suppressed for cooldownFrames frames so
    //                 MOG2 can stabilise before results are trusted again.
    DetectionResult detect(const cv::Mat& frame, bool isMoving);

    cv::Mat drawDebug(const cv::Mat& frame, const DetectionResult& result) const;

private:
    Config                                m_cfg;
    cv::Ptr<cv::BackgroundSubtractorMOG2> m_bgsub;
    cv::Mat                               m_mask;
    cv::Mat                               m_kernel;   // pre-allocated, never reallocated

    int  m_cooldownCounter = 0;   // frames remaining in post-motion cooldown
    bool m_wasMoving       = false;
};