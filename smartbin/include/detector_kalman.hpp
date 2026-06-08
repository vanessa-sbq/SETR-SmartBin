#pragma once
#include "detector.hpp"

// Kalman filters based algorithm
class KalmanDetector : public IDetector {
public:
    explicit KalmanDetector(DetectorConfig cfg = DetectorConfig{});
    DetectionResult detect(const cv::Mat& frame, bool isMoving = false) override;

private:
    DetectorConfig m_cfg;
    // TODO: Add fields
};
