#pragma once
#include "detector.hpp"

// Background-subtraction (motion) based detector.
class BgSubDetector : public IDetector {
public:
    explicit BgSubDetector(DetectorConfig cfg = DetectorConfig{});

    DetectionResult detect(const cv::Mat& frame, bool isMoving = false) override;

private:
    DetectorConfig m_cfg;
    // TODO: Add fields
};
