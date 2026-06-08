#pragma once
#include "detector.hpp"

// HSV colour threshold + contour / circularity / distance filtering.
class HsvDetector : public IDetector {
public:
    explicit HsvDetector(DetectorConfig cfg = DetectorConfig{});

    DetectionResult detect(const cv::Mat& frame, bool isMoving = false) override;

private:
    DetectorConfig m_cfg;
    cv::Mat m_hsv;
    cv::Mat m_mask;
    cv::Mat m_kernel;

    double m_focalPx;
    double m_objectSizeCm; 
};
