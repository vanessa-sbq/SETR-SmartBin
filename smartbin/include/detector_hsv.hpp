#pragma once
#include <vector>
#include "detector.hpp"

// Learned yellow colour model (Hue+Saturation Gaussian gate)
// + contour / circularity / distance filtering.
class HsvDetector : public IDetector {
public:
    explicit HsvDetector(DetectorConfig cfg = DetectorConfig{});

    DetectionResult detect(const cv::Mat& frame, bool isMoving = false) override;

    // Re-fit the yellow colour model from a handful of cropped sample images
    // (BGR patches that are mostly the target object).
    void trainColorModel(const std::vector<cv::Mat>& yellowPatches);

private:
    DetectorConfig m_cfg;
    cv::Mat m_hsv;
    cv::Mat m_mask;
    cv::Mat m_kernel;

    double m_focalPx;
    double m_objectSizeCm; 
};
