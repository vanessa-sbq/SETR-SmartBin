#include "detector_bgsub.hpp"
#include "config.hpp"

BgSubDetector::BgSubDetector(DetectorConfig cfg) : m_cfg(cfg) {
    // TODO: Implement
}

DetectionResult BgSubDetector::detect(const cv::Mat& frame, bool /*isMoving*/) {
    DetectionResult result{false, {0.f, 0.f}, 0.f, {}};

    if (frame.empty()) return result;

    // TODO: Implement

    return result;
}
