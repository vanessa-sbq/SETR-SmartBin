#include "detector_kalman.hpp"
#include "config.hpp"

KalmanDetector::KalmanDetector(DetectorConfig cfg) : m_cfg(cfg){
    // TODO:
}

DetectionResult KalmanDetector::detect(const cv::Mat& frame, bool /*isMoving*/) {
    DetectionResult result{false, {0.f, 0.f}, 0.f, {}};

    if (frame.empty()) return result;

    // TODO: implement algorithm based on Kalman filters

    return result;
}
