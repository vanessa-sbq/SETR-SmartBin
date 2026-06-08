#include "detector.hpp"
#include "detector_hsv.hpp"
#include "detector_bgsub.hpp"
#include "detector_kalman.hpp"

// Single place that knows every algorithm. Add new kinds here + in DetectorKind.
std::unique_ptr<IDetector> makeDetector(DetectorKind kind, DetectorConfig cfg) {
    switch (kind) {
        case DetectorKind::Hsv: return std::make_unique<HsvDetector>(cfg);
        case DetectorKind::BgSub: return std::make_unique<BgSubDetector>(cfg);
        case DetectorKind::Kalman: return std::make_unique<KalmanDetector>(cfg);
    }
    return std::make_unique<HsvDetector>(cfg); // fallback
}

// Shared debug overlay, identical for every algorithm.
cv::Mat IDetector::drawDebug(const cv::Mat& frame, const DetectionResult& result) const {
    cv::Mat out;
    frame.copyTo(out);

    if (result.detected) {
        cv::rectangle(out, result.boundingBox, {0, 255, 0}, 2);

        cv::Point c(static_cast<int>(result.centroid.x),
                    static_cast<int>(result.centroid.y));
        cv::drawMarker(out, c, {0, 0, 255}, cv::MARKER_CROSS, 20, 2);

        cv::putText(out, "Area: " + std::to_string(static_cast<int>(result.area)),
                    {result.boundingBox.x, result.boundingBox.y - 8},
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 255}, 1);
    } else {
        cv::putText(out, "No target", {10, 20},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 0, 255}, 2);
    }

    return out;
}
