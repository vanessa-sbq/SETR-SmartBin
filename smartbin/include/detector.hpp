#pragma once
#include <opencv2/opencv.hpp>
#include <memory>
#include "config.hpp"

// Every detection algorithm implements IDetector (one .cpp per algorithm, e.g. detector_hsv.cpp). 
// DetectionResult and DetectorConfig are algorithm-agnostic and live here. 
// Algorithm-specific state belongs in the concrete subclasses.
// The active algorithm can be picked through makeDetector(DetectorKind).

struct DetectionResult {
    bool detected;
    cv::Point2f centroid;
    float area;
    cv::Rect boundingBox;
};

struct DetectorConfig {
    int minArea = Config::DET_MIN_AREA;
    int erodeIterations = Config::DET_ERODE_ITER;
    int dilateIterations = Config::DET_DILATE_ITER;
    cv::Scalar lowerHSV = { 
        Config::DET_HSV_LO_H,
        Config::DET_HSV_LO_S,
        Config::DET_HSV_LO_V };
    cv::Scalar upperHSV = { 
        Config::DET_HSV_HI_H,
        Config::DET_HSV_HI_S,
        Config::DET_HSV_HI_V };
    float minCircularity = Config::DET_MIN_CIRCULARITY;
    float maxDistanceM = Config::DET_MAX_DIST_M;
};
        
// Detection Interface
class IDetector {
public:
    virtual ~IDetector() = default;

    // Run one detection pass. isMoving lets stateful algorithms manage learning
    // rate / cooldown (e.g. background subtraction).
    virtual DetectionResult detect(const cv::Mat& frame, bool isMoving = false) = 0;

    // Shared across all algorithms — only reads DetectionResult, so it is
    // implemented once in detector_common.cpp and is intentionally non-virtual.
    cv::Mat drawDebug(const cv::Mat& frame, const DetectionResult& result) const;
};

// Algorithm selection 
enum class DetectorKind {
    // TODO: Change names
    Hsv,    // detector_hsv.cpp     - HSV colour threshold + contour/circularity
    BgSub,  // detector_bgsub.cpp   - background subtraction (motion)
    Kalman, // detector_kalman.cpp  - Kalman filters based
};

std::unique_ptr<IDetector> makeDetector(DetectorKind kind, DetectorConfig cfg = DetectorConfig{});
