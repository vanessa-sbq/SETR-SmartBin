#pragma once
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
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

// Lightweight learned "is this pixel yellow" model.
// A 2-D Gaussian over (Hue, Saturation). Scoring is a single Mahalanobis²
// distance, so it runs as fast as the old inRange but adapts to the real object
// colour/lighting once fitted. No external dependencies, no model files needed.
struct YellowColorModel {
    cv::Vec2f   mean   { Config::DET_COLOR_MEAN_H, Config::DET_COLOR_MEAN_S };
    cv::Matx22f invCov { 1.f / Config::DET_COLOR_VAR_H, 0.f,
                         0.f, 1.f / Config::DET_COLOR_VAR_S };
    float       threshold = Config::DET_COLOR_GATE;   // Mahalanobis² gate

    // Fit mean + covariance from labelled yellow pixels (N×2 CV_32F rows of [H,S]).
    void fit(const cv::Mat& samplesHS);

    // Tiny text serialization so a fitted model survives restarts.
    bool save(const std::string& path) const;
    bool load(const std::string& path);
};

struct DetectorConfig {
    int minArea = Config::DET_MIN_AREA;
    int erodeIterations = Config::DET_ERODE_ITER;
    int dilateIterations = Config::DET_DILATE_ITER;
    YellowColorModel colorModel {};
    int minValue = Config::DET_COLOR_MIN_V;   // reject near-black pixels
    float minCircularity = Config::DET_MIN_CIRCULARITY;
    float maxDistanceM = Config::DET_MAX_DIST_M;
};
        
// Detection Interface
class IDetector {
public:
    virtual ~IDetector() = default;

    // Run one detection pass
    virtual DetectionResult detect(const cv::Mat& frame, bool isMoving = false) = 0;

    // Debug function shared across all algorithms
    cv::Mat drawDebug(const cv::Mat& frame, const DetectionResult& result) const;

protected:
    // Funciton for algorithm-specific debug drawing (e.g. Kalman trajectory)
    virtual void drawOverlay(cv::Mat& /*out*/) const {}
};

// Algorithm selection 
enum class DetectorKind {
    Hsv,    // detector_hsv.cpp     - HSV colour threshold + contour/circularity
    BgSub,  // detector_bgsub.cpp   - background subtraction (motion)
    Kalman, // detector_kalman.cpp  - Kalman filters based
};

std::unique_ptr<IDetector> makeDetector(DetectorKind kind, DetectorConfig cfg = DetectorConfig{});
