#pragma once
#include "config.hpp"
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// DetectionResult and DetectorConfig.
struct DetectionResult {
    bool detected;
    cv::Point2f centroid;
    float area;
    cv::Rect boundingBox;
};

struct YellowColorModel {
    cv::Vec2f mean{Config::DET_COLOR_MEAN_HUE, Config::DET_COLOR_MEAN_SATURATION};
    cv::Matx22f invCov{1.f / Config::DET_COLOR_VARIANCE_HUE, 0.f, 0.f, 1.f / Config::DET_COLOR_VARIANCE_SATURATION};
    float threshold = Config::DET_COLOR_THREASHOLD; // Mahalanobis² gate
};

struct DetectorConfig {
    int minArea = Config::DET_MIN_AREA;
    int erodeIterations = Config::DET_ERODE_ITER;
    int dilateIterations = Config::DET_DILATE_ITER;
    YellowColorModel colorModel{};
    int minValue = Config::DET_COLOR_MIN_HUE_VALUE; // reject near-black pixels
    float minCircularity = Config::DET_MIN_CIRCULARITY;
    float maxDistanceM = Config::DET_MAX_DIST_M;
};

class HsvDetector {
  public:
    explicit HsvDetector(DetectorConfig cfg = DetectorConfig{});

    DetectionResult detect(const cv::Mat &frame);

  private:
    DetectorConfig motor_config;
    cv::Mat m_hsv;
    cv::Mat m_mask;
    cv::Mat m_kernel;

    double m_focalPx;
    double m_objectSizeCm;
};
