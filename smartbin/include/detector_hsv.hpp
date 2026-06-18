#pragma once
#include "config.hpp"
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

struct DetectionResult {
    bool detected;
    cv::Point2f centroid;
    float area;
    cv::Rect boundingBox;
};

struct YellowColorModel {
    cv::Vec2f mean{Config::DET_COLOR_MEAN_HUE, Config::DET_COLOR_MEAN_SATURATION};
    cv::Matx22f invCov{1.f / Config::DET_COLOR_VARIANCE_HUE, 0.f, 0.f, 1.f / Config::DET_COLOR_VARIANCE_SATURATION};
    float threshold = Config::DET_COLOR_THREASHOLD;
};

struct DetectorConfig {
    int minArea = Config::DETECTOR_MIN_AREA;
    int erodeIterations = Config::DETECTOR_ERODE_ITER;
    int dilateIterations = Config::DETECTOR_DILATE_ITER;
    YellowColorModel colorModel{};
    int minValue = Config::DETECTOR_COLOR_MIN_HUE_VALUE;
    float minCircularity = Config::DETECTOR_MIN_CIRCULARITY;
    float maxDistanceM = Config::DETECTOR_MAX_DIST_M;
};

class HsvDetector {
  public:
    explicit HsvDetector(DetectorConfig cfg = DetectorConfig{});

    DetectionResult detect(const cv::Mat &frame);

  private:
    DetectorConfig motor_config;
    cv::Mat hsv_matrix;
    cv::Mat mask_matrix;
    cv::Mat kernel_matrix;

    double focalPx;
    double objectSizeCm;
};
