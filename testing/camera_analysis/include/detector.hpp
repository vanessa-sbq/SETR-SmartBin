#pragma once
#include <opencv2/opencv.hpp>
#include "config.hpp"

struct DetectionResult {
    bool        detected;
    cv::Point2f centroid;
    float       area;
    cv::Rect    boundingBox;
};

struct DetectorConfig {
    int        minArea          = Config::DET_MIN_AREA;
    int        erodeIterations  = Config::DET_ERODE_ITER;
    int        dilateIterations = Config::DET_DILATE_ITER;
    cv::Scalar lowerHSV        = { Config::DET_HSV_LO_H,
                                   Config::DET_HSV_LO_S,
                                   Config::DET_HSV_LO_V };
    cv::Scalar upperHSV        = { Config::DET_HSV_HI_H,
                                   Config::DET_HSV_HI_S,
                                   Config::DET_HSV_HI_V };
    float      minCircularity   = Config::DET_MIN_CIRCULARITY;
    float      maxDistanceM     = Config::DET_MAX_DIST_M;
};

class Detector {
public:
    using Config = DetectorConfig;

    explicit Detector(Config cfg = Config{});

    DetectionResult detect(const cv::Mat& frame, bool isMoving = false);

    cv::Mat drawDebug(const cv::Mat& frame, const DetectionResult& result) const;

private:
    Config  m_cfg;
    cv::Mat m_hsv;
    cv::Mat m_mask;
    cv::Mat m_kernel;

    double m_focalPx;      // derived from FRAME_W and H_FOV_DEG
    double m_objectSizeCm; // derived from MOT_OBJECT_HEIGHT_M
};
