#include "detector_hsv.hpp"
#include "config.hpp"
#include <cmath>

HsvDetector::HsvDetector(DetectorConfig cfg) : m_cfg(cfg) {
    // focal length in pixels from horizontal FOV
    m_focalPx = (::Config::FRAME_W / 2.0) / std::tan(::Config::H_FOV_DEG * M_PI / 360.0);
    m_objectSizeCm = ::Config::MOT_OBJECT_HEIGHT_M * 100.0;
    m_kernel = cv::getStructuringElement(cv::MORPH_RECT, {3, 3});
}

DetectionResult HsvDetector::detect(const cv::Mat& frame, bool /*isMoving*/) {
    DetectionResult result{false, {0.f, 0.f}, 0.f, {}};

    if (frame.empty()) return result;

    // HSV color filter
    cv::cvtColor(frame, m_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(m_hsv, m_cfg.lowerHSV, m_cfg.upperHSV, m_mask);

    // Morphological cleanup
    cv::erode (m_mask, m_mask, m_kernel, {-1,-1}, m_cfg.erodeIterations);
    cv::dilate(m_mask, m_mask, m_kernel, {-1,-1}, m_cfg.dilateIterations);

    // Contour detection
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(m_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return result;

    // Area + circularity filter
    std::vector<const std::vector<cv::Point>*> found;
    for (const auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < m_cfg.minArea) continue;

        double perim = cv::arcLength(c, true);
        if (perim == 0.0) continue;

        double circularity = 4.0 * M_PI * area / (perim * perim);
        if (circularity > m_cfg.minCircularity)
            found.push_back(&c);
    }

    if (found.empty()) return result;

    // Distance filter
    std::vector<const std::vector<cv::Point>*> filtered;
    for (const auto* cp : found) {
        cv::Rect br = cv::boundingRect(*cp);
        double pixelSize = std::max(br.width, br.height);
        if (pixelSize == 0.0) continue;
        double distM = (m_objectSizeCm * m_focalPx) / pixelSize / 100.0;
        if (distM > m_cfg.maxDistanceM) continue;
        filtered.push_back(cp);
    }

    if (filtered.empty()) return result;

    // Pick largest surviving contour
    const auto* best = *std::max_element(filtered.begin(), filtered.end(),
        [](const auto* a, const auto* b) {
            return cv::contourArea(*a) < cv::contourArea(*b);
        });

    cv::Rect bbox = cv::boundingRect(*best);

    result.detected    = true;
    result.area        = static_cast<float>(cv::contourArea(*best));
    result.centroid    = { static_cast<float>(bbox.x + bbox.width  / 2),
                           static_cast<float>(bbox.y + bbox.height / 2) };
    result.boundingBox = bbox;

    return result;
}
