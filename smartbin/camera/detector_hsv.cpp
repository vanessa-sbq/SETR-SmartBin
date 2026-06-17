#include "detector_hsv.hpp"
#include "config.hpp"
#include <cmath>
#include <fstream>
#include <iostream>

HsvDetector::HsvDetector(DetectorConfig cfg) : motor_config(cfg) {
    // focal length in pixels from horizontal FOV
    m_focalPx = (::Config::CAM_VIDEO_WIDTH / 2.0) / std::tan(::Config::CAM_H_FOV_DEG * M_PI / 360.0);
    m_objectSizeCm = ::Config::OBJECT_DIAMETER_M * 100.0;
    m_kernel = cv::getStructuringElement(cv::MORPH_RECT, {3, 3});
}

DetectionResult HsvDetector::detect(const cv::Mat &frame) {
    DetectionResult result{false, {0.f, 0.f}, 0.f, {}};

    if (frame.empty())
        return result;

    // Learned yellow colour model (Hue+Saturation Gaussian gate)
    cv::cvtColor(frame, m_hsv, cv::COLOR_BGR2HSV);

    cv::Mat ch[3];
    cv::split(m_hsv, ch); // ch[0]=H, ch[1]=S, ch[2]=V (8-bit)
    cv::Mat H, S;
    ch[0].convertTo(H, CV_32F);
    ch[1].convertTo(S, CV_32F);

    const YellowColorModel &m = motor_config.colorModel;
    cv::Mat dH = H - m.mean[0];
    cv::Mat dS = S - m.mean[1];

    // Mahalanobis² = a·dH² + (b+c)·dH·dS + d·dS²,  invCov = [[a,b],[c,d]]
    cv::Mat mahal = m.invCov(0, 0) * dH.mul(dH) + (m.invCov(0, 1) + m.invCov(1, 0)) * dH.mul(dS) + m.invCov(1, 1) * dS.mul(dS);

    m_mask = (mahal < m.threshold); // CV_8U, 255 where yellow
    m_mask.setTo(0, ch[2] < motor_config.minValue); // drop near-black (hue noise)

    // Morphological cleanup
    cv::erode(m_mask, m_mask, m_kernel, {-1, -1}, motor_config.erodeIterations);
    cv::dilate(m_mask, m_mask, m_kernel, {-1, -1}, motor_config.dilateIterations);

    // Contour detection
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(m_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty())
        return result;

    // Area + circularity filter
    std::vector<const std::vector<cv::Point> *> found;
    for (const auto &c : contours) {
        double area = cv::contourArea(c);
        if (area < motor_config.minArea)
            continue;

        double perim = cv::arcLength(c, true);
        if (perim == 0.0)
            continue;

        double circularity = 4.0 * M_PI * area / (perim * perim);
        if (circularity > motor_config.minCircularity)
            found.push_back(&c);
    }

    if (found.empty())
        return result;

    // Distance filter
    std::vector<const std::vector<cv::Point> *> filtered;
    for (const auto *cp : found) {
        cv::Rect br = cv::boundingRect(*cp);
        double pixelSize = std::max(br.width, br.height);
        if (pixelSize == 0.0)
            continue;
        double distM = (m_objectSizeCm * m_focalPx) / pixelSize / 100.0;
        if (distM > motor_config.maxDistanceM)
            continue;
        filtered.push_back(cp);
    }

    if (filtered.empty())
        return result;

    // Pick largest surviving contour
    const auto *best = *std::max_element(filtered.begin(), filtered.end(), [](const auto *a, const auto *b) { return cv::contourArea(*a) < cv::contourArea(*b); });

    cv::Rect bbox = cv::boundingRect(*best);

    result.detected = true;
    result.area = static_cast<float>(cv::contourArea(*best));
    result.centroid = {static_cast<float>(bbox.x + bbox.width / 2), static_cast<float>(bbox.y + bbox.height / 2)};
    result.boundingBox = bbox;

    return result;
}
