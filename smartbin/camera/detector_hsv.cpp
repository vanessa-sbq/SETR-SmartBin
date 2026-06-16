#include "detector_hsv.hpp"
#include "config.hpp"
#include <cmath>
#include <fstream>
#include <iostream>

HsvDetector::HsvDetector(DetectorConfig cfg) : m_cfg(cfg) {
    // focal length in pixels from horizontal FOV
    m_focalPx = (::Config::FRAME_W / 2.0) / std::tan(::Config::H_FOV_DEG * M_PI / 360.0);
    m_objectSizeCm = ::Config::MOT_OBJECT_HEIGHT_M * 100.0;
    m_kernel = cv::getStructuringElement(cv::MORPH_RECT, {3, 3});
}

DetectionResult HsvDetector::detect(const cv::Mat& frame, bool isMoving) {
    DetectionResult result{false, {0.f, 0.f}, 0.f, {}};

    if (frame.empty()) return result;

    // Learned yellow colour model (Hue+Saturation Gaussian gate)
    cv::cvtColor(frame, m_hsv, cv::COLOR_BGR2HSV);

    cv::Mat ch[3];
    cv::split(m_hsv, ch);                 // ch[0]=H, ch[1]=S, ch[2]=V (8-bit)
    cv::Mat H, S;
    ch[0].convertTo(H, CV_32F);
    ch[1].convertTo(S, CV_32F);

    const YellowColorModel& m = m_cfg.colorModel;
    cv::Mat dH = H - m.mean[0];
    cv::Mat dS = S - m.mean[1];

    // Mahalanobis² = a·dH² + (b+c)·dH·dS + d·dS²,  invCov = [[a,b],[c,d]]
    cv::Mat mahal = m.invCov(0, 0) * dH.mul(dH)
                  + (m.invCov(0, 1) + m.invCov(1, 0)) * dH.mul(dS)
                  + m.invCov(1, 1) * dS.mul(dS);

    m_mask = (mahal < m.threshold);                 // CV_8U, 255 where yellow
    m_mask.setTo(0, ch[2] < m_cfg.minValue);        // drop near-black (hue noise)

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

void HsvDetector::trainColorModel(const std::vector<cv::Mat>& yellowPatches) {
    cv::Mat samplesHS;   // N×2 CV_32F rows of [H, S]

    for (const auto& patch : yellowPatches) {
        if (patch.empty()) continue;

        cv::Mat hsv;
        cv::cvtColor(patch, hsv, cv::COLOR_BGR2HSV);
        hsv.convertTo(hsv, CV_32F);

        // Flatten to one row per pixel, keep only the H and S columns.
        cv::Mat flat = hsv.reshape(1, static_cast<int>(hsv.total())); // (N×3)
        samplesHS.push_back(flat(cv::Range::all(), cv::Range(0, 2)).clone());
    }

    if (samplesHS.rows >= 2)
        m_cfg.colorModel.fit(samplesHS);
    else
        std::cerr << "[Detector] trainColorModel: not enough sample pixels.\n";
}

// YellowColorModel
void YellowColorModel::fit(const cv::Mat& samplesHS) {
    cv::Mat cov, mu;
    cv::calcCovarMatrix(samplesHS, cov, mu,
                        cv::COVAR_NORMAL | cv::COVAR_ROWS | cv::COVAR_SCALE,
                        CV_32F);

    mean = { mu.at<float>(0), mu.at<float>(1) };

    // Regularize so the 2×2 covariance stays invertible on tight colour samples.
    cov.at<float>(0, 0) += 1.f;
    cov.at<float>(1, 1) += 1.f;

    invCov = cv::Matx22f(cov.at<float>(0, 0), cov.at<float>(0, 1),
                         cov.at<float>(1, 0), cov.at<float>(1, 1)).inv();
}

bool YellowColorModel::save(const std::string& path) const {
    std::ofstream f(path);
    if (!f) return false;
    f << mean[0] << ' ' << mean[1] << '\n'
      << invCov(0, 0) << ' ' << invCov(0, 1) << ' '
      << invCov(1, 0) << ' ' << invCov(1, 1) << '\n'
      << threshold << '\n';
    return static_cast<bool>(f);
}

bool YellowColorModel::load(const std::string& path) {
    std::ifstream f(path);
    if (!f) return false;
    f >> mean[0] >> mean[1]
      >> invCov(0, 0) >> invCov(0, 1) >> invCov(1, 0) >> invCov(1, 1)
      >> threshold;
    return !f.fail();
}
