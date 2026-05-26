#include "detector.hpp"
#include <iostream>

Detector::Detector(Config cfg)
    : m_cfg(cfg)
{
    m_bgsub = cv::createBackgroundSubtractorMOG2(
        m_cfg.history, m_cfg.varThreshold, /*detectShadows=*/false);

    // Pre-allocate the morphology kernel once — RECT is faster than ELLIPSE
    // (no diagonal pixel calculations).
    m_kernel = cv::getStructuringElement(cv::MORPH_RECT, {3, 3});
}

DetectionResult Detector::detect(const cv::Mat& frame, bool isMoving) {
    DetectionResult result{false, {0.f, 0.f}, 0.f, {}};

    if (frame.empty()) return result;

    // ── Cooldown management ───────────────────────────────────────────────────
    // When the can transitions from moving → stopped, start a cooldown.
    // During cooldown MOG2 still runs (to re-learn the new background) but
    // we don't trust its detections yet.
    if (m_wasMoving && !isMoving) {
        m_cooldownCounter = m_cfg.cooldownFrames;
    }
    m_wasMoving = isMoving;

    if (m_cooldownCounter > 0) --m_cooldownCounter;

    // ── MOG2 learning rate ────────────────────────────────────────────────────
    // While moving: high learning rate so MOG2 rapidly absorbs the shifting
    // background instead of treating it as foreground.
    // While still:  -1 = let OpenCV manage it automatically (stable, slow drift).
    float learnRate = isMoving ? m_cfg.movingLearnRate : -1.f;

    // Always apply MOG2 — even while moving — so the background model stays
    // current. We just discard the result when we can't trust it.
    m_bgsub->apply(frame, m_mask, learnRate);

    // Suppress detections while moving or during cooldown
    if (isMoving || m_cooldownCounter > 0) return result;

    // ── Morphological cleanup ─────────────────────────────────────────────────
    // MORPH_OPEN = erode then dilate in one optimised call.
    // Kills small noise specks (erode) then restores real blobs (dilate).
    cv::morphologyEx(m_mask, m_mask, cv::MORPH_OPEN, m_kernel);

    // ── Contour detection ─────────────────────────────────────────────────────
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(m_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return result;

    // Largest contour above the minimum area threshold
    auto best = std::max_element(contours.begin(), contours.end(),
        [](const auto& a, const auto& b){
            return cv::contourArea(a) < cv::contourArea(b);
        });

    float area = static_cast<float>(cv::contourArea(*best));
    if (area < m_cfg.minArea) return result;

    // Centroid via image moments
    cv::Moments M = cv::moments(*best);
    if (M.m00 == 0.0) return result;

    result.detected    = true;
    result.area        = area;
    result.centroid    = { static_cast<float>(M.m10 / M.m00),
                           static_cast<float>(M.m01 / M.m00) };
    result.boundingBox = cv::boundingRect(*best);

    return result;
}

cv::Mat Detector::drawDebug(const cv::Mat& frame, const DetectionResult& result) const {
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