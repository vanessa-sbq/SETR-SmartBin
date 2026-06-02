#include "detector.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>

Detector::Detector(Config cfg)
    : m_cfg(cfg)
{
    m_bgsub = cv::createBackgroundSubtractorMOG2(
        m_cfg.history, m_cfg.varThreshold, /*detectShadows=*/false);

    m_kernel = cv::getStructuringElement(cv::MORPH_RECT, {3, 3});
}

DetectionResult Detector::detect(const cv::Mat& frame, bool isMoving) {
    DetectionResult result{false, {0.f, 0.f}, 0.f, {}};
    if (frame.empty()) return result;

    cv::Mat hsv, trackingMask;

    // ── STATE 1: ACTIVE TARGET TRACKING (When moving or executing paths) ─────
    if (m_isTracking) {
        if (m_trackingFramesLeft > 0) {
            m_trackingFramesLeft--;

            // Extract matching target color spaces
            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
            cv::inRange(hsv, m_lowColor, m_highColor, trackingMask);
            cv::morphologyEx(trackingMask, trackingMask, cv::MORPH_OPEN, m_kernel);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(trackingMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            float bestMatchScore = 999999.f;
            bool foundMatch = false;
            cv::Rect bestRect;
            cv::Point2f bestCentroid;
            float bestArea = 0.f;

            for (const auto& contour : contours) {
                float area = static_cast<float>(cv::contourArea(contour));
                if (area < m_cfg.minArea) continue;

                cv::Rect r = cv::boundingRect(contour);
                float aspect = static_cast<float>(r.width) / static_cast<float>(r.height);
                
                // Score based on similarity to original bounding box proportions
                float score = std::abs(aspect - m_targetAspectRatio);
                if (score < bestMatchScore && score < 0.6f) {
                    bestMatchScore = score;
                    bestRect = r;
                    bestArea = area;

                    cv::Moments M = cv::moments(contour);
                    if (M.m00 != 0.0) {
                        bestCentroid = { static_cast<float>(M.m10 / M.m00), static_cast<float>(M.m01 / M.m00) };
                        foundMatch = true;
                    }
                }
            }

            if (foundMatch) {
                result.detected = true;
                result.area = bestArea;
                result.centroid = bestCentroid;
                result.boundingBox = bestRect;
                return result;
            }
        } else {
            m_isTracking = false; // Target tracking lifespan expired
        }
    }

    // ── STATE 2: BACKGROUND SUBTRACTION (When still, scanning the area) ──────
    if (m_wasMoving && !isMoving) {
        m_cooldownCounter = m_cfg.cooldownFrames;
    }
    m_wasMoving = isMoving;

    if (m_cooldownCounter > 0) --m_cooldownCounter;

    float learnRate = isMoving ? m_cfg.movingLearnRate : -1.f;
    m_bgsub->apply(frame, m_mask, learnRate);

    if (isMoving || m_cooldownCounter > 0) return result;

    cv::morphologyEx(m_mask, m_mask, cv::MORPH_OPEN, m_kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(m_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return result;

    auto best = std::max_element(contours.begin(), contours.end(),
        [](const auto& a, const auto& b){
            return cv::contourArea(a) < cv::contourArea(b);
        });

    float area = static_cast<float>(cv::contourArea(*best));
    if (area < m_cfg.minArea) return result;

    cv::Moments M = cv::moments(*best);
    if (M.m00 == 0.0) return result;

    result.detected    = true;
    result.area        = area;
    result.centroid    = { static_cast<float>(M.m10 / M.m00), static_cast<float>(M.m01 / M.m00) };
    result.boundingBox = cv::boundingRect(*best);

    // ── STATE 3: LOCK ON EVENT (Capture the target's visual signature) ──────
    cv::Rect roi = result.boundingBox & cv::Rect(0, 0, frame.cols, frame.rows);
    if (roi.width > 0 && roi.height > 0) {
        cv::cvtColor(frame(roi), hsv, cv::COLOR_BGR2HSV);
        cv::Scalar mean, stddev;
        cv::meanStdDev(hsv, mean, stddev);

        // Lock stable HSV variance thresholds based on the target center blob profile
        m_lowColor  = cv::Scalar(std::max(0.0, mean[0] - 22), std::max(40.0, mean[1] - 55), std::max(40.0, mean[2] - 55));
        m_highColor = cv::Scalar(std::min(179.0, mean[0] + 22), std::min(255.0, mean[1] + 55), std::min(255.0, mean[2] + 55));
        
        m_targetAspectRatio = static_cast<float>(roi.width) / static_cast<float>(roi.height);
        m_isTracking = true;
        m_trackingFramesLeft = 75; // Number of frames to persist tracking lock
    }

    return result;
}

cv::Mat Detector::drawDebug(const cv::Mat& frame, const DetectionResult& result) const {
    cv::Mat out;
    frame.copyTo(out);

    if (result.detected) {
        cv::rectangle(out, result.boundingBox, {0, 255, 0}, 2);

        cv::Point c(static_cast<int>(result.centroid.x), static_cast<int>(result.centroid.y));
        cv::drawMarker(out, c, {0, 0, 255}, cv::MARKER_CROSS, 20, 2);

        if (m_isTracking) {
            cv::putText(out, "LOCKED [" + std::to_string(m_trackingFramesLeft) + "]", 
                        {result.boundingBox.x, result.boundingBox.y - 24}, 
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, {0, 0, 255}, 1);
        }

        cv::putText(out, "Area: " + std::to_string(static_cast<int>(result.area)),
                    {result.boundingBox.x, result.boundingBox.y - 8},
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 255}, 1);
    } else {
        cv::putText(out, "No target", {10, 20},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 0, 255}, 2);
    }

    return out;
}