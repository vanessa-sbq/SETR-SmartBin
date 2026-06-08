#pragma once
#include "detector.hpp"

// Background-subtraction (motion) based detector.
class BgSubDetector : public IDetector {
public:
    explicit BgSubDetector(DetectorConfig cfg = DetectorConfig{});

    DetectionResult detect(const cv::Mat& frame, bool isMoving = false) override;

private:
    DetectorConfig m_cfg;

    cv::Ptr<cv::BackgroundSubtractor> m_bgsub;
    cv::Mat m_kernel;
    cv::Mat m_mask;

    // ── Tracking lock state ──────────────────────────────────────────────────
    bool       m_isTracking         = false;
    int        m_trackingFramesLeft = 0;
    cv::Scalar m_lowColor;
    cv::Scalar m_highColor;
    float      m_targetAspectRatio  = 1.f;

    // ── Background-subtraction cooldown state ────────────────────────────────
    bool m_wasMoving      = false;
    int  m_cooldownCounter = 0;

    // ── Tunables (were part of the original Config struct — not in the shared
    //    algorithm-agnostic DetectorConfig, so kept local to this algorithm) ──
    int    m_history        = 500;   // MOG2 history length
    double m_varThreshold   = 16.0;  // MOG2 variance threshold
    int    m_cooldownFrames = 10;    // frames to ignore after motion stops
    double m_movingLearnRate = 0.01; // BG learning rate while moving
};
