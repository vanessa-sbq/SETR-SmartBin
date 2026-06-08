#pragma once
#include "detector.hpp"
#include <memory>
#include <vector>

// Kalman trajectory detector
//
// Wraps a raw per-frame detector (HSV) to locate the object, then fits a
// constant-velocity trajectory and hands the motors a predicted intercept
// point so they lead the object instead of chasing it.
class KalmanDetector : public IDetector {
public:
    explicit KalmanDetector(DetectorConfig cfg = DetectorConfig{});
    DetectionResult detect(const cv::Mat& frame, bool isMoving = false) override;

protected:
    void drawOverlay(cv::Mat& out) const override; // past path + predicted line

private:
    enum class Phase { Idle, Collecting, Tracking };

    void reset();
    void initFilter(const cv::Point2f& pos, const cv::Point2f& vel);
    void recordSample(int frame, const cv::Point2f& pos, bool valid);
    cv::Point2f fitVelocity(const std::vector<int>& frames) const; // px/frame

    DetectorConfig m_cfg;
    std::unique_ptr<IDetector> m_raw; // underlying raw detector (HSV)
    cv::KalmanFilter m_kf; // constant-velocity [x, y, vx, vy]

    Phase m_phase = Phase::Idle;
    int   m_frame = 0; // frames since the first detection (frame 0)
    int   m_missing = 0; // consecutive frames without a raw detection
    bool  m_kfReady = false;
    bool  m_initialized = false;
    bool  m_corrected = false;

    struct Sample { cv::Point2f pos; bool valid; };
    std::vector<Sample> m_history; // indexed by frame number

    cv::Size m_lastSize{0, 0}; // last known bbox size (for distance)
    cv::Point2f m_lead{0.f, 0.f}; // last predicted intercept (for overlay)
};
