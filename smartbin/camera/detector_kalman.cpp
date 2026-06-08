#include "detector_kalman.hpp"
#include "detector_hsv.hpp"
#include "config.hpp"
#include <cmath>

// Build a DetectionResult from a centroid, reusing the last known bbox size
static DetectionResult makeResult(cv::Point2f c, cv::Size sz, float area) {
    DetectionResult r{true, c, area, {}};
    int w = std::max(1, sz.width);
    int h = std::max(1, sz.height);
    r.boundingBox = cv::Rect(cvRound(c.x - w / 2.0), cvRound(c.y - h / 2.0), w, h);
    return r;
}

// Constructor
// Kalman filter background: Sophie Zhao, "Kalman Filter Explained Simply" https://medium.com/@sophiezhao_2990/kalman-filter-explained-simply-2b5672429205
KalmanDetector::KalmanDetector(DetectorConfig cfg) : m_cfg(cfg), m_raw(std::make_unique<HsvDetector>(cfg)), m_kf(4, 2, 0) {
    // Constant-velocity model: state = [x, y, vx, vy], measurement = [x, y].
    // dt = 1 frame, so each predict step adds the velocity to the position.
    m_kf.transitionMatrix = (cv::Mat_<float>(4, 4) <<
        1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);
    cv::setIdentity(m_kf.measurementMatrix);
    cv::setIdentity(m_kf.processNoiseCov, cv::Scalar::all(Config::KAL_PROCESS_NOISE));
    cv::setIdentity(m_kf.measurementNoiseCov,cv::Scalar::all(Config::KAL_MEAS_NOISE));
    cv::setIdentity(m_kf.errorCovPost, cv::Scalar::all(1));
    reset();
}

// Reset values for next computation
void KalmanDetector::reset() {
    m_phase       = Phase::Idle;
    m_frame       = 0;
    m_missing     = 0;
    m_kfReady     = false;
    m_initialized = false;
    m_corrected   = false;
    m_history.clear();
    m_lastSize = cv::Size(0, 0);
}

// Function that sets initial state (writes the state vector into statePost) and starts the filters for future iterations
void KalmanDetector::initFilter(const cv::Point2f& pos, const cv::Point2f& vel) {
    m_kf.statePost = (cv::Mat_<float>(4, 1) << pos.x, pos.y, vel.x, vel.y);
    m_kf.statePost.copyTo(m_kf.statePre);
    cv::setIdentity(m_kf.errorCovPost, cv::Scalar::all(1));
    m_kfReady = true;
}

// Helper that collects frame samples that will be used for trajectory prediction
void KalmanDetector::recordSample(int frame, const cv::Point2f& pos, bool valid) {
    if (static_cast<int>(m_history.size()) <= frame){
        m_history.resize(frame + 1);
    }
    m_history[frame] = {pos, valid};
}

// This function corrects the trajectory prediction when more frames are recorded
cv::Point2f KalmanDetector::fitVelocity(const std::vector<int>& frames) const {
    // Least-squares slope of position vs. frame index over the given (valid) samples.
    double n = 0, sf = 0, sff = 0;
    double sx = 0, sfx = 0, sy = 0, sfy = 0;
    for (int f : frames) {
        if (f < 0 || f >= static_cast<int>(m_history.size()) || !m_history[f].valid){
            continue;
        }
        double fx = f;
        n += 1;     
        sf += fx;            
        sff += fx * fx;
        sx += m_history[f].pos.x;          
        sfx += fx * m_history[f].pos.x;
        sy += m_history[f].pos.y;          
        sfy += fx * m_history[f].pos.y;
    }
    if (n < 2) return {0.f, 0.f};

    double denom = n * sff - sf * sf;
    if (std::abs(denom) < 1e-9) return {0.f, 0.f};

    return {static_cast<float>((n * sfx - sf * sx) / denom), static_cast<float>((n * sfy - sf * sy) / denom)};
}

// Detection algorithm entry point
DetectionResult KalmanDetector::detect(const cv::Mat& frame, bool isMoving) {
    DetectionResult none{false, {0.f, 0.f}, 0.f, {}};
    if (frame.empty()) return none;

    // Locate the object in the first frame using another detector
    DetectionResult raw = m_raw->detect(frame, isMoving); // we use the HSV detector by default

    // Check if there was a first detection (= frame 0)
    if (m_phase == Phase::Idle) {
        if (!raw.detected) return none; // No object detected
        reset();
        m_phase = Phase::Collecting;
        m_frame = 0;
        m_lastSize = raw.boundingBox.size();
        recordSample(0, raw.centroid, true);
        return raw; // follow the live detection while we gather data
    }

    // Active detection (collecting/tracking): advance the frame clock
    m_frame++;

    // Drop the target once it has been missing for too long
    if (raw.detected) {
        m_missing  = 0;
        m_lastSize = raw.boundingBox.size();
    } else if (++m_missing > Config::KAL_LOST_FRAMES) {
        reset(); // next detection is treated as a new object
        return none;
    }

    // Advance the Kalman prediction one frame (do nothing until the filter is seeded)
    cv::Point2f predicted{0.f, 0.f};
    if (m_kfReady) {
        cv::Mat p = m_kf.predict();
        predicted = {p.at<float>(0), p.at<float>(1)};
    }

    // Record this frame
    recordSample(m_frame, raw.detected ? raw.centroid : predicted, raw.detected);

    // Fold a new measurement into the filter.
    if (m_kfReady && raw.detected) {
        cv::Mat meas = (cv::Mat_<float>(2, 1) << raw.centroid.x, raw.centroid.y);
        m_kf.correct(meas);
    }

    // Seed the trajectory from frames {0, mid, init}
    if (!m_initialized && m_frame >= Config::KAL_INIT_FRAME && raw.detected) {
        cv::Point2f vel = fitVelocity({0, Config::KAL_MID_FRAME, Config::KAL_INIT_FRAME});
        initFilter(raw.centroid, vel);
        m_initialized = true;
        m_phase = Phase::Tracking; // set state to tracking since we have enough points
    }

    // Refine the velocity from frames {0, init, correct}
    if (m_initialized && !m_corrected && m_frame >= Config::KAL_CORRECT_FRAME && raw.detected) {
        cv::Point2f vel = fitVelocity({0, Config::KAL_INIT_FRAME, Config::KAL_CORRECT_FRAME});
        m_kf.statePost.at<float>(2) = vel.x; // keep the measured position,
        m_kf.statePost.at<float>(3) = vel.y; // refresh the velocity estimate
        m_corrected = true;
    }

    /// Output /// 
    
    // Still collecting: no trajectory yet, so just follow the live detection.
    if (m_phase == Phase::Collecting){    
        return raw.detected ? raw : none;
    }

    // Tracking: lead the motors to where the object is predicted to be.
    m_lead = {
        m_kf.statePost.at<float>(0) + m_kf.statePost.at<float>(2) * Config::KAL_LOOKAHEAD,
        m_kf.statePost.at<float>(1) + m_kf.statePost.at<float>(3) * Config::KAL_LOOKAHEAD
    };
    float area = raw.detected ? raw.area : static_cast<float>(m_lastSize.area());
    return makeResult(m_lead, m_lastSize, area);
}

void KalmanDetector::drawOverlay(cv::Mat& out) const {
    // Past path: connect the real measurements gathered so far.
    bool havePrev = false;
    cv::Point prev;
    for (const auto& s : m_history) {
        if (!s.valid) continue;
        cv::Point p(cvRound(s.pos.x), cvRound(s.pos.y));
        if (havePrev) {
            cv::line(out, prev, p, {255, 200, 0}, 1, cv::LINE_AA);
        } 
        cv::circle(out, p, 2, {255, 200, 0}, cv::FILLED);
        prev = p;
        havePrev = true;
    }

    // Predicted trajectory: current filtered position -> lead/intercept point.
    if (m_phase != Phase::Tracking || !m_kfReady) return;

    cv::Point cur(cvRound(m_kf.statePost.at<float>(0)), cvRound(m_kf.statePost.at<float>(1)));
    cv::Point lead(cvRound(m_lead.x), cvRound(m_lead.y));

    cv::arrowedLine(out, cur, lead, {0, 140, 255}, 2, cv::LINE_AA, 0, 0.15);
    cv::circle(out, lead, 6, {0, 0, 255}, 2, cv::LINE_AA);
    cv::putText(out, "predict", {lead.x + 8, lead.y - 8}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {0, 140, 255}, 1);
}
