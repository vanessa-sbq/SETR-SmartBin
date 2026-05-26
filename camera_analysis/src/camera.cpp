#include "camera.hpp"
#include <iostream>

Camera::Camera(int deviceIndex, int width, int height, int fps)
    : m_deviceIndex(deviceIndex), m_width(width), m_height(height), m_fps(fps) {}

Camera::~Camera() {
    release();
}

bool Camera::open() {
    m_cap.open(m_deviceIndex, cv::CAP_V4L2);  // V4L2 works best on Linux/Pi
    if (!m_cap.isOpened()) {
        // Fallback: try default backend (works on macOS/Windows for testing)
        m_cap.open(m_deviceIndex);
    }
    if (!m_cap.isOpened()) {
        std::cerr << "[Camera] Failed to open device " << m_deviceIndex << "\n";
        return false;
    }

    m_cap.set(cv::CAP_PROP_FRAME_WIDTH,  m_width);
    m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, m_height);
    m_cap.set(cv::CAP_PROP_FPS,          m_fps);

    std::cout << "[Camera] Opened device " << m_deviceIndex
              << " at " << m_width << "x" << m_height << " " << m_fps << "fps\n";
    return true;
}

bool Camera::readFrame(cv::Mat& frame) {
    return m_cap.read(frame) && !frame.empty();
}

void Camera::release() {
    if (m_cap.isOpened()) {
        m_cap.release();
    }
}
