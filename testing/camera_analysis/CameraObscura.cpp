#include "camera.hpp"
#include <iostream>
#include <string>

Camera::Camera(int deviceIndex, int width, int height, int fps)
    : m_deviceIndex(deviceIndex), m_width(width), m_height(height), m_fps(fps) {}

Camera::~Camera() {
    release();
}

bool Camera::open() {
    // 1. Build the GStreamer pipeline string for Raspberry Pi 5 / libcamera
    std::string pipeline = "libcamerasrc ! video/x-raw, width=" + std::to_string(m_width) +
                           ", height=" + std::to_string(m_height) +
                           ", framerate=" + std::to_string(m_fps) +
                           "/1 ! videoconvert ! appsink";

    // 2. Try opening with the GStreamer backend first
    m_cap.open(pipeline, cv::CAP_GSTREAMER);

    // 3. Fallback: try default backend (works on macOS/Windows for testing)
    if (!m_cap.isOpened()) {
        std::cout << "[Camera] GStreamer failed or unavailable. Falling back to default backend...\n";
        m_cap.open(m_deviceIndex);
    }

    if (!m_cap.isOpened()) {
        std::cerr << "[Camera] Failed to open device " << m_deviceIndex << "\n";
        return false;
    }

    // 4. Handle Properties
    // Note: When using GStreamer, width/height/fps MUST be set in the pipeline string.
    // Setting them via m_cap.set() is usually ignored by the appsink backend.
    // We only apply m_cap.set() if we fell back to the default backend.
    if (m_cap.getBackendName() != "GSTREAMER") {
        m_cap.set(cv::CAP_PROP_FRAME_WIDTH,  m_width);
        m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, m_height);
        m_cap.set(cv::CAP_PROP_FPS,          m_fps);
    }

    std::cout << "[Camera] Opened device via " << m_cap.getBackendName()
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
