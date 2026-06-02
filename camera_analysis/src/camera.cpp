#include "camera.hpp"
#include <iostream>

Camera::Camera(int deviceIndex, int width, int height, int fps)
    : m_deviceIndex(deviceIndex), m_width(width), m_height(height), m_fps(fps) {}

Camera::~Camera() {
    release();
}

bool Camera::open() {
#ifdef USING_OPENCV_CAM
    // WSL / PC Mode: Open default USB webcam (device 0) or an offline video string path
    m_cam.open(m_deviceIndex, cv::CAP_ANY);
    if (!m_cam.isOpened()) return false;

    m_cam.set(cv::CAP_PROP_FRAME_WIDTH, m_width);
    m_cam.set(cv::CAP_PROP_FRAME_HEIGHT, m_height);
    m_cam.set(cv::CAP_PROP_FPS, m_fps);
    m_isRunning = true;
    std::cout << "[Camera] Opened WebCam/Video via standard cross-platform OpenCV\n";
#else
    // Raspberry Pi Native Hardware Mode
    m_cam.options->video_width = m_width;
    m_cam.options->video_height = m_height;
    m_cam.options->framerate = m_fps;
    m_cam.options->camera = m_deviceIndex; 

    m_cam.startVideo();
    m_isRunning = true;
    std::cout << "[Camera] Opened Camera Module 3 natively via LCCV\n";
#endif
    return true;
}

bool Camera::readFrame(cv::Mat& frame) {
    if (!m_isRunning) return false;

#ifdef USING_OPENCV_CAM
    m_cam.read(frame);
    return !frame.empty();
#else
    bool success = m_cam.getVideoFrame(frame, 1000);
    return success && !frame.empty();
#endif
}

void Camera::release() {
    if (m_isRunning) {
#ifdef USING_OPENCV_CAM
        m_cam.release();
#else
        m_cam.stopVideo();
#endif
        m_isRunning = false;
        std::cout << "[Camera] Released camera stream.\n";
    }
}