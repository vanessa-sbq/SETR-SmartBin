#include "camera.hpp"
#include <iostream>

Camera::Camera(int deviceIndex, int width, int height, int fps)
    : m_deviceIndex(deviceIndex), m_width(width), m_height(height), m_fps(fps) {}

Camera::~Camera() {
    release();
}

bool Camera::open() {
    // 1. Configure the native libcamera options
    m_cam.options->video_width = m_width;
    m_cam.options->video_height = m_height;
    m_cam.options->framerate = m_fps;
    m_cam.options->camera = m_deviceIndex; // Select camera 0 or 1

    // 2. Enable Autofocus for Camera Module 3!
    // Without this, Module 3 might be blurry depending on subject distance
    //m_cam.options->autofocusMode = libcamera::controls::AfModeEnum::AfModeContinuous;

    // 3. Start the libcamera stream
    m_cam.startVideo();
    m_isRunning = true;

    std::cout << "[Camera] Opened Camera Module 3 natively via LCCV at " 
              << m_width << "x" << m_height << " " << m_fps << "fps\n";
    return true;
}

bool Camera::readFrame(cv::Mat& frame) {
    if (!m_isRunning) return false;

    // Grab the frame natively from the ISP into a cv::Mat
    // The second parameter is a timeout in milliseconds
    bool success = m_cam.getVideoFrame(frame, 1000);
    
    return success && !frame.empty();
}

void Camera::release() {
    if (m_isRunning) {
        m_cam.stopVideo();
        m_isRunning = false;
        std::cout << "[Camera] Released libcamera stream.\n";
    }
}