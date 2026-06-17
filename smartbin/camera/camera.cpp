#include "camera.hpp"
#include <iostream>

Camera::Camera(int deviceIndex, int width, int height, int fps) : m_deviceIndex(deviceIndex), m_width(width), m_height(height), m_fps(fps) {}

Camera::~Camera() { release(); }

bool Camera::open() {
    // Configure the native lccv options
    m_cam.options->video_width = m_width;
    m_cam.options->video_height = m_height;
    m_cam.options->framerate = m_fps;
    m_cam.options->camera = m_deviceIndex;

    // Start the libcamera stream
    m_cam.startVideo();
    m_isRunning = true;

    std::cout << "[Camera] Opened Camera Module 3 natively via LCCV at " << m_width << "x" << m_height << " " << m_fps << "fps\n";
    return true;
}

/*
    Helper function. Grabs the a frame from the camera and places it inside the frame matrix.
*/
bool Camera::readFrame(cv::Mat &frame) {
    if (!m_isRunning)
        return false;
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
