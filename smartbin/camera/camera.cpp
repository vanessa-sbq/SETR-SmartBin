#include "camera.hpp"
#include <iostream>

Camera::Camera(int deviceIndex, int width, int height, int fps)
    : m_deviceIndex(deviceIndex), m_width(width), m_height(height), m_fps(fps) {}

Camera::~Camera() {
    release();
}

#ifdef SMARTBIN_DESKTOP
// Desktop backend: OpenCV VideoCapture (USB webcam / DroidCam)

bool Camera::open() {
    if (!m_cam.open(m_deviceIndex)) {
        std::cerr << "[Camera] Could not open device " << m_deviceIndex << " via OpenCV VideoCapture.\n";
        return false;
    }

    m_cam.set(cv::CAP_PROP_FRAME_WIDTH,  m_width);
    m_cam.set(cv::CAP_PROP_FRAME_HEIGHT, m_height);
    m_cam.set(cv::CAP_PROP_FPS,          m_fps);
    m_isRunning = true;

    std::cout << "[Camera] Opened device " << m_deviceIndex
              << " via OpenCV VideoCapture at "
              << static_cast<int>(m_cam.get(cv::CAP_PROP_FRAME_WIDTH))  << "x"
              << static_cast<int>(m_cam.get(cv::CAP_PROP_FRAME_HEIGHT)) << "\n";
    return true;
}

/*
    Helper function. Grabs a frame from the camera and places it inside the frame matrix.
*/
bool Camera::readFrame(cv::Mat& frame) {
    if (!m_isRunning) return false;
    bool success = m_cam.read(frame);
    return success && !frame.empty();
}

void Camera::release() {
    if (m_isRunning) {
        m_cam.release();
        m_isRunning = false;
        std::cout << "[Camera] Released VideoCapture.\n";
    }
}

#else
// Raspberry Pi backend: native libcamera via LCCV

bool Camera::open() {
    // Configure the native libcamera options
    m_cam.options->video_width = m_width;
    m_cam.options->video_height = m_height;
    m_cam.options->framerate = m_fps;
    m_cam.options->camera = m_deviceIndex; // Select camera 0 or 1

    // TODO: Enable Autofocus for Camera Module 3!
    //m_cam.options->autofocusMode = libcamera::controls::AfModeEnum::AfModeContinuous;

    // Start the libcamera stream
    m_cam.startVideo();
    m_isRunning = true;

    std::cout << "[Camera] Opened Camera Module 3 natively via LCCV at " << m_width << "x" << m_height << " " << m_fps << "fps\n";
    return true;
}

/*
    Helper function. Grabs the a frame from the camera and places it inside the frame matrix.
*/
bool Camera::readFrame(cv::Mat& frame) {
    if (!m_isRunning) return false;
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

#endif