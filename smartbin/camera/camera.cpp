#include "camera.hpp"
#include <iostream>

Camera::Camera(int deviceIndex, int width, int height, int fps) : deviceIndex(deviceIndex), camera_width(width), camera_height(height), camera_fps(fps) {}

Camera::~Camera() { release(); }

bool Camera::open() {
    // Configure the native lccv options
    lccv_camera.options->video_width = camera_width;
    lccv_camera.options->video_height = camera_height;
    lccv_camera.options->framerate = camera_fps;
    lccv_camera.options->camera = deviceIndex;

    // Start the libcamera stream
    lccv_camera.startVideo();
    isRunning = true;

    std::cout << "[Camera] Opened Camera Module 3 natively via LCCV at " << camera_width << "x" << camera_height << " " << camera_fps << "fps\n";
    return true;
}

/*
    Helper function. Grabs the a frame from the camera and places it inside the frame matrix.
*/
bool Camera::readFrame(cv::Mat &frame) {
    if (!isRunning)
        return false;
    bool success = lccv_camera.getVideoFrame(frame, 1000);
    return success && !frame.empty();
}

void Camera::release() {
    if (isRunning) {
        lccv_camera.stopVideo();
        isRunning = false;
        std::cout << "[Camera] Released libcamera stream.\n";
    }
}
