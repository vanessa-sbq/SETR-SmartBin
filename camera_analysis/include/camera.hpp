#pragma once
#include <opencv2/opencv.hpp>

// Only include the Raspberry Pi camera library if we are NOT on a simulated PC/WSL camera platform
#ifndef USING_OPENCV_CAM
    #include <lccv.hpp> 
#endif

class Camera {
public:
    Camera(int deviceIndex, int width, int height, int fps);
    ~Camera();

    bool open();
    bool readFrame(cv::Mat& frame);
    void release();

private:
    int m_deviceIndex;
    int m_width;
    int m_height;
    int m_fps;
    bool m_isRunning = false;

    // Use standard OpenCV capture on PC/WSL environments, and lccv on the Pi hardware
#ifdef USING_OPENCV_CAM
    cv::VideoCapture m_cam;
#else
    lccv::PiCamera m_cam; 
#endif
};