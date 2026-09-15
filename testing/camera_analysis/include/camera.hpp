#pragma once
#include <opencv2/opencv.hpp>
#include <lccv.hpp> // Include the new native libcamera wrapper

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

    // Replace cv::VideoCapture with LCCV
    lccv::PiCamera m_cam; 
    bool m_isRunning = false;
};