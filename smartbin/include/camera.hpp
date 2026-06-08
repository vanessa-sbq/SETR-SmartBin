#pragma once
#include <opencv2/opencv.hpp>

// On the Raspberry Pi we grab frames natively through libcamera (LCCV).
// On a desktop (SMARTBIN_DESKTOP) we fall back to OpenCV's VideoCapture so the
// camera/detector pipeline can be tested with a USB webcam or DroidCam.
#ifndef SMARTBIN_DESKTOP
#include <lccv.hpp> // native libcamera wrapper (Pi only)
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

#ifdef SMARTBIN_DESKTOP
    cv::VideoCapture m_cam;
#else
    lccv::PiCamera m_cam;
#endif
    bool m_isRunning = false;
};