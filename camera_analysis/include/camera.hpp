#pragma once
#include <opencv2/opencv.hpp>
#include <string>

#include "config.hpp"

class Camera {
public:
    // deviceIndex: 0 = first webcam/phone, or pass a path like "/dev/video0"
    explicit Camera(int deviceIndex = 0, int width = Config::FRAME_W, int height = Config::FRAME_H, int fps = Config::FRAME_FPS);
    ~Camera();

    bool open();
    bool readFrame(cv::Mat& frame);
    void release();

    int width()  const { return m_width; }
    int height() const { return m_height; }

private:
    int            m_deviceIndex;
    int            m_width, m_height, m_fps;
    cv::VideoCapture m_cap;
};
