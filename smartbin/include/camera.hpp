#pragma once
#include <opencv2/opencv.hpp>

#include <lccv.hpp>

class Camera {
  public:
    Camera(int deviceIndex, int width, int height, int fps);
    ~Camera();

    bool open();
    bool readFrame(cv::Mat &frame);
    void release();

  private:
    int m_deviceIndex;
    int m_width;
    int m_height;
    int m_fps;
    lccv::PiCamera m_cam;
    bool m_isRunning = false;
};