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
    int deviceIndex;
    int camera_width;
    int camera_height;
    int camera_fps;
    lccv::PiCamera lccv_camera;
    bool isRunning = false;
};