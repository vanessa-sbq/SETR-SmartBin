#include "camera.hpp"
#include "detector.hpp"
#include "motor_translation.hpp"
#include "config.hpp"
#include "hardware_drive.hpp"

#include <iostream>
#include <csignal>
#include <atomic>

static std::atomic<bool> g_running{true};
static void onSignal(int) { g_running = false; }

int main(int argc, char* argv[]) {
    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    int deviceIndex = (argc > 1) ? std::stoi(argv[1]) : Config::DEVICE_INDEX;

    Camera cam(deviceIndex, Config::FRAME_W, Config::FRAME_H, Config::FRAME_FPS);
    auto detector = makeDetector(DetectorKind::Hsv); // TODO: Select algorithm to use for detection
    MotorTranslation motor(Config::FRAME_W, Config::FRAME_H);

    if (!cam.open()) {
        std::cerr << "Could not open camera. Exiting.\n";
        return 1;
    }

    bool hwOk = hardwareInit();

    std::cout << "Trashcan tracker running. Press Ctrl-C to stop.\n";

    if (Config::SHOW_WINDOW)
        cv::namedWindow("Trashcan Tracker", cv::WINDOW_AUTOSIZE);

    cv::Mat frame;
    int frameCount = 0;

    while (g_running) {
        if (!cam.readFrame(frame)) {
            std::cerr << "[Main] Failed to read frame — retrying...\n";
            cv::waitKey(30);
            continue;
        }

        // Ask the motor controller if the can is currently moving, then pass
        // that flag to the detector so it can manage learning rate + cooldown.
        bool moving = motor.isMoving();
        DetectionResult det = detector->detect(frame, moving);

        MotorCommand cmd = motor.compute(det.detected, det.centroid, det.boundingBox);
        if (hwOk) hardwareApply(cmd);

        if (Config::SHOW_WINDOW) {
            cv::Mat debug = detector->drawDebug(frame, det);

            cv::Point centre(Config::FRAME_W / 2, Config::FRAME_H / 2);
            cv::drawMarker(debug, centre, {255, 255, 0}, cv::MARKER_CROSS, 24, 1);

            if (det.detected && cmd.distanceM > 0.f) {
                std::string distStr = "Dist: " +
                    std::to_string(static_cast<int>(cmd.distanceM * 100)) + " cm";
                cv::putText(debug, distStr, {Config::FRAME_W - 110, 18},
                            cv::FONT_HERSHEY_SIMPLEX, 0.45, {0, 255, 255}, 1);
            }

            cv::putText(debug, "Frame: " + std::to_string(frameCount++),
                        {5, Config::FRAME_H - 8},
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, {200, 200, 200}, 1);

            cv::imshow("Trashcan Tracker", debug);
            if (cv::waitKey(1) == 'q') break;
        }
    }

    std::cout << "\nShutting down.\n";
    hardwareShutdown();
    cam.release();
    cv::destroyAllWindows();
    return 0;
}