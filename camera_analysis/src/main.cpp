#include "camera.hpp"
#include "detector.hpp"
#include "motor_controller.hpp"
#include "config.hpp"

#include <iostream>
#include <csignal>
#include <atomic>
#include <cmath>

static std::atomic<bool> g_running{true};
static void onSignal(int) { g_running = false; }

int main(int argc, char* argv[]) {
    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    int deviceIndex = (argc > 1) ? std::stoi(argv[1]) : Config::DEVICE_INDEX;

    Camera          cam(deviceIndex, Config::FRAME_W, Config::FRAME_H, Config::FRAME_FPS);
    Detector        detector;
    MotorController motor(Config::FRAME_W, Config::FRAME_H);

    if (!cam.open()) {
        std::cerr << "Could not open camera. Exiting.\n";
        return 1;
    }

    std::cout << "Trashcan tracker running. Press Ctrl-C to stop.\n";

    if (Config::SHOW_WINDOW)
        cv::namedWindow("Trashcan Tracker", cv::WINDOW_AUTOSIZE);

    cv::Mat frame;
    int frameCount = 0;

    // Calculate focal length parameters mirroring the motor controller configuration
    float vFovRad = Config::V_FOV_DEG * 3.14159265f / 180.f;
    float focalLengthPx = (static_cast<float>(Config::FRAME_H) / 2.f) / std::tan(vFovRad / 2.f);

    while (g_running) {
        if (!cam.readFrame(frame)) {
            std::cerr << "[Main] Failed to read frame — trying again...\n";
            continue;
        }

        // Ask the motor controller if the can is currently moving, then pass
        // that flag to the detector so it can manage learning rate + cooldown.
        bool moving = motor.isMoving();
        DetectionResult det = detector.detect(frame, moving);

        // ── FIX SEQUENCE: Compute motor data BEFORE drawing the frame components ──
        int bboxH = det.detected ? det.boundingBox.height : 0;
        MotorCommand cmd = motor.compute(det.detected, det.centroid, bboxH);

        if (Config::SHOW_WINDOW) {
            cv::Mat debug = detector.drawDebug(frame, det);

            // Default middle coordinates
            cv::Point targetPoint(Config::FRAME_W / 2, Config::FRAME_H / 2);

            // Recalculate target point crosshair downward exactly 15 cm dynamically 
            if (det.detected && cmd.distanceM > 0.f) {
                int pixelOffsetY = static_cast<int>((0.15f * focalLengthPx) / cmd.distanceM);
                targetPoint.y += pixelOffsetY;
            }

            // Draw target point crosshair (Cyan marker)
            cv::drawMarker(debug, targetPoint, {255, 255, 0}, cv::MARKER_CROSS, 24, 1);

            if (det.detected && cmd.distanceM > 0.f) {
                std::string distStr = "Dist: " +
                    std::to_string(static_cast<int>(cmd.distanceM * 100)) + " cm";
                cv::putText(debug, distStr, {Config::FRAME_W - 110, 18},
                            cv::FONT_HERSHEY_SIMPLEX, 0.45, {0, 255, 255}, 1);
            }

            cv::putText(debug, "Frame: " + std::to_string(frameCount++),
                        {5, Config::FRAME_H - 10}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {255, 255, 255}, 1);

            cv::imshow("Trashcan Tracker", debug);
            
            // Break loop if ESC is pressed in the GUI window
            if (cv::waitKey(1) == 27) {
                g_running = false;
            }
        }
    }

    std::cout << "\n[Main] Shutting down streams cleanly...\n";
    cam.release();
    if (Config::SHOW_WINDOW) {
        cv::destroyAllWindows();
    }
    
    return 0;
}