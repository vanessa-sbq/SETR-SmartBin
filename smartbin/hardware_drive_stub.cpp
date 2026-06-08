#include "hardware_drive.hpp"
#include "config.hpp"
#include <iostream>

// Desktop (SMARTBIN_DESKTOP) stub for the motor hardware.
// On a PC there is no PCA9685/gpiod, so this replaces hardware_drive.cpp.
// hardwareInit() returns false, which makes main.cpp skip hardwareApply().

bool hardwareInit() {
    std::cout << "[HW] Desktop build - motor hardware disabled (stub).\n";
    return false;
}

void hardwareApply(const MotorCommand& cmd) {
    // Not called by main.cpp when hardwareInit() returns false, but kept so the motor decisions can still be logged.
    if (cmd.stop) {
        std::cout << "[HW-stub] STOP\n";
        return;
    }
    std::cout << "[HW-stub] FR=" << cmd.wheelFR << " FL=" << cmd.wheelFL
              << " RR=" << cmd.wheelRR << " RL=" << cmd.wheelRL << "\n";
}

void hardwareShutdown() {
    std::cout << "[HW] Desktop stub shutdown.\n";
}
