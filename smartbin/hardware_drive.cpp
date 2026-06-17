#include "hardware_drive.hpp"
#include "motor/MotorPins.h"
#include "motor/motor_controller.h"
#include <PiPCA9685/PCA9685.h>
#include <cmath>
#include <iostream>
#include <memory>

static std::unique_ptr<PiPCA9685::PCA9685> g_pwm;
static std::unique_ptr<MotorController> g_motors;

// Normalized wheel speed (-1..1) -> PCA9685 on-time (0..4095).
static uint16_t toPwm(float v) {
    float mag = std::abs(v);
    if (mag > 1.f)
        mag = 1.f;
    float pwm_value = mag * 4095.f;

    if (pwm_value < 2000)
        pwm_value = 2000;
    return static_cast<uint16_t>(pwm_value);
}

// // Thin wrapper so main.cpp never sees motor/motor_controller.h
// (that header also defines a class called MotorTranslation).
bool hardwareInit() {
    try {
        MotorPins pins;
        g_motors = std::make_unique<MotorController>("/dev/gpiochip0", pins);
        g_pwm = std::make_unique<PiPCA9685::PCA9685>("/dev/i2c-1", 0x7f);
        g_pwm->set_pwm_freq(60.0);
        std::cout << "[HW] Motor driver ready.\n";
        return true;
    } catch (const std::exception &e) {
        std::cerr << "[HW] Init failed: " << e.what() << "\n";
        return false;
    }
}

void hardwareApply(const MotorCommand &cmd) {
    if (!g_motors || !g_pwm)
        return;

    if (cmd.stop) {
        g_motors->stop_car();
        g_pwm->set_all_pwm(0, 0);
        return;
    }

    // PCA9685 channels
    // 0 = FR, 1 = FL, 2 = RR, 3 = RL
    g_pwm->set_pwm(0, 0, toPwm(cmd.wheelFR));
    g_pwm->set_pwm(1, 0, toPwm(cmd.wheelFL));
    g_pwm->set_pwm(2, 0, toPwm(cmd.wheelRR));
    g_pwm->set_pwm(3, 0, toPwm(cmd.wheelRL));

    g_motors->move_individual({cmd.wheelFR, cmd.wheelFL, cmd.wheelRR, cmd.wheelRL});
}

void hardwareShutdown() {
    if (g_motors)
        g_motors->stop_car();
    if (g_pwm)
        g_pwm->set_all_pwm(0, 0);
    g_motors.reset();
    g_pwm.reset();
    std::cout << "[HW] Motors stopped.\n";
}
