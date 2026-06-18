#include "hardware_drive.hpp"
#include "motor/MotorPins.h"
#include "motor/motor_controller.h"
#include <PiPCA9685/PCA9685.h>
#include <cmath>
#include <iostream>
#include <memory>

static std::unique_ptr<PiPCA9685::PCA9685> pwm;
static std::unique_ptr<MotorController> motors;

// Normalized wheel speed (-1 to 1) -> PCA9685 PWM values (0 to 4095).
static uint16_t toPwm(float v) {
    float mag = std::abs(v);
    if (mag > 1.f)
        mag = 1.f;
    float pwm_value = mag * 4095.f;

    if (pwm_value < 2000)
        pwm_value = 2000;
    return static_cast<uint16_t>(pwm_value);
}

/*
    Initializes all the hardware required for the motors to be able to spin.
*/
bool hardwareInit() {
    try {
        MotorPins pins;
        motors = std::make_unique<MotorController>("/dev/gpiochip0", pins);
        pwm = std::make_unique<PiPCA9685::PCA9685>("/dev/i2c-1", 0x7f);
        pwm->set_pwm_freq(60.0);
        std::cout << "[Motor Hardware] Motor driver ready.\n";
        return true;
    } catch (const std::exception &e) {
        std::cerr << "[Motor Hardware] Init failed: " << e.what() << "\n";
        return false;
    }
}

void hardwareApply(const MotorCommand &cmd) {
    if (!motors || !pwm)
        return;

    if (cmd.stop) {
        motors->stop_car();
        pwm->set_all_pwm(0, 0);
        return;
    }

    // PCA9685 channels
    // 0 = FR, 1 = FL, 2 = RR, 3 = RL
    pwm->set_pwm(0, 0, toPwm(cmd.wheelFR));
    pwm->set_pwm(1, 0, toPwm(cmd.wheelFL));
    pwm->set_pwm(2, 0, toPwm(cmd.wheelRR));
    pwm->set_pwm(3, 0, toPwm(cmd.wheelRL));

    motors->move_individual({cmd.wheelFR, cmd.wheelFL, cmd.wheelRR, cmd.wheelRL});
}

void hardwareShutdown() {
    if (motors)
        motors->stop_car();
    if (pwm)
        pwm->set_all_pwm(0, 0);
    motors.reset();
    pwm.reset();
    std::cout << "[Motor Hardware] Motors stopped.\n";
}
