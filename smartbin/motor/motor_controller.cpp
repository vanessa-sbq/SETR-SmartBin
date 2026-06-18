#include "motor_controller.h"

void MotorController::stop_car() {
    set_line(in1_front_offset, 0);
    set_line(in2_front_offset, 0);
    set_line(in3_front_offset, 0);
    set_line(in4_front_offset, 0);
    set_line(in1_rear_offset, 0);
    set_line(in2_rear_offset, 0);
    set_line(in3_rear_offset, 0);
    set_line(in4_rear_offset, 0);
}

void MotorController::go_ahead() {
    rl_ahead();
    rr_ahead();
    fl_ahead();
    fr_ahead();
}

void MotorController::go_back() {
    rr_back();
    rl_back();
    fr_back();
    fl_back();
}

void MotorController::turn_right() {
    rl_ahead();
    rr_back();
    fl_ahead();
    fr_back();
}

void MotorController::turn_left() {
    rr_ahead();
    rl_back();
    fr_ahead();
    fl_back();
}

void MotorController::shift_left() {
    fr_ahead();
    rr_back();
    rl_ahead();
    fl_back();
}

void MotorController::shift_right() {
    fr_back();
    rr_ahead();
    rl_back();
    fl_ahead();
}

void MotorController::upper_right() {
    rr_ahead();
    fl_ahead();
}

void MotorController::lower_left() {
    rr_back();
    fl_back();
}

void MotorController::upper_left() {
    fr_ahead();
    rl_ahead();
}

void MotorController::lower_right() {
    fr_back();
    rl_back();
}

/*
    This function sets the motor direction using GPIO for each wheel based on sign
*/
void MotorController::move_individual(std::vector<double> speeds) {
    // Front right
    if (speeds[0] >= 0.0) {
        fr_ahead();
    } else {
        fr_back();
    }

    // Front Left
    if (speeds[1] >= 0.0) {
        fl_ahead();
    } else {
        fl_back();
    }

    // Back right
    if (speeds[2] >= 0.0) {
        rr_ahead();
    } else {
        rr_back();
    }

    // Back Left
    if (speeds[3] >= 0.0) {
        rl_ahead();
    } else {
        rl_back();
    }
}

void MotorController::set_line(unsigned int offset, int value) {
    enum gpiod_line_value val = value ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE;
    if (gpiod_line_request_set_value(request_gpio, offset, val) < 0) {
        throw std::runtime_error("Failed to set GPIO value");
    }
}

void MotorController::fr_ahead() {
    set_line(in1_rear_offset, 1);
    set_line(in2_rear_offset, 0);
}

void MotorController::fr_back() {
    set_line(in1_rear_offset, 0);
    set_line(in2_rear_offset, 1);
}

void MotorController::fl_ahead() {
    set_line(in3_rear_offset, 1);
    set_line(in4_rear_offset, 0);
}

void MotorController::fl_back() {
    set_line(in3_rear_offset, 0);
    set_line(in4_rear_offset, 1);
}

void MotorController::rr_ahead() {
    set_line(in1_front_offset, 1);
    set_line(in2_front_offset, 0);
}

void MotorController::rr_back() {
    set_line(in1_front_offset, 0);
    set_line(in2_front_offset, 1);
}

void MotorController::rl_ahead() {
    set_line(in3_front_offset, 1);
    set_line(in4_front_offset, 0);
}

void MotorController::rl_back() {
    set_line(in3_front_offset, 0);
    set_line(in4_front_offset, 1);
}