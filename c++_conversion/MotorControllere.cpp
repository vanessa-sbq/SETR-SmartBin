#include "MotorControllere.h"

void MotorController::change_speed(uint16_t speed) {
    /* pwm_.set_duty_cycle(0, speed);
    pwm_.set_duty_cycle(1, speed);
    pwm_.set_duty_cycle(2, speed);
    pwm_.set_duty_cycle(3, speed); */
}

void MotorController::stop_car() {
    set_line(in1_front_offset_, 0);
    set_line(in2_front_offset_, 0);
    set_line(in3_front_offset_, 0);
    set_line(in4_front_offset_, 0);
    set_line(in1_rear_offset_, 0);
    set_line(in2_rear_offset_, 0);
    set_line(in3_rear_offset_, 0);
    set_line(in4_rear_offset_, 0);
    change_speed(0);
}

void MotorController::go_ahead(uint16_t speed) {
    rl_ahead();
    rr_ahead();
    fl_ahead();
    fr_ahead();
    change_speed(speed);
}

void MotorController::go_back(uint16_t speed) {
    rr_back();
    rl_back();
    fr_back();
    fl_back();
    change_speed(speed);
}

void MotorController::turn_right(uint16_t speed) {
    rl_ahead();
    rr_back();
    fl_ahead();
    fr_back();
    change_speed(speed);
}

void MotorController::turn_left(uint16_t speed) {
    rr_ahead();
    rl_back();
    fr_ahead();
    fl_back();
    change_speed(speed);
}

void MotorController::shift_left(uint16_t speed) {
    fr_ahead();
    rr_back();
    rl_ahead();
    fl_back();
    change_speed(speed);
}

void MotorController::shift_right(uint16_t speed) {
    fr_back();
    rr_ahead();
    rl_back();
    fl_ahead();
    change_speed(speed);
}

void MotorController::upper_right(uint16_t speed) {
    rr_ahead();
    fl_ahead();
    change_speed(speed);
}

void MotorController::lower_left(uint16_t speed) {
    rr_back();
    fl_back();
    change_speed(speed);
}

void MotorController::upper_left(uint16_t speed) {
    fr_ahead();
    rl_ahead();
    change_speed(speed);
}

void MotorController::lower_right(uint16_t speed) {
    fr_back();
    rl_back();
    change_speed(speed);
}

void MotorController::set_line(unsigned int offset, int value) {
    enum gpiod_line_value val = value ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE;
    if (gpiod_line_request_set_value(request_, offset, val) < 0) {
        throw std::runtime_error("Failed to set GPIO value");
    }
}

void MotorController::rr_ahead() {
    set_line(in1_rear_offset_, 1);
    set_line(in2_rear_offset_, 0);
}

void MotorController::rr_back() {
    set_line(in1_rear_offset_, 0);
    set_line(in2_rear_offset_, 1);
}

void MotorController::rl_ahead() {
    set_line(in3_rear_offset_, 1);
    set_line(in4_rear_offset_, 0);
}

void MotorController::rl_back() {
    set_line(in3_rear_offset_, 0);
    set_line(in4_rear_offset_, 1);
}

void MotorController::fr_ahead() {
    set_line(in1_front_offset_, 1);
    set_line(in2_front_offset_, 0);
}

void MotorController::fr_back() {
    set_line(in1_front_offset_, 0);
    set_line(in2_front_offset_, 1);
}

void MotorController::fl_ahead() {
    set_line(in3_front_offset_, 1);
    set_line(in4_front_offset_, 0);
}

void MotorController::fl_back() {
    set_line(in3_front_offset_, 0);
    set_line(in4_front_offset_, 1);
}