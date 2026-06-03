#pragma once

#include <iostream>
#include <cstdint>
#include <string>
#include <gpiod.h>
#include <PiPCA9685/PCA9685.h>
#include "MotorPins.h"
#include <stdexcept>
#include <vector>

class MotorController {
public:
    MotorController(const std::string &gpio_chip, const MotorPins &pins/*, PiPCA9685::PCA9685 &pwm */): chip_(nullptr) /*, pwm_(pwm) */ {
        chip_ = gpiod_chip_open(gpio_chip.c_str());
        if (!chip_) {
            throw std::runtime_error("Failed to open GPIO chip");
        }

        // Collect all 8 offsets into an array
        unsigned int offsets[8] = {
            static_cast<unsigned int>(pins.in1_front), //in1_front_offset_
            static_cast<unsigned int>(pins.in2_front), //in2_front_offset_
            static_cast<unsigned int>(pins.in3_front), //in3_front_offset_
            static_cast<unsigned int>(pins.in4_front), //in4_front_offset_
            static_cast<unsigned int>(pins.in1_rear), //in1_rear_offset_
            static_cast<unsigned int>(pins.in2_rear), //in2_rear_offset_
            static_cast<unsigned int>(pins.in3_rear), //in3_rear_offset_
            static_cast<unsigned int>(pins.in4_rear), //in4_rear_offset_
        };

        // Configure all lines as output, default low
        struct gpiod_line_settings *settings = gpiod_line_settings_new();
        if (!settings) {
            throw std::runtime_error("Failed to create line settings");
        }
        gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
        gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

        struct gpiod_line_config *line_cfg = gpiod_line_config_new();
        if (!line_cfg) {
            gpiod_line_settings_free(settings);
            throw std::runtime_error("Failed to create line config");
        }
        if (gpiod_line_config_add_line_settings(line_cfg, offsets, 8, settings) < 0) {
            gpiod_line_settings_free(settings);
            gpiod_line_config_free(line_cfg);
            throw std::runtime_error("Failed to add line settings");
        }
        gpiod_line_settings_free(settings);

        struct gpiod_request_config *req_cfg = gpiod_request_config_new();
        if (!req_cfg) {
            gpiod_line_config_free(line_cfg);
            throw std::runtime_error("Failed to create request config");
        }
        gpiod_request_config_set_consumer(req_cfg, "MotorController");

        request_ = gpiod_chip_request_lines(chip_, req_cfg, line_cfg);
        gpiod_request_config_free(req_cfg);
        gpiod_line_config_free(line_cfg);

        if (!request_) {
            throw std::runtime_error("Failed to request GPIO lines");
        }

        // Store offsets for later use
        in1_front_offset_ = offsets[0];
        in2_front_offset_ = offsets[1];
        in3_front_offset_ = offsets[2];
        in4_front_offset_ = offsets[3];
        in1_rear_offset_  = offsets[4];
        in2_rear_offset_  = offsets[5];
        in3_rear_offset_  = offsets[6];
        in4_rear_offset_  = offsets[7];
    }

    ~MotorController() {
        if (request_) {
            gpiod_line_request_release(request_);
        }
        if (chip_) {
            gpiod_chip_close(chip_);
        }
    }

    void change_speed(uint16_t speed);
    void stop_car();

    void go_ahead(uint16_t speed);
    void go_back(uint16_t speed);
    void turn_right(uint16_t speed);
    void turn_left(uint16_t speed);
    void shift_left(uint16_t speed);
    void shift_right(uint16_t speed);
    void upper_right(uint16_t speed);
    void lower_left(uint16_t speed);
    void upper_left(uint16_t speed);
    void lower_right(uint16_t speed);
    void move_individual(std::vector<double> speeds);
    void rr_back();
    void fr_ahead();
    void fl_ahead();
    void rl_back();
    void fr_back();
    void rr_ahead();
    void rl_ahead();
    void fl_back();

private:
   

    void set_line(unsigned int offset, int value);

    gpiod_chip *chip_;
    struct gpiod_line_request *request_ = nullptr;

    unsigned int in1_front_offset_;
    unsigned int in2_front_offset_;
    unsigned int in3_front_offset_;
    unsigned int in4_front_offset_;
    unsigned int in1_rear_offset_;
    unsigned int in2_rear_offset_;
    unsigned int in3_rear_offset_;
    unsigned int in4_rear_offset_;

    /* PiPCA9685::PCA9685 &pwm_; */
};