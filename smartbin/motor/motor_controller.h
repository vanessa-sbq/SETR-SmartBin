#pragma once

#include "MotorPins.h"
#include <PiPCA9685/PCA9685.h>
#include <cstdint>
#include <gpiod.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

class MotorController {
  public:
    MotorController(const std::string &gpio_chip, const MotorPins &pins) : chip_gpio(nullptr) {
        chip_gpio = gpiod_chip_open(gpio_chip.c_str());
        if (!chip_gpio) {
            throw std::runtime_error("Failed to open GPIO chip");
        }

        // Collect all 8 offsets into an array
        unsigned int offsets[8] = {
            static_cast<unsigned int>(pins.in1_front), // in1_front_offset
            static_cast<unsigned int>(pins.in2_front), // in2_front_offset
            static_cast<unsigned int>(pins.in3_front), // in3_front_offset
            static_cast<unsigned int>(pins.in4_front), // in4_front_offset
            static_cast<unsigned int>(pins.in1_rear), // in1_rear_offset
            static_cast<unsigned int>(pins.in2_rear), // in2_rear_offset
            static_cast<unsigned int>(pins.in3_rear), // in3_rear_offset
            static_cast<unsigned int>(pins.in4_rear), // in4_rear_offset
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

        request_gpio = gpiod_chip_request_lines(chip_gpio, req_cfg, line_cfg);
        gpiod_request_config_free(req_cfg);
        gpiod_line_config_free(line_cfg);

        if (!request_gpio) {
            throw std::runtime_error("Failed to request GPIO lines");
        }

        // Store offsets for later use
        in1_front_offset = offsets[0];
        in2_front_offset = offsets[1];
        in3_front_offset = offsets[2];
        in4_front_offset = offsets[3];
        in1_rear_offset = offsets[4];
        in2_rear_offset = offsets[5];
        in3_rear_offset = offsets[6];
        in4_rear_offset = offsets[7];
    }

    ~MotorController() {
        if (request_gpio) {
            gpiod_line_request_release(request_gpio);
        }
        if (chip_gpio) {
            gpiod_chip_close(chip_gpio);
        }
    }

    void move_individual(std::vector<double> speeds);

    void stop_car();
    void go_ahead();
    void go_back();
    void turn_right();
    void turn_left();
    void shift_left();
    void shift_right();
    void upper_right();
    void lower_left();
    void upper_left();
    void lower_right();
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

    gpiod_chip *chip_gpio;
    struct gpiod_line_request *request_gpio = nullptr;

    unsigned int in1_front_offset;
    unsigned int in2_front_offset;
    unsigned int in3_front_offset;
    unsigned int in4_front_offset;
    unsigned int in1_rear_offset;
    unsigned int in2_rear_offset;
    unsigned int in3_rear_offset;
    unsigned int in4_rear_offset;
};