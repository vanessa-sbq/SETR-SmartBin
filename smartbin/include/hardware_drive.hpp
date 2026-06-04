#pragma once
#include "motor_command.hpp"

// Thin wrapper so main.cpp never sees motor/motor_controller.h
// (that header also defines a class called MotorTranslation).
bool hardwareInit();
void hardwareApply(const MotorCommand& cmd);
void hardwareShutdown();
