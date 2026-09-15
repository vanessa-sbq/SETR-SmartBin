#pragma once
#include "motor_command.hpp"

bool hardwareInit();
void hardwareApply(const MotorCommand &cmd);
void hardwareShutdown();
