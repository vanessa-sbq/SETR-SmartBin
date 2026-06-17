#pragma once

struct MotorCommand {
    float velX; // forward velocity (m/s)
    float velY; // lateral velocity (m/s, +right)
    float distanceM;
    bool stop;
    float wheelFL;
    float wheelFR;
    float wheelRL;
    float wheelRR;
};
