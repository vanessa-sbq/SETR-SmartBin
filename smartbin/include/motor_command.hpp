#pragma once

struct MotorCommand {
    float velX;      // forward velocity (m/s)
    float velY;      // lateral velocity (m/s, +right)
    float distanceM;
    bool  stop;
    float wheelFL;   // normalized -1..1
    float wheelFR;
    float wheelRL;
    float wheelRR;
    float ballVx;    // ball velocity in robot frame (m/s), 0 if unknown
    float ballVy;
    float ballVz;
};
