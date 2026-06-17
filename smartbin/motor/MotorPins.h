#pragma once

/*
    Raspberry GPIO Pins for the PWM board.
*/
struct MotorPins {
    int in1_front = 23;
    int in2_front = 24;
    int in3_front = 27;
    int in4_front = 22;

    int in1_rear = 21;
    int in2_rear = 20;
    int in3_rear = 16;
    int in4_rear = 12;
};