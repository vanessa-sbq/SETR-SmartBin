# SmartBin - Balloon-catching robot

A four‑wheeled omnidirectional robot that uses camera-based detection and real‑time scheduling to catch a falling balloon in an onboard bin. Runs on a Raspberry Pi 5 under a PREEMPT_RT-patched kernel with tasks scheduled using SCHED_DEADLINE (EDF).

## Hardware
- Raspberry Pi 5 (8 GB)
- Raspberry Pi Camera Module 3
- Osoyoo FlexiRover (mecanum wheels)
- ESP8266 + NES controller (operation interface)
- PWM hat for motor drivers

## Software & Architecture
- Language: C++ with OpenCV and libcamera (LCCV)
- Concurrency: POSIX threads; shared-state protected with mutexes and condition variables
- Task set:
  - Vision Detection - periodic (T = 50 ms): HSV-based segmentation, contour extraction
  - Trajectory Prediction - sporadic (MIT ≈ 33 ms): compute interception target
  - Motor Control - periodic (T = 20 ms): issue PWM commands via I2C/PWM hat
  - Operation Interface - periodic (T = 100 ms): poll remote for start/stop
- Timing: drift-free periodic timing via `clock_nanosleep(CLOCK_MONOTONIC, TIME_ABSTIME)`

## Key results
- Measured total CPU utilization: U = 0.921 (schedulable under EDF)
- Vision processing dominates (≈0.871 of U)
- System reliably detects the balloon and repositions the bin in most tests
- Main limitations: mechanical stability, wheel speed, and lighting sensitivity of the vision pipeline

## Videos
- Balloon Catching: https://youtu.be/S61jXNjOkig
- Operation Interface: https://youtu.be/8Dc4dnWEBN0

## Where to look next
- See subfolder READMEs and source:
  - [NesController](NesController) - microcontroller firmware (PlatformIO)
  - [smartbin](smartbin) - main app, drivers, Makefile
  - [wifi_raspberry](wifi_raspberry) - Raspberry Pi Wi‑Fi helper
  - [testing](testing) - analysis and experiments
