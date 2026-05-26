# Trashcan Tracker

Movement-detection tracker for a Raspberry Pi–based autonomous trashcan.
Uses **OpenCV MOG2 background subtraction** — no neural network, runs at 30fps on a Pi Zero 2W.

---

## Project Structure

```
trashcan/
├── CMakeLists.txt
├── include/
│   ├── config.hpp            ← single file for all tuning parameters
│   ├── camera.hpp
│   ├── detector.hpp
│   └── motor_controller.hpp
└── src/
    ├── main.cpp
    ├── camera.cpp
    ├── detector.cpp
    └── motor_controller.cpp
```

---

## How It Works

```
Phone (USB camera) → Camera → Detector (MOG2) → centroid + bounding box
                                                          ↓
                                                 MotorController
                                                /               \
                                          velX (m/s)        velY (m/s)
                                       (strafe to centre   (fwd/back to centre
                                        object on X axis)   object on Y axis)
                                                \               /
                                                 prints command
                                               (→ motor driver later)
```

1. **Camera** — grabs frames via OpenCV/V4L2.
2. **Detector** — runs MOG2 background subtraction, morphological cleanup, and contour finding. Returns the centroid, bounding box, and area of the largest moving blob.
3. **MotorController** — estimates the object's real-world distance from the bounding-box height (pinhole camera model), then converts the pixel error on both axes into m/s velocity commands. Keeps the object centred in the frame on both X and Y.

### Distance estimation

No fixed distance assumption is needed. Every frame, the controller derives distance from the bounding-box height using the pinhole model:

```
focalLength_px = (frameHeight / 2) / tan(vFov / 2)   ← computed once at startup
distance_m     = (objectRealHeightM × focalLength_px) / bboxHeight_px
```

This is a single multiply + divide per frame — negligible cost on any hardware including a Pi Zero.

---

## Configuration

**All tuning parameters live in one place: `include/config.hpp`.**
You should never need to touch any other file for normal tuning.

```cpp
namespace Config {

    // Camera
    DEVICE_INDEX = 0        // 0 = first device; override via command-line arg
    FRAME_W      = 640      // 16:9 width  (px)
    FRAME_H      = 360      // 16:9 height (px)
    FRAME_FPS    = 30

    // Lens FOV — adjust to your phone camera
    H_FOV_DEG    = 78.f     // horizontal field of view (degrees)
    V_FOV_DEG    = 43.9f    // = H_FOV × (9/16) for a 16:9 sensor

    // Detector (MOG2)
    DET_MIN_AREA       = 800    // blobs smaller than this (px²) are ignored
    DET_HISTORY        = 120    // background model memory (frames)
    DET_VAR_THRESHOLD  = 40.f   // MOG2 sensitivity — lower = more sensitive
    DET_ERODE_ITER     = 1
    DET_DILATE_ITER    = 2

    // Motor
    MOT_DEAD_ZONE_X     = 30.f   // px — horizontal error ignored (reduces jitter)
    MOT_DEAD_ZONE_Y     = 30.f   // px — vertical error ignored
    MOT_MAX_VEL_X       = 10.f   // m/s — max strafe speed
    MOT_MAX_VEL_Y       = 10.f   // m/s — max fwd/back speed
    MOT_OBJECT_HEIGHT_M = 0.15f  // real height of the tracked object (m)
                                 // tennis ball ≈ 0.067, bottle ≈ 0.22

    // Display
    SHOW_WINDOW = true      // set false on headless Pi
}
```

### Finding your phone's FOV

For a 16:9 phone camera, vertical FOV is `H_FOV × (9/16)`. To measure your exact horizontal FOV:

1. Point the camera at a wall from a known distance `d`.
2. Measure the visible width `w` on the wall.
3. `H_FOV = 2 × atan(w / (2 × d)) × (180 / π)`

Typical phone cameras: standard ≈ 78°, wide-angle ≈ 90°, ultra-wide ≈ 120°.

---

## 1. Use Your Phone as a USB Camera

### Option A — DroidCam (easiest, Android & iOS)

1. Install **DroidCam** on your phone (free tier is fine).
2. On your PC/Pi: install the DroidCam client.

**On Ubuntu/Pi:**
```bash
sudo apt install v4l2loopback-dkms
cd /tmp
wget -O droidcam.zip https://files.dev47apps.net/linux/droidcam_2.1.3.zip
unzip droidcam.zip && cd droidcam
sudo ./install-client
```

3. Connect phone via USB-C, enable **USB Tethering** on the phone.
4. Launch DroidCam on phone → launch `droidcam-cli` on PC.
5. A virtual `/dev/video0` (or `/dev/video2`) appears — use that index.

### Option B — scrcpy + v4l2loopback (Android, no app needed on phone)

```bash
sudo apt install scrcpy v4l2loopback-dkms ffmpeg

# Load virtual camera
sudo modprobe v4l2loopback devices=1 video_nr=10 card_label="Phone" exclusive_caps=1

# Stream phone screen into virtual camera
scrcpy --v4l2-sink=/dev/video10 --no-display
```

### Option C — IP Webcam (Android, Wi-Fi fallback)

1. Install **IP Webcam** on Android.
2. Start server → note the URL shown (e.g. `http://192.168.1.5:8080/video`).
3. In `main.cpp`, replace the `Camera` construction with an HTTP stream:

```cpp
cv::VideoCapture cap("http://192.168.1.5:8080/video");
```

---

## 2. Build on Your Development Machine (Linux / macOS)

### Prerequisites

```bash
# Ubuntu / Debian / Raspberry Pi OS
sudo apt update
sudo apt install -y cmake build-essential libopencv-dev

# macOS (Homebrew)
brew install cmake opencv
```

### Compile

```bash
cd trashcan
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

### Run

```bash
# Camera index 0 (default, set in config.hpp)
./build/trashcan

# Override camera index at runtime
./build/trashcan 1
```

Press **q** in the debug window or **Ctrl-C** in the terminal to stop.

---

## 3. Build on Raspberry Pi (on-device)

```bash
sudo apt update
sudo apt install -y cmake build-essential libopencv-dev

cd trashcan
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4

./build/trashcan
```

On a Pi 4 this runs ~30fps at 640×360. On Pi Zero 2W, drop to 320×180 in `config.hpp` and expect ~15–20fps — still plenty for tracking.

### Run headless (no monitor on Pi)

In `config.hpp`, set:
```cpp
constexpr bool SHOW_WINDOW = false;
```
Then rebuild. Motor commands still print to stdout.

---

## 4. Tuning Parameters

All parameters are in `include/config.hpp`. Common adjustments:

| Parameter | What it does |
|---|---|
| `FRAME_W` / `FRAME_H` | Resolution — lower = faster. Keep 16:9 (e.g. 320×180, 640×360) |
| `H_FOV_DEG` / `V_FOV_DEG` | Match to your phone's lens for accurate m/s and distance readings |
| `MOT_OBJECT_HEIGHT_M` | Real height of the object being tracked — critical for distance accuracy |
| `DET_MIN_AREA` | Increase to ignore small noise blobs |
| `DET_VAR_THRESHOLD` | Lower = more sensitive to movement; higher = ignores subtle motion |
| `MOT_DEAD_ZONE_X/Y` | Increase if the can jitters when the object is roughly centred |
| `MOT_MAX_VEL_X/Y` | Cap to your motor driver's safe speed (m/s) |

---

## 5. Wiring the Motor (next step)

When ready, replace `MotorController::printCommand()` with actual GPIO/serial writes.
`MotorCommand` gives you `velX` and `velY` in **m/s**, plus the estimated `distanceM`.

For a holonomic (omnidirectional) drive with an L298N or similar:

```cpp
// Example with pigpio (Pi GPIO library)
#include <pigpio.h>

void writeMotors(const MotorCommand& cmd) {
    // Convert m/s to [-255, 255] PWM range for your specific motor
    int pwmX = static_cast<int>(cmd.velX / Config::MOT_MAX_VEL_X * 255.f);
    int pwmY = static_cast<int>(cmd.velY / Config::MOT_MAX_VEL_Y * 255.f);
    // write pwmX / pwmY to your PWM pins
}
```

---

## Output Example

```
[Camera] Opened device 0 at 640x360 30fps
Trashcan tracker running. Press Ctrl-C to stop.
[MOTOR] STOP
[MOTOR] velX=  0.340 m/s  velY=  0.210 m/s  dist= 0.82 m  (FWD RIGHT)  angle= 32deg
[MOTOR] velX=  0.120 m/s  velY=  0.210 m/s  dist= 0.79 m  (FWD RIGHT)  angle= 60deg
[MOTOR] velX=  0.000 m/s  velY=  0.100 m/s  dist= 0.75 m  (FWD )       angle= 90deg
[MOTOR] velX=  0.000 m/s  velY=  0.000 m/s  dist= 0.00 m  (HOLD)       angle= N/A
```