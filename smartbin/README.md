# SmartBin

## How to Run

### Desktop (test the camera without the Pi)

Uses OpenCV `VideoCapture`, so it works with a USB webcam or a DroidCam stream. Motor hardware is stubbed out.

```bash
sudo apt install libopencv-dev # one-time: OpenCV dev package
make desktop
```

### Raspberry Pi (full robot)

Requires OpenCV, [LCCV](https://github.com/kbarni/LCCV) (libcamera),
`PiPCA9685`, `libgpiod`, and `libi2c`.

```bash
make # default target = rpi
```

Switch platforms with `make clean` first, or pass `PLATFORM` explicitly:
`make PLATFORM=desktop` / `make PLATFORM=rpi`.

```bash
make clean
```

### Run

```bash
./smartbin # uses Config::DEVICE_INDEX (default 0)
```

Or if the device is on a different index:
```bash
./smartbin <device_index> # open device on index <device_index>
```