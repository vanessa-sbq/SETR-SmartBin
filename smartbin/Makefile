## Build platform:
#   make            -> rpi    (default; libcamera + PCA9685/gpiod motor hardware)
#   make desktop    -> PC     (OpenCV VideoCapture webcam/DroidCam, motors stubbed)
#   make PLATFORM=desktop     (equivalent to `make desktop`)
PLATFORM ?= rpi

CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Iinclude $(shell pkg-config --cflags opencv4)

TARGET := smartbin

# Sources shared by both platforms
SRCS := main.cpp \
        camera/camera.cpp \
        camera/detector_hsv.cpp \
        camera/detector_bgsub.cpp \
        camera/detector_kalman.cpp \
        camera/detector_factory.cpp \
        camera/motor_translation.cpp \
        operation_interface/operation_interface.cpp

ifeq ($(PLATFORM),desktop)
    CXXFLAGS += -DSMARTBIN_DESKTOP
    LDFLAGS  := $(shell pkg-config --libs opencv4) -lpthread
    # Motor hardware (PCA9685/gpiod) is not present off-Pi -> use the stub
    SRCS     += hardware_drive_stub.cpp
else
    CXXFLAGS += $(shell pkg-config --cflags libcamera)
    LDFLAGS  := $(shell pkg-config --libs opencv4) -llccv -lpthread \
                -L/usr/local/lib -lPiPCA9685 -lgpiod -li2c
    SRCS     += hardware_drive.cpp \
                motor/motor_controller.cpp
endif

OBJS := $(SRCS:.cpp=.o)

.PHONY: all desktop rpi clean

all: $(TARGET)

# Convenience targets so `make desktop` / `make rpi` just work.
desktop:
	$(MAKE) PLATFORM=desktop

rpi:
	$(MAKE) PLATFORM=rpi

$(TARGET): $(OBJS)
	$(CXX) $^ -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f main.o camera/*.o operation_interface/*.o hardware_drive.o hardware_drive_stub.o motor/*.o $(TARGET)
