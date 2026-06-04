CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Iinclude \
            $(shell pkg-config --cflags opencv4) \
            $(shell pkg-config --cflags libcamera)
LDFLAGS  := $(shell pkg-config --libs opencv4) -llccv -lpthread \
            -L/usr/local/lib -lPiPCA9685 -lgpiod -li2c

TARGET := smartbin
SRCS   := main.cpp \
           camera/camera.cpp \
           camera/detector.cpp \
           camera/motor_translation.cpp \
           hardware_drive.cpp \
           motor/motor_controller.cpp
OBJS   := $(SRCS:.cpp=.o)

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $^ -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)
