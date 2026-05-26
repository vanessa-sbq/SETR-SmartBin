CXX := g++
CXXFLAGS := -std=c++17 -Wextra -I../include
LDFLAGS := ../build/libPiPCA9685.a -lgpiod -li2c

TARGET := motor
SRC := test.cpp MotorControllere.cpp
OBJ := $(SRC:.cpp=.o)

.PHONY: all clean motor

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) $(OBJ) -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)