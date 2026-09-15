#include <Arduino.h>
#include <map>
#include "wifi.h"
#include "buttonState.h"

// Define NES controller pins
const int nesDataPin = 4;
const int nesClockPin = 2;
const int nesLatchPin = 5;

static std::map<std::string, ButtonState> currently_pressed = {
    {"A", {}}, {"B", {}}, {"Select", {}}, {"Start", {}},
    {"Up", {}}, {"Down", {}}, {"Left", {}}, {"Right", {}}
};



void setup() {
  Serial.begin(9600);

  wifiSetup();

  // Set pin modes
  pinMode(nesDataPin, INPUT_PULLUP);
  pinMode(nesClockPin, OUTPUT);
  pinMode(nesLatchPin, OUTPUT);

  // Initialize clock and latch to low
  digitalWrite(nesClockPin, LOW);
  digitalWrite(nesLatchPin, LOW);

  Serial.println("NES Controller Ready");
}

byte readNesController() {
  byte controllerData = 0;

  // Latch the button states into the controller's shift register
  digitalWrite(nesLatchPin, HIGH);
  delayMicroseconds(50);
  digitalWrite(nesLatchPin, LOW);

  // Read the 8 buttons
  // Standard Order: A, B, Select, Start, Up, Down, Left, Right
  for (int i = 0; i < 8; i++) {
    // Read the data pin (active low, so we invert it: LOW means pressed)
    int bit = digitalRead(nesDataPin) == LOW ? 1 : 0;

    // Store the bit in our byte
    controllerData |= (bit << i);

    // Pulse the clock pin to shift the next bit to the data line
    digitalWrite(nesClockPin, HIGH);
    delayMicroseconds(6);
    digitalWrite(nesClockPin, LOW);
  }

  return controllerData;
}

void loop() {
  byte state = readNesController();

  
  if (state >= 0) {

    const char* buttons[] = {"A", "B", "Select", "Start", "Up", "Down", "Left", "Right"};

    for (int bitNumber = 0; bitNumber < 8; bitNumber++) {
      const char* button = buttons[bitNumber];

      currently_pressed[button].previous = currently_pressed[button].pressed;

      // Check if a button is being pressed.
      if (state & (1 << bitNumber)) {
        currently_pressed[button].pressed = true;
      } else {
        currently_pressed[button].pressed = false;
      }
    }

  }

  // Convert std::string to Arduino String by getting a c string
  wifiLoop(currently_pressed);

  // Polling delay
  delay(16); // roughly 60Hz 
}