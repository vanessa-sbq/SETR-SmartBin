#include <Arduino.h>
#include "wifi.h"

// Define NES controller pins
const int nesDataPin = 4;
const int nesClockPin = 2;
const int nesLatchPin = 5;

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

  
  if (state > 0) {
    Serial.printf("%x\n", state);
    Serial.print("Pressed Buttons: ");
    if (state & (1 << 0)) Serial.print("A ");
    if (state & (1 << 1)) Serial.print("B ");
    if (state & (1 << 2)) Serial.print("Select ");
    if (state & (1 << 3)) Serial.print("Start ");
    if (state & (1 << 4)) Serial.print("Up ");
    if (state & (1 << 5)) Serial.print("Down ");
    if (state & (1 << 6)) Serial.print("Left ");
    if (state & (1 << 7)) Serial.print("Right ");
    Serial.println();
  }

  // Polling delay
  delay(16); // roughly 60Hz 
}