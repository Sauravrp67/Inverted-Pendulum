// Example: drive the stepper at a constant speed to validate STEP/DIR wiring.
// Refactor of archive/Stpper_Motor/Acc_Test using the StepperDriver library.
#include <Arduino.h>

#include "BoardConfig.h"
#include "StepperDriver.h"

StepperDriver stepper(STEPPER_PULSE_PIN, STEPPER_DIR_PIN);

void setup() {
  Serial.begin(SERIAL_BAUD);
  stepper.begin(/*max_speed=*/2000.0f, /*accel=*/8000.0f, /*speed_scale=*/1.0f);
  Serial.println("Stepper test: reversing every 2 s.");
}

void loop() {
  // Run forward for 2 s, then reverse, using the constant-speed control path.
  static unsigned long last_flip = 0;
  static float speed = 1500.0f;
  if (millis() - last_flip >= 2000) {
    last_flip = millis();
    speed = -speed;
  }
  stepper.applyControl(speed);
  stepper.service();
}
