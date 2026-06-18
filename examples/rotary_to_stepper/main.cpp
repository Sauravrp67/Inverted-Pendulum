// Example: slave the stepper speed to the encoder velocity.
// Refactor of archive/Rotary_Stepper_Test using both libraries. Turning the
// encoder by hand commands a proportional motor speed -- a quick end-to-end
// sanity check of the sensing + actuation chain before closing the PID loop.
#include <Arduino.h>

#include "BoardConfig.h"
#include "QuadratureEncoder.h"
#include "StepperDriver.h"

QuadratureEncoder encoder(ENC_A_PIN, ENC_B_PIN, ENC_Z_PIN, 1440.0f);
StepperDriver stepper(STEPPER_PULSE_PIN, STEPPER_DIR_PIN);

void setup() {
  Serial.begin(SERIAL_BAUD);
  encoder.begin();
  stepper.begin(/*max_speed=*/1000.0f, /*accel=*/100.0f, /*speed_scale=*/100.0f);
}

void loop() {
  // Command motor speed proportional to encoder displacement from zero.
  stepper.applyControl(encoder.angleDegrees());
  stepper.service();
}
