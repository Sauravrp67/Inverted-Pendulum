// StepperDriver -- thin wrapper over AccelStepper for the cart actuator.
//
// Centralises the STEP/DIR setup and the control-output-to-speed mapping that
// was scattered across the PID sketches. PID5 drove the motor with
// stepper.setSpeed(16 * output); that 16x gain is the default speed_scale.
//
// Arduino-only (AccelStepper depends on the Arduino core). Guarded so the host
// unit-test build can ignore it.
#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#ifdef ARDUINO

#include <AccelStepper.h>
#include <stdint.h>

class StepperDriver {
public:
  StepperDriver(uint8_t pulse_pin, uint8_t dir_pin);

  void begin(float max_speed = 4000.0f, float acceleration = 8000.0f,
             float speed_scale = 16.0f);

  // Map a PID output to a constant motor speed (steps/s).
  void applyControl(float output);

  // Must be called every loop iteration to emit step pulses.
  void service();

  long currentPosition();

private:
  AccelStepper stepper_;
  float speed_scale_;
};

#endif  // ARDUINO

#endif  // STEPPER_DRIVER_H
