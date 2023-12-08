#ifdef ARDUINO

#include "StepperDriver.h"

StepperDriver::StepperDriver(uint8_t pulse_pin, uint8_t dir_pin)
    : stepper_(AccelStepper::DRIVER, pulse_pin, dir_pin), speed_scale_(16.0f) {}

void StepperDriver::begin(float max_speed, float acceleration,
                          float speed_scale) {
  speed_scale_ = speed_scale;
  stepper_.setMaxSpeed(max_speed);
  stepper_.setAcceleration(acceleration);
}

void StepperDriver::applyControl(float output) {
  stepper_.setSpeed(speed_scale_ * output);
}

void StepperDriver::service() { stepper_.runSpeed(); }

long StepperDriver::currentPosition() { return stepper_.currentPosition(); }

#endif  // ARDUINO
