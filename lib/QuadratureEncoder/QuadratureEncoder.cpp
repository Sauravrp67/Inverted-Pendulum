// Arduino glue for QuadratureEncoder. The pure decoder lives header-inline so
// this translation unit is empty on the host build.
#ifdef ARDUINO

#include "QuadratureEncoder.h"

#include <Arduino.h>

QuadratureEncoder* QuadratureEncoder::instance_ = nullptr;

QuadratureEncoder::QuadratureEncoder(uint8_t pin_a, uint8_t pin_b, uint8_t pin_z,
                                     float pulses_per_rev)
    : pin_a_(pin_a),
      pin_b_(pin_b),
      pin_z_(pin_z),
      pulses_per_rev_(pulses_per_rev) {}

void QuadratureEncoder::begin() {
  instance_ = this;

  pinMode(pin_a_, INPUT_PULLUP);
  pinMode(pin_b_, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin_a_), isrAB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_b_), isrAB, CHANGE);

  if (pin_z_ != 255) {
    pinMode(pin_z_, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pin_z_), isrZ, RISING);
  }
}

float QuadratureEncoder::angleDegrees() const {
  return (static_cast<float>(decoder_.count()) / pulses_per_rev_) * 360.0f;
}

void QuadratureEncoder::reset() { decoder_.reset(); }

void QuadratureEncoder::handleAB() {
  decoder_.update(digitalRead(pin_a_), digitalRead(pin_b_));
}

void QuadratureEncoder::handleZ() { decoder_.reset(); }

void QuadratureEncoder::isrAB() {
  if (instance_) instance_->handleAB();
}

void QuadratureEncoder::isrZ() {
  if (instance_) instance_->handleZ();
}

#endif  // ARDUINO
