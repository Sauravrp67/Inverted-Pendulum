// QuadratureEncoder -- optical quadrature encoder driver.
//
// Two layers:
//   * QuadratureDecoder -- a pure, header-inline state machine (no <Arduino.h>)
//     that is unit-tested on the host (tests/test_encoder.cpp). It uses the
//     classic 16-entry transition lookup table and counts every valid A/B
//     transition, matching the +/-1-per-edge resolution the tuned PID5 baseline
//     was calibrated against (1440 counts per revolution).
//   * QuadratureEncoder -- the Arduino wrapper (compiled only when ARDUINO is
//     defined) that wires pins, attaches interrupts, and exposes angle.
#ifndef QUADRATURE_ENCODER_H
#define QUADRATURE_ENCODER_H

#include <stdint.h>

class QuadratureDecoder {
public:
  QuadratureDecoder() : old_ab_(0), count_(0) {}

  // Feed the current A/B channel levels. Returns the signed transition delta
  // (-1, 0, or +1) and folds it into the running count.
  int8_t update(bool a, bool b) {
    static const int8_t kStates[16] = {0, -1, 1, 0, 1, 0, 0, -1,
                                       -1, 0, 0, 1, 0, 1, -1, 0};
    old_ab_ <<= 2;
    if (a) old_ab_ |= 0x02;
    if (b) old_ab_ |= 0x01;
    int8_t delta = kStates[old_ab_ & 0x0f];
    count_ += delta;
    return delta;
  }

  long count() const { return count_; }
  void reset() { count_ = 0; }

private:
  uint8_t old_ab_;
  long count_;
};

#ifdef ARDUINO

// Arduino-facing driver. Because attachInterrupt() takes a free function, a
// single active instance is tracked so a static trampoline can dispatch to it.
class QuadratureEncoder {
public:
  QuadratureEncoder(uint8_t pin_a, uint8_t pin_b, uint8_t pin_z = 255,
                    float pulses_per_rev = 1440.0f);

  // Configure pins and attach CHANGE interrupts on A/B (and Z reset if wired).
  void begin();

  float angleDegrees() const;
  long count() const { return decoder_.count(); }
  void reset();

  // Called from the ISR trampolines; reads the pins and steps the decoder.
  void handleAB();
  void handleZ();

private:
  static QuadratureEncoder* instance_;
  static void isrAB();
  static void isrZ();

  QuadratureDecoder decoder_;
  uint8_t pin_a_;
  uint8_t pin_b_;
  uint8_t pin_z_;
  float pulses_per_rev_;
};

#endif  // ARDUINO

#endif  // QUADRATURE_ENCODER_H
