#pragma once

#include <stdint.h>

class QuadratureDecoder {
public:
  QuadratureDecoder() : old_ab_(0), count_(0) {}

  // Each valid edge adds or subtracts one count.
  int8_t update(bool a, bool b) {
    static const int8_t kStates[16] = {0,  -1, 1, 0, 1, 0, 0,  -1,
                                       -1, 0,  0, 1, 0, 1, -1, 0};

    old_ab_ <<= 2;
    if (a) {
      old_ab_ |= 0x02;
    }
    if (b) {
      old_ab_ |= 0x01;
    }

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
