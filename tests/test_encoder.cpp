// Host unit tests for the platform-independent QuadratureDecoder.
//
// The decoder is fed synthetic A/B channel sequences. A clockwise quadrature
// cycle visits 00 -> 10 -> 11 -> 01 -> 00; counter-clockwise reverses it.
#include "QuadratureEncoder.h"
#include "test_framework.h"

namespace {
// One full quadrature cycle in each direction, as {A, B} pairs.
const bool kForward[4][2]  = {{1, 0}, {1, 1}, {0, 1}, {0, 0}};
const bool kBackward[4][2] = {{0, 1}, {1, 1}, {1, 0}, {0, 0}};
}  // namespace

TEST(encoder_forward_increments) {
  QuadratureDecoder dec;
  long sum = 0;
  for (int i = 0; i < 4; ++i) sum += dec.update(kForward[i][0], kForward[i][1]);
  // Four valid transitions, each +1 -> net +4 per cycle (matches PID5 edges).
  CHECK(dec.count() == 4);
  CHECK(sum == 4);
}

TEST(encoder_backward_decrements) {
  QuadratureDecoder dec;
  for (int i = 0; i < 4; ++i) dec.update(kBackward[i][0], kBackward[i][1]);
  CHECK(dec.count() == -4);
}

TEST(encoder_net_zero_round_trip) {
  QuadratureDecoder dec;
  for (int rep = 0; rep < 10; ++rep)
    for (int i = 0; i < 4; ++i) dec.update(kForward[i][0], kForward[i][1]);
  for (int rep = 0; rep < 10; ++rep)
    for (int i = 0; i < 4; ++i) dec.update(kBackward[i][0], kBackward[i][1]);
  CHECK(dec.count() == 0);
}

TEST(encoder_invalid_double_transition_ignored) {
  QuadratureDecoder dec;
  // Jumping 00 -> 11 is an illegal double transition; the lookup table maps it
  // to 0 so the count must not move.
  dec.update(0, 0);
  int8_t delta = dec.update(1, 1);
  CHECK(delta == 0);
  CHECK(dec.count() == 0);
}

TEST(encoder_reset_zeros_count) {
  QuadratureDecoder dec;
  for (int i = 0; i < 4; ++i) dec.update(kForward[i][0], kForward[i][1]);
  dec.reset();
  CHECK(dec.count() == 0);
}
