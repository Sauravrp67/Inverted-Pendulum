// Host entry point for checking the same balance routines linked into firmware.
#include "../firmware/balance.h"

bool control_deadline_reached() { return false; }

extern "C" bool probe(double *x, double error, double initial, bool handoff,
                      double *rate) {
  Sample s{};
  for (int i = 0; i < 5; ++i) {
    s.state[i] = x[i];
  }
  s.error = error;
  s.initial_integral = initial;
  if (handoff) {
    begin_balance(s);
  }
  return balance_rate(s, *rate);
}
