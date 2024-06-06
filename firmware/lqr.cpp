#include "balance.h"
#include "parameters.h"
#include "controllers/state_feedback/lqr.h"

void begin_balance(const Sample &) {}

bool balance_rate(const Sample &s, double &rate) {
  rate = feedback(lqr_gain, s.state, params.arm_target_rad, 5) /
         params.rad_per_step;

  return true;
}
