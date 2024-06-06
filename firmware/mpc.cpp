#include "balance.h"
#include "parameters.h"
#include "design_values.inc"
#include "controllers/mpc/mpc.h"

static LinearMpc mpc(mpc_free_plan, mpc_inverse, MPC_HORIZON,
                     params.stepper.max_steps_s *params.rad_per_step,
                     coefficient);

void begin_balance(const Sample &) {}

bool balance_rate(const Sample &s, double &rate) {
  float state[5];
  for (int k = 0; k < 5; ++k) {
    state[k] = s.state[k];
  }

  float field_rate;
  const bool solved = mpc.step(state, params.arm_target_rad, field_rate,
                               control_deadline_reached);
  rate = field_rate / params.rad_per_step;
  return solved;
}
