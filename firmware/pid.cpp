#include "balance.h"
#include "parameters.h"
#include "controllers/classical/PIDController.h"

static PIDController pid(params.pid.kp, params.pid.ki, params.pid.kd);

void begin_balance(const Sample &s) {
  pid.reset();

  pid.setDerivativeTime(params.pid.derivative_time_s);
  pid.setOutputLimit(params.stepper.max_steps_s);
  pid.setConditionalIntegration(true);

  pid.prime(s.error, s.initial_integral, s.state[3]);
}

bool balance_rate(const Sample &s, double &rate) {
  rate = pid.update(-s.error, params.sample_s);

  return true;
}
