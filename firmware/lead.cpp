#include "balance.h"
#include "parameters.h"
#include "controllers/classical/step_rate.h"

static RateLead lead;

void begin_balance(const Sample &s) {
  lead = RateLead{};
  lead.prime(params.lead.gain * s.initial_integral);
}

bool balance_rate(const Sample &s, double &rate) {
  rate =
      lead.step(s.error, params.sample_s, params.lead.gain, params.lead.zero_s,
                params.lead.pole_s, params.stepper.max_steps_s);

  return true;
}
