#pragma once
#include "controllers/classical/lead.h"

inline double limit_symmetric(double value, double limit) {
  return value < -limit ? -limit : (value > limit ? limit : value);
}

// Lead shapes field acceleration
class RateLead {
  Lead lead_;
  double rate_ = 0;

public:
  void prime(double rate) { rate_ = rate; }

  double step(double error, double dt, double gain, double zero, double pole,
              double limit) {
    const double acceleration = lead_.update(error, dt, gain, zero, pole);
    rate_ = limit_symmetric(rate_ + dt * acceleration, limit);
    return rate_;
  }
};

// trim tilt using the commanded speed
class PulseTrim {
  double mean_speed_ = 0;

public:
  double step(double field_speed, double dt, double filter_time,
              double lean_time, double max_lean) {
    mean_speed_ += dt / (dt + filter_time) * (field_speed - mean_speed_);
    return limit_symmetric(lean_time * mean_speed_, max_lean);
  }
};
