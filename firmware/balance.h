#pragma once

// The selected .cpp implements these two calls. arm and tilt are in radians.
struct Sample {
  double state[5]; // estimated arm, tilt, their rates, field minus arm
  double error; // measured tilt plus the classical loop's reference correction
  double initial_integral;
};

void begin_balance(const Sample &s);
bool control_deadline_reached();
bool balance_rate(const Sample &s, double &steps_per_second);
