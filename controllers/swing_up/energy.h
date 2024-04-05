#pragma once
#include <math.h>

// arm and tilt are in radians. Tilt is zero upright; rod_rate is in rad/s.
inline double
pump_rate(double arm, double tilt, double rod_rate,
          double j,    // Rod inertia at the hinge (m*L^2/3 for a uniform rod).
          double mgl,  // Energy to lift the rod from horizontal to upright (J).
          double gain, // Arm speed at full effort (rad/s).
          double reserve,    // Extra energy above upright (J).
          double band,       // Energy range for easing off near the target (J).
          double phase_lead, // How far ahead to estimate tilt (s).
          double centring,   // Arm speed correction per radian from home (1/s).
          double kick        // Startup override (rad/s); zero uses feedback.
) {
  const double predicted_tilt = tilt + phase_lead * rod_rate;
  const double energy_gap =
      mgl * (1 - cos(tilt)) - 0.5 * j * rod_rate * rod_rate;

  const double phase = sin(predicted_tilt);
  const double effort = tanh((energy_gap + reserve) / band);

  double motion;
  if (kick != 0) {
    motion = kick;
  } else {
    motion = -gain * phase * effort;
  }

  return motion - centring * arm;
}
