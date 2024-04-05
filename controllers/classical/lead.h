#pragma once

// backward Euler form of k (1 + Tz s) / (1 + Tp s)
struct Lead {
  double previous = 0, output = 0;

  double update(double angle, double dt, double k, double tz, double tp) {
    output =
        (tp * output + k * ((dt + tz) * angle - tz * previous)) / (dt + tp);
    previous = angle;
    return output;
  }
};
