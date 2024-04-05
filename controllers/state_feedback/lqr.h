#pragma once

inline double feedback(const double *k, const double *x, double reference,
                       int size = 4) {
  double u = k[0] * reference;

  for (int i = 0; i < size; ++i) {
    u -= k[i] * x[i];
  }

  return u;
}
