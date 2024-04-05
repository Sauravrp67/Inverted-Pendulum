#pragma once
#include <math.h>

inline double wrap(double a) { return atan2(sin(a), cos(a)); }

// A, B and L are row-major.
inline void observe(double *x, const double *y, double applied, const double *A,
                    const double *B, const double *L) {
  double predicted[4];
  for (int row = 0; row < 4; ++row) {
    predicted[row] = B[row] * applied;
    for (int col = 0; col < 4; ++col) {
      predicted[row] += A[4 * row + col] * x[col];
    }
  }

  // the arm encoder counts whole turns. Only wrap the pole error
  const double arm_error = y[0] - predicted[0];
  const double tilt_error = wrap(y[1] - predicted[1]);
  for (int row = 0; row < 4; ++row) {
    x[row] =
        predicted[row] + L[2 * row] * arm_error + L[2 * row + 1] * tilt_error;
  }

  x[1] = wrap(x[1]);
}
