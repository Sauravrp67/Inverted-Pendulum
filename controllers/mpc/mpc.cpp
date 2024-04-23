#include "mpc.h"

#include <math.h>

LinearMpc::LinearMpc(const float *free_plan, const float *inverse, int horizon,
                     float rate_limit, Reader read)
    : free_plan_(free_plan), inverse_(inverse), n_(horizon), limit_(rate_limit),
      read_(read) {}

float LinearMpc::coefficient(const float *p) const {
  return read_ ? read_(p) : *p;
}

float LinearMpc::inverse(int row, int col) const {
  if (row < col) {
    int tmp = col;
    col = row;
    row = tmp;
  }

  return coefficient(inverse_ + row * (row + 1) / 2 + col);
}

bool LinearMpc::step(const float estimate[5], float reference, float &rate,
                     Deadline expired) {
  rate = 0;
  sweeps_ = 0;

  if (!free_plan_ || !inverse_ || n_ < 1 || n_ > max_horizon ||
      !isfinite(limit_) || limit_ <= 0 || !isfinite(reference)) {
    return false;
  }

  float error[5];
  for (int k = 0; k < 5; ++k) {
    error[k] = estimate[k] - (k == 0 ? reference : 0);
    if (!isfinite(error[k])) {
      return false;
    }
  }

  for (int i = 0; i < n_; ++i) {
    if (expired && expired()) {
      return false;
    }

    float u = 0;
    for (int k = 0; k < 5; ++k) {
      u += coefficient(free_plan_ + i * 5 + k) * error[k];
    }

    const float diag = inverse(i, i);
    if (!isfinite(u) || !isfinite(diag) || diag <= 0) {
      return false;
    }
    plan_[i] = u;
    dual_[i] = 0;
  }

  // Adjust one signed bound multiplier, then update the whole input plan.
  for (int sweep = 0; sweep < max_sweeps; ++sweep) {
    sweeps_ = sweep + 1;
    bool done = true;

    for (int i = 0; i < n_; ++i) {
      if (expired && expired()) {
        return false;
      }

      const float u = plan_[i];
      float residual = 0;
      if (dual_[i] > 0 || (dual_[i] == 0 && u > limit_)) {
        residual = u - limit_;
      } else if (dual_[i] < 0 || u < -limit_) {
        residual = u + limit_;
      }

      if (fabsf(residual) <= tolerance) {
        continue;
      }

      done = false;
      const float diag = inverse(i, i);
      const float free_u = u + diag * dual_[i];
      const float clipped = fmaxf(-limit_, fminf(limit_, free_u));
      const float next = (free_u - clipped) / diag;
      const float change = next - dual_[i];
      if (!isfinite(next)) {
        return false;
      }
      dual_[i] = next;

      for (int j = 0; j < n_; ++j) {
        plan_[j] -= inverse(i, j) * change;
        if (!isfinite(plan_[j])) {
          return false;
        }
      }
    }

    if (done) {
      rate = fmaxf(-limit_, fminf(limit_, plan_[0]));
      return true;
    }
  }

  return false;
}
