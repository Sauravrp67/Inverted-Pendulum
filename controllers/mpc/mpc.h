#pragma once

class LinearMpc {
public:
  static constexpr int max_horizon = 100;
  static constexpr int max_sweeps = 24;
  static constexpr float tolerance = 1e-4f; // field rad/s

  using Reader = float (*)(const float *);
  using Deadline = bool (*)();

  LinearMpc(const float *free_plan, const float *inverse, int horizon,
            float rate_limit, Reader read = nullptr);

  bool step(const float estimate[5], float reference, float &rate,
            Deadline expired = nullptr);

  const float *plan() const { return plan_; }

  int sweeps() const { return sweeps_; }

private:
  float coefficient(const float *p) const;
  float inverse(int row, int col) const;

  const float *free_plan_, *inverse_;
  int n_, sweeps_ = 0;
  float limit_;
  Reader read_;
  float plan_[max_horizon]{}, dual_[max_horizon]{};
};
