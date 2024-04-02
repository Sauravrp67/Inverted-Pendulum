// Shared by the board and simulator PID controllers.
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
  PIDController(float kp = 1.2f, float ki = 0.11f, float kd = 0.006f);

  void setGains(float kp, float ki, float kd);
  void setSetpoint(float setpoint);

  // Limit the integral to +/-limit.
  void setIntegralLimit(float limit);

  // A nonpositive limit disables output clipping.
  void setOutputLimit(float limit);

  void setDerivativeTime(float tau) { derivative_time_ = tau; }

  void setConditionalIntegration(bool enabled) { conditional_ = enabled; }

  float requested() const { return requested_; }

  float kp() const { return kp_; }

  float ki() const { return ki_; }

  float kd() const { return kd_; }

  float setpoint() const { return setpoint_; }

  float integral() const { return integral_; }

  float derivative() const { return derivative_; }

  float error() const { return error_; }

  float output() const { return output_; }

  // Match dt and filter time to the gains: ms in firmware, s in simulation.
  float update(float measurement, float dt);

  void reset();

  // Seed tilt history when switching from swing-up to balance.
  void prime(float error, float integral, float rate) {
    previous_error_ = error;
    integral_ = integral;
    derivative_ = rate;
  }

private:
  float clampIntegral();

  float kp_;
  float ki_;
  float kd_;
  float setpoint_;
  float integral_limit_;
  float output_limit_;
  float derivative_time_ = 0;
  bool conditional_ = false;
  float requested_ = 0;

  float integral_;
  float derivative_;
  float error_;
  float previous_error_;
  float output_;
};

#endif // PID_CONTROLLER_H
