// PIDController -- platform-independent PID controller.
//
// Deliberately free of <Arduino.h> so it compiles and is unit-tested on the
// host (see tests/test_pid.cpp). The control law and default gains reproduce
// the tuned "PID5" baseline: Kp=1.2, Ki=0.11, Kd=0.006, integral clamp +/-1000.
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
  PIDController(float kp = 1.2f, float ki = 0.11f, float kd = 0.006f);

  void setGains(float kp, float ki, float kd);
  void setSetpoint(float setpoint);
  // Symmetric clamp on the accumulated integral term (anti-windup).
  void setIntegralLimit(float limit);
  // Symmetric clamp on the controller output. <= 0 disables the clamp.
  void setOutputLimit(float limit);

  float kp() const { return kp_; }
  float ki() const { return ki_; }
  float kd() const { return kd_; }
  float setpoint() const { return setpoint_; }
  float integral() const { return integral_; }
  float derivative() const { return derivative_; }
  float error() const { return error_; }
  float output() const { return output_; }

  // Run one step. dt is the elapsed time in milliseconds (matching the
  // original sketches, which compute (now - last) and feed ms here). Returns
  // the clamped controller output.
  float update(float measurement, float dt);

  // Zero the accumulated state (integral, derivative, last error, output).
  void reset();

private:
  float clampIntegral();

  float kp_;
  float ki_;
  float kd_;
  float setpoint_;
  float integral_limit_;
  float output_limit_;

  float integral_;
  float derivative_;
  float error_;
  float previous_error_;
  float output_;
};

#endif  // PID_CONTROLLER_H
