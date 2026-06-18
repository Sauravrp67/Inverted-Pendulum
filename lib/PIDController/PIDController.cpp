#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      setpoint_(0.0f),
      integral_limit_(1000.0f),
      output_limit_(0.0f),
      integral_(0.0f),
      derivative_(0.0f),
      error_(0.0f),
      previous_error_(0.0f),
      output_(0.0f) {}

void PIDController::setGains(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PIDController::setSetpoint(float setpoint) { setpoint_ = setpoint; }

void PIDController::setIntegralLimit(float limit) { integral_limit_ = limit; }

void PIDController::setOutputLimit(float limit) { output_limit_ = limit; }

float PIDController::clampIntegral() {
  if (integral_ > integral_limit_) integral_ = integral_limit_;
  else if (integral_ < -integral_limit_) integral_ = -integral_limit_;
  return integral_;
}

float PIDController::update(float measurement, float dt) {
  if (dt <= 0.0f) return output_;

  error_ = setpoint_ - measurement;

  integral_ += error_ * dt;
  clampIntegral();

  derivative_ = (error_ - previous_error_) / dt;
  output_ = kp_ * error_ + ki_ * integral_ + kd_ * derivative_;

  if (output_limit_ > 0.0f) {
    if (output_ > output_limit_) output_ = output_limit_;
    else if (output_ < -output_limit_) output_ = -output_limit_;
  }

  previous_error_ = error_;
  return output_;
}

void PIDController::reset() {
  integral_ = 0.0f;
  derivative_ = 0.0f;
  error_ = 0.0f;
  previous_error_ = 0.0f;
  output_ = 0.0f;
}
