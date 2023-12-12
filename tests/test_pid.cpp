// Host unit tests for the platform-independent PIDController.
#include "PIDController.h"
#include "test_framework.h"

TEST(pid_proportional_sign) {
  // Pure P control: output should oppose the error toward the setpoint.
  PIDController pid(2.0f, 0.0f, 0.0f);
  pid.setSetpoint(0.0f);
  float out = pid.update(/*measurement=*/5.0f, /*dt=*/1.0f);
  // error = setpoint - measurement = -5; output = Kp*error = -10.
  CHECK_NEAR(pid.error(), -5.0f, 1e-6);
  CHECK_NEAR(out, -10.0f, 1e-6);
}

TEST(pid_integral_accumulates) {
  PIDController pid(0.0f, 1.0f, 0.0f);
  pid.setSetpoint(0.0f);
  pid.update(-2.0f, 1.0f);  // error=+2, integral=2
  float out = pid.update(-2.0f, 1.0f);  // error=+2, integral=4
  CHECK_NEAR(pid.integral(), 4.0f, 1e-6);
  CHECK_NEAR(out, 4.0f, 1e-6);  // Ki*integral
}

TEST(pid_integral_windup_clamp) {
  PIDController pid(0.0f, 1.0f, 0.0f);
  pid.setIntegralLimit(10.0f);
  pid.setSetpoint(0.0f);
  // Drive a large constant error; integral must saturate at the limit.
  for (int i = 0; i < 1000; ++i) pid.update(-100.0f, 1.0f);
  CHECK_NEAR(pid.integral(), 10.0f, 1e-6);
}

TEST(pid_output_clamp) {
  PIDController pid(1000.0f, 0.0f, 0.0f);
  pid.setOutputLimit(50.0f);
  pid.setSetpoint(0.0f);
  CHECK_NEAR(pid.update(1.0f, 1.0f), -50.0f, 1e-6);  // clamped negative
  CHECK_NEAR(pid.update(-1.0f, 1.0f), 50.0f, 1e-6);  // clamped positive
}

TEST(pid_derivative_responds_to_change) {
  PIDController pid(0.0f, 0.0f, 1.0f);
  pid.setSetpoint(0.0f);
  pid.update(0.0f, 1.0f);          // error=0, prev=0
  float out = pid.update(-3.0f, 1.0f);  // error=+3, derivative=(3-0)/1=3
  CHECK_NEAR(pid.derivative(), 3.0f, 1e-6);
  CHECK_NEAR(out, 3.0f, 1e-6);
}

TEST(pid_reset_clears_state) {
  PIDController pid(1.0f, 1.0f, 1.0f);
  pid.setSetpoint(0.0f);
  pid.update(-5.0f, 1.0f);
  pid.reset();
  CHECK_NEAR(pid.integral(), 0.0f, 1e-6);
  CHECK_NEAR(pid.error(), 0.0f, 1e-6);
  CHECK_NEAR(pid.output(), 0.0f, 1e-6);
}

TEST(pid_zero_dt_is_noop) {
  PIDController pid(1.0f, 1.0f, 1.0f);
  pid.setSetpoint(0.0f);
  float out = pid.update(-5.0f, 0.0f);  // dt<=0 returns prior output (0)
  CHECK_NEAR(out, 0.0f, 1e-6);
  CHECK_NEAR(pid.integral(), 0.0f, 1e-6);
}

TEST(pid_defaults_match_tuned_pid5) {
  PIDController pid;  // default ctor
  CHECK_NEAR(pid.kp(), 1.2f, 1e-6);
  CHECK_NEAR(pid.ki(), 0.11f, 1e-6);
  CHECK_NEAR(pid.kd(), 0.006f, 1e-6);
}
