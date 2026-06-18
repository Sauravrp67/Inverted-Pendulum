// Inverted pendulum -- canonical balancing firmware.
//
// Wires the shared libraries into the control loop that was tuned as "PID5":
//   encoder angle -> PIDController -> stepper speed.
// Gains, timing, the 1 Hz telemetry line, and the serial "reset" command are
// preserved from that baseline so the existing tools/visualization.py parser
// keeps working unchanged. Board pins come from BoardConfig.h via the -DBOARD_*
// macro set by the Makefile.
#include <Arduino.h>

#include "BoardConfig.h"
#include "PIDController.h"
#include "QuadratureEncoder.h"
#include "StepperDriver.h"

// Tuned PID5 baseline.
static const float kKp = 1.2f;
static const float kKi = 0.11f;
static const float kKd = 0.006f;
static const float kMaxIntegral = 1000.0f;
static const float kSetpoint = 0.0f;  // upright

// Control loop cadence in milliseconds, matching the original sketch's
// (micros()-delta / 1000.0) >= 0.01 gate.
static const float kLoopPeriodMs = 0.01f;
static const float kPulsesPerRev = 1440.0f;
static const float kSpeedScale = 16.0f;

PIDController pid(kKp, kKi, kKd);
QuadratureEncoder encoder(ENC_A_PIN, ENC_B_PIN, ENC_Z_PIN, kPulsesPerRev);
StepperDriver stepper(STEPPER_PULSE_PIN, STEPPER_DIR_PIN);

static unsigned long last_time = 0;
static unsigned long last_print_time = 0;

void resetValues() {
  noInterrupts();
  encoder.reset();
  pid.reset();
  interrupts();
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  pid.setSetpoint(kSetpoint);
  pid.setIntegralLimit(kMaxIntegral);

  encoder.begin();
  stepper.begin(4000.0f, 8000.0f, kSpeedScale);

  Serial.println("Encoder and PID controller initialized.");

  last_time = micros();
  last_print_time = micros();
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command == "reset") {
      resetValues();
      Serial.println("Values have been reset.");
    }
  }

  unsigned long now = micros();
  float dt = (now - last_time) / 1000.0f;  // milliseconds

  if (dt >= kLoopPeriodMs) {
    last_time = now;
    float angle = encoder.angleDegrees();
    float output = pid.update(angle, dt);
    stepper.applyControl(output);
    stepper.service();
  }

  // Telemetry once per second. The token layout is a contract with
  // tools/visualization.py (it reads parts[1,3,5,7,9,11,13,15]).
  if (now - last_print_time >= 1000000UL) {
    last_print_time = now;
    Serial.print("Angle: ");
    Serial.print(encoder.angleDegrees());
    Serial.print(" Error: ");
    Serial.print(pid.error());
    Serial.print(" Integral: ");
    Serial.print(pid.integral());
    Serial.print(" Derivative: ");
    Serial.print(pid.derivative());
    Serial.print(" Output: ");
    Serial.print(pid.output());
    Serial.print(" Kp: ");
    Serial.print(pid.kp(), 3);
    Serial.print(" Ki: ");
    Serial.print(pid.ki(), 3);
    Serial.print(" Kd: ");
    Serial.println(pid.kd(), 4);
  }
}
