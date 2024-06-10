#include "balance.h"
#include "parameters.h"
#include "controllers/classical/step_rate.h"
#include "controllers/swing_up/energy.h"
#include "estimators/kalman.h"
#include "pins.h"
#include "pulses.h"
#include "quadrature_decoder.h"
#include <string.h>

using namespace pins;

namespace {

enum Mode { uncalibrated, idle, swinging, balancing };

Mode mode = uncalibrated;
QuadratureDecoder arm_encoder, pole_encoder;

volatile long arm_ticks = 0, pole_ticks = 0;
long arm_zero, pole_zero, field_zero;

double est[4], last_arm, last_tilt, arm_rate, pole_rate, last_field,
    last_command;
PulseTrim trim;

unsigned long start_us, sample_us;
const unsigned long period = params.sample_s * 1000000 + 0.5;

void read_arm() {
  arm_encoder.update(digitalRead(arm_a), digitalRead(arm_b));
  arm_ticks = arm_encoder.count();
}

void read_pole() {
  pole_encoder.update(digitalRead(pole_a), digitalRead(pole_b));
  pole_ticks = pole_encoder.count();
}

bool active() { return mode == swinging || mode == balancing; }

char serial_command() {
  static char line[12];
  static uint8_t length = 0;

  if (!Serial.available()) {
    return 0;
  }
  const char ch = Serial.read();

  if (ch == '\r') {
    return 0;
  }
  if (ch != '\n') {
    if (length < sizeof(line)) {
      line[length++] = ch;
    }

    return 0;
  }

  bool valid = length < sizeof(line);
  if (valid) {
    line[length] = 0;
  }
  length = 0;

  if (!valid) {
    return 0;
  }

  if (!strcmp(line, "stop")) {
    return 'x';
  }

  if (!strcmp(line, "zero")) {
    return 'z';
  }

  if (!strcmp(line, "balance")) {
    return 'b';
  }

  if (!strcmp(line, "swing")) {
    return 's';
  }

  if (!strcmp(line, "status")) {
    return '?';
  }

  return 0;
}

void stop(const char *reason) {
  pulses::set(0);
  mode = uncalibrated;
  last_command = 0;

  Serial.println(reason);
}

} // namespace

bool control_deadline_reached() { return micros() - sample_us >= period; }

void setup() {
  Serial.begin(115200);

  pinMode(arm_a, INPUT_PULLUP);
  pinMode(arm_b, INPUT_PULLUP);
  pinMode(pole_a, INPUT_PULLUP);
  pinMode(pole_b, INPUT_PULLUP);

  read_arm();
  read_pole();

  attachInterrupt(digitalPinToInterrupt(arm_a), read_arm, CHANGE);
  attachInterrupt(digitalPinToInterrupt(arm_b), read_arm, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pole_a), read_pole, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pole_b), read_pole, CHANGE);

  pulses::begin(static_cast<int>(params.direction.motor));

  Serial.println(params.image);
  Serial.println("Stopped. Commands: zero, status, balance, swing, stop");
}

void loop() {
  // Stop between samples. Check start requests against fresh encoder readings.
  static char request = 0;
  const char incoming = serial_command();

  if (incoming == 'x') {
    stop("Stopped. Re-zero before starting.");
    request = 0;
  } else if (incoming && !active()) {
    request = incoming;
  }

  unsigned long now = micros();
  unsigned long elapsed = now - sample_us;

  if (elapsed < period) {
    return;
  }
  if (active() && elapsed >= 2 * period) {
    stop("Missed control sample.");
  }

  sample_us = elapsed >= 2 * period ? now : sample_us + period;
  const char key = request;
  request = 0;

  noInterrupts();
  long a = arm_ticks, q = pole_ticks, f = pulses::count;
  interrupts();

  if (key == 'z' && !active()) {
    pulses::set(0);
    arm_zero = a;
    pole_zero = q;
    field_zero = f;
    last_arm = last_field = arm_rate = pole_rate = last_command = 0;
    last_tilt = PI;
    trim = PulseTrim{};
    mode = idle;

    Serial.println("Zeroed: arm home, pole down.");
    return;
  }
  if (mode == uncalibrated) {
    return;
  }

  double arm = params.direction.arm * (a - arm_zero) *
               (2 * PI / params.encoder_counts[0]);
  double tilt = wrap(PI + params.direction.pole * (q - pole_zero) *
                              (2 * PI / params.encoder_counts[1]));
  double field = (f - field_zero) * params.rad_per_step;
  double field_speed = (field - last_field) / params.sample_s;

  double alpha =
      params.sample_s / (params.sample_s + params.swing.rate_filter_s);
  arm_rate += alpha * ((arm - last_arm) / params.sample_s - arm_rate);
  pole_rate += alpha * (wrap(tilt - last_tilt) / params.sample_s - pole_rate);

  last_arm = arm;
  last_tilt = tilt;

  if (mode == balancing) {
    est[0] -= last_field;
    double measured[] = {arm - field, tilt};
    observe(est, measured, field_speed, observer.F, observer.G, observer.L);
    est[0] += field;
  } else {
    est[0] = arm;
    est[1] = tilt;
    est[2] = arm_rate;
    est[3] = pole_rate;
  }

  last_field = field;

  const bool capture = fabs(tilt) < params.swing.capture_rad &&
                       fabs(pole_rate) < params.swing.capture_rate &&
                       fabs(arm_rate) < params.swing.capture_arm_rate;

  if (mode == idle && key == '?') {
    Serial.print("arm / tilt (rad): ");
    Serial.print(arm, 3);
    Serial.print(' ');
    Serial.println(tilt, 3);
  }

  if (mode == idle && key == 's') {
    if (!params.swing.enabled) {
      Serial.println("Swing-up disabled in firmware settings.");
      return;
    }

    if (fabs(tilt) < PI - params.swing.capture_rad || fabs(pole_rate) > 0.2 ||
        fabs(arm_rate) > 0.2) {
      Serial.println("Swing requires hanging rest.");
      return;
    }

    start_us = now;
    mode = swinging;
    Serial.println("Swing");
  }

  bool handoff = capture && (mode == swinging || (mode == idle && key == 'b'));
  
  if (handoff) {
    mode = balancing;
    Serial.println("Balance");
  } else if (mode == idle && key == 'b') {
    Serial.println("Outside capture range.");
  }

  if (!active()) {
    return;
  }

  if (fabs(arm) >= params.travel_stop_rad ||
      (mode == balancing && fabs(tilt) > params.swing.exit_rad) ||
      (mode == swinging && now - start_us > params.swing.timeout_s * 1e6)) {
    stop("Travel, tilt or swing time limit.");
    return;
  }

  double rate;
  if (mode == swinging) {
    double kick = now - start_us < params.swing.kick_s * 1e6
                      ? params.swing.kick_rad_s
                      : 0;

    rate = pump_rate(arm, tilt, pole_rate, params.hinge_inertia,
                     params.gravity_moment, params.swing.speed_rad_s,
                     params.swing.energy_reserve_J, params.swing.energy_band_J,
                     params.swing.phase_lead_s, params.swing.centring_per_s,
                     kick) /
           params.rad_per_step;
  } else {
    double outer =
        limit_symmetric(params.outer_gains[0] * (arm - params.arm_target_rad) +
                            params.outer_gains[1] * est[2],
                        params.tilt_ref_limit_rad);

    double lean =
        trim.step(field_speed, params.sample_s, params.pulse_trim.filter_s,
                  params.pulse_trim.lean_s, params.pulse_trim.limit_rad);

    Sample s{{est[0], est[1], est[2], est[3], field - est[0]},
             tilt + outer + lean,
             0};

    if (handoff) {
      s.initial_integral =
          (params.hinge_inertia * est[3] +
           params.coupling * last_command * params.rad_per_step +
           params.plant.hinge_damping * tilt) /
          params.gravity_moment;
      begin_balance(s);
    }

    if (!balance_rate(s, rate)) {
      stop(control_deadline_reached() ? "Missed control deadline."
                                      : "Controller solve failed.");
      return;
    }
  }

  if (!isfinite(rate) || micros() - sample_us >= period) {
    stop("Invalid or late command.");
    return;
  }

  last_command = limit_symmetric(rate, params.stepper.max_steps_s);
  pulses::set(lround(last_command));
}
