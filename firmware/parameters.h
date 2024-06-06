#pragma once
#include "storage.h"

typedef struct {
  double arm_length, rod_length, rod_mass, arm_inertia, gravity;
  double arm_damping, hinge_damping;
} PlantParameters;

typedef struct {
  double steps_per_turn, max_steps_s, stiffness_Nm_rad, damping_Nm_s_rad;
} StepperParameters;

typedef struct {
  double encoder_sd_rad;
} HardwareModelParameters;

typedef struct {
  double arm, pole, motor;
} DirectionParameters;

typedef struct {
  double enabled, speed_rad_s, energy_reserve_J, energy_band_J;
  double phase_lead_s, centring_per_s, kick_rad_s, kick_s;
  double capture_rad, capture_rate, capture_arm_rate, exit_rad, rate_filter_s,
      timeout_s;
} SwingParameters;

typedef struct {
  double kp, ki, kd, derivative_time_s;
} PIDParameters;

typedef struct {
  double gain, zero_s, pole_s;
} LeadParameters;

typedef struct {
  double filter_s, lean_s, limit_rad;
} PulseTrimParameters;

typedef struct FirmwareParameters {
  char image[32];
  char controller[8];

  double sample_s, encoder_counts[2], arm_target_rad, travel_stop_rad;

  StepperParameters stepper;
  PlantParameters plant;
  double process_variances[4], state_weights[5], rate_weight, horizon;

  HardwareModelParameters hardware_model;
  DirectionParameters direction;
  SwingParameters swing;
  PIDParameters pid;
  LeadParameters lead;
  PulseTrimParameters pulse_trim;

  double outer_gains[2], tilt_ref_limit_rad;

  double rad_per_step, hinge_inertia, gravity_moment, coupling;
} FirmwareParameters;

typedef struct ObserverMatrices {
  double F[16], G[4], L[8];
} ObserverMatrices;

#ifdef __cplusplus
extern "C" {
#endif
extern const FirmwareParameters params;
extern const ObserverMatrices observer;
extern const double lqr_gain[5];
extern const float mpc_free_plan[] FIRMWARE_FLASH, mpc_inverse[] FIRMWARE_FLASH;
#ifdef __cplusplus
}
#endif
