"""Encoder noise, motor noise and hinge drag. Values are not fitted to hardware."""

import numpy as np


def validate_hardware_model(p):
    if not isinstance(p["seed"], int) or p["seed"] < 0:
        raise ValueError("hardware_model.seed must be a nonnegative integer")
    for key in (
        "encoder_sd_rad",
        "motor_noise_sd_Nm",
        "hinge_dry_Nm",
        "drag_Nm_s2_rad2",
    ):
        if not np.isfinite(p[key]) or p[key] < 0:
            raise ValueError(f"hardware_model.{key} must be finite and nonnegative")
    if (
        not np.isfinite(p["friction_smoothing_rad_s"])
        or p["friction_smoothing_rad_s"] <= 0
    ):
        raise ValueError("friction_smoothing_rad_s must be finite and positive")


class TrialDisturbances:
    def __init__(self, p, trial_id):
        self.p = p
        streams = np.random.SeedSequence(p["seed"], spawn_key=(trial_id,)).spawn(2)
        self.encoder, self.motor = map(np.random.default_rng, streams)

    def encoder_error(self):
        return self.encoder.normal(scale=self.p["encoder_sd_rad"], size=2)

    def torque_error(self):
        return self.motor.normal(scale=self.p["motor_noise_sd_Nm"])

    def hinge_torque(self, rate):
        p = self.p
        dry_friction = p["hinge_dry_Nm"] * np.tanh(rate / p["friction_smoothing_rad_s"])
        drag = p["drag_Nm_s2_rad2"] * abs(rate) * rate
        return -dry_friction - drag


class Stepper:
    """Count step pulses and turn shaft lag into motor torque."""

    def __init__(self, p, limit, initial_arm):
        self.step = 2 * np.pi / p["steps_per_turn"]
        self.stiffness = p["stiffness_Nm_rad"]
        self.damping = p["damping_Nm_s_rad"]
        self.limit = limit
        self.origin = initial_arm
        self.phase = 0.0
        self.count = 0

    @property
    def target(self):
        return self.origin + self.count * self.step

    def advance(self, rate_steps_s, dt, angle, speed, noise):
        self.phase += rate_steps_s * dt
        pulses = int(self.phase)
        self.phase -= pulses
        self.count += pulses

        effort = self.stiffness * (self.target - angle) - self.damping * speed + noise
        return float(np.clip(effort, -self.limit, self.limit))
