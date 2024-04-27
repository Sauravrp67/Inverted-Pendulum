"""Read controller profiles and check their settings before a run or board build."""

import numpy as np
from ruamel.yaml import YAML, YAMLError

from simulation import ROOT
from simulation.hardware_model import validate_hardware_model


CONTROLLERS = ("pid", "lead", "lqr", "mpc")
OBJECTIVES = ("pendulum-arm-regulation", "pendulum-regulation")


def read_yaml(path):
    try:
        data = YAML(typ="safe", pure=True).load(path)
    except (OSError, YAMLError) as error:
        raise ValueError(f"{path}: {error}") from error
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a YAML mapping")
    return data


def overlay(base, overrides):
    if not isinstance(base, dict) or not isinstance(overrides, dict):
        raise ValueError("expected a configuration mapping")
    result = base.copy()
    for key, value in overrides.items():
        if isinstance(result.get(key), dict) and isinstance(value, dict):
            value = overlay(result[key], value)
        result[key] = value
    return result


def config(name, controller, domain="simulation", root=ROOT / "configs"):
    if domain not in ("simulation", "empirical") or controller not in CONTROLLERS:
        raise ValueError("unknown configuration domain or controller")
    path = root / domain / (controller + ".yaml")
    profile = read_yaml(path)
    if (
        profile.pop("import", None) != "../base.yaml"
        or profile.get("controller") != controller
    ):
        raise ValueError(f"{path}: invalid import or controller name")
    base = read_yaml(root / "base.yaml")
    c = overlay(overlay(base["common"], base[domain]), profile)
    scenarios = c.pop("scenarios")
    if name not in scenarios:
        raise ValueError(f"{path}: unknown objective {name}")
    c = overlay(c, scenarios[name])
    validate_config(c, domain)
    if domain == "simulation" and not c["swing"]["enabled"]:
        c["initial"] = c["balance_initial"]
    return c


def validate_config(c, domain="simulation"):
    sim = domain == "simulation"
    if sim:
        validate_hardware_model(c["hardware_model"])
        for key in ("initial", "balance_initial"):
            if np.shape(c[key]) != (4,) or not np.isfinite(c[key]).all():
                raise ValueError(f"{key} must contain four finite states")
    else:
        if (
            not 0 < c["travel_stop_rad"] < np.inf
            or not 0 < c["swing"]["timeout_s"] < 4294
        ):
            raise ValueError("invalid empirical travel stop or swing timeout")
        for joint in ("arm", "pole", "motor"):
            sign = c["direction"][joint]
            if type(sign) is not int or sign not in (-1, 1):
                raise ValueError(f"direction.{joint} must be -1 or 1")
        if not 0 <= c["hardware_model"]["encoder_sd_rad"] < np.inf:
            raise ValueError("invalid encoder standard deviation")
        if (
            c["arm_target_rad"] is not None
            and abs(c["arm_target_rad"]) >= c["travel_stop_rad"]
        ):
            raise ValueError("arm target must be inside the travel stop")
    if type(c["swing"]["enabled"]) is not bool:
        raise ValueError("swing.enabled must be true or false")
    for key in (
        "speed_rad_s",
        "energy_reserve_J",
        "phase_lead_s",
        "centring_per_s",
        "kick_rad_s",
        "kick_s",
        "capture_rate",
        "capture_arm_rate",
    ):
        if not np.isfinite(c["swing"].get(key, np.nan)):
            raise ValueError(f"swing.{key} must be finite")
    if "outer_gains" in c and np.shape(c["outer_gains"]) != (2,):
        raise ValueError("expected two outer_gains")
    if not 0 < c["swing"]["capture_rad"] < c["swing"]["exit_rad"] <= np.pi:
        raise ValueError("require 0 < capture_rad < exit_rad <= pi")
    if np.shape(c["process_variances"]) != (4,) or not np.all(
        np.asarray(c["process_variances"]) >= 0
    ):
        raise ValueError("expected four nonnegative process variances")
    if c["controller"] in ("lqr", "mpc"):
        if (
            np.shape(c["state_weights"]) != (5,)
            or not np.all(np.asarray(c["state_weights"]) >= 0)
            or not 0 < c["rate_weight"] < np.inf
        ):
            raise ValueError(
                "expected five nonnegative state weights and a positive rate weight"
            )
        if c["arm_target_rad"] is None and c["state_weights"][0] != 0:
            raise ValueError("pendulum-regulation must not penalise arm position")
    if c["controller"] == "mpc" and (
        type(c["horizon"]) is not int or not 1 <= c["horizon"] <= 100
    ):
        raise ValueError("MPC horizon must be an integer in 1..100")
    if c["controller"] == "pid" and not 0 <= c["pid"]["derivative_time_s"] < np.inf:
        raise ValueError("invalid PID derivative time")
    if c["controller"] == "lead" and not 0 < c["lead"]["pole_s"] < np.inf:
        raise ValueError("lead pole time must be positive")
    for key in (
        "steps_per_turn",
        "max_steps_s",
        "stiffness_Nm_rad",
        "damping_Nm_s_rad",
    ):
        value = c["stepper"][key]
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f"stepper.{key} must be finite and positive")
    for key in (
        ("updates_per_sample", "steps_per_turn") if sim else ("steps_per_turn",)
    ):
        if type(c["stepper"][key]) is not int or c["stepper"][key] < 1:
            raise ValueError(f"stepper.{key} must be a positive integer")

    for key in ("energy_band_J", "rate_filter_s"):
        if not np.isfinite(c["swing"][key]) or c["swing"][key] <= 0:
            raise ValueError(f"swing.{key} must be finite and positive")

    if "pulse_trim" in c:
        for key in ("filter_s", "lean_s", "limit_rad"):
            value = c["pulse_trim"][key]
            if not np.isfinite(value) or value < 0:
                raise ValueError(f"pulse_trim.{key} must be finite and nonnegative")

    if sim and (type(c["trial_id"]) is not int or c["trial_id"] < 0):
        raise ValueError("trial_id must be a nonnegative integer")

    for key in ("arm_length", "rod_length", "rod_mass", "arm_inertia", "gravity"):
        value = c["plant"][key]
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f"plant.{key} must be finite and positive")
    for key in ("arm_damping", "hinge_damping"):
        if not np.isfinite(c["plant"][key]) or c["plant"][key] < 0:
            raise ValueError(f"plant.{key} must be finite and nonnegative")

    for key in (("sample_s", "duration", "limit_Nm") if sim else ("sample_s",)):
        if not np.isfinite(c[key]) or c[key] <= 0:
            raise ValueError(f"{key} must be finite and positive")
    if len(c["encoder_counts"]) != 2 or any(
        not isinstance(n, int) or n <= 0 for n in c["encoder_counts"]
    ):
        raise ValueError("expected two positive integer encoder resolutions")
    if not sim:
        return
    if not isinstance(c["substeps"], int) or c["substeps"] < 1:
        raise ValueError("substeps must be a positive integer")

    ticks = c["duration"] / c["sample_s"]
    if abs(ticks - round(ticks)) > 1e-8:
        raise ValueError("duration must align to controller sample times")

    push = c["push"]
    for key in ("start_s", "duration_s"):
        value = push[key]
        if (
            not np.isfinite(value)
            or value < 0
            or not np.isclose(
                value / c["sample_s"], round(value / c["sample_s"]), atol=1e-8, rtol=0
            )
        ):
            raise ValueError(f"push.{key} must align to controller sample times")
    if (
        push["duration_s"] == 0
        or push["start_s"] + push["duration_s"] > c["duration"] + 1e-10
    ):
        raise ValueError("push must occupy a nonempty interval inside the recording")
    if not np.isfinite(push["hinge_Nm"]):
        raise ValueError("push.hinge_Nm must be finite")
