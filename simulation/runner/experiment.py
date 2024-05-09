"""Advance the plant, call the C++ controllers and record the sampled signals."""

import numpy as np
import furuta_control as control

from simulation.control_design import design, mpc_tables, observer_model
from simulation.dynamics.drake_plant import DrakePlant
from simulation.sensors.encoder import measure
from simulation.hardware_model import TrialDisturbances, Stepper


FIELDS = (
    "t arm angle arm_rate angle_rate arm_meas angle_meas arm_est angle_est "
    "arm_rate_est angle_rate_est requested_steps_s applied_Nm integral mode rate_limited feasible arm_feedback "
    "limited_steps_s motor_noise_Nm hinge_Nm arm_readout_error_rad rod_readout_error_rad "
    "step_count target_rad torque_limited pulse_trim_rad push_Nm mpc_sweeps"
).split()


def first_balance_loss(trace, threshold):
    """First post-capture sample outside the upright band, using recorded truth."""
    tilt = np.arctan2(np.sin(trace.angle), np.cos(trace.angle))
    lost = np.flatnonzero((trace.mode == 1) & (abs(tilt) > threshold))
    return int(lost[0]) if lost.size else None


def simulate(c, controller):
    if controller != c["controller"]:
        raise ValueError(controller)
    balance_estimator = c.get("balance_estimator", "kalman")
    if balance_estimator not in ("kalman", "difference"):
        raise ValueError(f"unknown balance estimator: {balance_estimator}")

    plant = DrakePlant(c["plant"])
    A, B, K, L = design(plant, c, with_lqr=controller == "lqr")
    dt, limit = c["sample_s"], c["limit_Nm"]
    motor = Stepper(c["stepper"], limit, c["initial"][0])
    rate_limit = c["stepper"]["max_steps_s"]
    F, G = observer_model(A, B, dt)

    if controller == "mpc":
        free, inverse = mpc_tables(A, B, c)
        mpc = control.LinearMpc(free, inverse, rate_limit * motor.step)
    solver_failed = False

    trial = TrialDisturbances(c["hardware_model"], c["trial_id"])
    x = np.array(c["initial"], dtype=float)
    updates = c["stepper"]["updates_per_sample"]
    h = dt / updates
    plant.start(x, h / c["substeps"])

    est = np.zeros(4)
    prev_y = None
    velocity = np.zeros(2)

    lead = control.RateLead()
    pulse_trim = control.PulseTrim()
    if controller == "pid":
        p = c["pid"]
        pid = control.PID(
            p["kp"], p["ki"], p["kd"], p["derivative_time_s"], rate_limit, True
        )
    pid_out = np.zeros(2)
    limited = 0.0
    field_prev = motor.target
    balancing = False

    reference = c["arm_target_rad"]
    s = c["swing"]
    coupling = (
        c["plant"]["rod_mass"] * c["plant"]["arm_length"] * c["plant"]["rod_length"] / 2
    )

    rows = []
    push = c["push"]
    start = round(push["start_s"] / dt)
    stop = start + round(push["duration_s"] / dt)
    last_tick = round(c["duration"] / dt)

    for tick in range(last_tick + 1):
        t = tick * dt
        encoder_error = trial.encoder_error()
        y = measure(x[:2], encoder_error, c["encoder_counts"])

        if prev_y is None:
            est[:2] = y
        else:
            delta = y - prev_y
            delta[1] = np.arctan2(np.sin(delta[1]), np.cos(delta[1]))
            velocity += dt / (dt + s["rate_filter_s"]) * (delta / dt - velocity)

            if balancing and balance_estimator == "kalman":
                relative = est.copy()
                relative[0] -= field_prev
                est[:] = control.kalman(
                    relative,
                    y - [motor.target, 0],
                    (motor.target - field_prev) / dt,
                    F,
                    G,
                    L,
                )
                est[0] += motor.target
            else:
                est[:] = [*y, *velocity]
        prev_y = y

        handoff = False
        if not balancing:
            est[:] = [*y, *velocity]
            handoff = (
                abs(y[1]) < s["capture_rad"]
                and abs(est[3]) < s["capture_rate"]
                and abs(est[2]) < s["capture_arm_rate"]
            )
            balancing = handoff

        arm_feedback = 0.0
        if balancing and reference is not None and controller in ("pid", "lead"):
            kp, kd = c["outer_gains"]
            arm_feedback = np.clip(
                control.arm_feedback(y[0] - reference, est[2], kp, kd),
                -c["tilt_ref_limit_rad"],
                c["tilt_ref_limit_rad"],
            )

        trim = 0.0
        if balancing and controller in ("pid", "lead") and "pulse_trim" in c:
            p = c["pulse_trim"]
            trim = pulse_trim.step(
                (motor.target - field_prev) / dt,
                dt,
                p["filter_s"],
                p["lean_s"],
                p["limit_rad"],
            )

        error = y[1] + arm_feedback + trim
        if handoff:
            # Integrate the local hinge equation J*q'' + b*a'' + d*q' = mgl*q.
            # Use the commanded speed here; shaft speed isn't measured.
            initial_i = (
                plant.j * est[3]
                + coupling * limited * motor.step
                + c["plant"]["hinge_damping"] * y[1]
            ) / plant.g
            if controller == "pid":
                pid.prime(error, initial_i, est[3])
            elif controller == "lead":
                lead.prime(c["lead"]["gain"] * initial_i)

        feasible = -1
        mpc_sweeps = 0
        if not balancing and not s["enabled"]:
            command = 0.0
        elif not balancing:
            command = (
                control.pump_rate(
                    y[0],
                    y[1],
                    est[3],
                    plant.j,
                    plant.g,
                    s["speed_rad_s"],
                    s["energy_reserve_J"],
                    s["energy_band_J"],
                    s["phase_lead_s"],
                    s["centring_per_s"],
                    s["kick_rad_s"] if t < s["kick_s"] else 0,
                )
                / motor.step
            )
        elif controller == "pid":
            pid_out[:] = pid.step(error, dt)
            command = pid_out[0]
        elif controller == "lead":
            p = c["lead"]
            command = lead.step(
                error, dt, p["gain"], p["zero_s"], p["pole_s"], rate_limit
            )
        elif controller == "lqr":
            command = (
                control.lqr(K, [*est, motor.target - est[0]], reference or 0.0)
                / motor.step
            )
        elif controller == "mpc":
            rate, feasible = 0.0, False
            if not solver_failed:
                rate, feasible = mpc.step(
                    [*est, motor.target - est[0]], reference or 0.0
                )
                mpc_sweeps = mpc.sweeps
                solver_failed = not feasible
            command = rate / motor.step
        else:
            raise ValueError(controller)

        limited = float(np.clip(command, -rate_limit, rate_limit))
        motor_noise = trial.torque_error()
        hinge = trial.hinge_torque(x[3])
        push_Nm = push["hinge_Nm"] if start <= tick < min(stop, last_tick) else 0.0

        field_prev = motor.target
        count = motor.count
        at_sample = x.copy()
        applied = 0.0
        torque_limited = False

        if tick < last_tick:
            for j in range(updates):
                torque = motor.advance(limited, h, x[0], x[2], motor_noise)
                torque_limited |= abs(torque) >= limit
                applied += torque / updates
                x = plant.advance(t + (j + 1) * h, torque, hinge + push_Nm)
        else:
            applied = motor.advance(0, 0, x[0], x[2], motor_noise)
            torque_limited = abs(applied) >= limit

        rows.append(
            [
                t,
                *at_sample,
                *y,
                *est,
                command,
                applied,
                pid_out[1],
                int(balancing),
                int(abs(limited) >= rate_limit - 1e-7),
                int(feasible),
                arm_feedback,
                limited,
                motor_noise,
                hinge,
                *encoder_error,
                count,
                field_prev,
                int(torque_limited),
                trim,
                push_Nm,
                mpc_sweeps,
            ]
        )

    return np.rec.fromrecords(rows, names=FIELDS)
