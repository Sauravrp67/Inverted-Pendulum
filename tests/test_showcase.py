import unittest
from itertools import combinations

import numpy as np
import furuta_control as native

from simulation.dynamics.drake_plant import DrakePlant
from simulation.run_demo import metrics
from simulation.dynamics.furuta import Furuta, rk4
from simulation.configuration import CONTROLLERS, config, validate_config
from simulation.control_design import design, observer_model
from simulation.runner.experiment import simulate
from simulation.sensors.encoder import measure
from simulation.hardware_model import TrialDisturbances, Stepper


class Showcase(unittest.TestCase):
    def test_balance_estimator_routing(self):
        for controller in ("pid", "lqr"):
            c = dict(
                config("pendulum-regulation", controller),
                duration=0.5,
                initial=[0, 0.03, 0, 0],
            )
            kalman = simulate(c, controller)
            difference = simulate(dict(c, balance_estimator="difference"), controller)

            np.testing.assert_array_equal(
                kalman.motor_noise_Nm, difference.motor_noise_Nm
            )
            self.assertTrue(np.all(difference.mode))
            np.testing.assert_array_equal(difference.angle_est, difference.angle_meas)
            self.assertFalse(
                np.array_equal(kalman.angle_rate_est, difference.angle_rate_est)
            )
            self.assertEqual(
                np.array_equal(kalman.requested_steps_s, difference.requested_steps_s),
                controller == "pid",
            )

    def test_push_timing_and_response(self):
        c = dict(config("pendulum-regulation", "pid"), duration=12.12)
        pushed = simulate(c, "pid")
        baseline = simulate(dict(c, push=dict(c["push"], hinge_Nm=0)), "pid")

        np.testing.assert_array_equal(pushed[:1200], baseline[:1200])
        np.testing.assert_array_equal(
            np.flatnonzero(pushed.push_Nm), np.arange(1200, 1210)
        )
        self.assertAlmostEqual(pushed.push_Nm.sum() * c["sample_s"], 0.006)
        self.assertEqual(pushed.angle_rate[1200], baseline.angle_rate[1200])
        self.assertEqual(
            pushed.requested_steps_s[1200], baseline.requested_steps_s[1200]
        )
        self.assertGreater(pushed.angle_rate[1201], baseline.angle_rate[1201])

    def test_pendulum_regulation_pid_holds_without_arm_feedback(self):
        c = config("pendulum-regulation", "pid")
        c["push"]["hinge_Nm"] = 0  # Test the hold without a push.
        for n in (c["substeps"], 2 * c["substeps"]):
            a = simulate(dict(c, substeps=n, duration=60), "pid")
            capture = np.flatnonzero(a.mode)
            self.assertTrue(capture.size)
            self.assertFalse(a.arm_feedback.any())
            tilt = np.abs(np.arctan2(np.sin(a.angle), np.cos(a.angle)))
            self.assertLess(max(tilt[capture]), c["swing"]["exit_rad"])
            self.assertLess(max(tilt[a.t >= a.t[capture[0]] + 1]), np.radians(1))
            self.assertLess(np.ptp(a.arm[a.t >= 55]), np.radians(10))

    def test_independent_trials_from_identical_rest(self):
        prefixes = []
        for name in ("pendulum-regulation", "pendulum-arm-regulation"):
            for run in CONTROLLERS:
                c = dict(config(name, run), duration=1.0)
                a = simulate(c, run)
                np.testing.assert_array_equal(
                    [a.arm[0], a.angle[0], a.arm_rate[0], a.angle_rate[0]],
                    [0.0, np.pi, 0.0, 0.0],
                )
                self.assertFalse(a.mode.any())
                prefixes.append(a)
        for a, b in combinations(prefixes, 2):
            for field in (
                "arm_readout_error_rad",
                "rod_readout_error_rad",
                "motor_noise_Nm",
            ):
                self.assertFalse(np.array_equal(a[field], b[field]))
            self.assertFalse(np.allclose(a.angle, b.angle, rtol=0, atol=1e-7))

    def test_noise_replay(self):
        c = dict(config("pendulum-arm-regulation", "pid"), duration=12.0)
        a = simulate(c, "pid")
        np.testing.assert_array_equal(a, simulate(c, "pid"))
        # A shared trial ID replays the same physical inputs on another controller.
        other = dict(
            config("pendulum-arm-regulation", "lqr"),
            trial_id=c["trial_id"],
            duration=c["duration"],
        )
        b = simulate(other, "lqr")
        k = np.flatnonzero(a.mode)[0]
        np.testing.assert_array_equal(a[:k], b[:k])
        for key in ("arm_readout_error_rad", "rod_readout_error_rad", "motor_noise_Nm"):
            np.testing.assert_array_equal(a[key], b[key])  # even after handoff
            self.assertGreater(np.std(a[key]), 0)
        for row in (a[0], a[k], a[-1]):
            expected = measure(
                [row.arm, row.angle],
                [row.arm_readout_error_rad, row.rod_readout_error_rad],
                c["encoder_counts"],
            )
            np.testing.assert_allclose([row.arm_meas, row.angle_meas], expected)
        other = simulate(
            dict(c, hardware_model=dict(c["hardware_model"], seed=43)), "pid"
        )
        self.assertEqual(a.angle[0], other.angle[0])
        self.assertEqual(a.angle_rate[0], other.angle_rate[0])
        self.assertFalse(np.array_equal(a.motor_noise_Nm, other.motor_noise_Nm))

    def test_step_direction_holding_and_hinge_loss(self):
        c = config("pendulum-regulation", "pid")
        trial = TrialDisturbances(c["hardware_model"], 0)
        limit = c["limit_Nm"]
        motor = Stepper(c["stepper"], limit, 0)
        self.assertEqual(motor.advance(50, 0.01, 0, 0, 0), 0)
        self.assertGreater(motor.advance(50, 0.01, 0, 0, 0), 0)
        self.assertEqual(motor.count, 1)
        self.assertAlmostEqual(motor.target, 2 * np.pi / c["stepper"]["steps_per_turn"])
        self.assertEqual(motor.advance(0, 0.01, 1, 0, 0), -limit)
        self.assertEqual(motor.count, 1)
        self.assertLess(motor.advance(-200, 0.01, 0, 0, 0), 0)
        self.assertEqual(motor.count, -1)
        for rate in (-4, -1, 0, 1, 4):
            self.assertLessEqual(trial.hinge_torque(rate) * rate, 0)

    def test_nominal_objectives_and_common_capture(self):
        starts = []
        for name in ("pendulum-regulation", "pendulum-arm-regulation"):
            c = config(name, "lqr")
            c["hardware_model"].update(
                encoder_sd_rad=0,
                motor_noise_sd_Nm=0,
                hinge_dry_Nm=0,
                drag_Nm_s2_rad2=0,
            )
            _, _, K, _ = design(Furuta(c["plant"]), c)
            if c["arm_target_rad"] is None:
                self.assertEqual(K[0, 0], 0)
                self.assertEqual(np.array(c["state_weights"])[[0, 2]].tolist(), [0, 0])
            else:
                self.assertNotEqual(K[0, 0], 0)
            for run in CONTROLLERS:
                c = config(name, run)
                c["hardware_model"].update(
                    encoder_sd_rad=0,
                    motor_noise_sd_Nm=0,
                    hinge_dry_Nm=0,
                    drag_Nm_s2_rad2=0,
                )
                traces = [simulate(dict(c, substeps=n), run) for n in (4, 8)]
                for a in traces:
                    capture = np.flatnonzero(a.mode)[0]
                    self.assertTrue(np.all(a.mode[capture:]))
                    tilt = abs(np.arctan2(np.sin(a.angle), np.cos(a.angle)))
                    self.assertLess(max(tilt[capture:]), c["swing"]["exit_rad"])
                    self.assertLess(max(tilt[a.t >= c["duration"] - 1]), np.radians(1))
                    self.assertLessEqual(max(abs(a.applied_Nm)), c["limit_Nm"] + 1e-14)
                    self.assertLessEqual(
                        max(abs(a.limited_steps_s)), c["stepper"]["max_steps_s"]
                    )
                    np.testing.assert_allclose(
                        a.target_rad,
                        a.step_count * 2 * np.pi / c["stepper"]["steps_per_turn"],
                    )
                    self.assertLess(abs(metrics(a)["final_angle_rad"]), 0.03)
                    if c["arm_target_rad"] is not None:
                        self.assertLess(abs(a.arm[-1] - c["arm_target_rad"]), 0.05)
                    if run in ("pid", "lead"):
                        self.assertEqual(
                            np.any(a.arm_feedback), name == "pendulum-arm-regulation"
                        )
                    if run == "mpc":
                        self.assertTrue(np.all(a.feasible[capture:] == 1))
                a = traces[0]
                k = np.flatnonzero(a.mode)[0]
                starts.append(a[:k].copy())
        for start in starts[1:]:
            np.testing.assert_array_equal(start, starts[0])

    def test_swing_disabled(self):
        from pathlib import Path
        from shutil import copytree
        from tempfile import TemporaryDirectory
        from simulation import ROOT

        with TemporaryDirectory() as tmp:
            profile = Path(tmp) / "simulation/pid.yaml"
            copytree(ROOT / "configs", Path(tmp), dirs_exist_ok=True)
            profile.write_text(profile.read_text() + "\nswing: {enabled: false}\n")
            c = config("pendulum-regulation", "pid", root=Path(tmp))
            self.assertEqual(c["initial"], c["balance_initial"])
            self.assertTrue(simulate(dict(c, duration=0.1), "pid").mode.all())
            down = simulate(dict(c, initial=[0, np.pi, 0, 0], duration=0.1), "pid")
            self.assertFalse(down.mode.any())
            self.assertFalse(down.requested_steps_s.any())

    def test_invalid_inputs(self):
        c = config("pendulum-arm-regulation", "pid")
        for key, bad in (
            ("seed", -1),
            ("motor_noise_sd_Nm", -0.1),
            ("hinge_dry_Nm", np.nan),
        ):
            with self.assertRaisesRegex(ValueError, key):
                validate_config(
                    dict(c, hardware_model=dict(c["hardware_model"], **{key: bad}))
                )
        with self.assertRaisesRegex(ValueError, "align"):
            validate_config(dict(c, duration=1.005))
        with self.assertRaisesRegex(ValueError, "steps_per_turn"):
            validate_config(dict(c, stepper=dict(c["stepper"], steps_per_turn=0)))
        with self.assertRaisesRegex(ValueError, "matrix dimensions"):
            native.lqr(np.ones((5, 1)), np.zeros(5), 0.0)

    def test_force_ports_against_reference(self):
        params = config("pendulum-arm-regulation", "pid")["plant"]
        reference, drake = Furuta(params), DrakePlant(params)
        x = np.array([0.25, -0.6, 0.4, -0.2])
        drake.start(x, 0.001)
        for tick, (u, pulse) in enumerate(((0.16, 0.0), (0.0, -0.04), (-0.1, 0.02))):
            for _ in range(20):
                x = rk4(lambda state: reference.rhs(state, u, pulse), x, 0.001)
            np.testing.assert_allclose(
                drake.advance((tick + 1) * 0.02, u, pulse), x, atol=1e-7
            )

    def test_mpc_against_independent_qp(self):
        # Independently eliminate the states and optimise just the N inputs.
        from scipy.optimize import minimize
        from simulation.control_design import mpc_tables

        c = config("pendulum-arm-regulation", "mpc")
        A, B, _, _ = design(Furuta(c["plant"]), c)
        q, r = np.array(c["state_weights"]), c["rate_weight"]
        for n, rate_limit in ((6, 0.3), (12, 1), (100, 3)):
            c["horizon"] = n
            mpc = native.LinearMpc(*mpc_tables(A, B, c), rate_limit)
            G = np.zeros((5 * n, n))
            F = np.vstack([np.linalg.matrix_power(A, i + 1) for i in range(n)])
            for i in range(n):
                for j in range(i + 1):
                    G[5 * i : 5 * i + 5, j] = (
                        np.linalg.matrix_power(A, i - j) @ B
                    ).ravel()
            w = np.tile(q, n)
            H = G.T @ (w[:, None] * G) + r * np.eye(n)
            for x in (
                [0.9, 0, 0, 0, 0],
                [0, 0.055, 0, 0, 0],
                [0.648, 0, 0.04, 0, -0.001],
                [0, -0.055, 0, 0, 0],
            ):
                error = F @ x - np.tile([0.9, 0, 0, 0, 0], n)
                g = G.T @ (w * error)
                opt = minimize(
                    lambda u: 0.5 * u @ H @ u + g @ u,
                    np.zeros(n),
                    jac=lambda u: H @ u + g,
                    method="SLSQP",
                    bounds=[(-rate_limit, rate_limit)] * n,
                    options={"ftol": 1e-12, "maxiter": 200},
                )
                self.assertTrue(opt.success, opt.message)
                u, solved = mpc.step(x, 0.9)
                self.assertTrue(solved)
                np.testing.assert_allclose(mpc.plan, opt.x, atol=1e-4, rtol=0)
                self.assertAlmostEqual(u, opt.x[0], delta=1e-4)
                self.assertLessEqual(abs(u), rate_limit + 1e-7)

        # This tighter problem is feasible, but needs more than 24 sweeps.
        c["horizon"] = 12
        mpc = native.LinearMpc(*mpc_tables(A, B, c), 0.3)
        u, solved = mpc.step([0, 0.055, 0, 0, 0], 0.9)
        self.assertFalse(solved)
        self.assertEqual(mpc.sweeps, native.MPC_MAX_SWEEPS)
        self.assertEqual(u, 0)
        u, solved = mpc.step([np.nan, 0, 0, 0, 0], 0)
        self.assertFalse(solved)
        self.assertEqual(u, 0)
        self.assertTrue(mpc.step([0, 0, 0, 0, 0], 0)[1])

    def test_mpc_failed_solve_stops_until_restart(self):
        from unittest.mock import patch

        c = config("pendulum-regulation", "mpc")
        c["initial"] = [0, 0.01, 0, 0]
        c["duration"] = 0.1
        with patch("simulation.runner.experiment.control.LinearMpc") as factory:
            solver = factory.return_value
            solver.step.return_value = (0.0, False)
            solver.sweeps = 24
            trace = simulate(c, "mpc")
            self.assertEqual(solver.step.call_count, 1)
            np.testing.assert_array_equal(trace.feasible, 0)
            np.testing.assert_array_equal(trace.requested_steps_s, 0)
            self.assertEqual(trace.mpc_sweeps[0], 24)
            self.assertEqual(trace.t[-1], c["duration"])

    def test_mpc_rejects_invalid_tables(self):
        free = np.zeros((2, 5))
        inverse = np.array([1, 0, 1])
        for f, inv, limit in (
            (free, inverse, 0),
            (free, inverse, np.inf),
            (free[:, :4], inverse, 1),
            (free, inverse[:2], 1),
            (np.zeros((101, 5)), np.ones(5151), 1),
            (free, [1, np.nan, 1], 1),
            (free, [1, 0, -1], 1),
        ):
            with self.assertRaises(ValueError):
                native.LinearMpc(f, inv, limit)

    def test_plant(self):
        c = config("pendulum-arm-regulation", "pid")
        p = Furuta(dict(c["plant"], arm_damping=0, hinge_damping=0))
        for angle in [0, np.pi]:
            np.testing.assert_allclose(p.rhs([0.4, angle, 0, 0], 0), 0, atol=1e-12)
        A, B = p.linear()
        x = np.array([0.2, 0.3, 0.4, -0.5])
        scale = 1e-4
        np.testing.assert_allclose(
            p.rhs(scale * x, scale * 0.1) / scale, A @ x + B[:, 0] * 0.1, rtol=1e-7
        )
        drake = DrakePlant(dict(c["plant"], arm_damping=0, hinge_damping=0))
        for actual, reference in zip(drake.linear(), (A, B)):
            np.testing.assert_allclose(actual, reference, atol=1e-11)
        drake.start(x, 0.001)
        drake_state = drake.advance(1.0, 0.0, 0.0)
        e = p.energy(x)
        for _ in range(1000):
            x = rk4(lambda z: p.rhs(z, 0), x, 0.001)
        self.assertAlmostEqual(p.energy(x), e, places=8)
        np.testing.assert_allclose(drake_state, x, atol=1e-6)

    def test_native_reference(self):
        lib = native
        c = config("pendulum-arm-regulation", "lqr")
        A, B, K, L = design(Furuta(c["plant"]), c)
        x = np.array([0.2, 0.13, -0.8, 0.3])
        y = np.array([0.201, 0.11])
        F, G = observer_model(A, B, c["sample_s"])
        prediction = F @ x + G[:, 0] * 0.1
        # The four-state observer subtracts a known moving field, rather than
        # treating the rate input as a torque or dropping the fifth state.
        field = 0.3
        full = np.array([x[0] + field, *x[1:], -x[0]])
        next_full = A @ full + B[:, 0] * 0.1
        np.testing.assert_allclose(
            prediction, next_full[:4] - [field + c["sample_s"] * 0.1, 0, 0, 0]
        )
        expected = prediction + L @ (y - prediction[:2])
        x = np.array(lib.kalman(x, y, 0.1, F, G, L))
        np.testing.assert_allclose(x, expected)
        state = np.r_[x, -0.002]
        self.assertAlmostEqual(
            lib.lqr(K, state, 0.3), -(K @ (state - [0.3, 0, 0, 0, 0])).item()
        )
        pid = lib.PID(2, 1, 0.3, 0.02, 0, False)
        out = pid.step(0.01, 0.01)
        np.testing.assert_allclose(
            out, [0.02 + 0.0001 + 0.3 * 0.01 / 0.03, 0.0001], rtol=1e-6
        )
        y = measure([2 * np.pi + 0.01, 2 * np.pi - 0.01], 0, [1000, 1000])
        np.testing.assert_allclose(y, np.array([1002, -2]) * 2 * np.pi / 1000)
