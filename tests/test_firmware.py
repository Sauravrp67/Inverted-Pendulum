import ctypes as ct
from pathlib import Path
import subprocess
from shutil import copytree
from tempfile import TemporaryDirectory
import unittest

import numpy as np
import furuta_control as native
from ruamel.yaml import YAML

from simulation import ROOT
from simulation.configuration import config
from simulation.control_design import design, mpc_tables, observer_model
from simulation.dynamics.drake_plant import DrakePlant
from tools.firmware_parameters import prepare_values


def compile_constants(folder):
    obj = folder / "parameters.o"
    subprocess.run(
        [
            "cc",
            "-std=c11",
            "-fPIC",
            "-Werror",
            "-I",
            folder,
            "-c",
            ROOT / "firmware/parameters.c",
            "-o",
            obj,
        ],
        check=True,
    )
    return obj


def compile_probe(folder, name, objective, *, root=ROOT / "configs"):
    prepare_values(folder, name, objective, root=root)
    out = folder / (name + ".so")
    subprocess.run(
        [
            "c++",
            "-std=c++17",
            "-O2",
            "-shared",
            "-fPIC",
            "-I.",
            "-I",
            folder,
            compile_constants(folder),
            f"firmware/{name}.cpp",
            "tests/controller_probe.cpp",
            "controllers/classical/PIDController.cpp",
            "controllers/mpc/mpc.cpp",
            "-o",
            str(out),
        ],
        cwd=ROOT,
        check=True,
    )
    step = ct.CDLL(str(out)).probe
    step.restype = ct.c_bool
    step.argtypes = [
        np.ctypeslib.ndpointer(dtype=np.float64),
        ct.c_double,
        ct.c_double,
        ct.c_bool,
        ct.POINTER(ct.c_double),
    ]
    return step


class FirmwareControllers(unittest.TestCase):
    def test_mpc_bounds_and_failed_solve_match_firmware(self):
        with TemporaryDirectory() as tmp:
            folder = Path(tmp)
            root = folder / "configs"
            copytree(ROOT / "configs", root)
            profile = root / "empirical/mpc.yaml"
            yaml = YAML()
            values = yaml.load(profile)
            values["horizon"] = 12
            objective = "pendulum-arm-regulation"
            c = config(objective, "mpc", "empirical", root=root)
            values["stepper"] = {
                "max_steps_s": 0.3 * c["stepper"]["steps_per_turn"] / (2 * np.pi)
            }
            yaml.dump(values, profile)
            c = config(objective, "mpc", "empirical", root=root)
            A, B, _, _ = design(DrakePlant(c["plant"]), c, with_lqr=False)
            rad_per_step = 2 * np.pi / c["stepper"]["steps_per_turn"]
            bound = c["stepper"]["max_steps_s"] * rad_per_step
            mpc = native.LinearMpc(*mpc_tables(A, B, c), bound)
            step = compile_probe(folder, "mpc", objective, root=root)
            active, failed = False, False
            for tilt in (0, 0.055, -0.055, 0.2, -0.2):
                state = np.array([0.0, tilt, 0.0, 0.0, 0.0])
                expected, solved = mpc.step(state, c["arm_target_rad"])
                rate = ct.c_double()
                self.assertEqual(step(state, tilt, 0, False, ct.byref(rate)), solved)
                self.assertAlmostEqual(rate.value, expected / rad_per_step, delta=1e-9)
                active |= solved and abs(expected) >= bound - 1e-4
                failed |= not solved
            self.assertTrue(active)
            self.assertTrue(failed)

    def test_yaml_list_override_and_duplicate_keys(self):
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            copytree(ROOT / "configs", root, dirs_exist_ok=True)
            profile = root / "empirical/pid.yaml"
            text = profile.read_text() + "\nprocess_variances: [1e-9, 2e-9, 0, 0]\n"
            profile.write_text(text)
            c = config("pendulum-regulation", "pid", "empirical", root=root)
            self.assertEqual(c["process_variances"], [1e-9, 2e-9, 0, 0])
            profile.write_text(text + "process_variances: [0, 0, 0, 0]\n")
            with self.assertRaisesRegex(ValueError, "duplicate key"):
                config("pendulum-regulation", "pid", "empirical", root=root)

    def test_common_domain_and_controller_overrides(self):
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            copytree(ROOT / "configs", root, dirs_exist_ok=True)
            base = root / "base.yaml"
            base.write_text(
                base.read_text().replace(
                    "empirical:\n", "empirical:\n  plant: {rod_mass: 0.12}\n"
                )
            )
            profile = root / "empirical/pid.yaml"
            profile.write_text(profile.read_text() + "\nplant: {rod_mass: 0.14}\n")

            for domain, controller, mass in (
                ("simulation", "pid", 0.1),
                ("empirical", "lqr", 0.12),
                ("empirical", "pid", 0.14),
            ):
                c = config("pendulum-arm-regulation", controller, domain, root=root)
                self.assertEqual(c["plant"]["rod_mass"], mass)
                self.assertEqual(c["plant"]["rod_length"], 0.3)

            profile.write_text(
                profile.read_text().replace("../base.yaml", "other.yaml")
            )
            with self.assertRaisesRegex(ValueError, "invalid import"):
                config("pendulum-regulation", "pid", "empirical", root=root)

    def test_serial_modes_and_pulse_timeout(self):
        with TemporaryDirectory() as tmp:
            folder = Path(tmp)
            root = folder / "configs"
            copytree(ROOT / "configs", root)
            profile = root / "empirical/pid.yaml"
            for enabled in (True, False):
                if not enabled:
                    profile.write_text(
                        profile.read_text() + "\nswing: {enabled: false}\n"
                    )
                prepare_values(folder, "pid", "pendulum-arm-regulation", root=root)
                out = folder / "board"
                subprocess.run(
                    [
                        "c++",
                        "-std=c++17",
                        "-DARDUINO_ARCH_STM32",
                        "-Itests/board_io",
                        "-I.",
                        compile_constants(folder),
                        "firmware/main.cpp",
                        "firmware/pid.cpp",
                        "controllers/classical/PIDController.cpp",
                        "tests/board_io/run.cpp",
                        "-o",
                        str(out),
                    ],
                    cwd=ROOT,
                    check=True,
                )
                subprocess.run([out] if enabled else [out, "disabled"], check=True)

    def test_inputs_and_derived_tables(self):
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            copytree(ROOT / "configs", root, dirs_exist_ok=True)
            out = root / "build"
            profile = root / "empirical/mpc.yaml"
            original = config("pendulum-regulation", "mpc", root=root)
            prepare_values(out, "mpc", "pendulum-regulation", root=root)
            self.assertEqual({p.name for p in out.iterdir()}, {"design_values.inc"})
            generated = (out / "design_values.inc").read_text()
            self.assertNotIn("YAML_", generated)
            self.assertIn(".rod_length = ", generated)
            self.assertIn(".steps_per_turn = ", generated)
            obj = compile_constants(out)
            before = obj.read_bytes()
            prepare_values(out, "mpc", "pendulum-regulation", root=root)
            self.assertEqual(before, compile_constants(out).read_bytes())
            profile.write_text(profile.read_text() + "\nplant: {rod_mass: 0.12}\n")
            prepare_values(out, "mpc", "pendulum-regulation", root=root)
            self.assertNotEqual(before, compile_constants(out).read_bytes())
            self.assertEqual(original, config("pendulum-regulation", "mpc", root=root))

            c = config("pendulum-regulation", "mpc", "empirical", root=root)
            A, B, _, L = design(DrakePlant(c["plant"]), c, with_lqr=False)
            F, G = observer_model(A, B, c["sample_s"])
            library = out / "params.so"
            subprocess.run(
                [
                    "cc",
                    "-shared",
                    "-fPIC",
                    "-I",
                    ROOT / "firmware",
                    obj,
                    "-x",
                    "c",
                    "-",
                    "-o",
                    library,
                ],
                input="""#include "parameters.h"
                void read_constants(double *out) {
                    out[0] = params.rad_per_step;
                    out[1] = params.hinge_inertia;
                    out[2] = params.gravity_moment;
                    out[3] = params.coupling;
                    out[4] = params.sample_s;
                    out[5] = params.swing.capture_rate;
                    out[6] = params.plant.rod_mass;
                    out[7] = params.stepper.steps_per_turn;
                }
                """,
                text=True,
                check=True,
            )
            linked = ct.CDLL(str(library))
            actual = (ct.c_double * 8)()
            linked.read_constants(actual)
            p = c["plant"]
            plant = DrakePlant(p)
            np.testing.assert_allclose(
                actual,
                [
                    2 * np.pi / c["stepper"]["steps_per_turn"],
                    plant.j,
                    plant.g,
                    p["rod_mass"] * p["arm_length"] * p["rod_length"] / 2,
                    c["sample_s"],
                    c["swing"]["capture_rate"],
                    p["rod_mass"],
                    c["stepper"]["steps_per_turn"],
                ],
                rtol=1e-14,
                atol=0,
            )
            observer = (ct.c_double * 28).in_dll(linked, "observer")
            np.testing.assert_array_equal(
                observer, np.r_[F.ravel(), G.ravel(), L.ravel()]
            )

    def test_board_commands_against_desktop(self):
        for objective in ("pendulum-regulation", "pendulum-arm-regulation"):
            for name in ("pid", "lead", "lqr", "mpc"):
                c = config(objective, name, "empirical")
                A, B, K, _ = design(DrakePlant(c["plant"]), c, with_lqr=name == "lqr")
                ref = c["arm_target_rad"] or 0
                rad_per_step = 2 * np.pi / c["stepper"]["steps_per_turn"]
                limit = c["stepper"]["max_steps_s"]
                if name == "mpc":
                    mpc = native.LinearMpc(
                        *mpc_tables(A, B, c),
                        limit * rad_per_step,
                    )
                with (
                    self.subTest(objective=objective, name=name),
                    TemporaryDirectory() as tmp,
                ):
                    step = compile_probe(Path(tmp), name, objective)
                    if name == "pid":
                        p = c["pid"]
                        pid = native.PID(
                            p["kp"],
                            p["ki"],
                            p["kd"],
                            p["derivative_time_s"],
                            limit,
                            True,
                        )
                    lead = native.RateLead()
                    for i, (tilt, velocity) in enumerate(
                        ((0.01, 0.1), (-0.12, -1.2), (0.15, 2.4), (-0.15, -2.4))
                    ):
                        x = np.array([0.1, tilt, 0.1, velocity, 0.02])
                        rate = ct.c_double()
                        self.assertTrue(step(x, tilt, 0.03, i == 0, ct.byref(rate)))
                        if i == 0 and name == "pid":
                            pid.prime(tilt, 0.03, velocity)
                        if i == 0 and name == "lead":
                            lead.prime(c["lead"]["gain"] * 0.03)
                        if name == "pid":
                            expected = np.clip(
                                pid.step(tilt, c["sample_s"])[0], -limit, limit
                            )
                        elif name == "lead":
                            p = c["lead"]
                            expected = lead.step(
                                tilt,
                                c["sample_s"],
                                p["gain"],
                                p["zero_s"],
                                p["pole_s"],
                                limit,
                            )
                        elif name == "lqr":
                            expected = native.lqr(K, x, ref) / rad_per_step
                        else:
                            u, ok = mpc.step(x, ref)
                            self.assertTrue(ok)
                            expected = u / rad_per_step
                        self.assertAlmostEqual(
                            rate.value, expected, delta=1e-9 if name == "mpc" else 0.1
                        )
