"""Render one swing-up objective for a selected controller or all four."""

import argparse
import csv
import json
from importlib.metadata import version
import hashlib
import platform
from tempfile import TemporaryDirectory
import furuta_control
from pathlib import Path
import numpy as np
from simulation.configuration import CONTROLLERS, OBJECTIVES, config
from simulation.control_design import design
from simulation.runner.experiment import first_balance_loss, simulate
from simulation import ROOT
from simulation.dynamics.drake_plant import DrakePlant
from simulation.render import render


def metrics(a):
    upright = np.arctan2(np.sin(a.angle), np.cos(a.angle))
    balance = a.mode == 1
    outside = np.flatnonzero((abs(upright) > 0.03) | (abs(a.angle_rate) > 0.15))
    settled = outside[-1] + 1 if len(outside) else 0
    return dict(
        final_angle_rad=float(upright[-1]),
        final_arm_rad=float(a.arm[-1]),
        arm_span_last_5s_rad=float(np.ptp(a.arm[a.t >= a.t[-1] - 5])),
        peak_arm_rad=float(max(abs(a.arm))),
        settled_from_s=float(a.t[settled]) if settled < len(a) else None,
        rate_limited_s=float(np.sum(a.rate_limited[:-1]) * (a.t[1] - a.t[0])),
        torque_limited_s=float(np.sum(a.torque_limited[:-1]) * (a.t[1] - a.t[0])),
        switches_s=a.t[np.diff(a.mode, prepend=a.mode[0]) != 0].tolist(),
        velocity_rmse=float(np.mean((a.angle_rate - a.angle_rate_est) ** 2) ** 0.5),
        balance_rod_rate_rmse_rad_s=(
            float(
                np.sqrt(
                    np.mean((a.angle_rate[balance] - a.angle_rate_est[balance]) ** 2)
                )
            )
            if balance.any()
            else None
        ),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", choices=OBJECTIVES)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--controller", choices=CONTROLLERS)
    parser.add_argument(
        "--estimator",
        choices=("kalman", "difference"),
        default="kalman",
        help="state estimator after capture (default: kalman)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        help="root seed for the per-run encoder and motor noise streams",
    )
    args = parser.parse_args()
    if args.seed is not None and args.seed < 0:
        parser.error("--seed must be nonnegative")

    folder = args.name if args.seed is None else f"{args.name}-seed-{args.seed}"
    if args.estimator != "kalman":
        folder += "-" + args.estimator
    if args.controller:
        folder += "-" + args.controller
    dest = args.output or ROOT / "build/showcase" / folder
    runs = (args.controller,) if args.controller else CONTROLLERS
    reproduce(args.name, dest, args.seed, args.estimator, runs)


def reproduce(name, dest, seed=None, estimator="kalman", runs=CONTROLLERS):
    dest.mkdir(parents=True, exist_ok=True)
    # Keep the previous files if rendering fails.
    with TemporaryDirectory(dir=dest) as tmp:
        staging = Path(tmp)
        generate(name, staging, seed, estimator, runs)
        for artifact in sorted(
            staging.iterdir(), key=lambda p: p.name == "manifest.json"
        ):
            artifact.replace(dest / artifact.name)
    print(dest)


def generate(name, dest, seed=None, estimator="kalman", runs=CONTROLLERS):
    settings = {}
    summary = {}
    for run in runs:
        c = config(name, run)
        c["balance_estimator"] = estimator
        if seed is not None:
            c["hardware_model"]["seed"] = seed
        c["design"] = {
            k: v.tolist()
            for k, v in zip(
                ("Ad", "Bd", "K", "L"),
                design(DrakePlant(c["plant"]), c, with_lqr=run == "lqr"),
            )
            if v is not None
        }
        settings[run] = c
        if run == "mpc":
            c["design"]["solver"] = {
                "method": "dual coordinate descent",
                "precision": "float32",
                "max_sweeps": furuta_control.MPC_MAX_SWEEPS,
                "tolerance_rad_s": furuta_control.MPC_TOLERANCE,
            }
        trace = simulate(c, run)
        with (dest / (run + ".csv")).open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(trace.dtype.names)
            writer.writerows(trace)

        data = np.genfromtxt(dest / (run + ".csv"), delimiter=",", names=True).view(
            np.recarray
        )
        if any(not np.isfinite(data[key]).all() for key in data.dtype.names):
            raise ValueError(f"{run}: nonnumeric or nonfinite CSV sample")
        render(name, {run: data}, c, dest)

        capture = np.flatnonzero(data.mode)
        lost = first_balance_loss(data, c["swing"]["exit_rad"])
        summary[run] = metrics(data)
        if run == "mpc":
            failed = np.flatnonzero(data.feasible == 0)
            summary[run].update(
                solver_stop_s=float(data.t[failed[0]]) if failed.size else None,
                max_mpc_sweeps=int(max(data.mpc_sweeps)),
            )

        pulse = np.flatnonzero(data.push_Nm)
        if pulse.size:
            tilt = np.abs(np.arctan2(np.sin(data.angle), np.cos(data.angle)))
            end = pulse[-1] + 1
            outside = np.flatnonzero(tilt[end:] > np.radians(1))
            back = end + outside[-1] + 1 if outside.size else end
            summary[run].update(
                push_peak_tilt_deg=float(np.degrees(max(tilt[pulse[0] :]))),
                push_recovery_s=(
                    float(data.t[back] - data.t[end]) if back < len(data) else None
                ),
            )
        summary[run].update(
            capture_s=float(data.t[capture[0]]) if capture.size else None,
            first_loss_s=float(data.t[lost]) if lost is not None else None,
            final_arm_error_rad=(
                None
                if c["arm_target_rad"] is None
                else float(data.arm[-1] - c["arm_target_rad"])
            ),
        )
    (dest / "config.json").write_text(json.dumps(settings, indent=2) + "\n")
    (dest / "summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n"
    )

    manifest = {
        "scenario": name,
        "python": platform.python_version(),
        "versions": {
            pkg: version(pkg)
            for pkg in (
                "furuta-control",
                "drake",
                "numpy",
                "scipy",
                "matplotlib",
                "pillow",
            )
        },
        "native_sha256": hashlib.sha256(
            Path(furuta_control.__file__).read_bytes()
        ).hexdigest(),
        "files": {},
    }
    for path in sorted(dest.iterdir()):
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        manifest["files"][path.name] = digest.hexdigest()
    (dest / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
