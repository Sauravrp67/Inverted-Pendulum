"""Build or upload one controller for the board. Run from the repository root."""

import argparse
import os
import subprocess
import sys

from simulation import ROOT
from simulation.configuration import CONTROLLERS, OBJECTIVES
from tools.firmware_parameters import prepare_values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["build", "flash"])
    parser.add_argument("board", choices=["mega", "bluepill"])
    parser.add_argument("controller", choices=CONTROLLERS)
    parser.add_argument(
        "--objective",
        choices=OBJECTIVES,
        default="pendulum-arm-regulation",
    )
    parser.add_argument("--port", help="Mega serial upload port")
    args = parser.parse_args()
    folder = ROOT / "build/firmware" / args.board / args.objective / args.controller
    prepare_values(folder, args.controller, args.objective)

    command = [
        sys.executable,
        "-m",
        "platformio",
        "run",
        "-d",
        str(ROOT / "firmware"),
        "-e",
        args.board,
    ]
    if args.action == "flash":
        command += ["-t", "upload"]
    if args.port:
        command += ["--upload-port", args.port]

    subprocess.run(
        command,
        cwd=ROOT,
        check=True,
        env={
            **os.environ,
            "FIRMWARE_CONTROLLER": args.controller,
            "FIRMWARE_VALUES_DIR": str(folder),
            "PLATFORMIO_CORE_DIR": str(ROOT / "build/platformio"),
            "PLATFORMIO_BUILD_DIR": str(folder / "pio"),
        },
    )


if __name__ == "__main__":
    main()
