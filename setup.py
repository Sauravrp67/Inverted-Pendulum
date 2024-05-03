from pathlib import Path

from pybind11.setup_helpers import Pybind11Extension
from setuptools import setup

setup(
    ext_modules=[
        Pybind11Extension(
            "furuta_control",
            [
                "simulation/runner/native.cpp",
                "controllers/mpc/mpc.cpp",
                "controllers/classical/PIDController.cpp",
            ],
            include_dirs=["."],
            depends=[
                str(p)
                for folder in ("controllers", "estimators")
                for p in Path(folder).rglob("*.h")
            ],
            cxx_std=17,
        )
    ]
)
