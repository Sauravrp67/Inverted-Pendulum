"""Prepare the numeric initializers included by firmware/parameters.c."""

import json
from tempfile import TemporaryDirectory
from pathlib import Path

import numpy as np

from simulation import ROOT
from simulation.configuration import config
from simulation.control_design import design, mpc_tables, observer_model
from simulation.dynamics.drake_plant import DrakePlant


def array_literal(values):
    values = np.asarray(values).ravel()

    if not np.isfinite(values).all():
        raise ValueError("nonfinite firmware coefficient")
    
    rows = [
        ", ".join(repr(float(v)) for v in values[i : i + 6])
        for i in range(0, len(values), 6)
    ]
    
    return "{" + ", \\\n    ".join(rows) + "}"


def c_literal(value):
    if value is None:
        return "0"
    
    if isinstance(value, str):
        return json.dumps(value)
    
    if isinstance(value, dict):
        fields = []
        
        for key, item in value.items():
            if not isinstance(key, str) or not key.isascii() or not key.isidentifier():
                raise ValueError(f"invalid firmware field name: {key!r}")
            fields.append(f".{key} = {c_literal(item)}")
        
        return "{" + ", ".join(fields) + "}"
    
    if isinstance(value, list):
        return "{" + ", ".join(c_literal(v) for v in value) + "}"
    
    if not isinstance(value, (int, float)) or not np.isfinite(value):
        raise ValueError(f"invalid firmware value: {value!r}")
    
    return repr(float(value))


def prepare_values(folder, controller, objective, *, root=ROOT / "configs"):
    c = config(objective, controller, "empirical", root)
    if c["stepper"]["max_steps_s"] > 8000 or not 0.000001 <= c["sample_s"] < 0.020:
        raise ValueError("sample period or step rate exceeds the pulse timer limits")

    plant = DrakePlant(c["plant"])
    A, B, K, L = design(plant, c, with_lqr=controller == "lqr")
    F, G = observer_model(A, B, c["sample_s"])
    values = {
        "IMAGE_LABEL": json.dumps(f"{controller}: {objective}"),
        "PARAMETER_VALUES": c_literal(c)[1:-1],
        "OBSERVER_F": array_literal(F),
        "OBSERVER_G": array_literal(G),
        "OBSERVER_L": array_literal(L),
    }
    if controller == "lqr":
        values["LQR_GAIN"] = array_literal(K)
    elif controller == "mpc":
        if not 1 <= c["horizon"] <= 100:
            raise ValueError("firmware MPC horizon must be 1..100")
        free, inverse = mpc_tables(A, B, c)
        values.update(
            MPC_HORIZON=c["horizon"],
            MPC_FREE_PLAN=array_literal(free.astype("float32")),
            MPC_INVERSE=array_literal(inverse.astype("float32")),
        )

    for group in (c["plant"], c["stepper"]):
        for key, value in group.items():
            if key.upper() in values:
                raise ValueError(f"duplicate constant name: {key}")
            values[key.upper()] = c_literal(value)
    text = "".join(f"#define {key} {value}\n" for key, value in values.items())
    folder.mkdir(parents=True, exist_ok=True)
    dest = folder / "design_values.inc"
    if not dest.exists() or dest.read_text() != text:
        with TemporaryDirectory(dir=folder) as tmp:
            staged = Path(tmp) / dest.name
            staged.write_text(text)
            staged.replace(dest)
