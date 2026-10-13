"""Load config.yaml into dataclasses, converting to SI units.

Everything downstream of this module is SI: meters, newtons, kilograms, seconds.
Angles stay in degrees in the config (they are API-boundary values).
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

M_PER_IN = 0.0254
N_PER_LBF = 4.4482216152605

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.yaml"


@dataclass(frozen=True, eq=False)
class ForceDrawCurve:
    """Measured force-draw table. Draw must strictly increase; force may do anything (let-off)."""

    draw_m: np.ndarray
    force_n: np.ndarray

    def __post_init__(self) -> None:
        draw = np.asarray(self.draw_m, dtype=float)
        force = np.asarray(self.force_n, dtype=float)
        if draw.ndim != 1 or draw.shape != force.shape or draw.size < 2:
            raise ValueError("force-draw table needs two equal-length 1D lists with at least 2 points")
        if np.any(np.diff(draw) <= 0):
            raise ValueError("force-draw draw lengths must strictly increase")
        draw.setflags(write=False)
        force.setflags(write=False)
        object.__setattr__(self, "draw_m", draw)
        object.__setattr__(self, "force_n", force)

    @property
    def brace_m(self) -> float:
        return float(self.draw_m[0])

    @property
    def max_draw_m(self) -> float:
        return float(self.draw_m[-1])


@dataclass(frozen=True, eq=False)
class BowParams:
    curve: ForceDrawCurve
    efficiency: float
    arrow_mass_kg: float
    nominal_draw_m: float


@dataclass(frozen=True, eq=False)
class FlightParams:
    gravity_mps2: float
    air_density_kgpm3: float
    wind_mps: tuple[float, float, float]
    arrow_mass_kg: float
    arrow_diameter_m: float
    drag_coefficient: float
    launch_height_m: float
    drag: bool = True

    @property
    def drag_area_m2(self) -> float:
        return float(np.pi * (self.arrow_diameter_m / 2.0) ** 2)

    @property
    def drag_k_per_m(self) -> float:
        """k in a_drag = -k |v_rel| v_rel. Zero when drag is switched off."""
        if not self.drag:
            return 0.0
        return 0.5 * self.air_density_kgpm3 * self.drag_coefficient * self.drag_area_m2 / self.arrow_mass_kg


@dataclass(frozen=True)
class SolverParams:
    dt_s: float
    pitch_bracket_deg: tuple[float, float]
    tolerance_m: float
    max_iterations: int
    yaw_passes: int
    warm_start_halfwidth_deg: float
    scan_step_deg: float


@dataclass(frozen=True, eq=False)
class Params:
    """Everything the engine needs for one preset."""

    preset_name: str
    bow: BowParams
    flight: FlightParams
    solver: SolverParams

    def with_flight(self, **changes) -> Params:
        """Copy with flight fields replaced, e.g. params.with_flight(drag=False, wind_mps=(0, 2, 0))."""
        if "wind_mps" in changes:
            changes["wind_mps"] = _vec3(changes["wind_mps"])
        return dataclasses.replace(self, flight=dataclasses.replace(self.flight, **changes))

    def without_drag(self) -> Params:
        return self.with_flight(drag=False)


def _vec3(values) -> tuple[float, float, float]:
    v = tuple(float(x) for x in values)
    if len(v) != 3:
        raise ValueError(f"expected a 3-vector, got {values!r}")
    return v  # type: ignore[return-value]


def load_raw(path: str | Path | None = None) -> dict:
    with open(path or DEFAULT_CONFIG_PATH) as f:
        return yaml.safe_load(f)


def preset_names(path: str | Path | None = None) -> list[str]:
    return list(load_raw(path)["presets"].keys())


def load_config(path: str | Path | None = None, preset: str | None = None) -> Params:
    """Load config.yaml and build Params for `preset` (default: active_preset)."""
    raw = load_raw(path)
    name = preset or raw["active_preset"]
    if name not in raw["presets"]:
        raise KeyError(f"unknown preset {name!r}; available: {', '.join(raw['presets'])}")
    p = raw["presets"][name]
    env = raw["environment"]
    s = raw["solver"]

    curve = ForceDrawCurve(
        draw_m=np.asarray(p["force_draw"]["draw_in"], dtype=float) * M_PER_IN,
        force_n=np.asarray(p["force_draw"]["force_lb"], dtype=float) * N_PER_LBF,
    )
    bow = BowParams(
        curve=curve,
        efficiency=float(p["efficiency"]),
        arrow_mass_kg=float(p["arrow_mass_kg"]),
        nominal_draw_m=float(p["nominal_draw_in"]) * M_PER_IN,
    )
    flight = FlightParams(
        gravity_mps2=float(env["gravity_mps2"]),
        air_density_kgpm3=float(env["air_density_kgpm3"]),
        wind_mps=_vec3(env["wind_mps"]),
        arrow_mass_kg=float(p["arrow_mass_kg"]),
        arrow_diameter_m=float(p["arrow_diameter_m"]),
        drag_coefficient=float(p["drag_coefficient"]),
        launch_height_m=float(p["launch_height_m"]),
    )
    lo, hi = (float(x) for x in s["pitch_bracket_deg"])
    solver = SolverParams(
        dt_s=float(s["dt_s"]),
        pitch_bracket_deg=(lo, hi),
        tolerance_m=float(s["tolerance_m"]),
        max_iterations=int(s["max_iterations"]),
        yaw_passes=int(s["yaw_passes"]),
        warm_start_halfwidth_deg=float(s["warm_start_halfwidth_deg"]),
        scan_step_deg=float(s["scan_step_deg"]),
    )
    return Params(preset_name=name, bow=bow, flight=flight, solver=solver)
