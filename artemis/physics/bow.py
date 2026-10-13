"""Bow model: force-draw curve -> stored energy -> arrow launch speed."""

from __future__ import annotations

import math
import warnings

import numpy as np

from artemis.config import BowParams, ForceDrawCurve


class DrawBeyondCurveWarning(UserWarning):
    """Draw length past the last measured point; energy is clamped, not extrapolated."""


def stored_energy_j(draw_m: float, curve: ForceDrawCurve) -> float:
    """Work done drawing the string from brace height to `draw_m`.

    Trapezoid rule on the table with linear interpolation between points, which is
    exact for a piecewise-linear curve. No monotonicity assumption, so compound
    let-off curves work. Draw below brace gives 0; draw past the table is clamped.
    """
    if draw_m > curve.max_draw_m:
        warnings.warn(
            f"draw {draw_m:.4f} m is past the last force-draw point {curve.max_draw_m:.4f} m; clamping",
            DrawBeyondCurveWarning,
            stacklevel=2,
        )
        draw_m = curve.max_draw_m
    if draw_m <= curve.brace_m:
        return 0.0

    inside = curve.draw_m < draw_m
    d = np.append(curve.draw_m[inside], draw_m)
    f = np.append(curve.force_n[inside], np.interp(draw_m, curve.draw_m, curve.force_n))
    return float(np.sum(0.5 * (f[1:] + f[:-1]) * np.diff(d)))


def launch_speed_mps(draw_m: float, bow: BowParams) -> float:
    """v = sqrt(2 * efficiency * E / m)."""
    energy_j = stored_energy_j(draw_m, bow.curve)
    return math.sqrt(2.0 * bow.efficiency * energy_j / bow.arrow_mass_kg)
