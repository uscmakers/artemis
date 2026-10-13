import numpy as np
import pytest

from artemis.config import ForceDrawCurve, load_config
from artemis.physics.bow import DrawBeyondCurveWarning, launch_speed_mps, stored_energy_j


@pytest.fixture
def params():
    return load_config(preset="recurve_30lb")


def test_linear_curve_energy_is_triangle():
    brace_m, full_m, f_max_n = 0.2, 0.7, 130.0
    curve = ForceDrawCurve(draw_m=np.linspace(brace_m, full_m, 6), force_n=np.linspace(0.0, f_max_n, 6))
    expected = 0.5 * f_max_n * (full_m - brace_m)
    assert stored_energy_j(full_m, curve) == pytest.approx(expected, rel=1e-3)


def test_linear_curve_partial_draw_between_points():
    curve = ForceDrawCurve(draw_m=np.array([0.2, 0.7]), force_n=np.array([0.0, 100.0]))
    draw_m = 0.45  # halfway: force 50 N
    assert stored_energy_j(draw_m, curve) == pytest.approx(0.5 * 50.0 * 0.25, rel=1e-3)


def test_zero_at_brace_and_monotonic(params):
    curve = params.bow.curve
    assert stored_energy_j(curve.brace_m, curve) == 0.0
    draws = np.linspace(curve.brace_m, curve.max_draw_m, 200)
    energies = [stored_energy_j(d, curve) for d in draws]
    assert np.all(np.diff(energies) > 0)


def test_below_brace_is_zero(params):
    curve = params.bow.curve
    assert stored_energy_j(curve.brace_m - 0.05, curve) == 0.0


def test_compound_let_off_curve():
    # Force peaks mid-draw then drops near full draw (let-off). Energy still rises,
    # and must equal the hand-computed trapezoid sum.
    draw_m = np.array([0.20, 0.35, 0.50, 0.60, 0.70])
    force_n = np.array([0.0, 200.0, 250.0, 120.0, 60.0])
    curve = ForceDrawCurve(draw_m=draw_m, force_n=force_n)
    expected = 0.15 * 100.0 + 0.15 * 225.0 + 0.10 * 185.0 + 0.10 * 90.0
    assert stored_energy_j(0.70, curve) == pytest.approx(expected, rel=1e-9)
    # Partial draw on the falling part: interpolate force at 0.65 -> 90 N
    assert stored_energy_j(0.65, curve) == pytest.approx(expected - 0.05 * (90.0 + 60.0) / 2, rel=1e-9)
    draws = np.linspace(0.2, 0.7, 101)
    assert np.all(np.diff([stored_energy_j(d, curve) for d in draws]) >= 0)


def test_beyond_curve_clamps_and_warns(params):
    curve = params.bow.curve
    with pytest.warns(DrawBeyondCurveWarning):
        e = stored_energy_j(curve.max_draw_m + 0.05, curve)
    assert e == pytest.approx(stored_energy_j(curve.max_draw_m, curve))


def test_recurve_sanity_numbers(params):
    draw_m = params.bow.nominal_draw_m
    assert stored_energy_j(draw_m, params.bow.curve) == pytest.approx(34.7, abs=0.1)
    assert launch_speed_mps(draw_m, params.bow) == pytest.approx(52.7, abs=0.1)
