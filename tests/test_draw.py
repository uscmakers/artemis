import math

import numpy as np
import pytest

from artemis.config import load_config
from artemis.physics.bow import draw_force_n, launch_speed_mps
from artemis.solver.aim import Target, solve_aim
from artemis.solver.draw import solve_draw


@pytest.fixture
def params():
    return load_config(preset="recurve_30lb")


def vacuum_height_m(x_m, theta_deg, v_mps, g):
    th = math.radians(theta_deg)
    return x_m * math.tan(th) - g * x_m**2 / (2 * v_mps**2 * math.cos(th) ** 2)


@pytest.mark.parametrize("range_m", [0.5, 1.0, 3.0, 6.0])
@pytest.mark.parametrize("pitch_deg", [2.0, 5.0])
def test_vacuum_lands_on_the_mark(params, range_m, pitch_deg):
    # Judge by where it lands: at short range a 0.5 mm landing tolerance allows ~1% of speed slop.
    vac = params.without_drag()
    sol = solve_draw(Target(range_m), pitch_deg, vac)
    assert sol.converged, sol.reason
    assert sol.v_launch_mps == pytest.approx(launch_speed_mps(sol.draw_m, vac.bow))
    landing_m = vacuum_height_m(range_m, pitch_deg, sol.v_launch_mps, vac.flight.gravity_mps2)
    assert landing_m == pytest.approx(0.0, abs=1e-3)


def test_draw_increases_with_distance(params):
    draws = [solve_draw(Target(r), 2.0, params).draw_m for r in np.arange(0.3, 6.01, 0.1)]
    assert not any(math.isnan(d) for d in draws)
    assert np.all(np.diff(draws) > 0)


def test_round_trip_with_pitch_solver(params):
    # Solve the draw for 5 m at 1.0°, then ask the pitch solver what angle that draw needs: 1.0°.
    sol = solve_draw(Target(5.0), 1.0, params)
    back = solve_aim(Target(5.0), sol.v_launch_mps, params)
    assert back.pitch_deg == pytest.approx(1.0, abs=0.01)


def test_force_reported_from_curve(params):
    sol = solve_draw(Target(3.0), 2.0, params)
    assert sol.force_n == pytest.approx(draw_force_n(sol.draw_m, params.bow.curve))
    assert 0.0 < sol.force_n < params.bow.curve.force_n[-1]


def test_unreachable_says_raise_the_bow(params):
    level = solve_draw(Target(3.0), 0.0, params)  # level shot always drops below the mark
    assert not level.converged
    assert "raise the bow" in level.reason
    far = solve_draw(Target(400.0), 2.0, params)
    assert not far.converged
    assert math.isnan(far.draw_m)


def test_sensor_config_defaults(params):
    assert params.sensors.tof_to_nock_m == 0.0
    assert params.sensors.tof_smoothing_samples == 5
