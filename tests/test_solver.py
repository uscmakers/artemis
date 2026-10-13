import math

import numpy as np
import pytest

from artemis.config import load_config
from artemis.physics.bow import launch_speed_mps
from artemis.solver.aim import Target, aim_error, solve_aim


@pytest.fixture
def params():
    return load_config(preset="recurve_30lb")


@pytest.fixture
def v_launch(params):
    return launch_speed_mps(params.bow.nominal_draw_m, params.bow)


def analytic_low_arc_deg(x_m, h_m, v_mps, g):
    disc = v_mps**4 - g * (g * x_m**2 + 2 * h_m * v_mps**2)
    return math.degrees(math.atan((v_mps**2 - math.sqrt(disc)) / (g * x_m)))


@pytest.mark.parametrize("range_m", [5.0, 10.0, 15.0, 30.0, 40.0])
@pytest.mark.parametrize("height_m", [-0.3, 0.0, 0.5])
def test_vacuum_matches_analytic_low_arc(params, v_launch, range_m, height_m):
    vac = params.without_drag()
    sol = solve_aim(Target(range_m, 0.0, height_m), v_launch, vac)
    assert sol.converged, sol.reason
    expected = analytic_low_arc_deg(range_m, height_m, v_launch, vac.flight.gravity_mps2)
    assert sol.pitch_deg == pytest.approx(expected, abs=0.01)


def test_converges_3_to_40m_and_pitch_increases(params, v_launch):
    ranges = np.arange(3.0, 40.01, 1.0)
    pitches = []
    for r in ranges:
        sol = solve_aim(Target(r), v_launch, params)
        assert sol.converged, f"{r} m: {sol.reason}"
        pitches.append(sol.pitch_deg)
    assert np.all(np.diff(pitches) > 0)


def test_spec_sanity_numbers(params, v_launch):
    sol = solve_aim(Target(15.0), v_launch, params)
    assert sol.pitch_deg == pytest.approx(1.56, abs=0.01)
    assert sol.time_of_flight_s == pytest.approx(0.29, abs=0.005)
    assert sol.impact_speed_mps == pytest.approx(50.7, abs=0.1)
    assert solve_aim(Target(15.0), v_launch, params.without_drag()).pitch_deg == pytest.approx(1.52, abs=0.01)
    assert solve_aim(Target(30.0), v_launch, params).pitch_deg == pytest.approx(3.21, abs=0.01)
    assert solve_aim(Target(30.0), v_launch, params.without_drag()).pitch_deg == pytest.approx(3.04, abs=0.01)


@pytest.mark.parametrize("target", [Target(400.0), Target(20.0, 0.0, 30.0)])
def test_unreachable_returns_not_converged(params, v_launch, target):
    sol = solve_aim(target, v_launch, params)
    assert not sol.converged
    assert sol.reason
    assert math.isnan(sol.pitch_deg)


def test_unreachable_toy_bow():
    toy = load_config(preset="toy_suction")
    sol = solve_aim(Target(60.0), launch_speed_mps(toy.bow.nominal_draw_m, toy.bow), toy)
    assert not sol.converged
    assert "out of reach" in sol.reason


def test_near_max_range_finds_low_arc_below_bracket_top():
    # Toy bow: with heavy drag the max-range pitch is below 45 deg, so the top of the
    # bracket can fall short while a lower pitch still reaches.
    toy = load_config(preset="toy_suction")
    v = launch_speed_mps(toy.bow.nominal_draw_m, toy.bow)
    for r in np.arange(2.0, 30.0, 0.5):
        sol = solve_aim(Target(r), v, toy)
        if not sol.converged:
            break
        last_ok = sol
    assert last_ok.converged
    assert last_ok.pitch_deg < 45.0


def test_left_target_needs_positive_yaw(params, v_launch):
    sol = solve_aim(Target(20.0, 1.0, 0.0), v_launch, params)
    assert sol.converged
    assert sol.yaw_deg > 0.0
    assert sol.yaw_deg == pytest.approx(math.degrees(math.atan2(1.0, 20.0)), abs=1e-6)


def test_crosswind_is_corrected(params, v_launch):
    windy = params.with_flight(wind_mps=(0.0, 3.0, 0.0))
    sol = solve_aim(Target(20.0), v_launch, windy)
    assert sol.converged, sol.reason
    # Wind blows the arrow left, so the archer must aim right (negative yaw).
    assert sol.yaw_deg < 0.0


def test_aim_error_at_solution_is_zero(params, v_launch):
    target = Target(18.0, -0.7, -0.2)
    draw_m = params.bow.nominal_draw_m
    windy = params.with_flight(wind_mps=(1.0, -2.0, 0.0))
    sol = solve_aim(target, v_launch, windy)
    err = aim_error(sol.pitch_deg, sol.yaw_deg, draw_m, target, windy)
    assert err.predicted_miss_m < 0.001
    assert err.pitch_err_deg == pytest.approx(0.0, abs=1e-9)
    assert err.yaw_err_deg == pytest.approx(0.0, abs=1e-9)


def test_aim_error_signs(params):
    draw_m = params.bow.nominal_draw_m
    err = aim_error(0.0, 0.0, draw_m, Target(15.0, 0.5, 0.0), params)
    assert err.pitch_err_deg > 0.0  # need to raise
    assert err.yaw_err_deg > 0.0  # need to turn left
    assert err.predicted_impact_z_m < 0.0  # currently low
    assert err.predicted_impact_y_m < 0.0  # currently right of the target


def test_warm_start_same_answer_fewer_iterations(params, v_launch):
    windy = params.with_flight(wind_mps=(0.0, 2.0, 0.0))
    cold_prev = solve_aim(Target(15.0, 0.3, 0.0), v_launch, windy)
    moved = Target(15.2, 0.32, 0.01)
    cold = solve_aim(moved, v_launch, windy)
    warm = solve_aim(moved, v_launch, windy, warm_start=cold_prev)
    assert cold.converged and warm.converged
    assert warm.pitch_deg == pytest.approx(cold.pitch_deg, abs=0.01)
    assert warm.yaw_deg == pytest.approx(cold.yaw_deg, abs=0.01)
    assert warm.iterations < cold.iterations


def test_warm_start_falls_back_when_target_jumps(params, v_launch):
    prev = solve_aim(Target(5.0), v_launch, params)
    sol = solve_aim(Target(40.0), v_launch, params, warm_start=prev)
    assert sol.converged
    assert sol.pitch_deg == pytest.approx(solve_aim(Target(40.0), v_launch, params).pitch_deg, abs=0.01)


def test_warm_start_tracks_sideways_target_motion(params, v_launch):
    # No wind: the solved yaw is just the bearing, so a warm start after the target slides
    # sideways should need only one pitch pass (no extra yaw pass).
    prev = solve_aim(Target(20.0, 0.50, 0.0), v_launch, params)
    assert prev.yaw_drift_deg == pytest.approx(0.0, abs=1e-12)
    moved = Target(20.0, 0.52, 0.0)
    warm = solve_aim(moved, v_launch, params, warm_start=prev)
    cold = solve_aim(moved, v_launch, params)
    assert warm.converged
    assert warm.yaw_deg == pytest.approx(math.degrees(math.atan2(0.52, 20.0)), abs=1e-9)
    assert warm.iterations < cold.iterations
