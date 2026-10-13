import math

import numpy as np
import pytest

from artemis.config import load_config
from artemis.physics.bow import launch_speed_mps
from artemis.physics.flight import launch_velocity, simulate


@pytest.fixture
def params():
    return load_config(preset="recurve_30lb")


@pytest.fixture
def v_launch(params):
    return launch_speed_mps(params.bow.nominal_draw_m, params.bow)


def vacuum_height_m(x_m, theta_deg, v_mps, g):
    th = math.radians(theta_deg)
    return x_m * math.tan(th) - g * x_m**2 / (2 * v_mps**2 * math.cos(th) ** 2)


def test_launch_velocity_frame():
    v = launch_velocity(10.0, 0.0, 0.0)
    assert v == pytest.approx([10.0, 0.0, 0.0])
    assert launch_velocity(10.0, 90.0, 0.0) == pytest.approx([0.0, 0.0, 10.0], abs=1e-12)
    # +yaw points left (+y)
    assert launch_velocity(10.0, 0.0, 90.0) == pytest.approx([0.0, 10.0, 0.0], abs=1e-12)
    assert np.linalg.norm(launch_velocity(7.0, 23.0, -41.0)) == pytest.approx(7.0)


@pytest.mark.parametrize("theta_deg", [-2.0, 0.0, 1.5, 5.0, 15.0])
@pytest.mark.parametrize("range_m", [5.0, 15.0, 30.0, 40.0])
def test_vacuum_matches_analytic(params, v_launch, theta_deg, range_m):
    # Raise the launch point so flat shots at 40 m (which drop ~3 m) reach the plane instead of the ground.
    vac = params.with_flight(drag=False, launch_height_m=100.0).flight
    traj = simulate(launch_velocity(v_launch, theta_deg, 0.0), vac, range_m, params.solver.dt_s)
    assert traj.hit_target_plane
    expected = vacuum_height_m(range_m, theta_deg, v_launch, vac.gravity_mps2)
    assert traj.impact_z_m == pytest.approx(expected, abs=1e-3)
    assert traj.impact_y_m == pytest.approx(0.0, abs=1e-12)
    assert traj.x[-1] == pytest.approx(range_m)


@pytest.mark.parametrize("range_m", [5.0, 15.0, 40.0])
def test_halving_dt_changes_impact_less_than_1mm(params, v_launch, range_m):
    v0 = launch_velocity(v_launch, 3.0, 1.0)
    windy = params.with_flight(wind_mps=(1.0, 3.0, 0.0)).flight
    a = simulate(v0, windy, range_m, params.solver.dt_s)
    b = simulate(v0, windy, range_m, params.solver.dt_s / 2)
    assert math.hypot(a.impact_y_m - b.impact_y_m, a.impact_z_m - b.impact_z_m) < 1e-3


def test_drag_lands_lower(params, v_launch):
    v0 = launch_velocity(v_launch, 2.0, 0.0)
    with_drag = simulate(v0, params.flight, 20.0, params.solver.dt_s)
    vacuum = simulate(v0, params.without_drag().flight, 20.0, params.solver.dt_s)
    assert with_drag.impact_z_m < vacuum.impact_z_m
    assert with_drag.impact_speed_mps < vacuum.impact_speed_mps
    assert with_drag.time_of_flight_s > vacuum.time_of_flight_s


def test_matches_scipy_solve_ivp(params, v_launch):
    integrate = pytest.importorskip("scipy.integrate")
    flight = params.with_flight(wind_mps=(-1.0, 2.0, 0.5)).flight
    v0 = launch_velocity(v_launch, 4.0, -2.0)
    range_m = 30.0
    traj = simulate(v0, flight, range_m, params.solver.dt_s)

    g, k, w = flight.gravity_mps2, flight.drag_k_per_m, np.array(flight.wind_mps)

    def rhs(_t, s):
        v = s[3:]
        v_rel = v - w
        a = -k * np.linalg.norm(v_rel) * v_rel + np.array([0.0, 0.0, -g])
        return np.concatenate([v, a])

    def reach_plane(_t, s):
        return s[0] - range_m

    reach_plane.terminal = True
    sol = integrate.solve_ivp(
        rhs, (0.0, 5.0), np.concatenate([[0, 0, 0], v0]), events=reach_plane, rtol=1e-10, atol=1e-12
    )
    y_ref, z_ref = sol.y_events[0][0][1], sol.y_events[0][0][2]
    assert math.hypot(traj.impact_y_m - y_ref, traj.impact_z_m - z_ref) < 1e-3
    assert traj.time_of_flight_s == pytest.approx(sol.t_events[0][0], abs=1e-4)


def test_crosswind_toward_left_pushes_impact_left(params, v_launch):
    v0 = launch_velocity(v_launch, 2.0, 0.0)
    windy = params.with_flight(wind_mps=(0.0, 3.0, 0.0)).flight
    assert simulate(v0, windy, 20.0, params.solver.dt_s).impact_y_m > 0.0
    calm = simulate(v0, params.flight, 20.0, params.solver.dt_s)
    assert calm.impact_y_m == pytest.approx(0.0, abs=1e-12)


def test_ground_stop(params, v_launch):
    traj = simulate(launch_velocity(v_launch, -10.0, 0.0), params.flight, 30.0, params.solver.dt_s)
    assert not traj.hit_target_plane
    assert traj.stop_reason == "ground"
    assert traj.z[-1] == pytest.approx(-params.flight.launch_height_m)
    assert math.isnan(traj.impact_z_m)


def test_straight_up_stalls(params):
    traj = simulate(launch_velocity(10.0, 90.0, 0.0), params.flight, 30.0, params.solver.dt_s)
    assert not traj.hit_target_plane


def test_spec_level_shot_drop_at_15m(params, v_launch):
    traj = simulate(launch_velocity(v_launch, 0.0, 0.0), params.flight, 15.0, params.solver.dt_s)
    assert traj.impact_z_m == pytest.approx(-0.41, abs=0.01)
    assert traj.time_of_flight_s == pytest.approx(0.29, abs=0.005)
