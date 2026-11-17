"""Draw solver: with the bow held at a fixed angle, how far to pull back to hit the target.

This is the other way round from `aim.solve_aim`, which fixes the draw and solves for pitch.
It's what the ToF wall demo uses: distance in, required draw (and string force) out.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from artemis.config import Params
from artemis.physics.bow import draw_force_n, launch_speed_mps
from artemis.physics.flight import Trajectory, launch_velocity, simulate
from artemis.solver.aim import Target


@dataclass(frozen=True)
class DrawSolution:
    draw_m: float  # distance from grip pivot to string, same reference as the force-draw table
    force_n: float  # string force at that draw
    v_launch_mps: float
    time_of_flight_s: float
    converged: bool
    iterations: int  # number of trajectory simulations used
    reason: str


def solve_draw(target: Target, pitch_deg: float, params: Params, yaw_deg: float | None = None) -> DrawSolution:
    """Bisect on draw length (brace to the last table point) until the arrow hits `target`.

    Yaw defaults to pointing straight at the target. Impact height rises with launch speed at
    any fixed angle, so the vertical miss is monotonic in draw and bisection is safe.
    """
    curve = params.bow.curve
    tol = params.solver.tolerance_m
    yaw = math.degrees(math.atan2(target.lateral_m, target.range_m)) if yaw_deg is None else yaw_deg
    sims = 0

    def vertical_miss(draw_m: float) -> tuple[float, Trajectory | None, float]:
        nonlocal sims
        v = launch_speed_mps(draw_m, params.bow)
        if v <= 0.0:
            return -math.inf, None, v
        sims += 1
        traj = simulate(launch_velocity(v, pitch_deg, yaw), params.flight, target.range_m, params.solver.dt_s)
        return (traj.impact_z_m - target.height_m if traj.hit_target_plane else -math.inf), traj, v

    def solved(draw_m: float, traj: Trajectory, v: float) -> DrawSolution:
        return DrawSolution(draw_m, draw_force_n(draw_m, curve), v, traj.time_of_flight_s, True, sims, "")

    def failed(reason: str) -> DrawSolution:
        return DrawSolution(math.nan, math.nan, math.nan, math.nan, False, sims, reason)

    if target.range_m <= 0.0:
        return failed("target distance must be positive")

    lo, hi = curve.brace_m, curve.max_draw_m
    f_hi, traj_hi, v_hi = vertical_miss(hi)
    if f_hi < 0.0:
        if math.isinf(f_hi):
            return failed("out of reach even at full draw: raise the bow")
        return failed(f"out of reach: full draw lands {-f_hi * 100:.0f} cm low, raise the bow")
    if abs(f_hi) < tol:
        return solved(hi, traj_hi, v_hi)
    for _ in range(params.solver.max_iterations):
        mid = 0.5 * (lo + hi)
        f, traj, v = vertical_miss(mid)
        if abs(f) < tol:
            return solved(mid, traj, v)
        if f < 0.0:
            lo = mid
        else:
            hi = mid
    return failed(f"draw bisection did not converge in {params.solver.max_iterations} iterations")
