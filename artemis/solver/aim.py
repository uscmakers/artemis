"""Aim solver: find the pitch/yaw that hits a target, and the error from the current aim.

Errors are always `required - current`: pitch_err_deg > 0 means raise the bow,
yaw_err_deg > 0 means turn left.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from artemis.config import Params
from artemis.physics.bow import launch_speed_mps
from artemis.physics.flight import Trajectory, launch_velocity, simulate


@dataclass(frozen=True)
class Target:
    """Target center in the world frame, relative to the nock at launch."""

    range_m: float  # downrange (x)
    lateral_m: float = 0.0  # +left (y)
    height_m: float = 0.0  # relative to launch height (z); 0.2 m below the nock -> -0.2


@dataclass(frozen=True)
class AimSolution:
    pitch_deg: float
    yaw_deg: float
    time_of_flight_s: float
    impact_speed_mps: float
    converged: bool
    iterations: int  # number of trajectory simulations used
    reason: str
    yaw_drift_deg: float = 0.0  # solved yaw minus the geometric bearing to the target (wind correction)


@dataclass(frozen=True, eq=False)
class AimError:
    pitch_err_deg: float
    yaw_err_deg: float
    predicted_impact_y_m: float  # relative to target center, +left
    predicted_impact_z_m: float  # relative to target center, +up
    predicted_miss_m: float
    solution: AimSolution
    v_launch_mps: float
    current_trajectory: Trajectory


class _PitchSearch:
    """Bisection on pitch at fixed yaw. Counts simulations across calls."""

    def __init__(self, target: Target, v_launch_mps: float, params: Params):
        self.target = target
        self.v = v_launch_mps
        self.params = params
        self.sims = 0

    def vertical_miss(self, pitch_deg: float, yaw_deg: float) -> tuple[float, Trajectory]:
        """impact height - target height at the target plane; -inf if the arrow never gets there."""
        self.sims += 1
        traj = simulate(
            launch_velocity(self.v, pitch_deg, yaw_deg),
            self.params.flight,
            self.target.range_m,
            self.params.solver.dt_s,
        )
        if not traj.hit_target_plane:
            return -math.inf, traj
        return traj.impact_z_m - self.target.height_m, traj

    def solve(self, yaw_deg: float, guess_deg: float | None) -> tuple[float, Trajectory | None, str]:
        """Return (pitch, trajectory, "") on success or (nan, None, reason) on failure."""
        s = self.params.solver
        lo_b, hi_b = s.pitch_bracket_deg

        if guess_deg is not None:
            lo = max(lo_b, guess_deg - s.warm_start_halfwidth_deg)
            hi = min(hi_b, guess_deg + s.warm_start_halfwidth_deg)
            f_lo, _ = self.vertical_miss(lo, yaw_deg)
            f_hi, traj_hi = self.vertical_miss(hi, yaw_deg)
            if f_lo < 0.0 <= f_hi:
                return self._bisect(lo, hi, f_hi, traj_hi, yaw_deg)
            # Target moved too far for the narrow bracket: fall through to a cold search.

        f_lo, _ = self.vertical_miss(lo_b, yaw_deg)
        if f_lo >= 0.0:
            return math.nan, None, f"target is below the arc at the bottom of the pitch bracket ({lo_b:g} deg)"
        f_hi, traj_hi = self.vertical_miss(hi_b, yaw_deg)
        if f_hi >= 0.0:
            return self._bisect(lo_b, hi_b, f_hi, traj_hi, yaw_deg)

        # Top of the bracket falls short. With drag the max-range angle is below 45 deg,
        # so a lower pitch might still reach: scan upward for the first (low-arc) crossing.
        best = f_lo
        prev = lo_b
        p = lo_b + s.scan_step_deg
        while p < hi_b:
            f, traj = self.vertical_miss(p, yaw_deg)
            if f >= 0.0:
                return self._bisect(prev, p, f, traj, yaw_deg)
            best = max(best, f)
            prev = p
            p += s.scan_step_deg
        if math.isinf(best):
            return math.nan, None, "out of reach: the arrow never reaches the target plane"
        return math.nan, None, f"out of reach: best pitch falls {-best:.2f} m short"

    def _bisect(
        self, lo: float, hi: float, f_hi: float, traj_hi: Trajectory, yaw_deg: float
    ) -> tuple[float, Trajectory | None, str]:
        tol = self.params.solver.tolerance_m
        if abs(f_hi) < tol:
            return hi, traj_hi, ""
        for _ in range(self.params.solver.max_iterations):
            mid = 0.5 * (lo + hi)
            f, traj = self.vertical_miss(mid, yaw_deg)
            if abs(f) < tol:
                return mid, traj, ""
            if f < 0.0:
                lo = mid
            else:
                hi = mid
        return math.nan, None, f"pitch bisection did not converge in {self.params.solver.max_iterations} iterations"


def solve_aim(
    target: Target, v_launch_mps: float, params: Params, warm_start: AimSolution | None = None
) -> AimSolution:
    """Find the low-arc pitch and the yaw that put the arrow on `target`.

    Pass the previous AimSolution as `warm_start` to narrow the search when the
    target and bow have only moved a little (the 50 Hz loop case).
    """
    search = _PitchSearch(target, v_launch_mps, params)
    tol = params.solver.tolerance_m

    def failed(reason: str, pitch: float = math.nan, yaw: float = math.nan) -> AimSolution:
        return AimSolution(pitch, yaw, math.nan, math.nan, False, search.sims, reason, math.nan)

    if target.range_m <= 0.0:
        return failed("target range must be positive")
    if v_launch_mps <= 0.0:
        return failed("launch speed is zero (draw at or below brace height?)")

    bearing_deg = math.degrees(math.atan2(target.lateral_m, target.range_m))
    if warm_start is not None and warm_start.converged:
        # The target may have moved sideways, so reuse the drift correction, not the raw yaw.
        yaw = bearing_deg + warm_start.yaw_drift_deg
        guess: float | None = warm_start.pitch_deg
    else:
        yaw = bearing_deg
        guess = None

    passes = max(1, params.solver.yaw_passes)
    lateral_miss_m = math.nan
    pitch = math.nan
    for i in range(passes):
        pitch, traj, reason = search.solve(yaw, guess)
        if traj is None:
            return failed(reason, yaw=yaw)
        lateral_miss_m = target.lateral_m - traj.impact_y_m
        if abs(lateral_miss_m) < tol:
            return AimSolution(
                pitch_deg=pitch,
                yaw_deg=yaw,
                time_of_flight_s=traj.time_of_flight_s,
                impact_speed_mps=traj.impact_speed_mps,
                converged=True,
                iterations=search.sims,
                reason="",
                yaw_drift_deg=yaw - bearing_deg,
            )
        if i < passes - 1:
            yaw += math.degrees(math.atan(lateral_miss_m / target.range_m))
            guess = pitch
    return failed(f"lateral miss {lateral_miss_m * 1000:.1f} mm after {passes} yaw passes", pitch, yaw)


def aim_error(
    current_pitch_deg: float,
    current_yaw_deg: float,
    draw_m: float,
    target: Target,
    params: Params,
    warm_start: AimSolution | None = None,
    current_roll_deg: float = 0.0,
) -> AimError:
    """How far the current aim is from the solved aim, and where the arrow would land now.

    Impact coordinates are relative to the target center (+y left, +z up). If the
    solver can't reach the target the angle errors are NaN; if the current aim never
    reaches the target plane the impact is NaN and the miss is infinite.
    """
    # TODO(v1): bow roll (cant) rotates the sight line and the launch direction; ignored in v0.
    del current_roll_deg

    v = launch_speed_mps(draw_m, params.bow)
    solution = solve_aim(target, v, params, warm_start)
    current = simulate(
        launch_velocity(v, current_pitch_deg, current_yaw_deg), params.flight, target.range_m, params.solver.dt_s
    )
    if current.hit_target_plane:
        dy = current.impact_y_m - target.lateral_m
        dz = current.impact_z_m - target.height_m
        miss = math.hypot(dy, dz)
    else:
        dy = dz = math.nan
        miss = math.inf

    return AimError(
        pitch_err_deg=solution.pitch_deg - current_pitch_deg,
        yaw_err_deg=solution.yaw_deg - current_yaw_deg,
        predicted_impact_y_m=dy,
        predicted_impact_z_m=dz,
        predicted_miss_m=miss,
        solution=solution,
        v_launch_mps=v,
        current_trajectory=current,
    )
