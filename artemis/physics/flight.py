"""Point-mass arrow flight: gravity + quadratic drag + optional wind, fixed-step RK4.

World frame (origin at the nock at launch): x downrange, y to the archer's left, z up.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from artemis.config import FlightParams


@dataclass(frozen=True, eq=False)
class Trajectory:
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    z: np.ndarray
    vx: np.ndarray
    vy: np.ndarray
    vz: np.ndarray
    hit_target_plane: bool
    impact_y_m: float  # NaN unless hit_target_plane
    impact_z_m: float
    time_of_flight_s: float
    impact_speed_mps: float
    stop_reason: str  # "target_plane", "ground", or "stalled"


def launch_velocity(speed_mps: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Velocity vector for an arrow leaving at `speed_mps`. +pitch is up, +yaw is left (+y)."""
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    return np.array(
        [
            speed_mps * math.cos(pitch) * math.cos(yaw),
            speed_mps * math.cos(pitch) * math.sin(yaw),
            speed_mps * math.sin(pitch),
        ]
    )


def simulate(v0, params: FlightParams, x_target_m: float, dt_s: float) -> Trajectory:
    """Integrate from the origin with initial velocity `v0` until x reaches `x_target_m`.

    The crossing of the target plane is linearly interpolated between steps. Also stops
    if the arrow drops more than `launch_height_m` below launch (ground) or stops moving
    downrange (vx <= 0).
    """
    g = params.gravity_mps2
    k = params.drag_k_per_m
    wx, wy, wz = params.wind_mps
    ground_z = -params.launch_height_m

    # Plain floats, not numpy: ~10x faster for a 6-element state, and the solver calls this a lot.
    def accel(vx: float, vy: float, vz: float) -> tuple[float, float, float]:
        rx, ry, rz = vx - wx, vy - wy, vz - wz
        c = k * math.sqrt(rx * rx + ry * ry + rz * rz)
        return -c * rx, -c * ry, -g - c * rz

    t = x = y = z = 0.0
    vx, vy, vz = (float(c) for c in v0)
    ts, xs, ys, zs, vxs, vys, vzs = [t], [x], [y], [z], [vx], [vy], [vz]
    h = dt_s
    h2 = 0.5 * h
    h6 = h / 6.0

    stop_reason = "stalled"
    while vx > 0.0:
        # RK4. Position derivative is velocity, so only acceleration needs evaluating.
        a1x, a1y, a1z = accel(vx, vy, vz)
        v2x, v2y, v2z = vx + h2 * a1x, vy + h2 * a1y, vz + h2 * a1z
        a2x, a2y, a2z = accel(v2x, v2y, v2z)
        v3x, v3y, v3z = vx + h2 * a2x, vy + h2 * a2y, vz + h2 * a2z
        a3x, a3y, a3z = accel(v3x, v3y, v3z)
        v4x, v4y, v4z = vx + h * a3x, vy + h * a3y, vz + h * a3z
        a4x, a4y, a4z = accel(v4x, v4y, v4z)

        nx = x + h6 * (vx + 2 * v2x + 2 * v3x + v4x)
        ny = y + h6 * (vy + 2 * v2y + 2 * v3y + v4y)
        nz = z + h6 * (vz + 2 * v2z + 2 * v3z + v4z)
        nvx = vx + h6 * (a1x + 2 * a2x + 2 * a3x + a4x)
        nvy = vy + h6 * (a1y + 2 * a2y + 2 * a3y + a4y)
        nvz = vz + h6 * (a1z + 2 * a2z + 2 * a3z + a4z)
        nt = t + h

        if nx >= x_target_m:
            stop_reason = "target_plane"
            frac = (x_target_m - x) / (nx - x)
        elif nz < ground_z:
            stop_reason = "ground"
            frac = (ground_z - z) / (nz - z)
        else:
            frac = None

        if frac is not None:
            t, x, y, z, vx, vy, vz = (
                a + frac * (b - a)
                for a, b in ((t, nt), (x, nx), (y, ny), (z, nz), (vx, nvx), (vy, nvy), (vz, nvz))
            )
        else:
            t, x, y, z, vx, vy, vz = nt, nx, ny, nz, nvx, nvy, nvz
        ts.append(t)
        xs.append(x)
        ys.append(y)
        zs.append(z)
        vxs.append(vx)
        vys.append(vy)
        vzs.append(vz)
        if frac is not None:
            break

    hit = stop_reason == "target_plane"
    nan = float("nan")
    return Trajectory(
        t=np.array(ts),
        x=np.array(xs),
        y=np.array(ys),
        z=np.array(zs),
        vx=np.array(vxs),
        vy=np.array(vys),
        vz=np.array(vzs),
        hit_target_plane=hit,
        impact_y_m=y if hit else nan,
        impact_z_m=z if hit else nan,
        time_of_flight_s=t if hit else nan,
        impact_speed_mps=math.sqrt(vx * vx + vy * vy + vz * vz) if hit else nan,
        stop_reason=stop_reason,
    )
