"""Time solve_aim cold vs warm-started over random targets.

Warm start models the 50 Hz loop: the previous frame's solution is passed in, and the
target has moved slightly since (range +/-5 cm, lateral +/-2 cm, height +/-1 cm per frame).
Budget on the Pi is 20 ms per loop for everything, so this is just where we are today.

Run: python -m explore.solver_timing [--preset toy_suction] [--n 200]
"""

from __future__ import annotations

import time

import numpy as np

from artemis.physics.bow import launch_speed_mps
from artemis.physics.flight import launch_velocity, simulate
from artemis.solver.aim import Target, aim_error, solve_aim
from explore._common import parse_args

SEED = 2
RANGE_M = (3.0, 40.0)
LATERAL_M = (-2.0, 2.0)
HEIGHT_M = (-1.0, 1.0)
FRAME_JITTER_M = (0.05, 0.02, 0.01)  # per-frame target motion for the warm-start case
LOOP_BUDGET_MS = 20.0


def summarize(name: str, ms: list[float], iters: list[int], n_failed: int) -> None:
    a = np.array(ms)
    print(
        f"{name:<24} mean {a.mean():6.2f} ms   p95 {np.percentile(a, 95):6.2f} ms   worst {a.max():6.2f} ms"
        f"   sims/solve {np.mean(iters):5.1f}   over {LOOP_BUDGET_MS:g} ms: {np.mean(a > LOOP_BUDGET_MS) * 100:4.1f}%"
        + (f"   ({n_failed} unreachable)" if n_failed else "")
    )


def reachable_range_m(v_mps: float, params) -> float:
    """Upper end of the random range: RANGE_M[1], or 95% of how far a level shot can reach if less."""
    r = RANGE_M[0]
    while r < RANGE_M[1] and solve_aim(Target(r + 0.5, 0.0, HEIGHT_M[1]), v_mps, params).converged:
        r += 0.5
    return min(RANGE_M[1], 0.95 * r) if r < RANGE_M[1] else RANGE_M[1]


def main() -> None:
    def extra(p):
        p.add_argument("--n", type=int, default=200, help="number of random targets")

    args, params = parse_args(__doc__, extra)
    rng = np.random.default_rng(SEED)
    v = launch_speed_mps(params.bow.nominal_draw_m, params.bow)
    max_range_m = reachable_range_m(v, params)
    targets = [
        Target(rng.uniform(RANGE_M[0], max_range_m), rng.uniform(*LATERAL_M), rng.uniform(*HEIGHT_M))
        for _ in range(args.n)
    ]
    jitter = rng.uniform(-1, 1, size=(args.n, 3)) * np.array(FRAME_JITTER_M)

    # Untimed warm-up so imports/JIT-ish caches don't skew the first sample.
    solve_aim(Target(15.0), v, params)

    cold_ms, cold_it, prev, failed = [], [], [], 0
    for t in targets:
        t0 = time.perf_counter()
        sol = solve_aim(t, v, params)
        cold_ms.append((time.perf_counter() - t0) * 1000)
        cold_it.append(sol.iterations)
        prev.append(sol)
        failed += not sol.converged

    warm_ms, warm_it, mismatch = [], [], 0
    loop_ms = []
    for t, j, p in zip(targets, jitter, prev):
        moved = Target(t.range_m + j[0], t.lateral_m + j[1], t.height_m + j[2])
        t0 = time.perf_counter()
        sol = solve_aim(moved, v, params, warm_start=p)
        warm_ms.append((time.perf_counter() - t0) * 1000)
        warm_it.append(sol.iterations)
        ref = solve_aim(moved, v, params)
        # Check the warm answer by where it lands, not by angle: near max range or up close, valid
        # answers can differ by >0.01° while both land within tolerance.
        if sol.converged:
            tr = simulate(launch_velocity(v, sol.pitch_deg, sol.yaw_deg), params.flight, moved.range_m,
                          params.solver.dt_s)  # fmt: skip
            miss_m = np.hypot(tr.impact_y_m - moved.lateral_m, tr.impact_z_m - moved.height_m)
            mismatch += not (miss_m < 2 * params.solver.tolerance_m)
        # Full per-frame engine cost: aim_error = warm solve + one simulation of the current aim.
        t0 = time.perf_counter()
        aim_error(ref.pitch_deg - 0.5, ref.yaw_deg + 0.3, params.bow.nominal_draw_m, moved, params, warm_start=p)
        loop_ms.append((time.perf_counter() - t0) * 1000)

    windy = params.with_flight(wind_mps=(0.0, 3.0, 0.0))
    wind_ms, wind_it = [], []
    for t in targets:
        t0 = time.perf_counter()
        sol = solve_aim(t, v, windy)
        wind_ms.append((time.perf_counter() - t0) * 1000)
        wind_it.append(sol.iterations)

    print(f"solve_aim timing: {params.preset_name}, {args.n} random targets, ranges {RANGE_M[0]:g}-{max_range_m:.1f} m,")
    print(f"dt {params.solver.dt_s * 1000:g} ms, tolerance {params.solver.tolerance_m * 1000:g} mm, this machine\n")
    summarize("cold", cold_ms, cold_it, failed)
    summarize("warm (50 Hz jitter)", warm_ms, warm_it, 0)
    summarize("aim_error, warm", loop_ms, [i + 1 for i in warm_it], 0)
    summarize("cold, 3 m/s crosswind", wind_ms, wind_it, 0)
    if mismatch:
        print(f"\nWARNING: {mismatch} warm solutions miss the target by more than 2x the tolerance")


if __name__ == "__main__":
    main()
