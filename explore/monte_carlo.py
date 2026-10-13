"""Monte Carlo grouping: the archer follows the cue perfectly, but the sensors are noisy.

Per shot:
  1. Sensors measure range and draw with noise; the solver computes the required aim
     from those measurements.
  2. The archer holds until the *measured* pitch/yaw equal the required aim, so the
     *true* pitch/yaw are off by the IMU/camera attitude noise.
  3. The arrow flies with the true draw and true attitude to the true target.

Run: python -m explore.monte_carlo [--preset toy_suction] [--range 20] [--shots 2000] [--no-show]
"""

from __future__ import annotations

import math

import matplotlib.pyplot as plt
import numpy as np

from artemis.physics.bow import launch_speed_mps
from artemis.physics.flight import launch_velocity, simulate
from artemis.solver.aim import Target, solve_aim
from explore._common import SERIES, TEXT_MUTED, draw_target_face, finish, parse_args

# Sensor noise, 1 sigma. Change these to trade sensor accuracy against group size.
SIGMA_PITCH_DEG = 0.3
SIGMA_YAW_DEG = 0.3
SIGMA_DRAW_M = 0.005
SIGMA_RANGE_M = 0.3

FACE_DIAMETER_M = 0.80
RINGS = 10
SEED = 1
BREAKDOWN_SHOTS = 500  # shots per single-source run in the breakdown table


def run(params, range_m, n_shots, sig_pitch, sig_yaw, sig_draw, sig_range, rng):
    """Return ((n_shots, 2) impact offsets from target center as (y_left_m, z_up_m), n_draw_clamped)."""
    dt = params.solver.dt_s
    true_draw_m = params.bow.nominal_draw_m
    v_true = launch_speed_mps(true_draw_m, params.bow)
    curve_max = params.bow.curve.max_draw_m
    out = np.full((n_shots, 2), np.nan)
    warm = None
    n_clamped = 0
    for i in range(n_shots):
        meas_range = range_m + rng.normal(0.0, sig_range)
        meas_draw = true_draw_m + rng.normal(0.0, sig_draw)
        if meas_draw > curve_max:
            # The bow model clamps past the last force-draw point, so the solver under-estimates
            # speed for these readings and aims high. Clamp here explicitly to count it.
            meas_draw = curve_max
            n_clamped += 1
        sol = solve_aim(Target(meas_range), launch_speed_mps(meas_draw, params.bow), params, warm_start=warm)
        if not sol.converged:
            continue
        warm = sol
        pitch = sol.pitch_deg + rng.normal(0.0, sig_pitch)
        yaw = sol.yaw_deg + rng.normal(0.0, sig_yaw)
        tr = simulate(launch_velocity(v_true, pitch, yaw), params.flight, range_m, dt)
        if tr.hit_target_plane:
            out[i] = tr.impact_y_m, tr.impact_z_m
    return out, n_clamped


def ring_stats(impacts):
    """(mean radius m, list of cumulative % inside ring k for k = 10 (center) .. 1, % on face)."""
    r = np.hypot(impacts[:, 0], impacts[:, 1])
    r = np.where(np.isnan(r), np.inf, r)
    ring_w = FACE_DIAMETER_M / 2 / RINGS
    inside = [100.0 * np.mean(r <= ring_w * k) for k in range(1, RINGS + 1)]  # inside 10-ring, 9-ring, ...
    finite = r[np.isfinite(r)]
    return (finite.mean() if finite.size else math.nan), inside


def main() -> None:
    def extra(p):
        p.add_argument("--range", type=float, default=15.0, help="target range in m (default 15)")
        p.add_argument("--shots", type=int, default=2000)

    args, params = parse_args(__doc__, extra)
    rng = np.random.default_rng(SEED)
    sigmas = dict(sig_pitch=SIGMA_PITCH_DEG, sig_yaw=SIGMA_YAW_DEG, sig_draw=SIGMA_DRAW_M, sig_range=SIGMA_RANGE_M)
    impacts, n_clamped = run(params, args.range, args.shots, rng=rng, **sigmas)

    mean_r, inside = ring_stats(impacts)
    print(f"Monte Carlo: {params.preset_name}, target {args.range:g} m, {args.shots} shots")
    print(
        f"sensor sigmas: pitch {SIGMA_PITCH_DEG}°, yaw {SIGMA_YAW_DEG}°, draw {SIGMA_DRAW_M * 1000:g} mm, "
        f"range {SIGMA_RANGE_M} m\n"
    )
    print(f"mean radius:  {mean_r * 100:.1f} cm")
    print(f"std (y, z):   {np.nanstd(impacts[:, 0]) * 100:.1f} cm, {np.nanstd(impacts[:, 1]) * 100:.1f} cm")
    print(f"mean (y, z):  {np.nanmean(impacts[:, 0]) * 100:+.1f} cm, {np.nanmean(impacts[:, 1]) * 100:+.1f} cm")
    if n_clamped:
        print(
            f"\nnote: {100 * n_clamped / args.shots:.0f}% of draw readings were past the last force-draw point and"
            "\n      were clamped, which biases the aim high. Measure the force-draw curve past full draw."
        )
    print("\nscore ring  radius   % inside")
    for k, pct in enumerate(inside, start=1):
        print(f"   {11 - k:>3}     {k * FACE_DIAMETER_M / 2 / RINGS * 100:4.0f} cm   {pct:6.1f}%")

    print(f"\nBreakdown: each noise source alone ({BREAKDOWN_SHOTS} shots each)")
    print(f"{'source':>8} {'mean radius':>12} {'% in 10':>8} {'% in gold (9+)':>15} {'% on face':>10}")
    for name in sigmas:
        alone = {k: (v if k == name else 0.0) for k, v in sigmas.items()}
        m, ins = ring_stats(run(params, args.range, BREAKDOWN_SHOTS, rng=rng, **alone)[0])
        print(f"{name[4:]:>8} {m * 100:10.1f} cm {ins[0]:7.1f}% {ins[1]:14.1f}% {ins[-1]:9.1f}%")

    fig, ax = plt.subplots(figsize=(7, 7.4))
    draw_target_face(ax, FACE_DIAMETER_M, RINGS, unit_scale=100)
    # Viewed from the archer: +y (left) plots to the left, so x axis = -y.
    ax.scatter(-impacts[:, 0] * 100, impacts[:, 1] * 100, s=12, color=SERIES[6], edgecolors="#ffffff", linewidths=0.5, alpha=0.8, zorder=3)
    lim = max(FACE_DIAMETER_M / 2 * 100 * 1.1, np.nanmax(np.abs(impacts)) * 100 * 1.05)
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect("equal")
    ax.grid(False)
    ax.set_xlabel("right of center (cm), archer's view")
    ax.set_ylabel("above center (cm)")
    ax.set_title(f"{args.shots} shots at {args.range:g} m, {params.preset_name}")
    ax.text(
        0.0, -0.12,
        f"σ pitch {SIGMA_PITCH_DEG}°, yaw {SIGMA_YAW_DEG}°, draw {SIGMA_DRAW_M * 1000:g} mm, range {SIGMA_RANGE_M} m"
        f"   ·   mean radius {mean_r * 100:.1f} cm, {inside[1]:.0f}% in gold, {inside[-1]:.0f}% on face",
        transform=ax.transAxes, color=TEXT_MUTED, fontsize=8.5,
    )  # fmt: skip
    fig.tight_layout()
    finish(fig, f"monte_carlo_{params.preset_name}_{args.range:g}m", args)


if __name__ == "__main__":
    main()
