"""Sight tape: required pitch vs range (3-40 m), with and without drag.

Run: python -m explore.sight_tape [--preset toy_suction] [--no-show]
"""

from __future__ import annotations

import math

import matplotlib.pyplot as plt
import numpy as np

from artemis.physics.bow import launch_speed_mps
from artemis.solver.aim import Target, solve_aim
from explore._common import SERIES, finish, parse_args

RANGES_M = np.arange(3.0, 40.01, 0.5)
TABLE_RANGES_M = np.arange(5.0, 40.01, 5.0)


def solve_curve(ranges_m, v_mps, params):
    out, warm = [], None
    for r in ranges_m:
        sol = solve_aim(Target(float(r)), v_mps, params, warm_start=warm)
        out.append(sol)
        warm = sol if sol.converged else None
    return out


def main() -> None:
    args, params = parse_args(__doc__)
    v = launch_speed_mps(params.bow.nominal_draw_m, params.bow)
    vacuum = params.without_drag()

    drag = solve_curve(RANGES_M, v, params)
    vac = solve_curve(RANGES_M, v, vacuum)

    print(f"Sight tape: {params.preset_name}, launch {v:.1f} m/s, target level with the nock")
    print(f"{'range m':>8} {'pitch drag':>11} {'pitch vac':>10} {'diff':>7} {'TOF s':>7} {'impact m/s':>11}")
    for r in TABLE_RANGES_M:
        d = solve_aim(Target(float(r)), v, params)
        n = solve_aim(Target(float(r)), v, vacuum)
        if not d.converged:
            print(f"{r:8.0f}   out of reach ({d.reason})")
            continue
        print(
            f"{r:8.0f} {d.pitch_deg:10.2f}° {n.pitch_deg:9.2f}° {d.pitch_deg - n.pitch_deg:6.2f}°"
            f" {d.time_of_flight_s:7.3f} {d.impact_speed_mps:11.1f}"
        )

    fig, (ax, ax_diff) = plt.subplots(2, 1, figsize=(9, 7), sharex=True, gridspec_kw={"height_ratios": [2, 1]})
    pd = np.array([s.pitch_deg if s.converged else math.nan for s in drag])
    pv = np.array([s.pitch_deg if s.converged else math.nan for s in vac])
    ax.plot(RANGES_M, pd, color=SERIES[0], label="with drag")
    ax.plot(RANGES_M, pv, color=SERIES[1], linestyle="--", label="vacuum")
    ax.set_ylabel("required pitch (°)")
    ax.set_title(f"Sight tape, {params.preset_name} ({v:.1f} m/s)")
    ax.legend(loc="upper left")

    ax_diff.plot(RANGES_M, pd - pv, color=SERIES[0])
    ax_diff.set_ylabel("drag adds (°)")
    ax_diff.set_xlabel("range (m)")
    fig.tight_layout()
    finish(fig, f"sight_tape_{params.preset_name}", args)


if __name__ == "__main__":
    main()
