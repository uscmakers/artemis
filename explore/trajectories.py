"""Side view (x vs z) of arrow arcs at several pitches, drag vs no drag, with the 15 m target.

Run: python -m explore.trajectories [--preset toy_suction] [--no-show]
"""

from __future__ import annotations

import matplotlib.pyplot as plt

from artemis.physics.bow import launch_speed_mps
from artemis.physics.flight import launch_velocity, simulate
from artemis.solver.aim import Target, solve_aim
from explore._common import SERIES, TEXT_MUTED, finish, parse_args

TARGET_RANGE_M = 15.0
FACE_DIAMETER_M = 0.80
FAR_X_M = 200.0  # simulate until the ground (or this far)


def main() -> None:
    args, params = parse_args(__doc__)
    v = launch_speed_mps(params.bow.nominal_draw_m, params.bow)
    vacuum = params.without_drag()
    sol = solve_aim(Target(TARGET_RANGE_M), v, params)
    pitches = [0.0, 5.0, 10.0, 20.0]
    if sol.converged:
        pitches.insert(1, round(sol.pitch_deg, 2))

    fig, (ax_full, ax_zoom) = plt.subplots(2, 1, figsize=(10, 8), gridspec_kw={"height_ratios": [1, 1]})
    for color, pitch in zip(SERIES, pitches):
        v0 = launch_velocity(v, pitch, 0.0)
        drag = simulate(v0, params.flight, FAR_X_M, params.solver.dt_s)
        vac = simulate(v0, vacuum.flight, FAR_X_M, params.solver.dt_s)
        label = f"{pitch:g}°" + (" (solved for 15 m)" if sol.converged and pitch == round(sol.pitch_deg, 2) else "")
        for ax in (ax_full, ax_zoom):
            ax.plot(drag.x, drag.z, color=color, label=f"{label}, drag")
            ax.plot(vac.x, vac.z, color=color, linestyle="--", linewidth=1.2, alpha=0.8, label=f"{label}, vacuum")
        ax_full.annotate(
            f"{pitch:g}°", (drag.x[-1], drag.z[-1]), textcoords="offset points", xytext=(4, 4), color=TEXT_MUTED
        )

    ground_z = -params.flight.launch_height_m
    for ax in (ax_full, ax_zoom):
        ax.axhline(ground_z, color=TEXT_MUTED, linewidth=1.0)
        ax.plot([TARGET_RANGE_M] * 2, [-FACE_DIAMETER_M / 2, FACE_DIAMETER_M / 2], color="#e34948", linewidth=6,
                solid_capstyle="butt", zorder=5)  # fmt: skip
        ax.set_xlabel("downrange x (m)")
        ax.set_ylabel("height above nock z (m)")

    ax_full.set_title(f"Arrow arcs, {params.preset_name} at {v:.1f} m/s launch (solid = drag, dashed = vacuum)")
    ax_full.set_ylim(ground_z - 0.5, None)
    ax_zoom.set_title(f"Zoom to the {TARGET_RANGE_M:g} m target (80 cm face shown in red)")
    ax_zoom.set_xlim(0, TARGET_RANGE_M + 2)
    ax_zoom.set_ylim(ground_z - 0.2, 1.2)
    ax_zoom.legend(ncol=2, fontsize=8, loc="upper left")
    fig.tight_layout()
    finish(fig, f"trajectories_{params.preset_name}", args)


if __name__ == "__main__":
    main()
