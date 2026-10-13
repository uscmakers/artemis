"""How far the impact point moves for each kind of sensor/estimate error.

For each range, the bow is aimed at the solved aim for a level target, then one thing
is perturbed at a time:
  +1° pitch error         bow pitched 1° higher than intended
  +1° yaw error           bow turned 1° left of intended
  1 cm short draw         arrow released 1 cm short of the draw the solver assumed
  +0.5 m range error      rangefinder reads 0.5 m long; solver aims for the wrong range
  2 m/s crosswind         wind toward +y (left) that the solver doesn't know about

Run: python -m explore.sensitivity [--preset toy_suction] [--no-show]
"""

from __future__ import annotations

import math

import matplotlib.pyplot as plt

from artemis.physics.bow import launch_speed_mps
from artemis.physics.flight import launch_velocity, simulate
from artemis.solver.aim import Target, solve_aim
from explore._common import SERIES, TEXT_MUTED, finish, parse_args

RANGES_M = [5.0, 10.0, 15.0, 20.0, 30.0]
PITCH_ERR_DEG = 1.0
YAW_ERR_DEG = 1.0
SHORT_DRAW_M = 0.01
RANGE_ERR_M = 0.5
CROSSWIND_MPS = 2.0

FACE_RADIUS_M = 0.40  # 80 cm face
RING_WIDTH_M = 0.04  # 10 rings on an 80 cm face

SOURCES = [
    ("+1° pitch", "°"),
    ("+1° yaw", "°"),
    ("1 cm short draw", "cm"),
    ("+0.5 m range", "m"),
    ("2 m/s crosswind", "m/s"),
]
SOURCE_SIZE = [PITCH_ERR_DEG, YAW_ERR_DEG, SHORT_DRAW_M * 100, RANGE_ERR_M, CROSSWIND_MPS]


def impact_shift_cm(range_m: float, params) -> dict[str, tuple[float, float]] | None:
    """Return {source: (dy_cm, dz_cm)} for one range, or None if the range is out of reach."""
    dt = params.solver.dt_s
    draw_m = params.bow.nominal_draw_m
    v = launch_speed_mps(draw_m, params.bow)
    sol = solve_aim(Target(range_m), v, params)
    if not sol.converged:
        return None

    def impact(v_mps, pitch, yaw, flight=params.flight):
        tr = simulate(launch_velocity(v_mps, pitch, yaw), flight, range_m, dt)
        return (tr.impact_y_m, tr.impact_z_m) if tr.hit_target_plane else (math.nan, -math.inf)

    base_y, base_z = impact(v, sol.pitch_deg, sol.yaw_deg)
    wrong_range = solve_aim(Target(range_m + RANGE_ERR_M), v, params)
    windy = params.with_flight(wind_mps=(0.0, CROSSWIND_MPS, 0.0)).flight
    cases = {
        "+1° pitch": impact(v, sol.pitch_deg + PITCH_ERR_DEG, sol.yaw_deg),
        "+1° yaw": impact(v, sol.pitch_deg, sol.yaw_deg + YAW_ERR_DEG),
        "1 cm short draw": impact(launch_speed_mps(draw_m - SHORT_DRAW_M, params.bow), sol.pitch_deg, sol.yaw_deg),
        "+0.5 m range": impact(v, wrong_range.pitch_deg, wrong_range.yaw_deg),
        "2 m/s crosswind": impact(v, sol.pitch_deg, sol.yaw_deg, windy),
    }
    return {k: ((y - base_y) * 100, (z - base_z) * 100) for k, (y, z) in cases.items()}


def main() -> None:
    args, params = parse_args(__doc__)
    v = launch_speed_mps(params.bow.nominal_draw_m, params.bow)
    results = {r: impact_shift_cm(r, params) for r in RANGES_M}
    names = [s for s, _ in SOURCES]

    print(f"Impact shift (cm) per error source, {params.preset_name} at {v:.1f} m/s, level target")
    print("(signed: +y = left, +z = up; |total| in brackets)\n")
    print(f"{'range':>6} " + " ".join(f"{n:>22}" for n in names))
    for r, res in results.items():
        if res is None:
            print(f"{r:5.0f}m   out of reach")
            continue
        cells = []
        for n in names:
            dy, dz = res[n]
            main_axis = f"z {dz:+.1f}" if abs(dz) >= abs(dy) else f"y {dy:+.1f}"
            cells.append(f"{main_axis} [{math.hypot(dy, dz):.1f}]")
        print(f"{r:5.0f}m " + " ".join(f"{c:>22}" for c in cells))

    print("\nError budget: how big can each error be (alone) before the arrow moves")
    print(f"one ring ({RING_WIDTH_M * 100:.0f} cm) / off the 80 cm face ({FACE_RADIUS_M * 100:.0f} cm)? Linear estimate.\n")
    print(f"{'range':>6} " + " ".join(f"{n:>22}" for n in names))
    for r, res in results.items():
        if res is None:
            continue
        cells = []
        for (n, unit), size in zip(SOURCES, SOURCE_SIZE):
            per_unit_cm = math.hypot(*res[n]) / size
            ring = RING_WIDTH_M * 100 / per_unit_cm
            face = FACE_RADIUS_M * 100 / per_unit_cm
            cells.append(f"{ring:.2f} / {face:.1f} {unit}")
        print(f"{r:5.0f}m " + " ".join(f"{c:>22}" for c in cells))

    fig, ax = plt.subplots(figsize=(9, 5.5))
    ok = [r for r in RANGES_M if results[r] is not None]
    ends = []
    for color, n in zip(SERIES, names):
        totals = [math.hypot(*results[r][n]) for r in ok]
        ax.plot(ok, totals, color=color, marker="o", markersize=5, label=n)
        ends.append([math.log10(totals[-1]), n])
    # Direct labels at the right end, nudged apart (in log space) so near-equal series don't collide.
    ends.sort()
    for i in range(1, len(ends)):
        ends[i][0] = max(ends[i][0], ends[i - 1][0] + 0.07)
    for log_y, n in ends:
        ax.annotate(n, (ok[-1], 10**log_y), xytext=(8, 0), textcoords="offset points", va="center",
                    color=TEXT_MUTED, fontsize=8.5)  # fmt: skip
    for level_cm, text in ((FACE_RADIUS_M * 100, "off the 80 cm face"), (RING_WIDTH_M * 100, "one ring (4 cm)")):
        ax.axhline(level_cm, color=TEXT_MUTED, linewidth=1, linestyle=":")
        ax.text(ok[0] - 0.8, level_cm, text, va="bottom", ha="left", color=TEXT_MUTED, fontsize=8)
    ax.set_yscale("log")
    ax.set_xlabel("range (m)")
    ax.set_ylabel("impact shift (cm, log scale)")
    ax.set_title(f"Impact shift per error source, {params.preset_name}")
    ax.legend(loc="lower right")
    ax.set_xlim(ok[0] - 1, ok[-1] + 7)
    fig.tight_layout()
    finish(fig, f"sensitivity_{params.preset_name}", args)


if __name__ == "__main__":
    main()
