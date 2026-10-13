"""Interactive explorer: drag sliders, watch the arrow and the aim error update live.

Panels: side view (x vs z), top view (x vs y), and the impact point on the target face.
Solid line = current aim, dashed = the aim the solver says hits the center.
"Snap to solution" sets pitch/yaw to the solved aim.

Run: python -m explore.explorer [--preset toy_suction]
(--no-show saves a snapshot and prints the update time instead of opening a window.)
"""

from __future__ import annotations

import math
import time

import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider

from artemis.config import M_PER_IN
from artemis.physics.bow import launch_speed_mps
from artemis.physics.flight import launch_velocity, simulate
from artemis.solver.aim import Target, aim_error, solve_aim
from explore._common import SERIES, TEXT, TEXT_MUTED, draw_target_face, finish, parse_args

FACE_DIAMETER_M = 0.80
RANGE_LIMITS_M = (3.0, 50.0)
DEFAULT_RANGE_M = 15.0
YAW_LIMITS_DEG = (-5.0, 5.0)
WIND_LIMITS_MPS = (-5.0, 5.0)
CD_LIMITS = (0.0, 4.0)


def describe(err_deg: float, pos: str, neg: str) -> str:
    if math.isnan(err_deg):
        return "n/a"
    return f"{pos if err_deg > 0 else neg} {abs(err_deg):.2f}°"


def main() -> None:
    args, params = parse_args(__doc__)
    curve = params.bow.curve
    v_nom = launch_speed_mps(params.bow.nominal_draw_m, params.bow)
    far = solve_aim(Target(RANGE_LIMITS_M[1]), v_nom, params)
    pitch_max = min(45.0, far.pitch_deg + 5.0) if far.converged else 45.0
    start = solve_aim(Target(DEFAULT_RANGE_M), v_nom, params)
    start_pitch = round(start.pitch_deg - 0.6, 2) if start.converged else 0.0  # start a bit low so there's an error

    fig = plt.figure(figsize=(13, 8.5))
    fig.canvas.manager.set_window_title(f"Artemis explorer — {params.preset_name}")
    ax_side = fig.add_axes([0.06, 0.66, 0.56, 0.25])
    ax_top = fig.add_axes([0.06, 0.35, 0.56, 0.22])
    ax_face = fig.add_axes([0.67, 0.35, 0.30, 0.56])
    ax_text = fig.add_axes([0.67, 0.03, 0.30, 0.26])
    ax_text.axis("off")

    slider_specs = [
        ("draw", "draw (in)", curve.brace_m / M_PER_IN + 1.0, curve.max_draw_m / M_PER_IN,
         params.bow.nominal_draw_m / M_PER_IN, 0.1),
        ("pitch", "pitch (°)", -5.0, pitch_max, start_pitch, 0.01),
        ("yaw", "yaw (°)", *YAW_LIMITS_DEG, 0.3, 0.01),
        ("range", "range (m)", *RANGE_LIMITS_M, DEFAULT_RANGE_M, 0.1),
        ("wind", "crosswind (m/s, +left)", *WIND_LIMITS_MPS, params.flight.wind_mps[1], 0.1),
        ("cd", "drag coeff Cd", *CD_LIMITS, params.flight.drag_coefficient, 0.05),
    ]  # fmt: skip
    sliders: dict[str, Slider] = {}
    for i, (key, label, lo, hi, init, step) in enumerate(slider_specs):
        ax = fig.add_axes([0.17, 0.265 - i * 0.037, 0.40, 0.022])
        sliders[key] = Slider(ax, label, lo, hi, valinit=init, valstep=step, color=SERIES[0])
    btn = Button(fig.add_axes([0.06, 0.012, 0.14, 0.035]), "Snap to solution")

    (side_cur,) = ax_side.plot([], [], color=SERIES[0], label="current aim")
    (side_req,) = ax_side.plot([], [], color=SERIES[1], linestyle="--", linewidth=1.5, label="solved aim")
    (side_tgt,) = ax_side.plot([], [], color=SERIES[7], linewidth=6, solid_capstyle="butt")
    ax_side.axhline(-params.flight.launch_height_m, color=TEXT_MUTED, linewidth=1)  # ground
    ax_side.set_title("Side view")
    ax_side.set_xlabel("downrange x (m)")
    ax_side.set_ylabel("height z (m)")
    ax_side.legend(loc="lower left", fontsize=8)

    (top_cur,) = ax_top.plot([], [], color=SERIES[0])
    (top_req,) = ax_top.plot([], [], color=SERIES[1], linestyle="--", linewidth=1.5)
    (top_tgt,) = ax_top.plot([], [], color=SERIES[7], linewidth=6, solid_capstyle="butt")
    ax_top.set_title("Top view (archer's left is up)")
    ax_top.set_xlabel("downrange x (m)")
    ax_top.set_ylabel("left y (m)")

    draw_target_face(ax_face, FACE_DIAMETER_M, unit_scale=100)
    (face_hit,) = ax_face.plot([], [], marker="X", markersize=14, color=SERIES[6], markeredgecolor="white",
                               markeredgewidth=1.5, linestyle="none", zorder=5)  # fmt: skip
    ax_face.set_aspect("equal")
    ax_face.grid(False)
    ax_face.set_title("Predicted impact (archer's view)")
    ax_face.set_xlabel("right (cm)")
    ax_face.set_ylabel("up (cm)")
    info = ax_text.text(0.0, 1.0, "", va="top", ha="left", family="monospace", fontsize=9, color=TEXT)

    state = {"warm": None, "ms": []}

    def update(_=None):
        t0 = time.perf_counter()
        s = {k: sl.val for k, sl in sliders.items()}
        p = params.with_flight(wind_mps=(0.0, s["wind"], 0.0), drag_coefficient=s["cd"])
        target = Target(s["range"])
        draw_m = s["draw"] * M_PER_IN
        err = aim_error(s["pitch"], s["yaw"], draw_m, target, p, warm_start=state["warm"])
        sol = err.solution
        state["warm"] = sol if sol.converged else None
        cur = err.current_trajectory

        side_cur.set_data(cur.x, cur.z)
        top_cur.set_data(cur.x, cur.y)
        if sol.converged:
            req = simulate(launch_velocity(err.v_launch_mps, sol.pitch_deg, sol.yaw_deg), p.flight,
                           target.range_m, p.solver.dt_s)  # fmt: skip
            side_req.set_data(req.x, req.z)
            top_req.set_data(req.x, req.y)
        else:
            side_req.set_data([], [])
            top_req.set_data([], [])
        half = FACE_DIAMETER_M / 2
        side_tgt.set_data([target.range_m] * 2, [target.height_m - half, target.height_m + half])
        top_tgt.set_data([target.range_m] * 2, [target.lateral_m - half, target.lateral_m + half])

        x_max = max(target.range_m, float(cur.x[-1])) * 1.05
        zs = list(cur.z) + list(side_req.get_ydata()) + [-params.flight.launch_height_m, half]
        ys = list(cur.y) + list(top_req.get_ydata()) + [-half, half]
        ax_side.set_xlim(0, x_max)
        ax_side.set_ylim(min(zs) - 0.2, max(zs) + 0.2)
        ax_top.set_xlim(0, x_max)
        ax_top.set_ylim(min(ys) - 0.2, max(ys) + 0.2)

        if cur.hit_target_plane:
            hx, hz = -err.predicted_impact_y_m * 100, err.predicted_impact_z_m * 100
            face_hit.set_data([hx], [hz])
            lim = max(half * 100 * 1.15, abs(hx) * 1.15, abs(hz) * 1.15)
            impact_line = f"impact     {hx:+6.1f} cm right, {hz:+6.1f} cm up"
            miss_line = f"miss       {err.predicted_miss_m * 100:6.1f} cm"
        else:
            face_hit.set_data([], [])
            lim = half * 100 * 1.15
            impact_line = f"impact     never reaches target ({cur.stop_reason})"
            miss_line = ""
        ax_face.set_xlim(-lim, lim)
        ax_face.set_ylim(-lim, lim)

        ms = (time.perf_counter() - t0) * 1000
        state["ms"].append(ms)
        if sol.converged:
            sol_lines = [
                f"solved     pitch {sol.pitch_deg:6.2f}°  yaw {sol.yaw_deg:+6.2f}°",
                f"error      {describe(err.pitch_err_deg, 'raise', 'lower'):>14}  "
                f"{describe(err.yaw_err_deg, 'left', 'right')}",
                f"flight     {sol.time_of_flight_s:.3f} s, impact {sol.impact_speed_mps:.1f} m/s",
            ]
        else:
            sol_lines = [f"solver     {sol.reason}"]
        info.set_text(
            "\n".join(
                [
                    f"launch     {err.v_launch_mps:5.1f} m/s  ({s['draw']:.1f} in draw)",
                    *sol_lines,
                    impact_line,
                    miss_line,
                    f"compute    {ms:5.1f} ms ({sol.iterations} sims)",
                ]
            )
        )
        fig.canvas.draw_idle()

    def snap(_event):
        sol = state["warm"]
        if sol is None:
            return
        sliders["pitch"].set_val(round(sol.pitch_deg, 2))
        sliders["yaw"].set_val(round(sol.yaw_deg, 2))

    for sl in sliders.values():
        sl.on_changed(update)
    btn.on_clicked(snap)
    fig.text(0.06, 0.97, f"Artemis explorer — {params.preset_name}", fontsize=13, fontweight="bold", color=TEXT)
    update()

    if args.no_show:
        # Headless smoke test: sweep a few sliders and report how long an update takes.
        for r in (10.0, 20.0, 30.0):
            sliders["range"].set_val(r)
        sliders["wind"].set_val(2.0)
        sliders["range"].set_val(DEFAULT_RANGE_M)
        snap(None)
        sliders["pitch"].set_val(sliders["pitch"].val - 0.5)
        ms = state["ms"]
        print(f"{len(ms)} updates, mean {sum(ms) / len(ms):.1f} ms, worst {max(ms):.1f} ms (engine only, no redraw)")
    finish(fig, f"explorer_{params.preset_name}", args)


if __name__ == "__main__":
    main()
