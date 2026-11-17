"""Interactive explorer: drag sliders, watch the arrow and the aim cue update live.

Panels: side view, top view, the predicted hit on the target face, and the aim cue the
archer would hear. Blue = where the bow points now; dashed gray = the aim that hits center.

Keys: up/down nudge pitch, left/right nudge yaw, space snaps to the solution, F toggles fullscreen.

Run: python -m explore.explorer [--preset toy_suction] [--dark]
(--no-show saves a snapshot and prints the update time instead of opening a window.)
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle
from matplotlib.widgets import Button, Slider

from artemis.config import M_PER_IN, Params, load_config, preset_names
from artemis.physics.bow import launch_speed_mps
from artemis.physics.flight import launch_velocity, simulate
from artemis.solver.aim import Target, aim_error, solve_aim
from explore._common import draw_target_face, finish, parse_args

FACE_DIAMETER_M = 0.80
DEADBAND_DEG = 0.3  # cue "hold" zone from the design notes
RANGE_LIMITS_M = (3.0, 50.0)
DEFAULT_RANGE_M = 15.0
START_OFFSET_DEG = 0.5  # open slightly low so there's something to correct
YAW_LIMITS_DEG = (-5.0, 5.0)
WIND_LIMITS_MPS = (-5.0, 5.0)
CD_LIMITS = (0.0, 4.0)
NUDGE_DEG = 0.05
FIG_SIZE_IN = (15.0, 9.0)

# One accent (the current shot). Everything else is ink and grays; the target keeps its real ring colors.
THEMES = {
    "light": dict(bg="#ffffff", ink="#111111", ink2="#555555", ink3="#8a8a8a", rule="#dddddd", grid="#eeeeee",
                  accent="#1257c4"),
    "dark": dict(bg="#111111", ink="#eeeeee", ink2="#aaaaaa", ink3="#777777", rule="#333333", grid="#1e1e1e",
                 accent="#5b9ae6"),
}  # fmt: skip
# The face seen edge-on, outermost band first: (half-height in m, ring color).
FACE_BANDS = [(0.40, "#f2f2f2"), (0.32, "#1b1b1b"), (0.24, "#41b7e6"), (0.16, "#e2231a"), (0.08, "#ffe14f")]
FONTS = ["Helvetica Neue", "Helvetica", "Arial", "DejaVu Sans"]

# Layout in figure fractions: scene on the left, cue + face on the right, controls along the bottom.
SIDE_RECT = [0.055, 0.56, 0.58, 0.285]
TOP_RECT = [0.055, 0.33, 0.58, 0.16]
FACE_RECT = [0.695, 0.465, 0.25, 0.265]
RIGHT_X = 0.695


@dataclass
class PresetView:
    """Slider limits and starting values for one preset."""

    params: Params
    draw_lo_in: float
    draw_hi_in: float
    draw_nom_in: float
    pitch_lo_deg: float
    pitch_hi_deg: float
    start_pitch_deg: float


def preset_view(params: Params) -> PresetView:
    curve = params.bow.curve
    v = launch_speed_mps(params.bow.nominal_draw_m, params.bow)
    far = solve_aim(Target(RANGE_LIMITS_M[1]), v, params)
    start = solve_aim(Target(DEFAULT_RANGE_M), v, params)
    return PresetView(
        params=params,
        draw_lo_in=round(curve.brace_m / M_PER_IN + 1.0, 1),
        draw_hi_in=round(curve.max_draw_m / M_PER_IN, 1),
        draw_nom_in=round(params.bow.nominal_draw_m / M_PER_IN, 1),
        pitch_lo_deg=-5.0,
        pitch_hi_deg=min(45.0, math.ceil(far.pitch_deg + 5.0)) if far.converged else 45.0,
        start_pitch_deg=round(start.pitch_deg - START_OFFSET_DEG, 2) if start.converged else 0.0,
    )


def preset_label(name: str) -> str:
    return {"recurve_30lb": "Recurve 30 lb", "toy_suction": "Toy, suction cups"}.get(name, name.replace("_", " "))


def apply_theme(c: dict) -> None:
    for key in ("keymap.back", "keymap.forward", "keymap.save", "keymap.pan", "keymap.zoom", "keymap.home",
                "keymap.grid", "keymap.grid_minor", "keymap.xscale", "keymap.yscale"):  # fmt: skip
        matplotlib.rcParams[key] = []  # free the arrow keys and space for the explorer
    matplotlib.rcParams.update(
        {
            "toolbar": "None",
            "font.family": FONTS,
            "figure.facecolor": c["bg"],
            "axes.facecolor": c["bg"],
            "axes.edgecolor": c["rule"],
            "axes.labelcolor": c["ink2"],
            "axes.grid": True,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "grid.color": c["grid"],
            "grid.linewidth": 0.8,
            "xtick.color": c["ink3"],
            "ytick.color": c["ink3"],
            "xtick.labelcolor": c["ink2"],
            "ytick.labelcolor": c["ink2"],
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
            "axes.labelsize": 10,
            "text.color": c["ink"],
        }
    )


def main() -> None:
    def extra(p):
        p.add_argument("--dark", action="store_true", help="near-black background")

    args, first_params = parse_args(__doc__, extra)
    c = THEMES["dark" if args.dark else "light"]
    apply_theme(c)
    names = preset_names(args.config)
    views = {first_params.preset_name: preset_view(first_params)}

    def view_for(name: str) -> PresetView:
        if name not in views:
            views[name] = preset_view(load_config(args.config, name))
        return views[name]

    fig = plt.figure(figsize=FIG_SIZE_IN)
    if fig.canvas.manager is not None:
        fig.canvas.manager.set_window_title("Artemis aim explorer")

    # ---------- Header ----------
    fig.text(0.055, 0.952, "Artemis aim explorer", fontsize=20, fontweight="bold", va="center")
    subtitle = fig.text(0.055, 0.917, "", fontsize=11.5, color=c["ink2"], va="center")

    # ---------- Side view ----------
    ax_side = fig.add_axes(SIDE_RECT)
    ground_line = ax_side.axhline(0, color=c["ink3"], linewidth=1.2, zorder=1)
    ground_label = ax_side.text(0.0, 0.0, "ground", fontsize=9, color=c["ink3"], va="bottom")
    ax_side.plot([0], [0], marker="o", markersize=5, color=c["ink"], zorder=6)
    ax_side.annotate("nock", (0, 0), xytext=(0, -14), textcoords="offset points", ha="center", fontsize=9,
                     color=c["ink2"])  # fmt: skip
    side_stand = ax_side.plot([], [], color=c["ink3"], linewidth=1.5, zorder=2)[0]
    side_bands = [ax_side.plot([], [], color=col, linewidth=7, solid_capstyle="butt", zorder=4)[0] for _, col in FACE_BANDS]
    side_req = ax_side.plot([], [], color=c["ink3"], linewidth=1.5, linestyle=(0, (5, 4)), zorder=5)[0]
    side_cur = ax_side.plot([], [], color=c["accent"], linewidth=2.2, zorder=6)[0]
    apex_note = ax_side.annotate("", (0, 0), xytext=(0, 8), textcoords="offset points", ha="center", fontsize=9,
                                 color=c["ink2"], zorder=8)  # fmt: skip
    stretch_note = ax_side.text(1.0, 1.02, "", transform=ax_side.transAxes, ha="right", fontsize=9, color=c["ink3"])
    ax_side.set_title("Side view", loc="left", fontsize=13, color=c["ink"], pad=8)
    ax_side.set_ylabel("height above nock (m)")

    # ---------- Top view ----------
    ax_top = fig.add_axes(TOP_RECT, sharex=ax_side)
    top_bands = [ax_top.plot([], [], color=col, linewidth=7, solid_capstyle="butt", zorder=4)[0] for _, col in FACE_BANDS]
    top_req = ax_top.plot([], [], color=c["ink3"], linewidth=1.5, linestyle=(0, (5, 4)), zorder=5)[0]
    top_cur = ax_top.plot([], [], color=c["accent"], linewidth=2.2, zorder=6)[0]
    ax_top.plot([0], [0], marker="o", markersize=5, color=c["ink"], zorder=6)
    ax_top.set_title("Top view, archer's left is up", loc="left", fontsize=13, color=c["ink"], pad=8)
    ax_top.set_xlabel("distance downrange (m)")
    ax_top.set_ylabel("left (m)")

    # ---------- Cue ----------
    fig.text(RIGHT_X, 0.875, "Cue", fontsize=13, color=c["ink2"], va="center")
    cue_main = fig.text(RIGHT_X, 0.83, "", fontsize=30, fontweight="bold", va="center")
    cue_second = fig.text(RIGHT_X, 0.785, "", fontsize=18, va="center")
    cue_sub = fig.text(RIGHT_X, 0.752, "", fontsize=10, color=c["ink2"], va="center")

    # ---------- Target face ----------
    ax_face = fig.add_axes(FACE_RECT)
    draw_target_face(ax_face, FACE_DIAMETER_M, unit_scale=100)
    for x0, x1, y0, y1 in ((-2, 2, 0, 0), (0, 0, -2, 2)):
        ax_face.plot([x0, x1], [y0, y1], color="#000000", linewidth=0.8, zorder=2)
    hold_ring = Circle((0, 0), 1, fill=False, edgecolor="#000000", linewidth=1.2, linestyle=(0, (3, 2)), zorder=3)
    ax_face.add_patch(hold_ring)
    hit_dot = ax_face.plot([], [], marker="o", markersize=10, color=c["accent"], markeredgecolor="#ffffff",
                           markeredgewidth=1.5, zorder=5)[0]  # fmt: skip
    ax_face.set_xlim(-46, 46)
    ax_face.set_ylim(-46, 46)
    ax_face.set_aspect("equal")
    ax_face.set_axis_off()
    face_note = fig.text(RIGHT_X, 0.44, "", fontsize=12, fontweight="bold", va="center")
    fig.text(RIGHT_X, 0.415, f"Dashed circle: hold zone, ±{DEADBAND_DEG}° on both axes", fontsize=9.5,
             color=c["ink2"], va="center")  # fmt: skip

    # ---------- Numbers, as a plain two-column list ----------
    stat_labels = ["Launch speed", "Flight time", "Speed at target", "Solved aim", "Solver"]
    fig.text(RIGHT_X, 0.375, "\n".join(stat_labels), fontsize=10.5, color=c["ink2"], va="top", linespacing=1.45)
    stat_values = fig.text(0.945, 0.375, "", fontsize=10.5, va="top", ha="right", linespacing=1.45)

    # ---------- Controls ----------
    fig.add_artist(Line2D([0.055, 0.965], [0.235, 0.235], color=c["rule"], linewidth=1, transform=fig.transFigure))
    for x, label in ((0.055, "Bow"), (0.375, "Shot"), (RIGHT_X, "Preset")):
        fig.text(x, 0.205, label, fontsize=13, color=c["ink2"], va="center")

    v0 = views[first_params.preset_name]
    slider_specs = [
        # key, label, column x, row, lo, hi, init, step, format
        ("draw", "Draw", 0.12, 0, v0.draw_lo_in, v0.draw_hi_in, v0.draw_nom_in, 0.1, "%.1f in"),
        ("pitch", "Pitch", 0.12, 1, v0.pitch_lo_deg, v0.pitch_hi_deg, v0.start_pitch_deg, 0.01, "%.2f°"),
        ("yaw", "Yaw", 0.12, 2, *YAW_LIMITS_DEG, 0.3, 0.01, "%+.2f°"),
        ("range", "Distance", 0.44, 0, *RANGE_LIMITS_M, DEFAULT_RANGE_M, 0.1, "%.1f m"),
        ("wind", "Crosswind", 0.44, 1, *WIND_LIMITS_MPS, first_params.flight.wind_mps[1], 0.1, "%+.1f m/s"),
        ("cd", "Drag Cd", 0.44, 2, *CD_LIMITS, first_params.flight.drag_coefficient, 0.05, "%.2f"),
    ]
    sliders: dict[str, Slider] = {}
    for key, label, x, row, lo, hi, init, step, fmt in slider_specs:
        ax = fig.add_axes([x, 0.158 - row * 0.05, 0.18, 0.012], facecolor=c["bg"])
        sl = Slider(ax, label, lo, hi, valinit=init, valstep=step, valfmt=fmt, color=c["ink3"],
                    track_color=c["rule"], initcolor="none",
                    handle_style={"facecolor": c["bg"], "edgecolor": c["ink"], "size": 12})  # fmt: skip
        sl.poly.set_visible(False)  # just a line and a handle
        sl.label.set_color(c["ink"])
        sl.label.set_fontsize(11)
        sl.valtext.set_color(c["ink"])
        sl.valtext.set_fontsize(11)
        sl.valtext.set_x(1.05)
        sliders[key] = sl

    def plain_button(rect, label):
        ax = fig.add_axes(rect)
        b = Button(ax, label, color=c["bg"], hovercolor=c["grid"])
        b.label.set_color(c["ink"])
        b.label.set_fontsize(11)
        for spine in ax.spines.values():
            spine.set_visible(True)
            spine.set_edgecolor(c["ink3"])
            spine.set_linewidth(1)
        return b

    preset_buttons: dict[str, Button] = {}
    width = 0.27 / len(names)
    for i, name in enumerate(names):
        preset_buttons[name] = plain_button([RIGHT_X + i * width, 0.14, width - 0.008, 0.042], preset_label(name))
    snap_btn = plain_button([RIGHT_X, 0.08, 0.16, 0.042], "Snap to solution")
    reset_btn = plain_button([0.863, 0.08, 0.102, 0.042], "Reset")
    fig.text(RIGHT_X, 0.045, "Keys: up/down pitch, left/right yaw, space snaps, F fullscreen", fontsize=9.5,
             color=c["ink3"], va="center")  # fmt: skip

    state = {"name": first_params.preset_name, "warm": None, "sol": None, "ms": []}

    def place_face(bands, x, center, ground=None, stand=None):
        for line, (half, _) in zip(bands, FACE_BANDS):
            line.set_data([x, x], [center - half, center + half])
        if stand is not None:
            stand.set_data([x, x], [ground, center - FACE_BANDS[0][0]])

    def highlight_preset():
        for name, b in preset_buttons.items():
            active = name == state["name"]
            b.color = c["ink"] if active else c["bg"]
            b.hovercolor = c["ink"] if active else c["grid"]
            b.ax.set_facecolor(b.color)
            b.label.set_color(c["bg"] if active else c["ink"])

    # ---------- Update ----------
    def update(_=None):
        t0 = time.perf_counter()
        params = view_for(state["name"]).params
        s = {k: sl.val for k, sl in sliders.items()}
        p = params.with_flight(wind_mps=(0.0, s["wind"], 0.0), drag_coefficient=s["cd"])
        target = Target(s["range"])
        err = aim_error(s["pitch"], s["yaw"], s["draw"] * M_PER_IN, target, p, warm_start=state["warm"])
        sol = err.solution
        state["warm"] = sol if sol.converged else None
        state["sol"] = sol
        cur = err.current_trajectory
        req = None
        if sol.converged:
            req = simulate(launch_velocity(err.v_launch_mps, sol.pitch_deg, sol.yaw_deg), p.flight,
                           target.range_m, p.solver.dt_s)  # fmt: skip
        ms = (time.perf_counter() - t0) * 1000
        state["ms"].append(ms)

        # Side and top views
        ground_z = -p.flight.launch_height_m
        ground_line.set_ydata([ground_z, ground_z])
        side_cur.set_data(cur.x, cur.z)
        top_cur.set_data(cur.x, cur.y)
        side_req.set_data(*((req.x, req.z) if req else ([], [])))
        top_req.set_data(*((req.x, req.y) if req else ([], [])))
        place_face(side_bands, target.range_m, target.height_m, ground_z, side_stand)
        place_face(top_bands, target.range_m, target.lateral_m)

        x_max = max(target.range_m, float(cur.x[-1])) * 1.06
        z_all = list(cur.z) + (list(req.z) if req else [])
        z_lo, z_hi = ground_z - 0.25, max(max(z_all), FACE_BANDS[0][0]) + 0.3
        ax_side.set_xlim(-0.04 * x_max, x_max)
        ax_side.set_ylim(z_lo, z_hi)
        ground_label.set_position((0.01 * x_max, ground_z + 0.02 * (z_hi - z_lo)))
        y_span = max(0.6, max(abs(v) for v in list(cur.y) + (list(req.y) if req else [0.0])) + 0.25)
        ax_top.set_ylim(-y_span, y_span)
        fig_w, fig_h = FIG_SIZE_IN
        stretch = (1.04 * x_max / (SIDE_RECT[2] * fig_w)) / ((z_hi - z_lo) / (SIDE_RECT[3] * fig_h))
        stretch_note.set_text(f"Height is drawn {stretch:.0f}× taller than distance" if stretch >= 1.5 else "")

        i_apex = max(range(len(cur.z)), key=lambda i: cur.z[i])
        z_apex = cur.z[i_apex]
        if z_apex > 0.03 and 0 < i_apex < len(cur.z) - 1:
            apex_note.xy = (cur.x[i_apex], z_apex)
            apex_note.set_text(f"peak {z_apex * 100:.0f} cm" if z_apex < 1 else f"peak {z_apex:.1f} m")
        else:
            apex_note.set_text("")

        # Target face
        hold_r_cm = target.range_m * math.tan(math.radians(DEADBAND_DEG)) * 100
        hold_ring.set_radius(hold_r_cm)
        hold_ring.set_visible(hold_r_cm < 44)
        if cur.hit_target_plane:
            hx, hz = -err.predicted_impact_y_m * 100, err.predicted_impact_z_m * 100
            r = math.hypot(hx, hz)
            off = r > 43
            if off:
                hx, hz = hx * 43 / r, hz * 43 / r
            hit_dot.set_data([hx], [hz])
            hit_dot.set_markerfacecolor(c["bg"] if off else c["accent"])
            hit_dot.set_markeredgecolor(c["accent"] if off else "#ffffff")
            dz, dy = err.predicted_impact_z_m * 100, err.predicted_impact_y_m * 100
            where = f"{abs(dz):.1f} cm {'high' if dz > 0 else 'low'}, {abs(dy):.1f} cm {'left' if dy > 0 else 'right'}"
            face_note.set_text(("Off the face: " if off else "Lands ") + where)
        else:
            hit_dot.set_data([], [])
            face_note.set_text("Never reaches the target" + (", hits the ground" if cur.stop_reason == "ground" else ""))

        # Cue
        if not sol.converged:
            cue_main.set_text("Out of reach")
            cue_main.set_color(c["ink"])
            cue_second.set_text("")
            cue_sub.set_text(sol.reason.capitalize())
        else:
            pe, ye = err.pitch_err_deg, err.yaw_err_deg
            parts = []
            if abs(pe) >= DEADBAND_DEG:
                parts.append((abs(pe), f"{'Raise' if pe > 0 else 'Lower'} {abs(pe):.2f}°"))
            if abs(ye) >= DEADBAND_DEG:
                parts.append((abs(ye), f"Turn {'left' if ye > 0 else 'right'} {abs(ye):.2f}°"))
            parts.sort(reverse=True)
            if parts:
                cue_main.set_text(parts[0][1])
                cue_main.set_color(c["ink"])
                cue_second.set_text(parts[1][1] if len(parts) > 1 else "")
            else:
                cue_main.set_text("Hold")
                cue_main.set_color(c["accent"])
                cue_second.set_text("Both errors inside the hold zone")
            cue_sub.set_text(f"Pitch error {pe:+.2f}°, yaw error {ye:+.2f}°")

        # Numbers and subtitle
        if sol.converged:
            values = [
                f"{err.v_launch_mps:.1f} m/s",
                f"{sol.time_of_flight_s:.2f} s",
                f"{sol.impact_speed_mps:.1f} m/s",
                f"{sol.pitch_deg:.2f}° up, {sol.yaw_deg:+.2f}° yaw",
                f"{sol.iterations} runs, {ms:.0f} ms",
            ]
        else:
            values = [f"{err.v_launch_mps:.1f} m/s", "–", "–", "–", f"{sol.iterations} runs, {ms:.0f} ms"]
        stat_values.set_text("\n".join(values))
        drag = f"drag coefficient {s['cd']:.2f}" if s["cd"] > 0 else "no drag"
        subtitle.set_text(f"{preset_label(state['name'])}, {s['draw']:.1f} in draw, {drag}. "
                          "Blue is where the bow points now; dashed gray is the aim that hits the center.")  # fmt: skip
        fig.canvas.draw_idle()

    # ---------- Actions ----------
    def set_quietly(key, value, lo=None, hi=None):
        sl = sliders[key]
        sl.eventson = False
        if lo is not None:
            sl.valmin, sl.valmax = lo, hi
            sl.ax.set_xlim(lo, hi)
        sl.set_val(min(max(value, sl.valmin), sl.valmax))
        sl.eventson = True

    def load_preset(name):
        state["name"], state["warm"] = name, None
        v = view_for(name)
        set_quietly("draw", v.draw_nom_in, v.draw_lo_in, v.draw_hi_in)
        set_quietly("pitch", v.start_pitch_deg, v.pitch_lo_deg, v.pitch_hi_deg)
        set_quietly("yaw", 0.3)
        set_quietly("range", DEFAULT_RANGE_M)
        set_quietly("wind", v.params.flight.wind_mps[1])
        set_quietly("cd", v.params.flight.drag_coefficient)
        highlight_preset()
        update()

    def snap(_=None):
        sol = state["sol"]
        if sol is None or not sol.converged:
            return
        set_quietly("pitch", round(sol.pitch_deg, 2))
        set_quietly("yaw", round(sol.yaw_deg, 2))
        update()

    def nudge(key, delta):
        set_quietly(key, round(sliders[key].val + delta, 2))
        update()

    def on_key(event):
        actions = {
            "up": lambda: nudge("pitch", NUDGE_DEG),
            "down": lambda: nudge("pitch", -NUDGE_DEG),
            "left": lambda: nudge("yaw", NUDGE_DEG),  # yaw + is left
            "right": lambda: nudge("yaw", -NUDGE_DEG),
            " ": snap,
        }
        if event.key in actions:
            actions[event.key]()

    for sl in sliders.values():
        sl.on_changed(update)
    for name, b in preset_buttons.items():
        b.on_clicked(lambda _e, n=name: load_preset(n))
    snap_btn.on_clicked(snap)
    reset_btn.on_clicked(lambda _e: load_preset(state["name"]))
    fig.canvas.mpl_connect("key_press_event", on_key)
    highlight_preset()
    update()

    if args.no_show:
        # Headless smoke test: exercise every control and report how long an update takes.
        for r in (10.0, 30.0, DEFAULT_RANGE_M):
            sliders["range"].set_val(r)
        sliders["wind"].set_val(2.0)
        for key in ("up", "up", "left", " "):
            on_key(type("KeyEvent", (), {"key": key})())
        for name in names:
            load_preset(name)
        load_preset(first_params.preset_name)
        ms = state["ms"]
        print(f"{len(ms)} updates, mean {sum(ms) / len(ms):.1f} ms, worst {max(ms):.1f} ms (engine only, no redraw)")
    finish(fig, f"explorer_{first_params.preset_name}{'_dark' if args.dark else ''}", args)


if __name__ == "__main__":
    main()
