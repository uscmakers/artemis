"""Interactive explorer: drag sliders, watch the arrow and the aim cue update live.

Panels: side view, top view, the predicted hit on the target face, and the aim cue the
archer would hear. Solid line = where the bow points now; dashed = the aim that hits center.

Keys: up/down nudge pitch, left/right nudge yaw, space snaps to the solution, F toggles fullscreen.

Run: python -m explore.explorer [--preset toy_suction] [--light]
(--no-show saves a snapshot and prints the update time instead of opening a window.)
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyBboxPatch
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

THEMES = {
    "dark": dict(
        bg="#0b111b", panel="#121b29", panel2="#1a2537", edge="#26344b", grid="#1a2535", text="#e8eef6",
        muted="#8696ae", current="#4ea8ff", solved="#f6c447", target="#ff5d52", hold="#3ddc84",
        ground="#16140f", ground_line="#4a3f2c",
    ),
    "light": dict(
        bg="#f3f5f9", panel="#ffffff", panel2="#eef2f7", edge="#d3dbe6", grid="#e4e9f0", text="#132033",
        muted="#5a6a80", current="#1f6fd6", solved="#b67f00", target="#d63a2f", hold="#178a4c",
        ground="#ece5d7", ground_line="#b9a986",
    ),
}  # fmt: skip
# The face seen edge-on, outermost band first: (half-height in m, ring color).
FACE_BANDS = [(0.40, "#f4f4f4"), (0.32, "#1b1b1b"), (0.24, "#41b7e6"), (0.16, "#e2231a"), (0.08, "#ffe14f")]
FONTS = ["Avenir Next", "Helvetica Neue", "Helvetica", "Arial", "DejaVu Sans"]

# Layout in figure fractions: scene on the left, cue + face on the right, controls along the bottom.
SIDE_RECT = [0.055, 0.57, 0.575, 0.325]
TOP_RECT = [0.055, 0.335, 0.575, 0.17]
CUE_RECT = [0.675, 0.755, 0.29, 0.15]
FACE_RECT = [0.675, 0.405, 0.29, 0.335]
PANEL_RECT = (0.035, 0.025, 0.93, 0.225)


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


def apply_theme(c: dict) -> None:
    for key in ("keymap.back", "keymap.forward", "keymap.save", "keymap.pan", "keymap.zoom", "keymap.home",
                "keymap.grid", "keymap.grid_minor", "keymap.xscale", "keymap.yscale"):  # fmt: skip
        matplotlib.rcParams[key] = []  # free the arrow keys and space for the explorer
    matplotlib.rcParams.update(
        {
            "toolbar": "None",
            "font.family": FONTS,  # a list, so missing glyphs (arrows, triangles) fall back to DejaVu Sans
            "figure.facecolor": c["bg"],
            "axes.facecolor": c["panel"],
            "axes.edgecolor": c["edge"],
            "axes.labelcolor": c["muted"],
            "axes.grid": True,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "grid.color": c["grid"],
            "grid.linewidth": 0.8,
            "xtick.color": c["muted"],
            "ytick.color": c["muted"],
            "xtick.labelsize": 8.5,
            "ytick.labelsize": 8.5,
            "axes.labelsize": 9,
            "text.color": c["text"],
            "legend.frameon": False,
        }
    )


def main() -> None:
    def extra(p):
        p.add_argument("--light", action="store_true", help="light theme (for bright rooms and projectors)")

    args, first_params = parse_args(__doc__, extra)
    c = THEMES["light" if args.light else "dark"]
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
    fig.text(0.055, 0.952, "ARTEMIS", fontsize=19, fontweight="bold", color=c["text"], va="center")
    fig.text(0.158, 0.952, "aim explorer", fontsize=15, color=c["muted"], va="center")
    header_right = fig.text(0.965, 0.952, "", fontsize=11, color=c["muted"], va="center", ha="right")

    # ---------- Side view ----------
    ax_side = fig.add_axes(SIDE_RECT)
    ground_fill = ax_side.axhspan(-100, 0, color=c["ground"], zorder=0, linewidth=0)
    ground_line = ax_side.axhline(0, color=c["ground_line"], linewidth=1.2, zorder=1)
    ax_side.plot([0], [0], marker="o", markersize=7, color=c["text"], zorder=6)
    ax_side.annotate("nock", (0, 0), xytext=(0, -14), textcoords="offset points", ha="center", fontsize=8,
                     color=c["muted"])  # fmt: skip
    side_stand = ax_side.plot([], [], color=c["muted"], linewidth=2, zorder=2)[0]
    side_face_outline = ax_side.plot([], [], color=c["edge"], linewidth=10, solid_capstyle="butt", zorder=3)[0]
    side_bands = [ax_side.plot([], [], color=col, linewidth=8, solid_capstyle="butt", zorder=4)[0] for _, col in FACE_BANDS]
    side_req = ax_side.plot([], [], color=c["solved"], linewidth=1.8, linestyle=(0, (6, 4)), zorder=5,
                            label="aim that hits center")[0]  # fmt: skip
    side_glow = ax_side.plot([], [], color=c["current"], linewidth=8, alpha=0.18, solid_capstyle="round", zorder=5)[0]
    side_cur = ax_side.plot([], [], color=c["current"], linewidth=2.4, zorder=6, label="where the bow points now")[0]
    side_end = ax_side.plot([], [], marker="o", markersize=6, color=c["current"], zorder=7)[0]
    apex_note = ax_side.annotate("", (0, 0), xytext=(0, 10), textcoords="offset points", ha="center", fontsize=8.5,
                                 color=c["muted"], zorder=8)  # fmt: skip
    apex_dot = ax_side.plot([], [], marker="o", markersize=4, color=c["muted"], zorder=7)[0]
    stretch_note = ax_side.text(0.995, 0.03, "", transform=ax_side.transAxes, ha="right", fontsize=8, color=c["muted"])
    ax_side.text(0.008, 0.95, "SIDE VIEW", transform=ax_side.transAxes, fontsize=9, fontweight="bold",
                 color=c["muted"], va="top")  # fmt: skip
    ax_side.set_xlabel("distance downrange (m)")
    ax_side.set_ylabel("height above nock (m)")
    leg = fig.legend(handles=[side_cur, side_req], loc="center", bbox_to_anchor=(0.47, 0.952), ncol=2,
                     fontsize=10, handlelength=2.6, columnspacing=2.2)  # fmt: skip
    for t in leg.get_texts():
        t.set_color(c["text"])

    # ---------- Top view ----------
    ax_top = fig.add_axes(TOP_RECT, sharex=ax_side)
    top_face_outline = ax_top.plot([], [], color=c["edge"], linewidth=10, solid_capstyle="butt", zorder=3)[0]
    top_bands = [ax_top.plot([], [], color=col, linewidth=8, solid_capstyle="butt", zorder=4)[0] for _, col in FACE_BANDS]
    top_req = ax_top.plot([], [], color=c["solved"], linewidth=1.8, linestyle=(0, (6, 4)), zorder=5)[0]
    top_glow = ax_top.plot([], [], color=c["current"], linewidth=8, alpha=0.18, zorder=5)[0]
    top_cur = ax_top.plot([], [], color=c["current"], linewidth=2.4, zorder=6)[0]
    ax_top.plot([0], [0], marker="o", markersize=7, color=c["text"], zorder=6)
    ax_top.axhline(0, color=c["edge"], linewidth=1, zorder=1)
    ax_top.text(0.008, 0.92, "TOP VIEW  ·  archer's left is up", transform=ax_top.transAxes, fontsize=9,
                fontweight="bold", color=c["muted"], va="top")  # fmt: skip
    ax_top.set_xlabel("distance downrange (m)")
    ax_top.set_ylabel("left (m)")

    # ---------- Cue card ----------
    ax_cue = fig.add_axes(CUE_RECT)
    ax_cue.set_axis_off()
    ax_cue.add_patch(FancyBboxPatch((0, 0), 1, 1, boxstyle="round,pad=0,rounding_size=0.06",
                                    transform=ax_cue.transAxes, fc=c["panel"], ec=c["edge"], lw=1.2))  # fmt: skip
    cue_stripe = ax_cue.add_patch(FancyBboxPatch((0.0, 0.0), 0.018, 1, boxstyle="round,pad=0,rounding_size=0.009",
                                                 transform=ax_cue.transAxes, fc=c["current"], ec="none"))  # fmt: skip
    ax_cue.text(0.06, 0.85, "AIM CUE", fontsize=9, fontweight="bold", color=c["muted"], va="center")
    cue_sub = ax_cue.text(0.96, 0.85, "", fontsize=8.5, color=c["muted"], va="center", ha="right")
    cue_main = ax_cue.text(0.06, 0.52, "", fontsize=25, fontweight="bold", va="center")
    cue_second = ax_cue.text(0.06, 0.18, "", fontsize=15, fontweight="bold", va="center")

    # ---------- Target face ----------
    ax_face = fig.add_axes(FACE_RECT)
    ax_face.set_facecolor(c["bg"])
    draw_target_face(ax_face, FACE_DIAMETER_M, unit_scale=100)
    for x0, x1, y0, y1 in ((-2, 2, 0, 0), (0, 0, -2, 2)):
        ax_face.plot([x0, x1], [y0, y1], color="#000000", linewidth=0.8, zorder=2)
    hold_ring = Circle((0, 0), 1, fill=False, edgecolor=c["hold"], linewidth=1.6, linestyle=(0, (4, 3)), zorder=3)
    ax_face.add_patch(hold_ring)
    hit_glow = ax_face.plot([], [], marker="o", markersize=26, color=c["current"], alpha=0.25, zorder=4)[0]
    hit_dot = ax_face.plot([], [], marker="o", markersize=11, color=c["current"], markeredgecolor="#ffffff",
                           markeredgewidth=1.8, zorder=5)[0]  # fmt: skip
    face_note = ax_face.text(0, -51.5, "", ha="center", va="top", fontsize=10, fontweight="bold", color=c["text"])
    ax_face.text(0, -58, f"dashed green ring = hold zone (±{DEADBAND_DEG}°)", ha="center", va="top", fontsize=8.5,
                 color=c["hold"])  # fmt: skip
    ax_face.text(-50, 50, "WHERE IT LANDS", fontsize=9, fontweight="bold", color=c["muted"], va="top")
    ax_face.text(50, 50, "archer's view", fontsize=8, color=c["muted"], va="top", ha="right")
    ax_face.set_xlim(-50, 50)
    ax_face.set_ylim(-63, 52)
    ax_face.set_aspect("equal")
    ax_face.set_axis_off()

    # ---------- Stats ----------
    stat_specs = ["LAUNCH", "FLIGHT TIME", "AT TARGET", "SOLVED AIM", "MISS NOW", "COMPUTE"]
    stats = {}
    for i, label in enumerate(stat_specs):
        x = 0.675 + (i % 3) * 0.1
        y = 0.372 - (i // 3) * 0.054
        fig.text(x, y, label, fontsize=8, fontweight="bold", color=c["muted"])
        stats[label] = fig.text(x, y - 0.024, "", fontsize=12.5, fontweight="bold", color=c["text"])

    # ---------- Controls panel ----------
    fig.patches.append(FancyBboxPatch(PANEL_RECT[:2], *PANEL_RECT[2:], boxstyle="round,pad=0,rounding_size=0.008",
                                      transform=fig.transFigure, fc=c["panel"], ec=c["edge"], lw=1.2, zorder=-1))  # fmt: skip
    for x, label in ((0.05, "BOW"), (0.375, "SHOT"), (0.69, "PRESET")):
        fig.text(x, 0.222, label, fontsize=9, fontweight="bold", color=c["muted"])

    v0 = views[first_params.preset_name]
    slider_specs = [
        # key, label, column x, row, lo, hi, init, step, format
        ("draw", "Draw", 0.115, 0, v0.draw_lo_in, v0.draw_hi_in, v0.draw_nom_in, 0.1, "%.1f in"),
        ("pitch", "Pitch", 0.115, 1, v0.pitch_lo_deg, v0.pitch_hi_deg, v0.start_pitch_deg, 0.01, "%.2f°"),
        ("yaw", "Yaw", 0.115, 2, *YAW_LIMITS_DEG, 0.3, 0.01, "%+.2f°"),
        ("range", "Distance", 0.44, 0, *RANGE_LIMITS_M, DEFAULT_RANGE_M, 0.1, "%.1f m"),
        ("wind", "Crosswind", 0.44, 1, *WIND_LIMITS_MPS, first_params.flight.wind_mps[1], 0.1, "%+.1f m/s"),
        ("cd", "Drag Cd", 0.44, 2, *CD_LIMITS, first_params.flight.drag_coefficient, 0.05, "%.2f"),
    ]
    sliders: dict[str, Slider] = {}
    for key, label, x, row, lo, hi, init, step, fmt in slider_specs:
        ax = fig.add_axes([x, 0.168 - row * 0.055, 0.19, 0.03], facecolor=c["panel"])
        sl = Slider(ax, label, lo, hi, valinit=init, valstep=step, valfmt=fmt, color=c["current"],
                    track_color=c["panel2"], initcolor="none",
                    handle_style={"facecolor": c["text"], "edgecolor": c["current"], "size": 13})  # fmt: skip
        sl.label.set_color(c["text"])
        sl.label.set_fontsize(10.5)
        sl.valtext.set_color(c["text"])
        sl.valtext.set_fontsize(10.5)
        sl.valtext.set_fontweight("bold")
        sl.valtext.set_x(1.05)  # clear of the handle at full travel
        if lo < 0 < hi:
            sl.poly.set_visible(False)  # centered controls: show just the handle, not a fill from the left
        sliders[key] = sl

    def styled_button(rect, label):
        ax = fig.add_axes(rect)
        b = Button(ax, label, color=c["panel2"], hovercolor=c["edge"])
        b.label.set_color(c["text"])
        b.label.set_fontsize(10.5)
        b.label.set_fontweight("bold")
        for spine in ax.spines.values():
            spine.set_visible(True)
            spine.set_edgecolor(c["edge"])
        return b

    preset_buttons: dict[str, Button] = {}
    width = 0.26 / len(names)
    for i, name in enumerate(names):
        preset_buttons[name] = styled_button([0.69 + i * width, 0.152, width - 0.006, 0.045], name.replace("_", " "))
    snap_btn = styled_button([0.69, 0.088, 0.16, 0.045], "Snap to solution")
    reset_btn = styled_button([0.856, 0.088, 0.094, 0.045], "Reset")
    fig.text(0.69, 0.05, "↑↓ pitch    ←→ yaw    space snap    F fullscreen", fontsize=8.5, color=c["muted"])

    state = {"name": first_params.preset_name, "warm": None, "sol": None, "ms": []}

    # ---------- Drawing helpers ----------
    def place_face(bands, outline, x, center, ground=None, stand=None):
        outline.set_data([x, x], [center - FACE_BANDS[0][0] - 0.01, center + FACE_BANDS[0][0] + 0.01])
        for line, (half, _) in zip(bands, FACE_BANDS):
            line.set_data([x, x], [center - half, center + half])
        if stand is not None:
            stand.set_data([x, x], [ground, center - FACE_BANDS[0][0]])

    def highlight_preset():
        for name, b in preset_buttons.items():
            active = name == state["name"]
            b.color = c["current"] if active else c["panel2"]
            b.hovercolor = c["current"] if active else c["edge"]
            b.ax.set_facecolor(b.color)
            b.label.set_color(c["bg"] if active else c["text"])

    def set_cue(main, second, sub, color):
        cue_main.set_text(main)
        cue_main.set_color(color)
        cue_second.set_text(second)
        cue_second.set_color(color)
        cue_sub.set_text(sub)
        cue_stripe.set_facecolor(color)

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
        ground_fill.set_y(ground_z - 100)
        for line, glow, data in ((side_cur, side_glow, (cur.x, cur.z)), (top_cur, top_glow, (cur.x, cur.y))):
            line.set_data(*data)
            glow.set_data(*data)
        side_end.set_data([cur.x[-1]], [cur.z[-1]])
        side_req.set_data(*((req.x, req.z) if req else ([], [])))
        top_req.set_data(*((req.x, req.y) if req else ([], [])))
        place_face(side_bands, side_face_outline, target.range_m, target.height_m, ground_z, side_stand)
        place_face(top_bands, top_face_outline, target.range_m, target.lateral_m)

        x_max = max(target.range_m, float(cur.x[-1])) * 1.06
        z_all = list(cur.z) + (list(req.z) if req else [])
        z_lo, z_hi = ground_z - 0.25, max(max(z_all), FACE_BANDS[0][0]) + 0.3
        ax_side.set_xlim(-0.04 * x_max, x_max)
        ax_side.set_ylim(z_lo, z_hi)
        y_span = max(0.6, max(abs(v) for v in list(cur.y) + (list(req.y) if req else [0.0])) + 0.25)
        ax_top.set_ylim(-y_span, y_span)
        fig_w, fig_h = FIG_SIZE_IN
        stretch = (1.04 * x_max / (SIDE_RECT[2] * fig_w)) / ((z_hi - z_lo) / (SIDE_RECT[3] * fig_h))
        stretch_note.set_text(f"height stretched {stretch:.0f}×" if stretch >= 1.5 else "")

        i_apex = max(range(len(cur.z)), key=lambda i: cur.z[i])
        if cur.z[i_apex] > 0.03 and 0 < i_apex < len(cur.z) - 1:
            apex_dot.set_data([cur.x[i_apex]], [cur.z[i_apex]])
            apex_note.xy = (cur.x[i_apex], cur.z[i_apex])
            apex_note.set_text(f"peak +{cur.z[i_apex] * 100:.0f} cm" if cur.z[i_apex] < 1 else f"peak +{cur.z[i_apex]:.1f} m")
        else:
            apex_dot.set_data([], [])
            apex_note.set_text("")

        # Target face
        hold_r_cm = target.range_m * math.tan(math.radians(DEADBAND_DEG)) * 100
        hold_ring.set_radius(hold_r_cm)
        hold_ring.set_visible(hold_r_cm < 48)
        if cur.hit_target_plane:
            hx, hz = -err.predicted_impact_y_m * 100, err.predicted_impact_z_m * 100
            r = math.hypot(hx, hz)
            off = r > 46
            if off:
                hx, hz = hx * 46 / r, hz * 46 / r
            hit_dot.set_data([hx], [hz])
            hit_glow.set_data([hx], [hz])
            hit_dot.set_markerfacecolor(c["panel"] if off else c["current"])
            vertical = f"{abs(err.predicted_impact_z_m) * 100:.1f} cm {'high' if err.predicted_impact_z_m > 0 else 'low'}"
            lateral = f"{abs(err.predicted_impact_y_m) * 100:.1f} cm {'left' if err.predicted_impact_y_m > 0 else 'right'}"
            face_note.set_text(("off the face: " if off else "") + f"{vertical}, {lateral}")
            stats["MISS NOW"].set_text(f"{err.predicted_miss_m * 100:.1f} cm")
        else:
            hit_dot.set_data([], [])
            hit_glow.set_data([], [])
            face_note.set_text("never reaches the target" + (" (hits the ground)" if cur.stop_reason == "ground" else ""))
            stats["MISS NOW"].set_text("—")

        # Cue card
        if not sol.converged:
            set_cue("OUT OF REACH", "", sol.reason, c["target"])
        else:
            pe, ye = err.pitch_err_deg, err.yaw_err_deg
            parts = []
            if abs(pe) >= DEADBAND_DEG:
                parts.append((abs(pe), f"{'▲ RAISE' if pe > 0 else '▼ LOWER'} {abs(pe):.2f}°"))
            if abs(ye) >= DEADBAND_DEG:
                parts.append((abs(ye), f"{'◀ LEFT' if ye > 0 else '▶ RIGHT'} {abs(ye):.2f}°"))
            parts.sort(reverse=True)
            sub = f"pitch err {pe:+.2f}°   yaw err {ye:+.2f}°"
            if parts:
                set_cue(parts[0][1], parts[1][1] if len(parts) > 1 else "", sub, c["current"])
            else:
                set_cue("● HOLD", f"inside ±{DEADBAND_DEG}° on both axes", sub, c["hold"])

        # Stats and header
        stats["LAUNCH"].set_text(f"{err.v_launch_mps:.1f} m/s")
        stats["COMPUTE"].set_text(f"{ms:.0f} ms · {sol.iterations} sims")
        if sol.converged:
            stats["FLIGHT TIME"].set_text(f"{sol.time_of_flight_s:.2f} s")
            stats["AT TARGET"].set_text(f"{sol.impact_speed_mps:.1f} m/s")
            stats["SOLVED AIM"].set_text(f"{sol.pitch_deg:.2f}° / {sol.yaw_deg:+.2f}°")
        else:
            for k in ("FLIGHT TIME", "AT TARGET", "SOLVED AIM"):
                stats[k].set_text("—")
        drag = "drag on" if s["cd"] > 0 else "no drag"
        header_right.set_text(f"{state['name'].replace('_', ' ')}   ·   {s['draw']:.1f} in draw   ·   {drag}")
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
    finish(fig, f"explorer_{first_params.preset_name}{'_light' if args.light else ''}", args)


if __name__ == "__main__":
    main()
