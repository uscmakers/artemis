"""Wall demo (MVP): the ToF sensor's distance to the wall sets how far to pull the bow back.

The bow is held at a fixed angle (slider). For the current distance, the engine solves the draw
length that puts the arrow on a mark on the wall level with the nock, and shows a model bow
pulled back that far, the string force, and the flight to the wall.

Distance comes from the ToF sensor when one is connected; otherwise drag the distance slider.

Run:
    python -m explore.wall_demo                       # auto-detects a USB sensor, else slider only
    python -m explore.wall_demo --port /dev/cu.usbmodem1101
    ssh pi@raspberrypi.local "python3 -u ~/artemis/sensor_testing/tof_sensor_testing/tof_distance_pi.py" \\
        | python -m explore.wall_demo --stdin       # stream from the Pi
    python -m explore.wall_demo --fake                # simulated sensor, for testing without hardware
Options: --preset toy_suction, --dark, --no-sensor, --no-show (save a snapshot instead of a window).
"""

from __future__ import annotations

import math
import random
import sys
import time
from collections.abc import Iterator
from dataclasses import dataclass

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.widgets import Button, Slider

from artemis.config import M_PER_IN, N_PER_LBF, Params, load_config, preset_names
from artemis.io.tof import ToFReader, find_serial_port, serial_lines
from artemis.physics.bow import draw_force_n, stored_energy_j
from artemis.physics.flight import launch_velocity, simulate
from artemis.solver.aim import Target
from artemis.solver.draw import solve_draw
from explore._common import finish, parse_args
from explore.explorer import THEMES, apply_theme, preset_label

DISTANCE_LIMITS_M = (0.1, 10.0)  # bow max range for the demo; the VL53L4CX itself reads ~1.3 m (L4CD driver) to ~6 m
DEFAULT_DISTANCE_M = 1.0
ANGLE_LIMITS_DEG = (-5.0, 30.0)
DEFAULT_ANGLE_DEG = 2.0
POLL_MS = 100
FIG_SIZE_IN = (15.0, 9.0)

BOW_RECT = [0.04, 0.29, 0.31, 0.6]
FLIGHT_RECT = [0.43, 0.5, 0.535, 0.22]
CURVE_RECT = [0.43, 0.29, 0.24, 0.13]
RIGHT_X = 0.43


@dataclass
class BowShape:
    """Proportions for the model bow drawing, scaled from the preset's draw length."""

    half_height_m: float
    brace_m: float
    max_draw_m: float
    arrow_length_m: float


def bow_shape(params: Params) -> BowShape:
    curve = params.bow.curve
    return BowShape(
        half_height_m=0.87 * curve.max_draw_m,  # a 28 in recurve is ~62 in tip to tip
        brace_m=curve.brace_m,
        max_draw_m=curve.max_draw_m,
        arrow_length_m=curve.max_draw_m + 0.06,
    )


def bezier(p0, p1, p2, n=24):
    return [((1 - t) ** 2 * p0[0] + 2 * (1 - t) * t * p1[0] + t**2 * p2[0],
             (1 - t) ** 2 * p0[1] + 2 * (1 - t) * t * p1[1] + t**2 * p2[1]) for t in (i / n for i in range(n + 1))]  # fmt: skip


def bow_geometry(shape: BowShape, draw_m: float, pitch_deg: float) -> dict[str, list[tuple[float, float]]]:
    """Polylines for the bow at `draw_m`, in meters, x forward and y up, grip pivot at the origin."""
    h = shape.half_height_m
    frac = min(max((draw_m - shape.brace_m) / (shape.max_draw_m - shape.brace_m), 0.0), 1.0)
    tip = (-shape.brace_m - 0.16 * h * frac, h * (1 - 0.1 * frac))
    parts: dict[str, list[tuple[float, float]]] = {
        "riser": [(0.02 * h, -0.3 * h), (0.02 * h, 0.3 * h)],
        "grip": [(0.0, -0.07 * h), (0.0, 0.07 * h)],
    }
    for sign, key in ((1, "upper_limb"), (-1, "lower_limb")):
        ctrl = (0.05 * h - 0.06 * h * frac, sign * 0.72 * h)
        parts[key] = bezier((0.02 * h, sign * 0.3 * h), ctrl, (tip[0], sign * tip[1]))
    nock = (-max(draw_m, shape.brace_m), 0.0)
    parts["string"] = [(tip[0], tip[1]), nock, (tip[0], -tip[1])]
    tip_x = nock[0] + shape.arrow_length_m
    parts["arrow"] = [nock, (tip_x, 0.0)]
    head = 0.045 * h
    parts["arrowhead"] = [(tip_x - head, 0.5 * head), (tip_x, 0.0), (tip_x - head, -0.5 * head)]
    # Feathers: joined to the shaft at the front, flaring out toward the nock.
    vane = 0.035 * h
    parts["fletching"] = [(nock[0] + 0.03 * h, vane), (nock[0] + 0.14 * h, 0.0), (nock[0] + 0.03 * h, -vane)]
    y_dim = -1.12 * h
    parts["dimension"] = [(nock[0], y_dim), (0.0, y_dim)]
    parts["dimension_ticks"] = [(nock[0], y_dim - 0.03 * h), (nock[0], y_dim + 0.03 * h), (math.nan, math.nan),
                                (0.0, y_dim - 0.03 * h), (0.0, y_dim + 0.03 * h)]  # fmt: skip
    a = math.radians(pitch_deg)
    ca, sa = math.cos(a), math.sin(a)
    return {k: [(x * ca - y * sa, x * sa + y * ca) for x, y in pts] for k, pts in parts.items()}


def fake_sensor_lines() -> Iterator[str]:
    """A wall that drifts between ~0.5 and ~1.2 m, with a few mm of noise, at 10 readings/s."""
    t0 = time.monotonic()
    while True:
        t = time.monotonic() - t0
        mm = 850 + 350 * math.sin(t / 3.0) + random.gauss(0.0, 3.0)
        yield f"Distance: {mm:.0f} mm\n"
        time.sleep(0.1)


def main() -> None:
    def extra(p):
        p.add_argument("--dark", action="store_true", help="near-black background")
        src = p.add_mutually_exclusive_group()
        src.add_argument("--port", default="auto", help="serial port of the sensor board (default: auto-detect)")
        src.add_argument("--stdin", action="store_true", help="read sensor lines from stdin (e.g. piped from the Pi)")
        src.add_argument("--fake", action="store_true", help="simulated sensor, for testing without hardware")
        src.add_argument("--no-sensor", action="store_true", help="slider only")
        p.add_argument("--baud", type=int, default=115200)

    args, first_params = parse_args(__doc__, extra)
    c = THEMES["dark" if args.dark else "light"]
    apply_theme(c)
    names = preset_names(args.config)
    all_params = {first_params.preset_name: first_params}

    def params_for(name: str) -> Params:
        if name not in all_params:
            all_params[name] = load_config(args.config, name)
        return all_params[name]

    # ---------- Sensor ----------
    sensors = first_params.sensors
    reader: ToFReader | None = None
    source_note = "No sensor. Drag the distance slider."
    if args.fake:
        reader = ToFReader(sensors.tof_smoothing_samples, label="simulated sensor").start(fake_sensor_lines())
    elif args.stdin:
        reader = ToFReader(sensors.tof_smoothing_samples, label="stdin").start(sys.stdin)
    elif not args.no_sensor:
        port = find_serial_port() if args.port == "auto" else args.port
        if port:
            reader = ToFReader(sensors.tof_smoothing_samples, label=port).start(serial_lines(port, args.baud))
        elif args.port == "auto":
            source_note = "No sensor found on USB. Drag the distance slider, or pass --port."

    fig = plt.figure(figsize=FIG_SIZE_IN)
    if fig.canvas.manager is not None:
        fig.canvas.manager.set_window_title("Artemis wall demo")

    # ---------- Header ----------
    fig.text(0.04, 0.955, "Artemis wall demo", fontsize=20, fontweight="bold", va="center")
    fig.text(0.04, 0.92, "The sensor measures the distance to the wall. The engine works out how far to pull back "
             "so the arrow hits a mark on the wall level with the nock.", fontsize=11.5, color=c["ink2"],
             va="center")  # fmt: skip
    status = fig.text(0.965, 0.955, "", fontsize=11, color=c["ink2"], va="center", ha="right")

    # ---------- Model bow ----------
    ax_bow = fig.add_axes(BOW_RECT)
    ax_bow.set_aspect("equal")
    ax_bow.set_axis_off()
    ax_bow.set_title("Bow, side view", loc="left", fontsize=13, color=c["ink"], pad=4)
    full_string = ax_bow.plot([], [], color=c["ink3"], linewidth=1, linestyle=(0, (3, 3)), zorder=1)[0]
    full_label = ax_bow.text(0, 0, "full draw", fontsize=9, color=c["ink3"], ha="right", va="bottom")
    bow_lines = {
        "upper_limb": ax_bow.plot([], [], color=c["ink"], linewidth=4, solid_capstyle="round", zorder=3)[0],
        "lower_limb": ax_bow.plot([], [], color=c["ink"], linewidth=4, solid_capstyle="round", zorder=3)[0],
        "riser": ax_bow.plot([], [], color=c["ink"], linewidth=8, solid_capstyle="round", zorder=3)[0],
        "grip": ax_bow.plot([], [], color=c["ink2"], linewidth=13, solid_capstyle="round", zorder=4)[0],
        "string": ax_bow.plot([], [], color=c["ink2"], linewidth=1.3, zorder=2)[0],
        "arrow": ax_bow.plot([], [], color=c["accent"], linewidth=2.5, zorder=5)[0],
        "fletching": ax_bow.plot([], [], color=c["accent"], linewidth=1.5, zorder=5)[0],
        "arrowhead": ax_bow.plot([], [], color=c["accent"], linewidth=2.5, zorder=5)[0],
        "dimension": ax_bow.plot([], [], color=c["ink2"], linewidth=1, zorder=2)[0],
        "dimension_ticks": ax_bow.plot([], [], color=c["ink2"], linewidth=1, zorder=2)[0],
    }
    dim_label = ax_bow.text(0, 0, "", fontsize=10, color=c["ink2"], ha="center", va="top")

    # ---------- Big readout ----------
    pull_main = fig.text(RIGHT_X, 0.845, "", fontsize=30, fontweight="bold", va="center")
    pull_second = fig.text(RIGHT_X, 0.795, "", fontsize=18, va="center")
    pull_sub = fig.text(RIGHT_X, 0.762, "", fontsize=10, color=c["ink2"], va="center")

    # ---------- Flight to the wall ----------
    ax_fl = fig.add_axes(FLIGHT_RECT)
    ax_fl.set_title("Flight to the wall", loc="left", fontsize=13, color=c["ink"], pad=6)
    wall = ax_fl.axvspan(0, 1, color=c["grid"], zorder=0, linewidth=0)
    wall_edge = ax_fl.axvline(0, color=c["ink3"], linewidth=1.5, zorder=1)
    mark = ax_fl.plot([], [], color=c["ink"], linewidth=3, solid_capstyle="butt", zorder=4)[0]
    mark_label = ax_fl.text(0, 0, "mark", fontsize=9, color=c["ink2"], ha="left", va="center")
    wall_label = ax_fl.text(0, 0, "wall", fontsize=9, color=c["ink3"], ha="left", va="top")
    path = ax_fl.plot([], [], color=c["accent"], linewidth=2.2, zorder=5)[0]
    hit = ax_fl.plot([], [], marker="o", markersize=7, color=c["accent"], zorder=6)[0]
    ax_fl.plot([0], [0], marker="o", markersize=5, color=c["ink"], zorder=6)
    ax_fl.set_xlabel("distance from the nock (m)")
    ax_fl.set_ylabel("height above nock (cm)")

    # ---------- Force-draw curve ----------
    ax_fd = fig.add_axes(CURVE_RECT)
    ax_fd.set_title("String force vs draw", loc="left", fontsize=11, color=c["ink"], pad=4)
    fd_line = ax_fd.plot([], [], color=c["ink2"], linewidth=1.5)[0]
    fd_fill = [None]
    fd_dot = ax_fd.plot([], [], marker="o", markersize=7, color=c["accent"], zorder=5)[0]
    ax_fd.set_xlabel("draw (in)")
    ax_fd.set_ylabel("force (lb)")

    # ---------- Numbers ----------
    stat_labels = ["Sensor reading", "Nock to wall, level", "Launch speed", "Stored energy", "Flight time"]
    fig.text(0.72, 0.42, "\n".join(stat_labels), fontsize=10.5, color=c["ink2"], va="top", linespacing=1.45)
    stat_values = fig.text(0.965, 0.42, "", fontsize=10.5, va="top", ha="right", linespacing=1.45)

    # ---------- Controls ----------
    fig.add_artist(Line2D([0.04, 0.965], [0.215, 0.215], color=c["rule"], linewidth=1, transform=fig.transFigure))
    sliders: dict[str, Slider] = {}
    for key, label, row, lo, hi, init, step, fmt in (
        ("distance", "Distance to wall", 0, *DISTANCE_LIMITS_M, DEFAULT_DISTANCE_M, 0.005, "%.3f m"),
        ("angle", "Bow angle", 1, *ANGLE_LIMITS_DEG, DEFAULT_ANGLE_DEG, 0.1, "%.1f° up"),
    ):
        ax = fig.add_axes([0.15, 0.15 - row * 0.065, 0.36, 0.012], facecolor=c["bg"])
        sl = Slider(ax, label, lo, hi, valinit=init, valstep=step, valfmt=fmt, color=c["ink3"],
                    track_color=c["rule"], initcolor="none",
                    handle_style={"facecolor": c["bg"], "edgecolor": c["ink"], "size": 13})  # fmt: skip
        sl.poly.set_visible(False)
        for t in (sl.label, sl.valtext):
            t.set_color(c["ink"])
            t.set_fontsize(11.5)
        sl.valtext.set_x(1.04)
        sliders[key] = sl

    def plain_button(rect, label):
        ax = fig.add_axes(rect)
        b = Button(ax, label, color=c["bg"], hovercolor=c["grid"])
        b.label.set_color(c["ink"])
        b.label.set_fontsize(11)
        for spine in ax.spines.values():
            spine.set_visible(True)
            spine.set_edgecolor(c["ink3"])
        return b

    fig.text(0.65, 0.185, "Preset", fontsize=12, color=c["ink2"], va="center")
    preset_buttons: dict[str, Button] = {}
    width = 0.315 / len(names)
    for i, name in enumerate(names):
        preset_buttons[name] = plain_button([0.65 + i * width, 0.125, width - 0.008, 0.042], preset_label(name))
    follow_btn = plain_button([0.65, 0.055, 0.155, 0.042], "Follow sensor")
    follow_hint = fig.text(0.815, 0.076, "Dragging the distance\nslider pauses the sensor.", fontsize=9,
                           color=c["ink3"], va="center", linespacing=1.3)  # fmt: skip

    state = {"name": first_params.preset_name, "follow": reader is not None, "busy": False, "last_status": None}

    def set_button(b: Button, active: bool):
        b.color = c["ink"] if active else c["bg"]
        b.hovercolor = c["ink"] if active else c["grid"]
        b.ax.set_facecolor(b.color)
        b.label.set_color(c["bg"] if active else c["ink"])

    def refresh_buttons():
        for name, b in preset_buttons.items():
            set_button(b, name == state["name"])
        set_button(follow_btn, state["follow"])
        follow_btn.ax.set_visible(reader is not None)
        follow_hint.set_visible(reader is not None)

    def draw_curve(params: Params):
        curve = params.bow.curve
        fd_line.set_data(curve.draw_m / M_PER_IN, curve.force_n / N_PER_LBF)
        ax_fd.set_xlim(curve.brace_m / M_PER_IN - 1, curve.max_draw_m / M_PER_IN + 1)
        ax_fd.set_ylim(0, curve.force_n.max() / N_PER_LBF * 1.15)

    def set_lines(lines, geom):
        for key, line in lines.items():
            xs, ys = zip(*geom[key])
            line.set_data(xs, ys)

    # ---------- Update ----------
    def update(_=None):
        params = params_for(state["name"])
        shape = bow_shape(params)
        curve = params.bow.curve
        reading_m = sliders["distance"].val
        angle = sliders["angle"].val
        nock_to_wall_m = reading_m + params.sensors.tof_to_nock_m
        level_m = nock_to_wall_m * math.cos(math.radians(angle))  # the sensor looks along the tilted bow
        sol = solve_draw(Target(level_m), angle, params)
        draw_m = sol.draw_m if sol.converged else curve.max_draw_m
        force_n = draw_force_n(draw_m, curve)
        energy_j = stored_energy_j(draw_m, curve)
        v = math.sqrt(2 * params.bow.efficiency * energy_j / params.bow.arrow_mass_kg)
        traj = simulate(launch_velocity(v, angle, 0.0), params.flight, level_m, params.solver.dt_s)

        # Bow
        set_lines(bow_lines, bow_geometry(shape, draw_m, angle))
        full = bow_geometry(shape, shape.max_draw_m, angle)
        full_string.set_data(*zip(*full["string"]))
        full_label.set_position(full["string"][1])
        (x0, y0), (x1, y1) = bow_geometry(shape, draw_m, angle)["dimension"]
        dim_label.set_position(((x0 + x1) / 2, (y0 + y1) / 2 - 0.08 * shape.half_height_m))
        dim_label.set_text(f"draw {draw_m / M_PER_IN:.1f} in from the grip")
        h = shape.half_height_m
        ax_bow.set_xlim(-shape.max_draw_m - 0.35 * h, shape.arrow_length_m - shape.brace_m + 0.25 * h)
        ax_bow.set_ylim(-1.45 * h, 1.3 * h)

        # Readout
        if sol.converged:
            pull_main.set_text(f"Pull back {draw_m / M_PER_IN:.1f} in")
            pull_second.set_text(f"{force_n / N_PER_LBF:.1f} lb on the string")
            pull_sub.set_text(f"{draw_m * 100:.1f} cm from the grip. Full draw is {curve.max_draw_m / M_PER_IN:.1f} in "
                              f"({curve.force_n[-1] / N_PER_LBF:.0f} lb).")  # fmt: skip
        else:
            pull_main.set_text("Can't reach the mark")
            pull_second.set_text(sol.reason[0].upper() + sol.reason[1:])
            pull_sub.set_text("Showing the bow at full draw.")

        # Flight
        xs, zs = traj.x, traj.z * 100
        path.set_data(xs, zs)
        hit.set_data([xs[-1]], [zs[-1]])
        span = max(0.05 * level_m, 0.02)
        wall.set_x(level_m)
        wall.set_width(1000)
        wall_edge.set_xdata([level_m, level_m])
        mark.set_data([level_m, level_m + span], [0, 0])
        z_lo, z_hi = min(min(zs), -2.0), max(max(zs), 2.0)
        pad = 0.15 * (z_hi - z_lo)
        ax_fl.set_xlim(-0.03 * level_m, level_m * 1.12)
        ax_fl.set_ylim(z_lo - pad, z_hi + pad)
        mark_label.set_position((level_m + span * 1.25, 0))
        wall_label.set_position((level_m + 0.01 * level_m, z_hi + 0.85 * pad))

        # Force-draw marker
        fd_dot.set_data([draw_m / M_PER_IN], [force_n / N_PER_LBF])
        if fd_fill[0] is not None:
            fd_fill[0].remove()
        d_in = [d / M_PER_IN for d in curve.draw_m if d < draw_m] + [draw_m / M_PER_IN]
        f_lb = [f / N_PER_LBF for d, f in zip(curve.draw_m, curve.force_n) if d < draw_m] + [force_n / N_PER_LBF]
        fd_fill[0] = ax_fd.fill_between(d_in, 0, f_lb, color=c["grid"], linewidth=0, zorder=0)

        stat_values.set_text("\n".join([
            f"{reading_m:.3f} m",
            f"{level_m:.3f} m",
            f"{v:.1f} m/s",
            f"{energy_j:.1f} J",
            f"{traj.time_of_flight_s * 1000:.0f} ms" if traj.hit_target_plane else "–",
        ]))  # fmt: skip
        fig.canvas.draw_idle()

    # ---------- Sensor polling ----------
    def sensor_status() -> str:
        if reader is None:
            return source_note
        if reader.error:
            return f"Sensor error on {reader.label}: {reader.error}"
        r = reader.latest()
        if r is None:
            return f"Waiting for readings from {reader.label}…"
        if r.age_s > sensors.tof_stale_s:
            return f"No reading from {reader.label} for {r.age_s:.1f} s"
        mode = "following" if state["follow"] else "paused, slider in control"
        note = ", out of range" if r.out_of_range else ""
        return f"Sensor {reader.label}: {r.readings_per_s:.0f} readings/s{note}, {mode}"

    def poll():
        text = sensor_status()
        changed = text != state["last_status"]
        state["last_status"] = text
        status.set_text(text)
        if reader is not None and state["follow"]:
            r = reader.latest()
            if r is not None and r.age_s <= sensors.tof_stale_s:
                lo, hi = DISTANCE_LIMITS_M
                value = min(max(r.distance_m, lo), hi)
                if abs(value - sliders["distance"].val) >= 0.001:
                    state["busy"] = True
                    sliders["distance"].set_val(value)
                    state["busy"] = False
                    return  # set_val already redrew
        if changed:
            fig.canvas.draw_idle()

    # ---------- Actions ----------
    def on_distance(_):
        if not state["busy"] and state["follow"]:
            state["follow"] = False  # the user grabbed the slider
            refresh_buttons()
        update()

    def load_preset(name):
        state["name"] = name
        draw_curve(params_for(name))
        refresh_buttons()
        update()

    def toggle_follow(_):
        state["follow"] = not state["follow"]
        refresh_buttons()
        poll()
        fig.canvas.draw_idle()

    sliders["distance"].on_changed(on_distance)
    sliders["angle"].on_changed(update)
    for name, b in preset_buttons.items():
        b.on_clicked(lambda _e, n=name: load_preset(n))
    follow_btn.on_clicked(toggle_follow)
    timer = fig.canvas.new_timer(interval=POLL_MS)
    timer.add_callback(poll)
    load_preset(state["name"])
    poll()

    if args.no_show:
        if reader is not None:  # give a live source a moment, then take the snapshot from its data
            deadline = time.monotonic() + 3.0
            while reader.latest() is None and time.monotonic() < deadline and (reader.running or not args.stdin):
                time.sleep(0.05)
            time.sleep(0.6)
            poll()
            print(sensor_status())
        print(f"distance {sliders['distance'].val:.3f} m -> {pull_main.get_text()}, {pull_second.get_text()}")
    else:
        timer.start()
    finish(fig, f"wall_demo_{first_params.preset_name}{'_dark' if args.dark else ''}", args)
    if reader is not None:
        reader.stop()


if __name__ == "__main__":
    main()
