"""Shared CLI, plot style, and output handling for the exploration scripts."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

from artemis.config import Params, load_config, preset_names

OUT_DIR = Path(__file__).resolve().parent / "out"

# Categorical slots, used in this fixed order (validated colorblind-safe order).
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948"]
TEXT = "#0b0b0b"
TEXT_MUTED = "#52514e"
GRID = "#e4e3df"
SURFACE = "#fcfcfb"

# World Archery 10-ring face, outer ring first: (fill, line). Rings 1-2 white, 3-4 black,
# 5-6 blue, 7-8 red, 9-10 gold.
FACE_RING_COLORS = [
    ("#ffffff", "#000000"), ("#ffffff", "#000000"),
    ("#000000", "#ffffff"), ("#000000", "#ffffff"),
    ("#41b7e6", "#000000"), ("#41b7e6", "#000000"),
    ("#e2231a", "#000000"), ("#e2231a", "#000000"),
    ("#ffe552", "#000000"), ("#ffe552", "#000000"),
]  # fmt: skip


def parse_args(description: str, extra=None) -> tuple[argparse.Namespace, Params]:
    """Standard flags: --preset, --config, --no-show. `extra(parser)` adds script-specific ones."""
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--config", default=None, help="path to config.yaml (default: repo root)")
    parser.add_argument("--preset", default=None, help="preset name (default: active_preset)")
    parser.add_argument("--no-show", action="store_true", help="save plots without opening a window")
    if extra is not None:
        extra(parser)
    args = parser.parse_args()
    names = preset_names(args.config)
    if args.preset is not None and args.preset not in names:
        parser.error(f"unknown preset {args.preset!r}; choose from {', '.join(names)}")
    if args.no_show:
        matplotlib.use("Agg")
    apply_style()
    return args, load_config(args.config, args.preset)


def apply_style() -> None:
    matplotlib.rcParams.update(
        {
            "figure.facecolor": SURFACE,
            "axes.facecolor": SURFACE,
            "axes.edgecolor": GRID,
            "axes.labelcolor": TEXT_MUTED,
            "axes.titlecolor": TEXT,
            "axes.titlesize": 11,
            "axes.titleweight": "bold",
            "axes.grid": True,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.prop_cycle": matplotlib.cycler(color=SERIES),
            "grid.color": GRID,
            "grid.linewidth": 0.8,
            "xtick.color": TEXT_MUTED,
            "ytick.color": TEXT_MUTED,
            "legend.frameon": False,
            "lines.linewidth": 2.0,
            "font.size": 9.5,
        }
    )


def draw_target_face(ax, face_diameter_m: float, rings: int = 10, center=(0.0, 0.0), unit_scale: float = 1.0) -> None:
    """Draw a ring target face (outer ring first) on `ax` in data units of meters * unit_scale."""
    from matplotlib.patches import Circle

    ring_w = face_diameter_m / 2 / rings
    colors = FACE_RING_COLORS if rings == 10 else [("#ffffff", "#000000")] * rings
    for i in range(rings):
        r = (face_diameter_m / 2 - i * ring_w) * unit_scale
        fill, edge = colors[i]
        ax.add_patch(Circle(center, r, facecolor=fill, edgecolor=edge, linewidth=0.6, zorder=0))


def finish(fig, name: str, args: argparse.Namespace) -> Path:
    """Save to explore/out/<name>.png and show unless --no-show."""
    import matplotlib.pyplot as plt

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    path = OUT_DIR / f"{name}.png"
    fig.savefig(path, dpi=150, bbox_inches="tight")
    print(f"saved {path.relative_to(OUT_DIR.parent.parent)}")
    if not args.no_show:
        plt.show()
    return path
