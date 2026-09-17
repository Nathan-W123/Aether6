"""Shared plotting style: a validated categorical palette and Matplotlib defaults.

The eight categorical hues below are the reference instance of the data-visualisation
palette used throughout the project. They were checked with a colour-vision-deficiency
validator: worst adjacent-pair CVD dE is 9.1 and worst adjacent normal-vision dE is 19.6
(OKLab x100), both above their gates. Hues are assigned **in fixed slot order and never
cycled** - a ninth series is folded into "other" or split into small multiples instead.

Three of the light-mode slots (aqua, yellow, magenta) sit below 3:1 contrast on a white
surface, so every figure that uses them also carries a legend or direct labels; identity
is never conveyed by colour alone.
"""

from __future__ import annotations

import matplotlib as mpl
import matplotlib.pyplot as plt

#: Categorical palette, in the fixed order hues must be assigned.
PALETTE = [
    "#2a78d6",  # 1 blue
    "#eb6834",  # 2 orange
    "#1baf7a",  # 3 aqua
    "#eda100",  # 4 yellow
    "#e87ba4",  # 5 magenta
    "#008300",  # 6 green
    "#4a3aa7",  # 7 violet
    "#e34948",  # 8 red
]

#: Single-hue sequential ramp (light -> dark) for continuous magnitude.
SEQUENTIAL = [
    "#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5",
    "#256abf", "#184f95", "#0d366b",
]

#: Diverging pair with a neutral midpoint, for signed quantities.
DIVERGING = ("#2a78d6", "#f0efec", "#e34948")

#: Text / structural inks. Values, labels and legends wear these, never a series colour.
INK = {
    "primary": "#0b0b0b",
    "secondary": "#52514e",
    "muted": "#8a8880",
    "grid": "#dedcd6",
    "surface": "#ffffff",
    "truth": "#2b2b2b",
}


def series_color(index: int) -> str:
    """Return the categorical colour for slot ``index`` (0-based), clamped to the palette.

    Colours are never generated or cycled: asking for a slot past the palette returns the
    last hue, which is a deliberate signal that the chart has too many series.
    """
    return PALETTE[min(index, len(PALETTE) - 1)]


def apply_style() -> None:
    """Install the project-wide Matplotlib defaults.

    Thin marks, recessive grid and axes, text in ink colours, and the categorical palette
    installed as the property cycle so any un-coloured plot still lands on slot order.
    """
    mpl.rcParams.update(
        {
            "figure.facecolor": INK["surface"],
            "axes.facecolor": INK["surface"],
            "savefig.facecolor": INK["surface"],
            "savefig.dpi": 140,
            "figure.dpi": 110,
            "font.size": 9.5,
            "font.family": "DejaVu Sans",
            "axes.titlesize": 11,
            "axes.titleweight": "bold",
            "axes.titlecolor": INK["primary"],
            "axes.labelsize": 9.5,
            "axes.labelcolor": INK["secondary"],
            "axes.edgecolor": INK["grid"],
            "axes.linewidth": 0.8,
            "axes.grid": True,
            "axes.axisbelow": True,
            "axes.prop_cycle": mpl.cycler(color=PALETTE),
            "grid.color": INK["grid"],
            "grid.linewidth": 0.6,
            "grid.alpha": 0.9,
            "xtick.color": INK["secondary"],
            "ytick.color": INK["secondary"],
            "xtick.labelsize": 8.5,
            "ytick.labelsize": 8.5,
            "xtick.direction": "out",
            "ytick.direction": "out",
            "legend.frameon": True,
            "legend.framealpha": 0.92,
            "legend.edgecolor": INK["grid"],
            "legend.fontsize": 8.5,
            "legend.labelcolor": INK["secondary"],
            "lines.linewidth": 1.6,
            "lines.markersize": 4,
            "lines.solid_capstyle": "round",
            "figure.autolayout": False,
            "figure.constrained_layout.use": True,
        }
    )


def finish_axes(ax, title: str | None = None, xlabel: str | None = None,
                ylabel: str | None = None, legend: bool = False,
                legend_loc: str = "best") -> None:
    """Apply the shared axis furniture: recessive spines, labels and an optional legend."""
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    if title:
        ax.set_title(title, loc="left", pad=6)
    if xlabel:
        ax.set_xlabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)
    if legend:
        ax.legend(loc=legend_loc)


def annotate_source(fig, text: str) -> None:
    """Add a small provenance note (which run/command produced the figure)."""
    fig.text(0.005, 0.004, text, fontsize=7, color=INK["muted"], ha="left", va="bottom")
