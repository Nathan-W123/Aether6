"""Aether-6 analysis and visualisation package.

Loaders for the CSV/JSON artefacts written by the C++ tools, plus the figure
generators used by ``python/scripts/make_figures.py``.

All plotting uses a single validated categorical palette (see :mod:`aether_viz.style`)
and follows one rule set: fixed hue order, one y-axis per panel, a legend whenever more
than one series is drawn, and recessive grids/axes.
"""

from .style import PALETTE, apply_style, series_color, SEQUENTIAL, INK
from .io import (
    RunData,
    load_run,
    load_linear_model,
    load_monte_carlo,
    load_integrator_study,
)

__all__ = [
    "PALETTE",
    "SEQUENTIAL",
    "INK",
    "apply_style",
    "series_color",
    "RunData",
    "load_run",
    "load_linear_model",
    "load_monte_carlo",
    "load_integrator_study",
]

__version__ = "1.0.0"
