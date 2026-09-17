#!/usr/bin/env python3
"""Render the animated 3-D flight GIF for one simulation run.

Examples::

    python3 python/scripts/animate.py                                  # results/nominal
    python3 python/scripts/animate.py results/wind --output wind.gif
    python3 python/scripts/animate.py --seconds 300 --speedup 40
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "python"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")

from aether_viz import apply_style, load_run  # noqa: E402
from aether_viz.animation import animate_run  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", nargs="?", default=str(REPO_ROOT / "results" / "nominal"),
                        help="simulation output directory (default: results/nominal)")
    parser.add_argument("--output", default=None, help="output GIF path")
    parser.add_argument("--seconds", type=float, default=150.0,
                        help="simulated seconds to animate (default: 150)")
    parser.add_argument("--speedup", type=float, default=30.0,
                        help="playback speed relative to real time (default: 30)")
    parser.add_argument("--fps", type=int, default=16, help="frames per second (default: 16)")
    parser.add_argument("--dpi", type=int, default=70, help="raster resolution (default: 70)")
    parser.add_argument("--colors", type=int, default=32,
                        help="GIF palette size; 0 disables re-quantisation (default: 32)")
    args = parser.parse_args()

    run_dir = Path(args.run)
    output = Path(args.output) if args.output else run_dir.parent / "figures" / "flight.gif"

    apply_style()
    run = load_run(run_dir)
    path = animate_run(run, output, duration_s=args.seconds, fps=args.fps,
                       speedup=args.speedup, dpi=args.dpi, colors=args.colors)
    size_mb = path.stat().st_size / 1e6
    print(f"wrote {path} ({size_mb:.2f} MB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
