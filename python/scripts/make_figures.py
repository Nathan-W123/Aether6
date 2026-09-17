#!/usr/bin/env python3
"""Generate every figure in the project from the CSV/JSON artefacts under ``results/``.

Run the C++ tools first (``make run-all``), then::

    python3 python/scripts/make_figures.py                 # everything that is available
    python3 python/scripts/make_figures.py --only nominal  # one group
    python3 python/scripts/make_figures.py --no-animation  # skip the (slow) GIF

Missing inputs are reported and skipped rather than being treated as errors, so the script
is usable after running only part of the pipeline.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "python"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")

from aether_viz import (  # noqa: E402
    apply_style,
    load_integrator_study,
    load_linear_model,
    load_monte_carlo,
    load_run,
)
from aether_viz import figures  # noqa: E402
from aether_viz.animation import animate_run  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--results", default=str(REPO_ROOT / "results"),
                        help="directory holding the simulator output (default: results/)")
    parser.add_argument("--output", default=None,
                        help="figure directory (default: <results>/figures)")
    parser.add_argument("--only", action="append", default=None,
                        choices=["nominal", "linear", "monte-carlo", "integrators",
                                 "comparison", "animation"],
                        help="generate only the named group (repeatable)")
    parser.add_argument("--no-animation", action="store_true",
                        help="skip the animated GIF, which is the slowest output")
    parser.add_argument("--animation-seconds", type=float, default=150.0,
                        help="simulated seconds to animate (default: 150)")
    args = parser.parse_args()

    results = Path(args.results)
    outdir = Path(args.output) if args.output else results / "figures"
    outdir.mkdir(parents=True, exist_ok=True)
    apply_style()

    groups = set(args.only) if args.only else {
        "nominal", "linear", "monte-carlo", "integrators", "comparison", "animation"}
    if args.no_animation:
        groups.discard("animation")

    written: list[Path] = []
    skipped: list[str] = []

    def attempt(label: str, fn):
        start = time.perf_counter()
        try:
            path = fn()
        except FileNotFoundError as exc:
            skipped.append(f"{label}: {exc}")
            return
        except (KeyError, ValueError) as exc:
            skipped.append(f"{label}: {type(exc).__name__}: {exc}")
            return
        written.append(path)
        print(f"  {path.relative_to(REPO_ROOT) if path.is_relative_to(REPO_ROOT) else path}"
              f"   ({time.perf_counter() - start:.1f} s)")

    if "nominal" in groups:
        print("nominal run figures")
        try:
            run = load_run(results / "nominal")
        except FileNotFoundError as exc:
            skipped.append(f"nominal: {exc}")
            run = None
        if run is not None:
            attempt("trajectory_3d", lambda: figures.plot_trajectory_3d(run, outdir / "trajectory_3d.png"))
            attempt("ground_track", lambda: figures.plot_ground_track(run, outdir / "ground_track.png"))
            attempt("states", lambda: figures.plot_states(run, outdir / "states.png"))
            attempt("controls", lambda: figures.plot_controls(run, outdir / "control_inputs.png"))
            attempt("air_data", lambda: figures.plot_air_data(run, outdir / "air_data.png"))
            attempt("tracking", lambda: figures.plot_tracking(run, outdir / "waypoint_tracking.png"))
            attempt("estimator", lambda: figures.plot_estimator(run, outdir / "estimator_detail.png"))
            attempt("estimator_summary",
                    lambda: figures.plot_estimator_summary(run, outdir / "estimator_summary.png"))
            attempt("wind", lambda: figures.plot_wind_estimate(run, outdir / "wind_response.png"))

    if "linear" in groups:
        print("linear analysis figures")
        try:
            linear = load_linear_model(results / "linear")
        except FileNotFoundError as exc:
            skipped.append(f"linear: {exc}")
            linear = None
        if linear is not None:
            attempt("eigenvalues", lambda: figures.plot_eigenvalues(linear, outdir / "eigenvalues.png"))
            if "trim_sweep" in linear:
                attempt("trim_sweep", lambda: figures.plot_trim_sweep(linear, outdir / "trim_sweep.png"))
            else:
                skipped.append("trim_sweep: run 'aether_trim --sweep 16:34:19' to produce it")

    if "monte-carlo" in groups:
        print("Monte-Carlo figures")
        try:
            mc = load_monte_carlo(results / "monte_carlo")
        except FileNotFoundError as exc:
            skipped.append(f"monte-carlo: {exc}")
            mc = None
        if mc is not None:
            attempt("monte_carlo", lambda: figures.plot_monte_carlo(mc, outdir / "monte_carlo.png"))
            attempt("monte_carlo_statistics",
                    lambda: figures.plot_monte_carlo_statistics(mc, outdir / "monte_carlo_statistics.png"))

    if "integrators" in groups:
        print("integrator study figures")
        try:
            study = load_integrator_study(results / "integrators")
        except FileNotFoundError as exc:
            skipped.append(f"integrators: {exc}")
            study = None
        if study is not None:
            attempt("integrators", lambda: figures.plot_integrators(study, outdir / "integrators.png"))

    if "comparison" in groups:
        print("controller comparison figure")
        runs = {}
        for label, name in (("LQR", "nominal"), ("PID", "nominal_pid"),
                            ("LQR, strong wind", "wind")):
            try:
                runs[label] = load_run(results / name)
            except FileNotFoundError:
                pass
        if len(runs) >= 2:
            attempt("controller_comparison",
                    lambda: figures.plot_controller_comparison(runs, outdir / "controller_comparison.png"))
        else:
            skipped.append("comparison: needs at least two of results/{nominal,nominal_pid,wind}")

    if "animation" in groups:
        print("flight animation (this one takes a while)")
        try:
            run = load_run(results / "nominal")
        except FileNotFoundError as exc:
            skipped.append(f"animation: {exc}")
            run = None
        if run is not None:
            attempt("animation",
                    lambda: animate_run(run, outdir / "flight.gif",
                                        duration_s=args.animation_seconds))

    print(f"\n{len(written)} figure(s) written to {outdir}")
    if skipped:
        print("skipped:")
        for item in skipped:
            print(f"  - {item}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
