"""Committed reference results, loaded once at start-up.

Two things the dashboard shows are far too expensive to compute per visitor:

* the **256-trial Monte-Carlo campaign** (160 s of wall clock on four threads), and
* the **linearisation and modal analysis** at the reference trim point.

Both are checked into ``results/`` and reproducible with ``make results``, so they are read
from disk once and served as static JSON. A visitor's live campaign is a separate, much
smaller run (see :mod:`app.runner`); this module is what proves the published numbers.

Everything here is read-only and takes no request input, so there is nothing to validate.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

from .settings import Settings

#: Trial columns worth shipping to the browser. The full CSV has 33 columns; the scatter
#: plots need the dispersed inputs and the outcome metrics, not the bookkeeping.
_TRIAL_COLUMNS = [
    "index", "ok", "mass_kg", "inertia_scale", "cl_alpha", "cm_alpha", "wind_speed_mps",
    "w20_mps", "density_scale", "sensor_scale", "rms_cross_track_m", "max_cross_track_m",
    "rms_altitude_error_m", "rms_airspeed_error_mps", "max_bank_deg", "max_alpha_deg",
    "laps_completed", "estimator_position_rmse_m", "estimator_attitude_rmse_deg",
]

#: Envelope columns: the percentile bands over time across every recorded trajectory.
_ENVELOPE_COLUMNS = [
    "time_s", "n", "cross_track_p05", "cross_track_p50", "cross_track_p95",
    "cross_track_min", "cross_track_max", "altitude_error_p05", "altitude_error_p50",
    "altitude_error_p95", "est_pos_error_p50", "est_pos_error_p95",
]


def _read_numeric_csv(path: Path, columns: list[str]) -> dict[str, list[float]]:
    """Read the named columns of a numeric CSV into parallel lists."""
    if not path.is_file():
        return {}
    out: dict[str, list[float]] = {name: [] for name in columns}
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        available = [c for c in columns if c in (reader.fieldnames or [])]
        out = {name: [] for name in available}
        for row in reader:
            for name in available:
                try:
                    out[name].append(round(float(row[name]), 5))
                except (TypeError, ValueError):
                    out[name].append(0.0)
    return out


def _read_modes(path: Path) -> list[dict]:
    """Read one modes CSV, collapsing each complex-conjugate pair to a single entry."""
    if not path.is_file():
        return []
    seen: set[tuple[str, float, float]] = set()
    out: list[dict] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            try:
                real = float(row["real"])
                imag = float(row["imag"])
                name = str(row["mode"])
            except (KeyError, ValueError):
                continue
            # Keep one representative per pair: the root with non-negative imaginary part.
            if imag < 0.0:
                continue
            key = (name, round(real, 9), round(abs(imag), 9))
            if key in seen:
                continue
            seen.add(key)
            out.append({
                "group": str(row.get("group", "")),
                "mode": name,
                "real": round(real, 6),
                "imag": round(imag, 6),
                "natural_frequency_rad_s": round(float(row.get("natural_frequency_rad_s", 0.0)), 6),
                "damping_ratio": round(float(row.get("damping_ratio", 0.0)), 6),
                "period_s": round(float(row.get("period_s", 0.0)), 4),
                "time_to_half_s": round(float(row.get("time_to_half_s", 0.0)), 4),
                "stable": str(row.get("stable", "0")).strip() in ("1", "1.0", "true", "True"),
            })
    return out


def _read_reference_point(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    out = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            try:
                out.append({"quantity": row["quantity"], "value": float(row["value"]),
                            "unit": row.get("unit", "")})
            except (KeyError, ValueError):
                continue
    return out


class Precomputed:
    """Immutable snapshot of the committed results, built once at start-up."""

    def __init__(self, settings: Settings) -> None:
        root = settings.results_dir
        mc_dir = root / "monte_carlo"
        linear_dir = root / "linear"

        summary_path = mc_dir / "summary.json"
        summary: dict = {}
        if summary_path.is_file():
            try:
                summary = json.loads(summary_path.read_text(encoding="utf-8"))
            except json.JSONDecodeError:
                summary = {}

        self.montecarlo = {
            "available": bool(summary),
            "scenario": summary.get("scenario", ""),
            "controller": summary.get("controller", ""),
            "feedback": summary.get("feedback", ""),
            "trials": int(summary.get("trials", 0)),
            "master_seed": int(summary.get("master_seed", 0)),
            "duration_per_trial_s": float(summary.get("duration_per_trial_s", 0.0)),
            "successes": int(summary.get("successes", 0)),
            "failures": int(summary.get("failures", 0)),
            "failure_rate": float(summary.get("failure_rate", 0.0)),
            "failure_reasons": {str(k): int(v)
                                for k, v in (summary.get("failure_reasons") or {}).items()},
            "wall_clock_s": float(summary.get("wall_clock_s", 0.0)),
            "threads": int(summary.get("threads", 0)),
            "statistics": summary.get("statistics", []),
            "trials_table": _read_numeric_csv(mc_dir / "trials.csv", _TRIAL_COLUMNS),
            "envelope": _read_numeric_csv(mc_dir / "envelope.csv", _ENVELOPE_COLUMNS),
        }

        modes = (_read_modes(linear_dir / "modes_longitudinal.csv")
                 + _read_modes(linear_dir / "modes_lateral.csv"))
        self.modes = {
            "available": bool(modes),
            "modes": modes,
            "reference_point": _read_reference_point(linear_dir / "reference_point.csv"),
        }

    @property
    def available(self) -> bool:
        return bool(self.montecarlo["available"] and self.modes["available"])

    def describe(self) -> dict:
        """Compact summary for /health."""
        return {
            "montecarlo_trials": self.montecarlo["trials"],
            "montecarlo_rows": len(self.montecarlo["trials_table"].get("index", [])),
            "envelope_rows": len(self.montecarlo["envelope"].get("time_s", [])),
            "modes": len(self.modes["modes"]),
        }
