#!/usr/bin/env python3
"""Payloads for the two standalone figures.

Usage:  python3 media/plate/prepare_figures.py MC_DIR LINEAR_DIR OUT_DIR

MC_DIR is a Monte-Carlo campaign directory (trials.csv, trajectories.csv, summary.json).
LINEAR_DIR holds the modal analysis written by ``aether_trim``.
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path


def dispersion(mc_dir: Path) -> dict:
    with (mc_dir / "trials.csv").open(newline="") as handle:
        trials = list(csv.DictReader(handle))
    failed = {int(float(r["index"])) for r in trials if float(r["ok"]) < 0.5}

    tracks: dict[int, dict[str, list[float]]] = {}
    with (mc_dir / "trajectories.csv").open(newline="") as handle:
        for row in csv.DictReader(handle):
            i = int(float(row["trial"]))
            track = tracks.setdefault(i, {"n": [], "e": []})
            track["n"].append(round(float(row["north_m"]), 1))
            track["e"].append(round(float(row["east_m"]), 1))

    summary = json.loads((mc_dir / "summary.json").read_text())
    return {
        "tracks": [{"i": i, "n": t["n"], "e": t["e"], "ok": i not in failed}
                   for i, t in sorted(tracks.items())],
        "meta": {
            "trials": summary["trials"],
            "successes": summary["successes"],
            "failures": summary["failures"],
            "failure_rate": summary["failure_rate"],
            "reasons": summary["failure_reasons"],
        },
    }


def modes(linear_dir: Path) -> list[dict]:
    """The distinct modes, one representative per complex-conjugate pair."""
    rows: list[dict[str, str]] = []
    for name in ("modes_longitudinal.csv", "modes_lateral.csv"):
        path = linear_dir / name
        if path.is_file():
            with path.open(newline="") as handle:
                rows += list(csv.DictReader(handle))

    seen: set[str] = set()
    out = []
    for row in rows:
        if float(row["imag"]) < 0 or row["mode"] in seen:
            continue
        seen.add(row["mode"])
        out.append({
            "mode": row["mode"], "group": row["group"],
            "re": float(row["real"]), "im": float(row["imag"]),
            "wn": float(row["natural_frequency_rad_s"]),
            "zeta": float(row["damping_ratio"]),
            "period": float(row["period_s"]),
            "stable": row["stable"].strip() in ("1", "1.0"),
        })
    return out


def main(mc_dir: Path, linear_dir: Path, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    disp = dispersion(mc_dir)
    mode_list = modes(linear_dir)
    (out_dir / "figures-data.js").write_text(
        "window.DISP = " + json.dumps(disp, separators=(",", ":")) + ";\n"
        "window.MODES = " + json.dumps(mode_list, separators=(",", ":")) + ";\n")
    print(f"dispersion: {len(disp['tracks'])} recorded tracks of {disp['meta']['trials']} trials, "
          f"{disp['meta']['failures']} stopped by a safety limit")
    print("modes: " + ", ".join(
        f"{m['mode']}{'' if m['stable'] else ' (divergent)'}" for m in mode_list))


if __name__ == "__main__":
    if len(sys.argv) != 4:
        raise SystemExit(__doc__)
    main(Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3]))
