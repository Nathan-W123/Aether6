#!/usr/bin/env python3
"""Turn a simulator run into the payload the plate renderer reads.

Usage:  python3 media/plate/prepare.py RUN_DIR OUT_DIR

RUN_DIR is a directory written by ``aether_sim`` (states.csv, waypoints.csv, summary.json).
Only the columns the drawing needs are carried across, at the log's own rate.

Ground speed is computed here because it is what the plate encodes as colour: with a steady
wind, ground speed swings far more than airspeed around a circuit, so the gradient shows the
wind without drawing a single arrow.
"""
from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path


def main(run_dir: Path, out_dir: Path) -> None:
    with (run_dir / "states.csv").open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise SystemExit(f"no rows in {run_dir / 'states.csv'}")

    def col(name: str, places: int = 4) -> list[float]:
        return [round(float(r[name]), places) for r in rows]

    ground_speed = [round(math.hypot(float(r["vn_mps"]), float(r["ve_mps"])), 3) for r in rows]

    payload = {
        "t": col("time_s"), "n": col("pn_m"), "e": col("pe_m"), "alt": col("altitude_m"),
        "qw": col("qw"), "qx": col("qx"), "qy": col("qy"), "qz": col("qz"),
        "gs": ground_speed, "va": col("airspeed_mps"),
        "xtrack": col("cross_track_m"),
        "en": col("est_pn_m"), "ee": col("est_pe_m"),
        "ealt": [round(-float(r["est_pd_m"]), 4) for r in rows],
        "wp": [int(float(r["wp_index"])) for r in rows],
    }

    with (run_dir / "waypoints.csv").open(newline="") as handle:
        payload["waypoints"] = [
            {"i": int(float(r["index"])), "n": float(r["north_m"]),
             "e": float(r["east_m"]), "alt": float(r["altitude_m"])}
            for r in csv.DictReader(handle)
        ]

    summary = json.loads((run_dir / "summary.json").read_text())
    middle = rows[len(rows) // 2]
    payload["meta"] = {
        "controller": summary["controller"].upper(),
        "xtrack_rms": round(summary["tracking"]["rms_cross_track_m"], 2),
        "nav_rmse": round(summary["estimator"]["rmse_position_m"], 2),
        "laps": summary["tracking"]["laps_completed"],
        "duration": summary["simulated_time_s"],
        "wind": round(math.hypot(float(middle["wind_n_mps"]), float(middle["wind_e_mps"])), 1),
        "gs_min": round(min(ground_speed), 1),
        "gs_max": round(max(ground_speed), 1),
    }

    out_dir.mkdir(parents=True, exist_ok=True)
    text = json.dumps(payload, separators=(",", ":"))
    (out_dir / "flight.js").write_text(f"window.FLIGHT = {text};\n")
    print(f"flight.js: {len(text) / 1024:.0f} KB, {len(rows)} samples, "
          f"{payload['meta']['duration']:.0f} s")
    print(f"ground speed {payload['meta']['gs_min']}–{payload['meta']['gs_max']} m/s "
          f"in {payload['meta']['wind']} m/s wind")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    main(Path(sys.argv[1]), Path(sys.argv[2]))
