"""Parse simulator output into a compact, browser-ready payload.

Parsing uses only the standard library. The log the service reads is already decimated by
the simulator (10 Hz by default), and the series are resampled again here to a fixed length,
so a response stays a few hundred kilobytes regardless of how long the run was.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

from .models import MonteCarloTrial, RunMetrics, TrimPoint, Waypoint

_RAD2DEG = 180.0 / math.pi

#: Output name -> (source column, transform). ``None`` means the series is derived below.
_DIRECT: dict[str, tuple[str, str]] = {
    "t": ("time_s", "id"),
    "north": ("pn_m", "id"),
    "east": ("pe_m", "id"),
    "altitude": ("altitude_m", "id"),
    "qw": ("qw", "id"),
    "qx": ("qx", "id"),
    "qy": ("qy", "id"),
    "qz": ("qz", "id"),
    "roll_deg": ("roll_rad", "deg"),
    "pitch_deg": ("pitch_rad", "deg"),
    "yaw_deg": ("yaw_rad", "deg"),
    "p_dps": ("p_radps", "deg"),
    "q_dps": ("q_radps", "deg"),
    "r_dps": ("r_radps", "deg"),
    "vn": ("vn_mps", "id"),
    "ve": ("ve_mps", "id"),
    "vd": ("vd_mps", "id"),
    "airspeed": ("airspeed_mps", "id"),
    "alpha_deg": ("alpha_rad", "deg"),
    "beta_deg": ("beta_rad", "deg"),
    "cross_track": ("cross_track_m", "id"),
    "cmd_altitude": ("cmd_altitude_m", "id"),
    "cmd_airspeed": ("cmd_airspeed_mps", "id"),
    "wp_index": ("wp_index", "id"),
    "elevator_deg": ("act_elevator_rad", "deg"),
    "aileron_deg": ("act_aileron_rad", "deg"),
    "rudder_deg": ("act_rudder_rad", "deg"),
    "throttle": ("act_throttle", "id"),
    "est_north": ("est_pn_m", "id"),
    "est_east": ("est_pe_m", "id"),
    "est_altitude": ("est_pd_m", "neg"),
    "est_airspeed": ("est_airspeed_mps", "id"),
    "est_beta_deg": ("est_beta_rad", "deg"),
    "sig_north": ("sig_pn", "id"),
    "sig_east": ("sig_pe", "id"),
    "sig_down": ("sig_pd", "id"),
    "sig_roll_deg": ("sig_att_x", "deg"),
    "sig_pitch_deg": ("sig_att_y", "deg"),
    "sig_yaw_deg": ("sig_att_z", "deg"),
    "est_pos_err": ("est_pos_err_m", "id"),
    "est_att_err_deg": ("est_att_err_rad", "deg"),
    "wind_n": ("wind_n_mps", "id"),
    "wind_e": ("wind_e_mps", "id"),
    "est_wind_n": ("est_wind_n_mps", "id"),
    "est_wind_e": ("est_wind_e_mps", "id"),
    "est_roll_deg": ("est_roll_rad", "deg"),
    "est_pitch_deg": ("est_pitch_rad", "deg"),
    "est_yaw_deg": ("est_yaw_rad", "deg"),
    "est_vel_err": ("est_vel_err_mps", "id"),
    "climb_rate": ("climb_rate_mps", "id"),
    "course_deg": ("course_rad", "deg"),
    "cmd_course_deg": ("cmd_course_rad", "deg"),
    "roll_cmd_deg": ("roll_cmd_rad", "deg"),
    "pitch_cmd_deg": ("pitch_cmd_rad", "deg"),
    "sig_bg_x_dps": ("sig_bgx", "deg"),
    "sig_bg_y_dps": ("sig_bgy", "deg"),
    "sig_bg_z_dps": ("sig_bgz", "deg"),
}

#: Derived series: (output name, minuend, subtrahend, transform).
_DIFFERENCES = [
    ("err_north", "est_pn_m", "pn_m", "id"),
    ("err_east", "est_pe_m", "pe_m", "id"),
    ("err_down", "est_pd_m", "pd_m", "id"),
    ("err_roll_deg", "est_roll_rad", "roll_rad", "wrapdeg"),
    ("err_pitch_deg", "est_pitch_rad", "pitch_rad", "wrapdeg"),
    ("err_yaw_deg", "est_yaw_rad", "yaw_rad", "wrapdeg"),
    ("altitude_error", "altitude_m", "cmd_altitude_m", "id"),
    ("airspeed_error", "airspeed_mps", "cmd_airspeed_mps", "id"),
    ("bg_err_x_dps", "est_bgx", "true_bgx", "deg"),
    ("bg_err_y_dps", "est_bgy", "true_bgy", "deg"),
    ("bg_err_z_dps", "est_bgz", "true_bgz", "deg"),
]


def _wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _apply(kind: str, value: float) -> float:
    if kind == "deg":
        return value * _RAD2DEG
    if kind == "neg":
        return -value
    if kind == "wrapdeg":
        return _wrap_pi(value) * _RAD2DEG
    return value


def _sample_indices(n_rows: int, target: int) -> list[int]:
    """Evenly spaced row indices, always including the first and last row."""
    if n_rows <= 0:
        return []
    if n_rows <= target:
        return list(range(n_rows))
    return [round(i * (n_rows - 1) / (target - 1)) for i in range(target)]


def read_states(path: Path, points: int) -> dict[str, list[float]]:
    """Read ``states.csv`` and return the resampled, browser-facing series.

    Missing columns are skipped rather than raising, so a log written by an older build still
    yields whatever it does contain.
    """
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        try:
            header = next(reader)
        except StopIteration:
            return {}
        index = {name: i for i, name in enumerate(header)}
        rows = [row for row in reader if len(row) == len(header)]

    picks = _sample_indices(len(rows), points)
    if not picks:
        return {}

    def column(name: str, kind: str) -> list[float] | None:
        position = index.get(name)
        if position is None:
            return None
        out = []
        for i in picks:
            try:
                out.append(round(_apply(kind, float(rows[i][position])), 4))
            except (ValueError, IndexError):
                out.append(0.0)
        return out

    series: dict[str, list[float]] = {}
    for out_name, (src, kind) in _DIRECT.items():
        values = column(src, kind)
        if values is not None:
            series[out_name] = values

    for out_name, lhs, rhs, kind in _DIFFERENCES:
        a, b = index.get(lhs), index.get(rhs)
        if a is None or b is None:
            continue
        values = []
        for i in picks:
            try:
                delta = float(rows[i][a]) - float(rows[i][b])
            except (ValueError, IndexError):
                delta = 0.0
            values.append(round(_apply(kind, delta), 4))
        series[out_name] = values

    return series


def read_waypoints(path: Path) -> list[Waypoint]:
    if not path.is_file():
        return []
    out = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            try:
                out.append(Waypoint(index=int(float(row["index"])),
                                    north=float(row["north_m"]),
                                    east=float(row["east_m"]),
                                    altitude=float(row["altitude_m"])))
            except (KeyError, ValueError):
                continue
    return out


def read_summary(path: Path) -> tuple[TrimPoint, RunMetrics]:
    """Parse ``summary.json`` into the typed trim point and metric block."""
    data = json.loads(path.read_text(encoding="utf-8"))
    trim = data.get("trim", {})
    tracking = data.get("tracking", {})
    estimator = data.get("estimator", {})
    performance = data.get("performance", {})

    trim_point = TrimPoint(
        alpha_deg=float(trim.get("alpha_deg", 0.0)),
        theta_deg=float(trim.get("theta_deg", 0.0)),
        elevator_deg=float(trim.get("elevator_deg", 0.0)),
        throttle=float(trim.get("throttle", 0.0)),
        residual_inf_norm=float(trim.get("residual_inf_norm", 0.0)),
    )
    metrics = RunMetrics(
        completed=bool(data.get("completed", False)),
        stable=bool(data.get("stable", False)),
        termination_reason=str(data.get("termination_reason", "unknown")),
        simulated_time_s=float(data.get("simulated_time_s", 0.0)),
        metrics_window_s=float(data.get("metrics_window_s", 0.0)),
        waypoints_reached=int(tracking.get("waypoints_reached", 0)),
        laps_completed=int(tracking.get("laps_completed", 0)),
        rms_cross_track_m=float(tracking.get("rms_cross_track_m", 0.0)),
        max_cross_track_m=float(tracking.get("max_cross_track_m", 0.0)),
        rms_altitude_error_m=float(tracking.get("rms_altitude_error_m", 0.0)),
        max_altitude_error_m=float(tracking.get("max_altitude_error_m", 0.0)),
        rms_airspeed_error_mps=float(tracking.get("rms_airspeed_error_mps", 0.0)),
        max_bank_deg=float(tracking.get("max_bank_deg", 0.0)),
        max_alpha_deg=float(tracking.get("max_alpha_deg", 0.0)),
        rmse_position_m=float(estimator.get("rmse_position_m", 0.0)),
        rmse_velocity_mps=float(estimator.get("rmse_velocity_mps", 0.0)),
        rmse_attitude_deg=float(estimator.get("rmse_attitude_deg", 0.0)),
        rmse_yaw_deg=float(estimator.get("rmse_yaw_deg", 0.0)),
        min_covariance_eigenvalue=float(estimator.get("min_covariance_eigenvalue", 0.0)),
        wall_clock_s=float(performance.get("wall_clock_s", 0.0)),
        real_time_factor=float(performance.get("real_time_factor", 0.0)),
    )
    return trim_point, metrics


def read_montecarlo(directory: Path) -> dict:
    """Parse a campaign directory into the live Monte-Carlo payload."""
    summary = json.loads((directory / "summary.json").read_text(encoding="utf-8"))
    rows: list[MonteCarloTrial] = []
    trials_csv = directory / "trials.csv"
    if trials_csv.is_file():
        with trials_csv.open(newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                try:
                    rows.append(MonteCarloTrial(
                        index=int(float(row["index"])),
                        ok=float(row["ok"]) > 0.5,
                        mass_kg=round(float(row["mass_kg"]), 4),
                        cl_alpha=round(float(row["cl_alpha"]), 4),
                        wind_speed_mps=round(float(row["wind_speed_mps"]), 4),
                        rms_cross_track_m=round(float(row["rms_cross_track_m"]), 4),
                        estimator_position_rmse_m=round(float(row["estimator_position_rmse_m"]), 4),
                        max_bank_deg=round(float(row["max_bank_deg"]), 4),
                    ))
                except (KeyError, ValueError):
                    continue
    return {
        "trials": int(summary.get("trials", len(rows))),
        "successes": int(summary.get("successes", 0)),
        "failures": int(summary.get("failures", 0)),
        "failure_rate": float(summary.get("failure_rate", 0.0)),
        "failure_reasons": {str(k): int(v) for k, v in summary.get("failure_reasons", {}).items()},
        "statistics": summary.get("statistics", []),
        "trial_rows": rows,
    }
