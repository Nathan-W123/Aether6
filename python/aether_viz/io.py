"""Loaders for the CSV/JSON artefacts produced by the C++ tools.

Every loader returns plain pandas/NumPy objects so the plotting code stays independent of
how the simulator writes its logs. Column names carry their SI unit as a suffix
(``_m``, ``_mps``, ``_rad``, ...) exactly as written by ``aether::util::CsvWriter``.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd


@dataclass
class RunData:
    """Everything produced by one ``aether_sim`` run."""

    name: str
    directory: Path
    states: pd.DataFrame
    waypoints: pd.DataFrame
    summary: dict = field(default_factory=dict)
    sensors: dict = field(default_factory=dict)

    @property
    def time(self) -> np.ndarray:
        """Log time base [s]."""
        return self.states["time_s"].to_numpy()

    def has(self, column: str) -> bool:
        """True when the state log contains ``column``."""
        return column in self.states.columns


def _read_csv(path: Path) -> pd.DataFrame | None:
    return pd.read_csv(path) if path.is_file() else None


def load_run(directory: str | Path) -> RunData:
    """Load a simulation output directory written by ``aether_sim``.

    Parameters
    ----------
    directory:
        Directory containing ``states.csv`` (required) and, optionally, ``waypoints.csv``,
        ``summary.json`` and the ``sensor_*.csv`` files.

    Raises
    ------
    FileNotFoundError
        If ``states.csv`` is missing.
    """
    directory = Path(directory)
    states_path = directory / "states.csv"
    if not states_path.is_file():
        raise FileNotFoundError(
            f"{states_path} not found - run 'aether_sim' for this scenario first"
        )
    states = pd.read_csv(states_path)

    waypoints = _read_csv(directory / "waypoints.csv")
    if waypoints is None:
        waypoints = pd.DataFrame(
            columns=["index", "north_m", "east_m", "altitude_m", "airspeed_mps"]
        )

    summary = {}
    summary_path = directory / "summary.json"
    if summary_path.is_file():
        summary = json.loads(summary_path.read_text())

    sensors = {}
    for key in ("gps", "imu", "baro", "mag", "airspeed"):
        frame = _read_csv(directory / f"sensor_{key}.csv")
        if frame is not None:
            sensors[key] = frame

    return RunData(
        name=summary.get("scenario", directory.name),
        directory=directory,
        states=states,
        waypoints=waypoints,
        summary=summary,
        sensors=sensors,
    )


def _read_matrix(path: Path) -> pd.DataFrame:
    frame = pd.read_csv(path)
    return frame.set_index("row")


def load_linear_model(directory: str | Path) -> dict:
    """Load the linearised model exported by ``aether_trim`` / ``aether_sim``.

    Returns a dict with keys ``A_full``, ``B_full``, ``A_lon``, ``B_lon``, ``A_lat``,
    ``B_lat`` (DataFrames indexed by state name), ``modes`` (DataFrame) and ``reference``
    (DataFrame of the trim point). Optional ``trim_sweep`` is present when
    ``aether_trim --sweep`` was used.
    """
    directory = Path(directory)
    if not (directory / "modes.csv").is_file():
        raise FileNotFoundError(
            f"{directory}/modes.csv not found - run 'aether_trim' first"
        )
    out = {
        "modes": pd.read_csv(directory / "modes.csv"),
        "reference": pd.read_csv(directory / "reference_point.csv"),
    }
    for name in ("A_full", "B_full", "A_lon", "B_lon", "A_lat", "B_lat"):
        path = directory / f"{name}.csv"
        if path.is_file():
            out[name] = _read_matrix(path)
    sweep = directory / "trim_sweep.csv"
    if sweep.is_file():
        out["trim_sweep"] = pd.read_csv(sweep)
    return out


def load_monte_carlo(directory: str | Path) -> dict:
    """Load a Monte-Carlo campaign written by ``aether_mc``.

    Returns a dict with ``trials``, ``trajectories``, ``envelope`` (DataFrames) and
    ``summary`` (dict).
    """
    directory = Path(directory)
    trials_path = directory / "trials.csv"
    if not trials_path.is_file():
        raise FileNotFoundError(
            f"{trials_path} not found - run 'aether_mc' first"
        )
    summary_path = directory / "summary.json"
    return {
        "trials": pd.read_csv(trials_path),
        "trajectories": _read_csv(directory / "trajectories.csv"),
        "envelope": _read_csv(directory / "envelope.csv"),
        "summary": json.loads(summary_path.read_text()) if summary_path.is_file() else {},
    }


def load_integrator_study(directory: str | Path) -> dict:
    """Load the integrator comparison written by ``aether_integrators``."""
    directory = Path(directory)
    convergence = directory / "convergence.csv"
    if not convergence.is_file():
        raise FileNotFoundError(
            f"{convergence} not found - run 'aether_integrators' first"
        )
    return {
        "convergence": pd.read_csv(convergence),
        "tolerance": _read_csv(directory / "dopri_tolerance.csv"),
        "aircraft": _read_csv(directory / "aircraft_comparison.csv"),
    }


def quaternion_to_rotation(q: np.ndarray) -> np.ndarray:
    """Rotation matrices from an (N, 4) array of Hamilton quaternions ``[w, x, y, z]``.

    The result maps **body** vectors into **NED**, matching ``aether::math::quatToRotation``.
    """
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    n = q.shape[0]
    R = np.empty((n, 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - w * z)
    R[:, 0, 2] = 2 * (x * z + w * y)
    R[:, 1, 0] = 2 * (x * y + w * z)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - w * x)
    R[:, 2, 0] = 2 * (x * z - w * y)
    R[:, 2, 1] = 2 * (y * z + w * x)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R
