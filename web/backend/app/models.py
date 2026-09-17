"""Request and response models.

Every field a visitor can influence is declared here with an explicit type and explicit
bounds. Nothing else reaches the simulator: the service builds the scenario YAML itself from
these validated values, so there is no path from user text to a filename, a YAML fragment or
a command-line argument.
"""

from __future__ import annotations

import hashlib
import json
from enum import Enum
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field


class Controller(str, Enum):
    """Which control law flies the mission."""

    LQR = "lqr"
    PID = "pid"


class Feedback(str, Enum):
    """What the control law is allowed to see."""

    ESTIMATE = "estimate"  # the navigation filter's output, as on a real vehicle
    TRUTH = "truth"        # perfect sensing, for comparison


# --------------------------------------------------------------------------------------
# Bounds. These are the single source of truth: the frontend fetches them from /api/meta so
# the sliders cannot present a value the backend would reject.
# --------------------------------------------------------------------------------------
WIND_SPEED_MIN, WIND_SPEED_MAX = 0.0, 15.0        # m/s
ALTITUDE_MIN, ALTITUDE_MAX = 60.0, 250.0          # m
AIRSPEED_MIN, AIRSPEED_MAX = 22.0, 32.0           # m/s
SENSOR_NOISE_MIN, SENSOR_NOISE_MAX = 0.25, 4.0    # multiple of the nominal sensor sigmas


class SimulationRequest(BaseModel):
    """The complete set of knobs the dashboard exposes.

    Every numeric field is ``strict``, so a string is refused rather than coerced, and
    ``allow_inf_nan=False`` refuses ``NaN``/``Infinity`` outright. With ``extra="forbid"``
    that leaves no field a visitor can smuggle an unexpected value through. The enum fields
    are deliberately not strict: they are how a JSON string like ``"lqr"`` becomes a member,
    and an unlisted string is still rejected.
    """

    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)

    controller: Controller = Controller.LQR
    feedback: Feedback = Feedback.ESTIMATE

    wind_speed: float = Field(
        5.0, ge=WIND_SPEED_MIN, le=WIND_SPEED_MAX, strict=True,
        description="Steady wind magnitude at the reference altitude [m/s]. The turbulence "
                    "intensity is scaled with it, so one slider moves the whole disturbance.")
    target_altitude: float = Field(
        120.0, ge=ALTITUDE_MIN, le=ALTITUDE_MAX, strict=True,
        description="Altitude of the first waypoint [m]; the circuit's altitude profile is "
                    "shifted to match.")
    airspeed: float = Field(
        25.0, ge=AIRSPEED_MIN, le=AIRSPEED_MAX, strict=True,
        description="Commanded true airspeed [m/s]; the per-leg airspeed offsets scale with it.")
    sensor_noise: float = Field(
        1.0, ge=SENSOR_NOISE_MIN, le=SENSOR_NOISE_MAX, strict=True,
        description="Multiplier on every sensor noise and bias sigma. 1.0 is the nominal suite.")

    seed: int = Field(
        20240917, ge=0, le=2**31 - 1, strict=True,
        description="Master random seed. Fixed by default so a run is reproducible.")

    def cache_key(self) -> str:
        """Stable hash of the validated request, used as the result-cache key."""
        canonical = json.dumps(self.model_dump(mode="json"), sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:32]


class MonteCarloRequest(BaseModel):
    """A small live Monte-Carlo demo. The full campaign is served precomputed."""

    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)

    trials: int = Field(
        12, ge=8, le=16, strict=True,
        description="Number of dispersed trials. Deliberately small: the published 256-trial "
                    "campaign is served from precomputed results.")
    controller: Controller = Controller.LQR
    wind_speed: float = Field(5.0, ge=WIND_SPEED_MIN, le=WIND_SPEED_MAX, strict=True)
    airspeed: float = Field(25.0, ge=AIRSPEED_MIN, le=AIRSPEED_MAX, strict=True)
    seed: int = Field(987654321, ge=0, le=2**31 - 1, strict=True)

    def cache_key(self) -> str:
        canonical = json.dumps(self.model_dump(mode="json"), sort_keys=True, separators=(",", ":"))
        return "mc-" + hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:32]


# --------------------------------------------------------------------------------------
# Responses
# --------------------------------------------------------------------------------------
class TrimPoint(BaseModel):
    """The equilibrium the run was initialised from and the LQR was designed at."""

    alpha_deg: float
    theta_deg: float
    elevator_deg: float
    throttle: float
    residual_inf_norm: float


class RunMetrics(BaseModel):
    """Headline numbers for one run."""

    completed: bool
    stable: bool
    termination_reason: str
    simulated_time_s: float
    #: Seconds of data the RMS figures below cover. Zero means the run ended before the
    #: simulator's settling window opened, so those figures are zero for want of samples.
    metrics_window_s: float
    waypoints_reached: int
    laps_completed: int
    rms_cross_track_m: float
    max_cross_track_m: float
    rms_altitude_error_m: float
    max_altitude_error_m: float
    rms_airspeed_error_mps: float
    max_bank_deg: float
    max_alpha_deg: float
    rmse_position_m: float
    rmse_velocity_mps: float
    rmse_attitude_deg: float
    rmse_yaw_deg: float
    min_covariance_eigenvalue: float
    wall_clock_s: float
    real_time_factor: float


class Waypoint(BaseModel):
    index: int
    north: float
    east: float
    altitude: float


class SimulationResponse(BaseModel):
    """One completed interactive run, ready to plot."""

    request: SimulationRequest
    cached: bool
    server_runtime_s: float
    trim: TrimPoint
    metrics: RunMetrics
    waypoints: list[Waypoint]
    #: Column name -> resampled values. Names and units are documented in /api/meta.
    series: dict[str, list[float]]


class MonteCarloTrial(BaseModel):
    index: int
    ok: bool
    mass_kg: float
    cl_alpha: float
    wind_speed_mps: float
    rms_cross_track_m: float
    estimator_position_rmse_m: float
    max_bank_deg: float


class MonteCarloResponse(BaseModel):
    """A small live campaign."""

    request: MonteCarloRequest
    cached: bool
    server_runtime_s: float
    trials: int
    successes: int
    failures: int
    failure_rate: float
    failure_reasons: dict[str, int]
    statistics: list[dict]
    trial_rows: list[MonteCarloTrial]


class HealthResponse(BaseModel):
    """Railway health-check payload."""

    status: Literal["ok", "degraded"]
    version: str
    simulator_available: bool
    precomputed_available: bool
    active_runs: int
    max_concurrency: int
    settings: dict
