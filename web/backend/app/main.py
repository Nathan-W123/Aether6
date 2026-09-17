"""FastAPI application for the Aether-6 dashboard.

Routes
------
``GET  /health``                     Railway health check; also reports what the process found.
``GET  /api/meta``                   Presets, slider bounds, series units and service limits.
``GET  /api/precomputed/montecarlo`` The committed 256-trial campaign.
``GET  /api/precomputed/modes``      The committed modal analysis at the reference trim.
``POST /api/simulate``               One interactive closed-loop run.
``POST /api/montecarlo``             A small live dispersion campaign (8-16 trials).

Everything under ``/`` that is not an API route is served from the built frontend.
"""

from __future__ import annotations

import os
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request
from fastapi.exceptions import RequestValidationError
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from . import models as m
from .precomputed import Precomputed
from .presets import as_meta
from .runner import RunError, SimulationRunner
from .scenario import CIRCUIT, TURBULENCE_PER_WIND
from .settings import SETTINGS

VERSION = "1.0.0"

#: Unit label per returned series name, so the frontend never hard-codes a unit string.
SERIES_UNITS: dict[str, str] = {
    "t": "s", "north": "m", "east": "m", "altitude": "m", "cross_track": "m",
    "cmd_altitude": "m", "altitude_error": "m",
    "qw": "-", "qx": "-", "qy": "-", "qz": "-",
    "roll_deg": "deg", "pitch_deg": "deg", "yaw_deg": "deg",
    "roll_cmd_deg": "deg", "pitch_cmd_deg": "deg",
    "course_deg": "deg", "cmd_course_deg": "deg",
    "p_dps": "deg/s", "q_dps": "deg/s", "r_dps": "deg/s",
    "vn": "m/s", "ve": "m/s", "vd": "m/s", "climb_rate": "m/s",
    "airspeed": "m/s", "cmd_airspeed": "m/s", "airspeed_error": "m/s",
    "alpha_deg": "deg", "beta_deg": "deg",
    "wp_index": "-",
    "elevator_deg": "deg", "aileron_deg": "deg", "rudder_deg": "deg", "throttle": "-",
    "est_north": "m", "est_east": "m", "est_altitude": "m", "est_airspeed": "m/s",
    "est_beta_deg": "deg", "est_roll_deg": "deg", "est_pitch_deg": "deg", "est_yaw_deg": "deg",
    "est_pos_err": "m", "est_vel_err": "m/s", "est_att_err_deg": "deg",
    "err_north": "m", "err_east": "m", "err_down": "m",
    "err_roll_deg": "deg", "err_pitch_deg": "deg", "err_yaw_deg": "deg",
    "sig_north": "m", "sig_east": "m", "sig_down": "m",
    "sig_roll_deg": "deg", "sig_pitch_deg": "deg", "sig_yaw_deg": "deg",
    "sig_bg_x_dps": "deg/s", "sig_bg_y_dps": "deg/s", "sig_bg_z_dps": "deg/s",
    "bg_err_x_dps": "deg/s", "bg_err_y_dps": "deg/s", "bg_err_z_dps": "deg/s",
    "wind_n": "m/s", "wind_e": "m/s", "est_wind_n": "m/s", "est_wind_e": "m/s",
}

app = FastAPI(
    title="Aether-6 flight simulator",
    version=VERSION,
    description="Six-degree-of-freedom fixed-wing flight dynamics, guidance, navigation and "
                "control, served from the compiled C++ simulator.",
    docs_url="/api/docs",
    openapi_url="/api/openapi.json",
)
# The series payload is a few hundred kilobytes of decimal text and compresses ~10:1.
app.add_middleware(GZipMiddleware, minimum_size=2048)

RUNNER = SimulationRunner(SETTINGS)
PRECOMPUTED = Precomputed(SETTINGS)


@app.exception_handler(RunError)
async def _run_error_handler(_: Request, exc: RunError) -> JSONResponse:
    return JSONResponse(status_code=exc.status_code, content={"detail": exc.detail})


@app.exception_handler(RequestValidationError)
async def _validation_error_handler(_: Request, exc: RequestValidationError) -> JSONResponse:
    """Report *which* field was rejected and why, without echoing what was sent.

    FastAPI's default handler includes the offending input in the response, which both
    reflects a visitor's payload back at them and fails outright on a value the JSON encoder
    cannot represent — a body containing the bare token ``NaN`` parses fine but then makes
    the error response itself raise, turning a 422 into a 500. Reporting only the location,
    message and error type avoids both problems.
    """
    detail = [{"field": ".".join(str(part) for part in error.get("loc", ())[1:]) or "body",
               "message": str(error.get("msg", "invalid value")),
               "type": str(error.get("type", "value_error"))}
              for error in exc.errors()]
    return JSONResponse(status_code=422, content={"detail": detail})


# ------------------------------------------------------------------------------------------
# Service
# ------------------------------------------------------------------------------------------
@app.get("/health", response_model=m.HealthResponse, tags=["service"])
async def health() -> m.HealthResponse:
    """Liveness and readiness in one payload.

    The service is ``ok`` only when the compiled binaries are executable; a container that
    built the frontend but not the simulator reports ``degraded`` rather than pretending.
    """
    available = RUNNER.simulator_available()
    return m.HealthResponse(
        status="ok" if available else "degraded",
        version=VERSION,
        simulator_available=available,
        precomputed_available=PRECOMPUTED.available,
        active_runs=RUNNER.active_runs,
        max_concurrency=SETTINGS.max_concurrency,
        settings={**SETTINGS.describe(),
                  "precomputed": PRECOMPUTED.describe(),
                  "cache": RUNNER.cache_stats()},
    )


@app.get("/api/meta", tags=["service"])
async def meta() -> dict:
    """Everything the frontend needs to render its controls without hard-coding limits."""
    return {
        "version": VERSION,
        "presets": as_meta(),
        "bounds": {
            "wind_speed": {"min": m.WIND_SPEED_MIN, "max": m.WIND_SPEED_MAX,
                           "step": 0.5, "unit": "m/s", "label": "Wind speed"},
            "target_altitude": {"min": m.ALTITUDE_MIN, "max": m.ALTITUDE_MAX,
                                "step": 5.0, "unit": "m", "label": "Target altitude"},
            "airspeed": {"min": m.AIRSPEED_MIN, "max": m.AIRSPEED_MAX,
                         "step": 0.5, "unit": "m/s", "label": "Commanded airspeed"},
            "sensor_noise": {"min": m.SENSOR_NOISE_MIN, "max": m.SENSOR_NOISE_MAX,
                             "step": 0.25, "unit": "x nominal", "label": "Sensor noise"},
        },
        "controllers": [
            {"id": "lqr", "label": "LQR",
             "detail": "Servo-LQR with integral augmentation, designed on the linearised "
                       "model at the trim point."},
            {"id": "pid", "label": "PID",
             "detail": "Classical successive loop closure with anti-windup."},
        ],
        "feedback": [
            {"id": "estimate", "label": "EKF estimate",
             "detail": "The controller sees only the navigation filter's output."},
            {"id": "truth", "label": "Truth",
             "detail": "Perfect sensing, for isolating control error from estimation error."},
        ],
        "limits": {
            "duration_s": SETTINGS.duration_s,
            "dt_s": SETTINGS.dt_s,
            "series_points": SETTINGS.series_points,
            "mc_min_trials": SETTINGS.mc_min_trials,
            "mc_max_trials": SETTINGS.mc_max_trials,
            "mc_duration_s": SETTINGS.mc_duration_s,
            "max_concurrency": SETTINGS.max_concurrency,
            "simulate_timeout_s": SETTINGS.simulate_timeout_s,
            "montecarlo_timeout_s": SETTINGS.montecarlo_timeout_s,
        },
        "mission": {
            "waypoints": [{"north": w[0], "east": w[1]} for w in CIRCUIT],
            "turbulence_per_wind": TURBULENCE_PER_WIND,
            "terminal": "loop",
        },
        "series_units": SERIES_UNITS,
    }


# ------------------------------------------------------------------------------------------
# Precomputed results
# ------------------------------------------------------------------------------------------
@app.get("/api/precomputed/montecarlo", tags=["precomputed"])
async def precomputed_montecarlo() -> dict:
    """The committed 256-trial campaign, far too expensive to run per visitor."""
    if not PRECOMPUTED.montecarlo["available"]:
        raise HTTPException(status_code=503, detail="precomputed results are not bundled")
    return PRECOMPUTED.montecarlo


@app.get("/api/precomputed/modes", tags=["precomputed"])
async def precomputed_modes() -> dict:
    """Eigenvalues of the numerically linearised model at the reference trim point."""
    if not PRECOMPUTED.modes["available"]:
        raise HTTPException(status_code=503, detail="precomputed results are not bundled")
    return PRECOMPUTED.modes


# ------------------------------------------------------------------------------------------
# Live runs
# ------------------------------------------------------------------------------------------
@app.post("/api/simulate", response_model=m.SimulationResponse, tags=["simulate"])
async def simulate(request: m.SimulationRequest) -> m.SimulationResponse:
    """Run one closed-loop mission and return its trajectory, metrics and estimator history.

    The request body is the *only* input: FastAPI rejects anything outside the declared
    bounds before this function is entered, and the scenario is generated from the validated
    values.
    """
    return await RUNNER.simulate(request)


@app.post("/api/montecarlo", response_model=m.MonteCarloResponse, tags=["simulate"])
async def montecarlo(request: m.MonteCarloRequest) -> m.MonteCarloResponse:
    """Run a small live dispersion campaign. The published campaign is served precomputed."""
    return await RUNNER.montecarlo(request)


# ------------------------------------------------------------------------------------------
# Static frontend
# ------------------------------------------------------------------------------------------
def _mount_frontend() -> None:
    """Serve the built single-page app from the site root, when it is present.

    Mounted last so it cannot shadow an API route. In a development checkout the directory
    does not exist and the API simply runs on its own.
    """
    dist = SETTINGS.static_dir
    if not dist.is_dir():
        return

    assets = dist / "assets"
    if assets.is_dir():
        app.mount("/assets", StaticFiles(directory=str(assets)), name="assets")

    @app.get("/{path:path}", include_in_schema=False)
    async def spa(path: str) -> FileResponse:
        candidate = (dist / path).resolve() if path else dist / "index.html"
        # Only ever serve a file that really lives under the built bundle.
        if path and candidate.is_file() and str(candidate).startswith(str(dist)):
            return FileResponse(candidate)
        index = dist / "index.html"
        if not index.is_file():
            raise HTTPException(status_code=404, detail="frontend bundle not found")
        return FileResponse(index)


_mount_frontend()


def main() -> None:  # pragma: no cover - container entry point
    """Run the service. Railway injects ``PORT``; bind every interface inside the container."""
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("PORT", "8000")),
                log_level=os.environ.get("AETHER_LOG_LEVEL", "info"), access_log=False)


if __name__ == "__main__":  # pragma: no cover
    main()
