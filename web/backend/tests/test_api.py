"""Tests for the dashboard service.

The emphasis is on the boundary: what a visitor can and cannot make the service do. The
simulator itself is covered by the C++ suite (``make test``); here we check validation,
bounds, caching, cleanup and the shape of every response.
"""

from __future__ import annotations

import asyncio
import dataclasses
import glob
import sys
from pathlib import Path

import pytest
import yaml
from fastapi.testclient import TestClient

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from app import models  # noqa: E402
from app.main import RUNNER, app  # noqa: E402
from app.runner import RunError, SimulationRunner  # noqa: E402
from app.scenario import (assert_plain_data, build_montecarlo_scenario, build_scenario,  # noqa: E402
                          write_scenario)
from app.settings import SETTINGS  # noqa: E402

client = TestClient(app)

_HAVE_BINARIES = RUNNER.simulator_available()
needs_binaries = pytest.mark.skipif(not _HAVE_BINARIES,
                                    reason="simulator binaries are not built")


# ---------------------------------------------------------------------------------------
# Service endpoints
# ---------------------------------------------------------------------------------------
def test_health_reports_readiness():
    body = client.get("/health").json()
    assert body["status"] in ("ok", "degraded")
    assert body["status"] == ("ok" if _HAVE_BINARIES else "degraded")
    assert body["max_concurrency"] == SETTINGS.max_concurrency
    assert body["active_runs"] == 0


def test_meta_bounds_match_the_models():
    bounds = client.get("/api/meta").json()["bounds"]
    assert bounds["wind_speed"]["max"] == models.WIND_SPEED_MAX
    assert bounds["airspeed"]["min"] == models.AIRSPEED_MIN
    assert bounds["target_altitude"]["max"] == models.ALTITUDE_MAX
    assert bounds["sensor_noise"]["max"] == models.SENSOR_NOISE_MAX


def test_meta_presets_are_valid_requests():
    for preset in client.get("/api/meta").json()["presets"]:
        # Every preset must survive the same validation a visitor's request does.
        models.SimulationRequest(**preset["request"])


# ---------------------------------------------------------------------------------------
# Input validation: the only surface a visitor can reach
# ---------------------------------------------------------------------------------------
@pytest.mark.parametrize("payload", [
    {"airspeed": 5.0},                       # below the stall-safe floor
    {"airspeed": 99.0},                      # above the envelope
    {"wind_speed": -1.0},                    # negative magnitude
    {"wind_speed": 400.0},
    {"target_altitude": 1e9},
    {"target_altitude": 0.0},
    {"sensor_noise": 0.0},
    {"sensor_noise": 1e6},
    {"seed": -1},
    {"seed": 2**63},
    {"controller": "mpc"},                   # not an offered control law
    {"feedback": "oracle"},
    {"controller": "lqr; rm -rf /"},         # shell metacharacters are simply not a value
    {"extra_field": 1},                      # extra="forbid"
    {"aircraft": "/etc/passwd"},             # no path may be supplied
    {"duration": 100000},                    # cost is not user-controllable
    {"airspeed": "25"},                      # right value, wrong type is still refused
])
def test_simulate_rejects_out_of_bounds_input(payload):
    assert client.post("/api/simulate", json=payload).status_code == 422


@pytest.mark.parametrize("payload", [
    {"trials": 0},
    {"trials": 7},        # below the published floor
    {"trials": 17},       # above the public ceiling
    {"trials": 256},      # the full campaign is precomputed, never live
    {"threads": 64},      # not a field a visitor may set
    {"output_dir": "/tmp/x"},
])
def test_montecarlo_rejects_out_of_bounds_input(payload):
    assert client.post("/api/montecarlo", json=payload).status_code == 422


@pytest.mark.parametrize("body", [
    '{"airspeed": NaN}',
    '{"airspeed": Infinity}',
    '{"wind_speed": -Infinity}',
    '{"seed": 1e400}',
    'not json at all',
    '[]',
])
def test_simulate_rejects_malformed_bodies(body):
    """Raw bodies a JSON encoder would not produce are refused, not coerced."""
    response = client.post("/api/simulate", content=body,
                           headers={"content-type": "application/json"})
    assert response.status_code == 422


def test_montecarlo_trial_ceiling_is_small():
    assert SETTINGS.mc_max_trials <= 16
    assert models.MonteCarloRequest.model_fields["trials"].metadata[-1].le <= 16


# ---------------------------------------------------------------------------------------
# Scenario generation: no user text ever reaches a file or an argument vector
# ---------------------------------------------------------------------------------------
def test_generated_scenario_is_plain_data(tmp_path):
    request = models.SimulationRequest(wind_speed=12.0, target_altitude=200.0, airspeed=31.0,
                                       sensor_noise=3.0, controller=models.Controller.PID)
    scenario = build_scenario(request, SETTINGS, tmp_path)
    assert_plain_data(scenario)

    path = tmp_path / "scenario.yaml"
    write_scenario(scenario, path)
    reloaded = yaml.safe_load(path.read_text())
    assert reloaded["control"]["type"] == "pid"
    assert reloaded["logging"]["output_dir"] == str(tmp_path)
    # Only paths the service owns appear in the file.
    assert reloaded["aircraft"] == str(SETTINGS.aircraft_file)


def test_scenario_waypoints_stay_inside_the_flight_envelope(tmp_path):
    for altitude in (models.ALTITUDE_MIN, 120.0, models.ALTITUDE_MAX):
        for airspeed in (models.AIRSPEED_MIN, models.AIRSPEED_MAX):
            scenario = build_scenario(
                models.SimulationRequest(target_altitude=altitude, airspeed=airspeed),
                SETTINGS, tmp_path)
            for wp in scenario["guidance"]["waypoints"]:
                assert 40.0 <= wp["altitude"] <= 330.0
                assert models.AIRSPEED_MIN <= wp["airspeed"] <= models.AIRSPEED_MAX + 2.0


def test_montecarlo_scenario_disables_logging(tmp_path):
    scenario = build_montecarlo_scenario(models.MonteCarloRequest(trials=8), SETTINGS, tmp_path)
    assert scenario["logging"]["enabled"] is False          # no per-trial CSV to clean up
    assert scenario["monte_carlo"]["trials"] == 8
    assert scenario["monte_carlo"]["threads"] == SETTINGS.mc_threads
    assert_plain_data(scenario)


def test_assert_plain_data_rejects_injected_objects():
    with pytest.raises(ValueError):
        assert_plain_data({"ok": object()})
    with pytest.raises(ValueError):
        assert_plain_data({"a b; rm -rf /": 1})
    with pytest.raises(ValueError):
        assert_plain_data({"value": float("inf")})


# ---------------------------------------------------------------------------------------
# Caching
# ---------------------------------------------------------------------------------------
def test_cache_key_is_stable_and_discriminating():
    a = models.SimulationRequest(wind_speed=5.0)
    b = models.SimulationRequest(wind_speed=5.0)
    c = models.SimulationRequest(wind_speed=5.5)
    assert a.cache_key() == b.cache_key()
    assert a.cache_key() != c.cache_key()
    assert a.cache_key() != models.MonteCarloRequest().cache_key()


def test_request_models_are_immutable():
    request = models.SimulationRequest()
    with pytest.raises(Exception):
        request.airspeed = 40.0


# ---------------------------------------------------------------------------------------
# Live runs
# ---------------------------------------------------------------------------------------
@needs_binaries
def test_simulate_returns_a_complete_payload():
    response = client.post("/api/simulate", json={"wind_speed": 4.0, "seed": 7})
    assert response.status_code == 200
    body = response.json()

    assert body["metrics"]["completed"] is True
    assert body["metrics"]["laps_completed"] >= 1
    assert body["metrics"]["rms_cross_track_m"] < 60.0
    assert abs(body["trim"]["residual_inf_norm"]) < 1e-6

    series = body["series"]
    length = SETTINGS.series_points
    assert {"t", "north", "east", "altitude", "qw", "elevator_deg", "est_pos_err"} <= set(series)
    assert all(len(v) == length for v in series.values())
    assert series["t"][0] == 0.0 and series["t"][-1] > 0.0
    assert len(body["waypoints"]) == 6


@needs_binaries
def test_identical_requests_are_served_from_cache():
    payload = {"wind_speed": 6.5, "seed": 4242}
    first = client.post("/api/simulate", json=payload).json()
    second = client.post("/api/simulate", json=payload).json()
    assert first["cached"] is False
    assert second["cached"] is True
    assert second["series"]["altitude"] == first["series"]["altitude"]
    assert second["server_runtime_s"] == first["server_runtime_s"]


@needs_binaries
def test_temporary_directories_are_deleted():
    before = set(glob.glob("/tmp/aether-*"))
    client.post("/api/simulate", json={"wind_speed": 2.0, "seed": 31337})
    assert set(glob.glob("/tmp/aether-*")) == before


@needs_binaries
def test_metrics_window_distinguishes_no_samples_from_zero_error():
    """A run cut short before the settling window must say its statistics are empty.

    The simulator only accumulates tracking and estimator errors after a settling window, so
    an early abort leaves every RMS at zero. Without the window length a reader cannot tell
    that from perfect tracking, and the dashboard would show six tiles of excellent numbers
    for a departure.
    """
    completed = client.post("/api/simulate", json={"wind_speed": 4.0, "seed": 7}).json()
    assert completed["metrics"]["metrics_window_s"] > 0.8 * SETTINGS.duration_s
    assert completed["metrics"]["rms_cross_track_m"] > 0.0

    # 14 m/s wind with this seed departs about nine seconds in.
    departed = client.post("/api/simulate", json={"wind_speed": 14.0, "seed": 5}).json()
    assert departed["metrics"]["completed"] is False
    assert departed["metrics"]["simulated_time_s"] < SETTINGS.duration_s
    if departed["metrics"]["simulated_time_s"] < 20.0:
        assert departed["metrics"]["metrics_window_s"] == 0.0
        assert departed["metrics"]["rms_cross_track_m"] == 0.0
    # Whatever it did, the series must still cover the flight that happened.
    assert departed["series"]["t"][-1] > 1.0


@needs_binaries
def test_truth_feedback_beats_estimated_feedback():
    """Sanity check that the estimator really is in the loop when asked for."""
    truth = client.post("/api/simulate",
                        json={"feedback": "truth", "wind_speed": 5.0, "seed": 11}).json()
    estimate = client.post("/api/simulate",
                           json={"feedback": "estimate", "wind_speed": 5.0, "seed": 11}).json()
    assert truth["metrics"]["rmse_position_m"] == 0.0 or truth["series"]["est_pos_err"]
    assert estimate["metrics"]["rmse_position_m"] > 0.0
    assert estimate["metrics"]["rms_cross_track_m"] >= truth["metrics"]["rms_cross_track_m"] * 0.5


@needs_binaries
def test_execution_timeout_is_enforced():
    """A run that overruns its budget is killed, not left to consume the server."""
    tight = dataclasses.replace(SETTINGS, simulate_timeout_s=0.25)
    runner = SimulationRunner(tight)
    before = set(glob.glob("/tmp/aether-*"))
    with pytest.raises(RunError) as excinfo:
        asyncio.run(runner.simulate(models.SimulationRequest(seed=99)))
    assert excinfo.value.status_code == 504
    assert runner.active_runs == 0
    assert set(glob.glob("/tmp/aether-*")) == before


@needs_binaries
def test_concurrency_limit_refuses_rather_than_queues():
    """With every slot taken and a short queue timeout, extra work is refused with 503."""
    tight = dataclasses.replace(SETTINGS, max_concurrency=1, queue_timeout_s=0.2)
    runner = SimulationRunner(tight)

    async def both():
        return await asyncio.gather(
            runner.simulate(models.SimulationRequest(seed=1)),
            runner.simulate(models.SimulationRequest(seed=2)),
            return_exceptions=True)

    results = asyncio.run(both())
    refused = [r for r in results if isinstance(r, RunError)]
    assert len(refused) == 1
    assert refused[0].status_code == 503
    assert runner.active_runs == 0


@needs_binaries
@pytest.mark.slow
def test_live_montecarlo_runs_the_minimum_campaign():
    response = client.post("/api/montecarlo", json={"trials": 8, "wind_speed": 6.0})
    assert response.status_code == 200
    body = response.json()
    assert body["trials"] == 8
    assert body["successes"] + body["failures"] == 8
    assert len(body["trial_rows"]) == 8
    assert any(s["name"] == "rms_cross_track" for s in body["statistics"])


# ---------------------------------------------------------------------------------------
# Precomputed results
# ---------------------------------------------------------------------------------------
def test_precomputed_campaign_is_the_full_256_trials():
    body = client.get("/api/precomputed/montecarlo").json()
    assert body["trials"] == 256
    assert body["successes"] + body["failures"] == 256
    assert len(body["trials_table"]["index"]) == 256
    assert len(body["envelope"]["time_s"]) > 100


def test_precomputed_modes_cover_the_classical_five():
    names = {mode["mode"] for mode in client.get("/api/precomputed/modes").json()["modes"]}
    assert {"short_period", "phugoid", "roll_subsidence", "dutch_roll", "spiral"} <= names
