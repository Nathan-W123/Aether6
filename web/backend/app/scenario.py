"""Build a simulator scenario from a validated request.

The scenario is assembled here as a plain Python dict of numbers and fixed strings and then
serialised with ``yaml.safe_dump``. No string a visitor supplied ever reaches the file: the
request model only carries bounded floats, ints and enums, so there is nothing to escape and
nothing to inject.
"""

from __future__ import annotations

import math
from pathlib import Path

import yaml

from .models import AIRSPEED_MAX, AIRSPEED_MIN, MonteCarloRequest, SimulationRequest
from .settings import Settings

#: The published six-waypoint circuit, as (north [m], east [m], altitude offset [m],
#: airspeed offset [m/s]) relative to the reference 120 m / 25 m/s condition. The dashboard
#: moves the whole profile with the altitude and airspeed sliders instead of exposing
#: twenty-four separate numbers.
CIRCUIT = [
    (0.0, 0.0, 0.0, 0.0),
    (600.0, 0.0, 30.0, 0.0),
    (900.0, 500.0, 30.0, 2.0),
    (600.0, 900.0, -10.0, 0.0),
    (0.0, 900.0, -10.0, -2.0),
    (-300.0, 450.0, 10.0, 0.0),
]

#: Fixed wind bearing: the unit vector of the published nominal wind (blowing towards the
#: north-west, i.e. coming from the south-east). Only its magnitude is user-controllable.
WIND_DIRECTION = (0.6, -0.8)

#: Turbulence intensity per unit of steady wind. One slider therefore moves both the steady
#: field and the gust intensity, which is how they co-vary in the real atmosphere.
TURBULENCE_PER_WIND = 1.4

_ALTITUDE_FLOOR = 45.0
_ALTITUDE_CEILING = 320.0


def _waypoints(target_altitude: float, airspeed: float) -> list[dict]:
    out = []
    for i, (north, east, d_alt, d_va) in enumerate(CIRCUIT):
        altitude = min(_ALTITUDE_CEILING, max(_ALTITUDE_FLOOR, target_altitude + d_alt))
        leg_airspeed = airspeed + d_va * airspeed / 25.0
        leg_airspeed = min(AIRSPEED_MAX + 2.0, max(AIRSPEED_MIN, leg_airspeed))
        out.append({
            "north": round(north, 3),
            "east": round(east, 3),
            "altitude": round(altitude, 3),
            "airspeed": round(leg_airspeed, 3),
        })
    return out


def _sensor_block(scale: float) -> dict:
    """Sensor suite with every noise and bias sigma scaled by ``scale``."""
    s = float(scale)
    return {
        "enable_gps": True,
        "imu": {
            "rate": 200.0,
            "gyro_noise": round(0.0035 * s, 8),
            "accel_noise": round(0.05 * s, 8),
            "gyro_bias_initial": round(0.010 * s, 8),
            "accel_bias_initial": round(0.08 * s, 8),
            "gyro_bias_walk": round(3.0e-4 * s, 10),
            "accel_bias_walk": round(3.0e-3 * s, 10),
        },
        "gps": {
            "rate": 5.0,
            "position_noise_ne": round(1.2 * s, 6),
            "position_noise_d": round(2.5 * s, 6),
            "velocity_noise": round(0.15 * s, 6),
            "slow_error_sigma": round(1.5 * s, 6),
            "slow_error_tau": 300.0,
        },
        "baro": {"rate": 20.0, "noise": round(0.6 * s, 6), "bias_sigma": round(2.0 * s, 6)},
        "magnetometer": {
            "rate": 50.0,
            "field_strength": 50.0e-6,
            "inclination_deg": 65.0,
            "declination_deg": 11.0,
            "noise": round(0.4e-6 * s, 12),
            "bias_sigma": round(0.6e-6 * s, 12),
        },
        "airspeed": {"rate": 50.0, "noise": round(0.35 * s, 6), "bias_sigma": round(0.25 * s, 6)},
    }


def _estimator_block(scale: float) -> dict:
    """Filter tuning, tracking the sensor grade.

    A real vehicle tunes its filter for the sensors it carries, so the assumed measurement
    noise scales with the slider too. Without that, moving the slider would mostly show a
    mistuned filter rather than the effect of noisier sensors.
    """
    s = float(scale)
    return {
        "gyro_noise": round(0.0045 * s, 8),
        "accel_noise": round(0.07 * s, 8),
        "gyro_bias_walk": round(5.0e-4 * s, 10),
        "accel_bias_walk": round(0.04 * s, 8),
        "wind_walk": 0.05,
        "baro_bias_walk": 0.01,
        "gps_position_ne": round(2.2 * s, 6),
        "gps_position_d": round(3.5 * s, 6),
        "gps_velocity": round(0.25 * s, 6),
        "baro_noise": round(1.5 * s, 6),
        "mag_noise": round(0.8e-6 * s, 12),
        "airspeed_noise": round(0.6 * s, 6),
        "init_position_sigma": round(5.0 * s, 6),
        "init_velocity_sigma": round(1.0 * s, 6),
        "init_attitude_sigma_deg": round(min(20.0, 5.0 * s), 6),
        "init_gyro_bias_sigma": round(0.02 * s, 8),
        "init_accel_bias_sigma": round(0.2 * s, 6),
        "init_wind_sigma": 6.0,
        "init_baro_bias_sigma": round(3.0 * s, 6),
        "exact_discretisation": False,
        "innovation_gate": 25.0,
        "use_baro": True,
        "use_magnetometer": True,
        "use_airspeed": True,
        "initialise_wind_from_airspeed": True,
    }


def build_scenario(request: SimulationRequest, settings: Settings, output_dir: Path) -> dict:
    """Assemble the scenario dict for one interactive run."""
    wind = float(request.wind_speed)
    turbulence = TURBULENCE_PER_WIND * wind
    return {
        "name": "dashboard",
        "aircraft": str(settings.aircraft_file),
        "duration": float(settings.duration_s),
        "control_rate": 100.0,
        "seed": int(request.seed),
        "integration": {"integrator": "rk4", "dt": float(settings.dt_s)},
        "initial": {
            "from_trim": True,
            "airspeed": float(request.airspeed),
            "altitude": float(request.target_altitude),
            "north": 0.0,
            "east": 0.0,
            "heading_deg": 0.0,
            "flight_path_angle_deg": 0.0,
        },
        "environment": {
            "density_scale": 1.0,
            "wind": {
                "steady_ned": [round(WIND_DIRECTION[0] * wind, 6),
                               round(WIND_DIRECTION[1] * wind, 6), 0.0],
                "enable_shear": True,
                "shear_reference_altitude": float(request.target_altitude),
                "shear_roughness": 0.05,
                "enable_turbulence": turbulence > 0.05,
                "altitude_scaled": True,
                "w20": round(max(turbulence, 0.1), 6),
            },
        },
        "control": {
            "type": request.controller.value,
            "feedback": request.feedback.value,
            "lqr": {
                "limits": {
                    "max_bank_deg": 35.0,
                    "max_pitch_deg": 20.0,
                    "max_climb_rate": 4.0,
                    "integrator_authority": 0.25,
                    "protection_enabled": True,
                    "alpha_max_deg": 13.8,
                    "alpha_protection_gain": 2.5,
                    "alpha_protection_rate_gain": 0.35,
                    "alpha_protection_blend_deg": 5.0,
                }
            },
        },
        "guidance": {
            "terminal": "loop",
            "path_gain": 0.035,
            "chi_infinity_deg": 60.0,
            "capture_radius": 45.0,
            "use_half_plane_switching": True,
            "waypoints": _waypoints(request.target_altitude, request.airspeed),
        },
        "sensors": _sensor_block(request.sensor_noise),
        "estimator": _estimator_block(request.sensor_noise),
        "logging": {
            "output_dir": str(output_dir),
            "decimation": int(settings.log_decimation),
            "enabled": True,
            "log_sensors": False,
            "log_estimator": True,
        },
        "safety": {
            "ground_altitude": 0.0,
            "max_altitude": 3000.0,
            "min_airspeed": 8.0,
            "max_airspeed": 60.0,
            "max_bank_deg": 80.0,
            "max_pitch_deg": 75.0,
            "stop_on_ground_contact": True,
        },
    }


def build_montecarlo_scenario(request: MonteCarloRequest, settings: Settings,
                              output_dir: Path) -> dict:
    """Assemble the scenario plus campaign definition for a small live Monte-Carlo demo."""
    base = build_scenario(
        SimulationRequest(controller=request.controller, wind_speed=request.wind_speed,
                          airspeed=request.airspeed, seed=request.seed),
        settings, output_dir)
    base["name"] = "dashboard_montecarlo"
    base["duration"] = float(settings.mc_duration_s)
    base["logging"]["enabled"] = False
    base["monte_carlo"] = {
        "trials": int(request.trials),
        "master_seed": int(request.seed),
        "threads": int(settings.mc_threads),
        "trajectory_interval": 2.0,
        "recorded_trajectories": int(request.trials),
        "output_dir": str(output_dir),
        "dispersions": {
            "vary_mass_properties": True,
            "mass_relative": 0.08,
            "inertia_relative": 0.12,
            "vary_aerodynamics": True,
            "aero_relative": 0.12,
            "thrust_relative": 0.08,
            "vary_initial_conditions": True,
            "initial_altitude": 10.0,
            "initial_airspeed": 1.5,
            "initial_roll_deg": 5.7,
            "initial_pitch_deg": 4.0,
            "initial_yaw_deg": 20.0,
            "vary_environment": True,
            "wind_speed_mean": float(request.wind_speed),
            "wind_speed_sigma": max(1.0, 0.5 * float(request.wind_speed)),
            "w20_mean": round(TURBULENCE_PER_WIND * float(request.wind_speed), 4),
            "w20_sigma": max(1.0, 0.5 * TURBULENCE_PER_WIND * float(request.wind_speed)),
            "density_relative": 0.05,
            "vary_sensors": True,
            "sensor_scale_sigma": 0.30,
        },
    }
    return base


def write_scenario(scenario: dict, path: Path) -> None:
    """Serialise a scenario dict to YAML with the safe dumper."""
    text = yaml.safe_dump(scenario, sort_keys=False, default_flow_style=False)
    path.write_text(text, encoding="utf-8")


def assert_plain_data(value, _depth: int = 0) -> None:
    """Defence in depth: refuse to serialise anything that is not plain scalar data.

    ``yaml.safe_dump`` already refuses arbitrary objects, but asserting the shape here makes
    the guarantee explicit and fails loudly if a future edit lets a raw request field through.
    """
    if _depth > 8:
        raise ValueError("scenario nesting too deep")
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str) or not key.replace("_", "").isalnum():
                raise ValueError(f"unsafe scenario key: {key!r}")
            assert_plain_data(item, _depth + 1)
    elif isinstance(value, (list, tuple)):
        for item in value:
            assert_plain_data(item, _depth + 1)
    elif isinstance(value, bool) or value is None:
        return
    elif isinstance(value, (int, float)):
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError("non-finite number in scenario")
    elif isinstance(value, str):
        return
    else:
        raise ValueError(f"unsupported scenario value type: {type(value)!r}")
