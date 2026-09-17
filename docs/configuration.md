# Configuration reference

Configuration is YAML. There are two kinds of file:

* **airframe** files under `configs/aircraft/` — mass, geometry, aerodynamic coefficients,
  propulsion and actuator limits;
* **scenario** files under `configs/scenarios/` — everything about a run: integration,
  initial condition, environment, control law, guidance, sensors, estimator, logging, safety
  limits, and optionally a Monte-Carlo campaign definition.

Two rules apply everywhere:

1. **Any key may be omitted.** The loader starts from the C++ struct defaults and overwrites
   only what the file states, so a scenario file only has to say what it changes. The
   defaults are the ones documented in the corresponding header.
2. **Units are SI unless the key name says otherwise.** A `_deg` suffix means degrees and a
   `_deg_s` suffix means degrees per second; both are converted to radians on load.

A malformed file is a hard error with the file name and the parser message; a scenario with
fewer than two waypoints, a non-positive step size or a non-positive control rate is rejected
before anything runs.

---

## Airframe file

`configs/aircraft/aether6_uav.yaml` is the shipped example. Every value in it is
**synthetic** — see `docs/model.md` §13.

```yaml
name: "Aether-6 (synthetic small UAV)"
synthetic: true                 # printed in every banner; keep it true for made-up data

mass_properties:
  mass: 13.5                    # kg
  Jx: 0.824                     # kg m^2
  Jy: 1.135
  Jz: 1.759
  Jxz: 0.120                    # product of inertia in the x-z plane

geometry:
  wing_area: 0.52               # m^2   S
  wing_span: 2.60               # m     b
  mean_chord: 0.20              # m     c

aerodynamics:
  longitudinal:                 # all per radian unless noted
    CL0: 0.28                   # -
    CL_alpha: 5.00              # 1/rad
    CL_q: 5.50                  # 1/rad, non-dimensionalised by q*c/(2 Va)
    CL_de: 0.43
    CD0: 0.030                  # -
    CD_q: 0.0
    CD_de: 0.020                # multiplies |delta_e|
    oswald: 0.85                # -, span efficiency, must be in (0, 1]
    Cm0: 0.060
    Cm_alpha: -1.20             # negative = statically stable
    Cm_q: -19.0                 # negative = pitch damped
    Cm_de: -1.50                # negative with the TE-down-positive convention
    alpha_stall_deg: 17.2       # blend point of the flat-plate post-stall model
    stall_sharpness: 50.0       # 1/rad, sigmoid sharpness M
    enable_stall_model: true
  lateral:
    CY0, CY_beta, CY_p, CY_r, CY_da, CY_dr
    Cl0, Cl_beta, Cl_p, Cl_r, Cl_da, Cl_dr
    Cn0, Cn_beta, Cn_p, Cn_r, Cn_da, Cn_dr

propulsion:
  disk_area: 0.050              # m^2, propeller disk
  efficiency: 1.0               # -,  C_prop
  k_motor: 45.0                 # m/s, slipstream exit velocity at full throttle
  k_torque: 5.0e-6              # N m s^2, reaction-torque constant
  k_omega: 300.0                # rad/s per unit throttle
  thrust_offset: [0.0, 0.0, 0.0]  # m, thrust application point relative to the CG, body axes

actuators:
  elevator_min_deg: -25.0       # magnitude limits
  elevator_max_deg:  25.0
  aileron_min_deg:  -20.0
  aileron_max_deg:   20.0
  rudder_min_deg:   -25.0
  rudder_max_deg:    25.0
  throttle_min: 0.0
  throttle_max: 1.0
  surface_rate_limit_deg_s: 300.0
  throttle_rate_limit: 2.0      # 1/s
  surface_time_constant: 0.04   # s, first-order lag
  throttle_time_constant: 0.20  # s
```

`AircraftParameters::validate()` runs on load and rejects: non-positive mass, area, span or
chord; an inertia tensor that is not positive definite or whose principal moments violate
`I1 + I2 >= I3`; an Oswald factor outside `(0, 1]`; non-positive `CL_alpha`, `CD0`, propeller
disk area or `k_motor`; inverted actuator limits; non-positive actuator time constants.

---

## Scenario file

### Top level

| Key | Default | Meaning |
|---|---|---|
| `name` | `nominal` | scenario name, used in banners and summaries |
| `aircraft` | `../aircraft/aether6_uav.yaml` | airframe file, resolved relative to the scenario file |
| `duration` | 240 | simulated duration [s] |
| `control_rate` | 100 | autopilot update rate [Hz]; must satisfy `1/control_rate >= integration.dt` |
| `seed` | 20240917 | master seed for every random stream |

### `integration`

| Key | Default | Meaning |
|---|---|---|
| `integrator` | `rk4` | `rk4` (fixed step) or `dopri54` (adaptive Dormand-Prince 5(4)) |
| `dt` | 0.002 | outer fixed frame [s]; also the fixed step for `rk4` |
| `tolerances.rel_tol` | 1e-8 | adaptive relative tolerance |
| `tolerances.abs_tol` | 1e-10 | adaptive absolute tolerance |
| `tolerances.min_step` | 1e-7 | smallest step the controller may propose [s] |
| `tolerances.max_step` | 0.1 | largest step [s] |
| `tolerances.safety` | 0.9 | step-size safety factor |

A step already at `min_step` is accepted even if it misses the tolerance, and counted in
`IntegrationStats::tolerance_not_met`, so an unachievable tolerance degrades gracefully
instead of stalling the run.

### `initial`

| Key | Default | Meaning |
|---|---|---|
| `from_trim` | true | start from the trimmed condition |
| `airspeed` | 25.0 | trim/initial true airspeed [m/s] |
| `altitude` | 120.0 | initial altitude [m] |
| `north`, `east` | 0.0 | initial horizontal position [m] |
| `heading_deg` | 0.0 | initial heading |
| `flight_path_angle_deg` | 0.0 | trim flight-path angle |
| `delta_altitude`, `delta_airspeed` | 0.0 | additive offsets on top of trim [m], [m/s] |
| `delta_roll_deg`, `delta_pitch_deg`, `delta_yaw_deg` | 0.0 | additive attitude offsets |

The vehicle starts trimmed **relative to the air mass**: the ground velocity is the trimmed
air-relative velocity plus the local wind, so `Va`, `alpha` and `beta` match the trim point
exactly even in a steady wind.

### `environment`

| Key | Default | Meaning |
|---|---|---|
| `density_scale` | 1.0 | multiplier on the ISA density |
| `wind.steady_ned` | `[0, 0, 0]` | steady wind at the reference altitude [m/s] |
| `wind.enable_shear` | true | logarithmic boundary-layer profile |
| `wind.shear_reference_altitude` | 50.0 | altitude at which `steady_ned` is specified [m AGL] |
| `wind.shear_roughness` | 0.05 | surface roughness length `z0` [m] |
| `wind.enable_turbulence` | true | Dryden turbulence |
| `wind.altitude_scaled` | true | MIL-F-8785C low-altitude sigma/L scaling |
| `wind.w20` | 5.0 | wind speed at 6 m, which sets the turbulence intensity [m/s] |
| `wind.sigma`, `wind.length_scale` | see header | used directly when `altitude_scaled: false` |

### `control`

| Key | Default | Meaning |
|---|---|---|
| `type` | `lqr` | `lqr` or `pid` |
| `feedback` | `estimate` | `estimate` (EKF output) or `truth` (perfect sensing) |

`control.lqr.weights` holds the seventeen LQR weights (see
`include/aether/control/LqrAutopilot.hpp` for each one's units); `control.lqr.limits` holds:

| Key | Default | Meaning |
|---|---|---|
| `max_bank_deg` | 35 | bank limit the course-error clamp is derived from |
| `max_pitch_deg` | 20 | pitch limit the altitude-error clamp is derived from |
| `max_climb_rate` | 4.0 | slew limit on the altitude command [m/s] |
| `derive_limits_from_gains` | true | compute the error-state clamps from the synthesised gains |
| `integrator_authority` | 0.25 | fraction of the attitude budget reserved for integral action |
| `course_error_limit_deg`, `altitude_error_limit`, `integrator_*_limit` | — | upper bounds the derived values are clipped against |
| `protection_enabled` | true | angle-of-attack envelope protection |
| `alpha_max_deg` | 13.8 | protection threshold |
| `alpha_protection_gain` | 2.5 | nose-down elevator per radian of exceedance |
| `alpha_protection_rate_gain` | 0.35 | pitch-rate damping inside the protection |
| `alpha_protection_blend_deg` | 5.0 | exceedance over which the protection fades in |

`control.pid` holds the six loops (`roll_to_aileron`, `pitch_to_elevator`,
`sideslip_to_rudder`, `course_to_roll`, `altitude_to_pitch`, `airspeed_to_throttle`), each
with `kp`, `ki`, `kd`, `anti_windup_gain` and `derivative_filter_tau`, plus `max_bank_deg`,
`max_pitch_deg`, `max_climb_rate`, `yaw_damper_gain`, `yaw_damper_tau` and the same
protection keys. Output saturations are **not** configurable per loop: they are derived from
the actuator limits and the command limits, because they are properties of the airframe
rather than tuning choices.

### `guidance`

| Key | Default | Meaning |
|---|---|---|
| `terminal` | `loop` | `loop`, `orbit` or `hold` |
| `path_gain` | 0.04 | cross-track gain `k_path` [1/m] |
| `chi_infinity_deg` | 60 | approach-course limit |
| `capture_radius` | 30.0 | waypoint capture radius [m] |
| `orbit_radius`, `orbit_gain`, `orbit_direction` | 90, 2.5, +1 | terminal loiter circle |
| `use_half_plane_switching` | true | bisector half-plane switching |
| `waypoints` | — | list of `{north, east, altitude, airspeed}`; at least two |

`airspeed` defaults to `initial.airspeed` when a waypoint omits it.

### `sensors`

`enable_gps`, `gps_outage_start`, `gps_outage_end` (seconds; a negative start disables the
scripted outage) plus one block per sensor:

| Block | Keys |
|---|---|
| `imu` | `rate`, `gyro_noise`, `accel_noise`, `gyro_bias_initial`, `accel_bias_initial`, `gyro_bias_walk`, `accel_bias_walk` |
| `gps` | `rate`, `position_noise_ne`, `position_noise_d`, `velocity_noise`, `slow_error_sigma`, `slow_error_tau` |
| `baro` | `rate`, `noise`, `bias_sigma` |
| `magnetometer` | `rate`, `field_strength`, `inclination_deg`, `declination_deg`, `noise`, `bias_sigma` |
| `airspeed` | `rate`, `noise`, `bias_sigma` |

`slow_error_sigma` / `slow_error_tau` define a first-order Gauss-Markov correlated GNSS
position error on top of the white noise.

### `estimator`

Process noise (`gyro_noise`, `accel_noise`, `gyro_bias_walk`, `accel_bias_walk`, `wind_walk`,
`baro_bias_walk`), measurement noise (`gps_position_ne`, `gps_position_d`, `gps_velocity`,
`baro_noise`, `mag_noise`, `airspeed_noise`), initial 1-sigma values (`init_position_sigma`,
`init_velocity_sigma`, `init_attitude_sigma_deg`, `init_gyro_bias_sigma`,
`init_accel_bias_sigma`, `init_wind_sigma`, `init_baro_bias_sigma`) and switches
(`exact_discretisation`, `innovation_gate`, `use_baro`, `use_magnetometer`, `use_airspeed`,
`initialise_wind_from_airspeed`).

The estimator's assumed noise is deliberately *not* equal to the sensor truth — the shipped
scenarios inflate it, as one would on a real vehicle. See `docs/validation.md` for the
resulting consistency.

### `logging`

| Key | Default | Meaning |
|---|---|---|
| `output_dir` | `results/nominal` | destination directory |
| `decimation` | 10 | write one row every N simulation steps |
| `enabled` | true | set false to run with no file output (used by Monte Carlo) |
| `log_sensors` | true | also write `sensor_*.csv` |
| `log_estimator` | true | include the estimator columns |

### `safety`

`ground_altitude`, `max_altitude`, `min_airspeed`, `max_airspeed`, `max_bank_deg`,
`max_pitch_deg`, `stop_on_ground_contact`. A violation ends the run and is reported as the
`termination_reason` in `summary.json`; Monte Carlo counts it as a failure.

### `monte_carlo`

Read by `aether_mc`; ignored by `aether_sim`.

| Key | Default | Meaning |
|---|---|---|
| `trials` | 200 | number of trials |
| `master_seed` | 987654321 | master seed; trial seeds derive from it |
| `threads` | 0 | worker threads, 0 = hardware concurrency |
| `trajectory_interval` | 1.0 | in-memory trajectory sampling interval [s] |
| `recorded_trajectories` | 40 | how many trials keep a full trajectory |
| `output_dir` | `results/monte_carlo` | destination |
| `dispersions.vary_mass_properties` | true | enable the mass/inertia block |
| `dispersions.mass_relative` | 0.08 | relative 1-sigma on mass |
| `dispersions.inertia_relative` | 0.12 | relative 1-sigma on each inertia term |
| `dispersions.vary_aerodynamics` | true | enable the aerodynamic block |
| `dispersions.aero_relative` | 0.12 | relative 1-sigma on each derivative |
| `dispersions.thrust_relative` | 0.08 | relative 1-sigma on `k_motor` |
| `dispersions.vary_initial_conditions` | true | enable the initial-state block |
| `dispersions.initial_altitude` | 10.0 | 1-sigma [m] |
| `dispersions.initial_airspeed` | 1.5 | 1-sigma [m/s] |
| `dispersions.initial_roll_deg`, `initial_pitch_deg`, `initial_yaw_deg` | 5.7, 4.0, 20.0 | 1-sigma |
| `dispersions.vary_environment` | true | enable the wind/density block |
| `dispersions.wind_speed_mean`, `wind_speed_sigma` | 5.0, 3.0 | steady wind magnitude [m/s]; direction is uniform |
| `dispersions.w20_mean`, `w20_sigma` | 7.0, 3.5 | turbulence-driving wind [m/s] |
| `dispersions.density_relative` | 0.05 | relative 1-sigma on air density |
| `dispersions.vary_sensors` | true | enable the sensor block |
| `dispersions.sensor_scale_sigma` | 0.30 | 1-sigma of a log-normal scale applied to every sensor sigma |

Multiplicative dispersions are clipped so the airframe stays physically valid, and the
inertia tensor is repaired after dispersion if the independent draws happen to violate the
triangle inequality on the principal moments.

---

## Command-line overrides

Every application accepts `--help`. Flags override the file:

```
aether_sim  [SCENARIO] [--output DIR] [--duration S] [--dt S] [--integrator NAME]
            [--controller pid|lqr] [--feedback truth|estimate] [--seed N] [--no-log] [--quiet]

aether_trim [SCENARIO] [--output DIR] [--airspeed V] [--altitude H] [--gamma DEG]
            [--turn-rate DEG_S] [--sweep V0:V1:N]

aether_mc   [SCENARIO] [--trials N] [--seed N] [--threads N] [--output DIR]
            [--duration S] [--dt S] [--controller pid|lqr] [--trajectories N]

aether_integrators [SCENARIO] [--output DIR] [--horizon S]
```

`aether_sim` exits 0 when the run completed inside the safety envelope, 1 when a limit was
tripped and 2 on a usage or configuration error, so it composes with shell scripting.

---

## Shipped scenarios

| File | What it is |
|---|---|
| `nominal.yaml` | LQR on EKF feedback, six-waypoint circuit, 5 m/s wind with Dryden turbulence. The reference run. |
| `ideal.yaml` | Same mission, still air, perfect (truth) feedback. The deterministic regression baseline. |
| `nominal_pid.yaml` | Same mission and seed flown with the PID cascade, for a like-for-like controller comparison. |
| `wind_disturbance.yaml` | 10.8 m/s steady wind (43% of cruise airspeed) with severe turbulence. |
| `monte_carlo.yaml` | The nominal mission at 250 Hz plus the campaign definition; used by `aether_mc`. |
