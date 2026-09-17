/** TypeScript mirrors of the FastAPI response models in `web/backend/app/models.py`. */

export type ControllerId = 'lqr' | 'pid'
export type FeedbackId = 'estimate' | 'truth'

export interface SimulationRequest {
  controller: ControllerId
  feedback: FeedbackId
  wind_speed: number
  target_altitude: number
  airspeed: number
  sensor_noise: number
  seed: number
}

export interface MonteCarloRequest {
  trials: number
  controller: ControllerId
  wind_speed: number
  airspeed: number
  seed: number
}

export interface TrimPoint {
  alpha_deg: number
  theta_deg: number
  elevator_deg: number
  throttle: number
  residual_inf_norm: number
}

export interface RunMetrics {
  completed: boolean
  stable: boolean
  termination_reason: string
  simulated_time_s: number
  /** Seconds of data the RMS figures cover; 0 means the run ended before they began. */
  metrics_window_s: number
  waypoints_reached: number
  laps_completed: number
  rms_cross_track_m: number
  max_cross_track_m: number
  rms_altitude_error_m: number
  max_altitude_error_m: number
  rms_airspeed_error_mps: number
  max_bank_deg: number
  max_alpha_deg: number
  rmse_position_m: number
  rmse_velocity_mps: number
  rmse_attitude_deg: number
  rmse_yaw_deg: number
  min_covariance_eigenvalue: number
  wall_clock_s: number
  real_time_factor: number
}

export interface Waypoint {
  index: number
  north: number
  east: number
  altitude: number
}

/** Resampled time histories, keyed by the names documented in `/api/meta`. */
export type Series = Record<string, number[]>

export interface SimulationResponse {
  request: SimulationRequest
  cached: boolean
  server_runtime_s: number
  trim: TrimPoint
  metrics: RunMetrics
  waypoints: Waypoint[]
  series: Series
}

export interface MonteCarloTrial {
  index: number
  ok: boolean
  mass_kg: number
  cl_alpha: number
  wind_speed_mps: number
  rms_cross_track_m: number
  estimator_position_rmse_m: number
  max_bank_deg: number
}

export interface Statistic {
  name: string
  unit: string
  count: number
  mean: number
  stddev: number
  min: number
  p05: number
  median: number
  p95: number
  p99: number
  max: number
}

export interface MonteCarloResponse {
  request: MonteCarloRequest
  cached: boolean
  server_runtime_s: number
  trials: number
  successes: number
  failures: number
  failure_rate: number
  failure_reasons: Record<string, number>
  statistics: Statistic[]
  trial_rows: MonteCarloTrial[]
}

export interface PrecomputedMonteCarlo {
  available: boolean
  scenario: string
  controller: string
  feedback: string
  trials: number
  master_seed: number
  duration_per_trial_s: number
  successes: number
  failures: number
  failure_rate: number
  failure_reasons: Record<string, number>
  wall_clock_s: number
  threads: number
  statistics: Statistic[]
  trials_table: Record<string, number[]>
  envelope: Record<string, number[]>
}

export interface Mode {
  group: string
  mode: string
  real: number
  imag: number
  natural_frequency_rad_s: number
  damping_ratio: number
  period_s: number
  time_to_half_s: number
  stable: boolean
}

export interface PrecomputedModes {
  available: boolean
  modes: Mode[]
  reference_point: { quantity: string; value: number; unit: string }[]
}

export interface Bound {
  min: number
  max: number
  step: number
  unit: string
  label: string
}

export interface Preset {
  id: string
  label: string
  tagline: string
  detail: string
  request: SimulationRequest
}

export interface Meta {
  version: string
  presets: Preset[]
  bounds: Record<'wind_speed' | 'target_altitude' | 'airspeed' | 'sensor_noise', Bound>
  controllers: { id: ControllerId; label: string; detail: string }[]
  feedback: { id: FeedbackId; label: string; detail: string }[]
  limits: {
    duration_s: number
    dt_s: number
    series_points: number
    mc_min_trials: number
    mc_max_trials: number
    mc_duration_s: number
    max_concurrency: number
    simulate_timeout_s: number
    montecarlo_timeout_s: number
  }
  mission: {
    waypoints: { north: number; east: number }[]
    turbulence_per_wind: number
    terminal: string
  }
  series_units: Record<string, string>
}
