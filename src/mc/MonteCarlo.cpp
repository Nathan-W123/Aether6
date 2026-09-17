#include "aether/mc/MonteCarlo.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>

#include "aether/core/Constants.hpp"
#include "aether/util/Csv.hpp"
#include "aether/util/Random.hpp"

namespace aether::mc {

namespace {

constexpr std::uint64_t kStreamVehicle = 11;
constexpr std::uint64_t kStreamEnv = 12;
constexpr std::uint64_t kStreamInit = 13;
constexpr std::uint64_t kStreamSensor = 14;
constexpr std::uint64_t kStreamRun = 15;

/// Multiplicative dispersion clipped to keep the value physically sensible.
double disperse(double nominal, double relative_sigma, util::Rng& rng, double min_factor = 0.4,
                double max_factor = 1.6) {
  const double f = std::clamp(1.0 + relative_sigma * rng.gaussian(), min_factor, max_factor);
  return nominal * f;
}

/// Restore the physical constraints of an inertia tensor after independent dispersion.
///
/// Dispersing Jx, Jy, Jz independently can violate the triangle inequality on the principal
/// moments (I1 + I2 >= I3), which no rigid body can satisfy. When that happens the largest
/// moment is shrunk until the inequality holds with a small margin; Jxz is shrunk with it so
/// the tensor stays positive definite.
void repairInertia(dynamics::AircraftParameters* p) {
  for (int attempt = 0; attempt < 20; ++attempt) {
    const double jxz_max = 0.95 * std::sqrt(p->Jx * p->Jz);
    p->Jxz = std::clamp(p->Jxz, -jxz_max, jxz_max);
    Eigen::SelfAdjointEigenSolver<Mat3> es(p->inertia());
    const Vec3 pm = es.eigenvalues();  // ascending
    if (pm(0) > 0.0 && pm(0) + pm(1) >= 1.001 * pm(2)) return;
    // Shrink whichever body-axis moment dominates the largest principal moment.
    const double scale = std::max(0.5, (pm(0) + pm(1)) / (1.02 * pm(2)));
    if (p->Jz >= p->Jx && p->Jz >= p->Jy) {
      p->Jz *= scale;
    } else if (p->Jy >= p->Jx) {
      p->Jy *= scale;
    } else {
      p->Jx *= scale;
    }
    p->Jxz *= scale;
  }
}

}  // namespace

MetricStatistics computeStatistics(std::string name, std::string unit,
                                   std::vector<double> data) {
  MetricStatistics s;
  s.name = std::move(name);
  s.unit = std::move(unit);
  s.count = static_cast<int>(data.size());
  if (data.empty()) return s;
  std::sort(data.begin(), data.end());
  const double n = static_cast<double>(data.size());
  double sum = 0.0;
  for (double v : data) sum += v;
  s.mean = sum / n;
  double sq = 0.0;
  for (double v : data) sq += (v - s.mean) * (v - s.mean);
  s.stddev = data.size() > 1 ? std::sqrt(sq / (n - 1.0)) : 0.0;
  auto percentile = [&](double p) {
    const double pos = p * (n - 1.0);
    const std::size_t lo = static_cast<std::size_t>(std::floor(pos));
    const std::size_t hi = std::min(lo + 1, data.size() - 1);
    const double frac = pos - static_cast<double>(lo);
    return data[lo] * (1.0 - frac) + data[hi] * frac;
  };
  s.min = data.front();
  s.max = data.back();
  s.p05 = percentile(0.05);
  s.median = percentile(0.50);
  s.p95 = percentile(0.95);
  s.p99 = percentile(0.99);
  return s;
}

MonteCarloRunner::MonteCarloRunner(sim::ScenarioConfig base,
                                   dynamics::AircraftParameters aircraft,
                                   MonteCarloConfig config)
    : base_(std::move(base)), aircraft_(std::move(aircraft)), cfg_(std::move(config)) {
  base_.logging.enabled = false;
}

void MonteCarloRunner::buildTrial(int index, dynamics::AircraftParameters* aircraft,
                                  sim::ScenarioConfig* scenario, TrialResult* record) const {
  *aircraft = aircraft_;
  *scenario = base_;
  record->index = index;
  record->seed = util::deriveSeed(cfg_.master_seed, kStreamRun, static_cast<std::uint64_t>(index));
  scenario->seed = record->seed;

  const auto& d = cfg_.dispersions;

  // --- Vehicle -------------------------------------------------------------------
  util::Rng rng_v(util::deriveSeed(cfg_.master_seed, kStreamVehicle,
                                   static_cast<std::uint64_t>(index)));
  if (d.vary_mass_properties) {
    aircraft->mass = disperse(aircraft->mass, d.mass_relative, rng_v, 0.6, 1.4);
    aircraft->Jx = disperse(aircraft->Jx, d.inertia_relative, rng_v, 0.5, 1.5);
    aircraft->Jy = disperse(aircraft->Jy, d.inertia_relative, rng_v, 0.5, 1.5);
    aircraft->Jz = disperse(aircraft->Jz, d.inertia_relative, rng_v, 0.5, 1.5);
    aircraft->Jxz = disperse(aircraft->Jxz, d.inertia_relative, rng_v, 0.0, 2.0);
    repairInertia(aircraft);
  }
  if (d.vary_aerodynamics) {
    auto& l = aircraft->lon;
    auto& t = aircraft->lat;
    l.CL0 = disperse(l.CL0, d.aero_relative, rng_v);
    l.CL_alpha = disperse(l.CL_alpha, d.aero_relative, rng_v, 0.6, 1.4);
    l.CL_q = disperse(l.CL_q, d.aero_relative, rng_v);
    l.CL_de = disperse(l.CL_de, d.aero_relative, rng_v, 0.5, 1.5);
    l.CD0 = disperse(l.CD0, d.aero_relative, rng_v, 0.5, 1.8);
    l.CD_de = disperse(l.CD_de, d.aero_relative, rng_v);
    l.oswald = std::clamp(disperse(l.oswald, 0.5 * d.aero_relative, rng_v), 0.5, 0.99);
    l.Cm0 = disperse(l.Cm0, d.aero_relative, rng_v);
    l.Cm_alpha = disperse(l.Cm_alpha, d.aero_relative, rng_v, 0.5, 1.5);
    l.Cm_q = disperse(l.Cm_q, d.aero_relative, rng_v, 0.5, 1.5);
    l.Cm_de = disperse(l.Cm_de, d.aero_relative, rng_v, 0.5, 1.5);
    t.CY_beta = disperse(t.CY_beta, d.aero_relative, rng_v);
    t.CY_p = disperse(t.CY_p, d.aero_relative, rng_v);
    t.CY_r = disperse(t.CY_r, d.aero_relative, rng_v);
    t.CY_dr = disperse(t.CY_dr, d.aero_relative, rng_v);
    t.Cl_beta = disperse(t.Cl_beta, d.aero_relative, rng_v);
    t.Cl_p = disperse(t.Cl_p, d.aero_relative, rng_v, 0.5, 1.5);
    t.Cl_r = disperse(t.Cl_r, d.aero_relative, rng_v);
    t.Cl_da = disperse(t.Cl_da, d.aero_relative, rng_v, 0.5, 1.5);
    t.Cl_dr = disperse(t.Cl_dr, d.aero_relative, rng_v);
    t.Cn_beta = disperse(t.Cn_beta, d.aero_relative, rng_v, 0.4, 1.6);
    t.Cn_p = disperse(t.Cn_p, d.aero_relative, rng_v);
    t.Cn_r = disperse(t.Cn_r, d.aero_relative, rng_v, 0.5, 1.5);
    t.Cn_da = disperse(t.Cn_da, d.aero_relative, rng_v);
    t.Cn_dr = disperse(t.Cn_dr, d.aero_relative, rng_v, 0.5, 1.5);
    aircraft->prop.k_motor = disperse(aircraft->prop.k_motor, d.thrust_relative, rng_v, 0.7, 1.3);
  }
  aircraft->validate();

  record->mass = aircraft->mass;
  record->inertia_scale =
      (aircraft->Jx / aircraft_.Jx + aircraft->Jy / aircraft_.Jy + aircraft->Jz / aircraft_.Jz) / 3.0;
  record->cl_alpha = aircraft->lon.CL_alpha;
  record->cm_alpha = aircraft->lon.Cm_alpha;

  // --- Environment ---------------------------------------------------------------
  util::Rng rng_e(util::deriveSeed(cfg_.master_seed, kStreamEnv,
                                   static_cast<std::uint64_t>(index)));
  if (d.vary_environment) {
    const double speed =
        std::max(0.0, d.wind_speed_mean + d.wind_speed_sigma * rng_e.gaussian());
    const double dir = rng_e.uniform(-constants::kPi, constants::kPi);
    scenario->wind.steady_ned = Vec3(speed * std::cos(dir), speed * std::sin(dir), 0.0);
    scenario->wind.w20 = std::max(0.5, d.w20_mean + d.w20_sigma * rng_e.gaussian());
    scenario->density_scale =
        std::clamp(1.0 + d.density_relative * rng_e.gaussian(), 0.7, 1.3);
    record->wind_speed = speed;
    record->wind_direction = dir;
    record->w20 = scenario->wind.w20;
    record->density_scale = scenario->density_scale;
  } else {
    record->wind_speed = scenario->wind.steady_ned.norm();
    record->w20 = scenario->wind.w20;
    record->density_scale = scenario->density_scale;
  }

  // --- Initial conditions ---------------------------------------------------------
  util::Rng rng_i(util::deriveSeed(cfg_.master_seed, kStreamInit,
                                   static_cast<std::uint64_t>(index)));
  if (d.vary_initial_conditions) {
    scenario->initial.delta_altitude = d.initial_altitude * rng_i.gaussian();
    scenario->initial.delta_airspeed = d.initial_airspeed * rng_i.gaussian();
    scenario->initial.delta_roll = d.initial_roll * rng_i.gaussian();
    scenario->initial.delta_pitch = d.initial_pitch * rng_i.gaussian();
    scenario->initial.delta_yaw = d.initial_yaw * rng_i.gaussian();
    record->init_altitude_offset = scenario->initial.delta_altitude;
    record->init_airspeed_offset = scenario->initial.delta_airspeed;
    record->init_yaw_offset = scenario->initial.delta_yaw;
  }

  // --- Sensors ---------------------------------------------------------------------
  util::Rng rng_s(util::deriveSeed(cfg_.master_seed, kStreamSensor,
                                   static_cast<std::uint64_t>(index)));
  double scale = 1.0;
  if (d.vary_sensors) {
    scale = std::exp(std::clamp(d.sensor_scale_sigma * rng_s.gaussian(), -1.2, 1.2));
    auto& s = scenario->sensors;
    s.imu.gyro_noise_density *= scale;
    s.imu.accel_noise_density *= scale;
    s.imu.gyro_bias_initial *= scale;
    s.imu.accel_bias_initial *= scale;
    s.imu.gyro_bias_walk *= scale;
    s.imu.accel_bias_walk *= scale;
    s.gps.position_noise_ne *= scale;
    s.gps.position_noise_d *= scale;
    s.gps.velocity_noise *= scale;
    s.gps.slow_error_sigma *= scale;
    s.baro.noise *= scale;
    s.baro.bias_sigma *= scale;
    s.magnetometer.noise *= scale;
    s.magnetometer.bias_sigma *= scale;
    s.airspeed.noise *= scale;
    s.airspeed.bias_sigma *= scale;
  }
  record->sensor_scale = scale;
}

MonteCarloResult MonteCarloRunner::run(bool verbose) {
  const auto wall_start = std::chrono::steady_clock::now();

  MonteCarloResult result;
  result.trials.resize(static_cast<std::size_t>(std::max(cfg_.trials, 0)));

  int threads = cfg_.threads > 0 ? cfg_.threads
                                 : static_cast<int>(std::thread::hardware_concurrency());
  threads = std::clamp(threads, 1, std::max(1, cfg_.trials));
  result.threads_used = threads;

  std::atomic<int> next_index{0};
  std::atomic<int> completed{0};
  std::mutex print_mutex;

  auto worker = [&]() {
    for (;;) {
      const int i = next_index.fetch_add(1);
      if (i >= cfg_.trials) return;

      TrialResult trial;
      dynamics::AircraftParameters aircraft;
      sim::ScenarioConfig scenario;
      try {
        buildTrial(i, &aircraft, &scenario, &trial);
        sim::Simulator simulator(scenario, aircraft);
        if (i < cfg_.recorded_trajectories && cfg_.trajectory_interval > 0.0) {
          simulator.setTrajectoryRecorder(cfg_.trajectory_interval,
                                          [&trial](const sim::TrajectorySample& s) {
                                            trial.trajectory.push_back(s);
                                          });
        }
        const sim::SimulationResult r = simulator.run();
        trial.metrics = r.metrics;
        trial.ok = r.metrics.stable && r.metrics.completed;
        trial.termination_reason = r.metrics.termination_reason;
      } catch (const std::exception& e) {
        trial.ok = false;
        trial.termination_reason = std::string("exception: ") + e.what();
      }
      result.trials[static_cast<std::size_t>(i)] = std::move(trial);

      const int done = completed.fetch_add(1) + 1;
      if (verbose && (done % std::max(1, cfg_.trials / 20) == 0 || done == cfg_.trials)) {
        std::lock_guard<std::mutex> lock(print_mutex);
        std::cout << "  progress: " << done << "/" << cfg_.trials << " trials\r" << std::flush;
      }
    }
  };

  std::vector<std::thread> pool;
  pool.reserve(static_cast<std::size_t>(threads));
  for (int t = 0; t < threads; ++t) pool.emplace_back(worker);
  for (auto& t : pool) t.join();
  if (verbose) std::cout << "\n";

  // --- Aggregate -------------------------------------------------------------------
  std::vector<double> cross, alt, va, pos_rmse, vel_rmse, att_rmse, yaw_rmse, bank, runtime,
      max_cross, max_alt, laps, gyro_bias_err, accel_bias_err;
  double trial_time_sum = 0.0;
  for (const auto& t : result.trials) {
    trial_time_sum += t.metrics.wall_clock_seconds;
    if (t.ok) {
      ++result.successes;
      cross.push_back(t.metrics.rms_cross_track);
      max_cross.push_back(t.metrics.max_cross_track);
      alt.push_back(t.metrics.rms_altitude_error);
      max_alt.push_back(t.metrics.max_altitude_error);
      va.push_back(t.metrics.rms_airspeed_error);
      pos_rmse.push_back(t.metrics.rmse_position);
      vel_rmse.push_back(t.metrics.rmse_velocity);
      att_rmse.push_back(t.metrics.rmse_attitude * constants::kRadToDeg);
      yaw_rmse.push_back(t.metrics.rmse_yaw * constants::kRadToDeg);
      bank.push_back(t.metrics.max_bank * constants::kRadToDeg);
      laps.push_back(static_cast<double>(t.metrics.laps_completed));
      gyro_bias_err.push_back(t.metrics.final_gyro_bias_error);
      accel_bias_err.push_back(t.metrics.final_accel_bias_error);
    } else {
      ++result.failures;
    }
    runtime.push_back(t.metrics.wall_clock_seconds);
  }
  result.failure_rate =
      cfg_.trials > 0 ? static_cast<double>(result.failures) / cfg_.trials : 0.0;

  result.statistics.push_back(computeStatistics("rms_cross_track", "m", cross));
  result.statistics.push_back(computeStatistics("max_cross_track", "m", max_cross));
  result.statistics.push_back(computeStatistics("rms_altitude_error", "m", alt));
  result.statistics.push_back(computeStatistics("max_altitude_error", "m", max_alt));
  result.statistics.push_back(computeStatistics("rms_airspeed_error", "m/s", va));
  result.statistics.push_back(computeStatistics("estimator_position_rmse", "m", pos_rmse));
  result.statistics.push_back(computeStatistics("estimator_velocity_rmse", "m/s", vel_rmse));
  result.statistics.push_back(computeStatistics("estimator_attitude_rmse", "deg", att_rmse));
  result.statistics.push_back(computeStatistics("estimator_yaw_rmse", "deg", yaw_rmse));
  result.statistics.push_back(computeStatistics("final_gyro_bias_error", "rad/s", gyro_bias_err));
  result.statistics.push_back(
      computeStatistics("final_accel_bias_error", "m/s^2", accel_bias_err));
  result.statistics.push_back(computeStatistics("max_bank", "deg", bank));
  result.statistics.push_back(computeStatistics("laps_completed", "-", laps));
  result.statistics.push_back(computeStatistics("trial_wall_clock", "s", runtime));

  result.wall_clock_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
  result.mean_trial_seconds = cfg_.trials > 0 ? trial_time_sum / cfg_.trials : 0.0;
  return result;
}

void MonteCarloRunner::write(const MonteCarloResult& result) const {
  namespace fs = std::filesystem;
  fs::create_directories(cfg_.output_dir);

  // --- Per-trial table --------------------------------------------------------------
  {
    util::CsvWriter w(cfg_.output_dir + "/trials.csv",
                      {"index", "seed", "ok", "mass_kg", "inertia_scale", "cl_alpha",
                       "cm_alpha", "wind_speed_mps", "wind_direction_rad", "w20_mps",
                       "density_scale", "sensor_scale", "init_altitude_offset_m",
                       "init_airspeed_offset_mps", "init_yaw_offset_rad", "simulated_time_s",
                       "rms_cross_track_m", "max_cross_track_m", "rms_altitude_error_m",
                       "max_altitude_error_m", "rms_airspeed_error_mps", "max_bank_deg",
                       "max_alpha_deg", "laps_completed", "waypoints_reached",
                       "estimator_position_rmse_m", "estimator_velocity_rmse_mps",
                       "estimator_attitude_rmse_deg", "estimator_yaw_rmse_deg",
                       "final_gyro_bias_error_radps", "final_accel_bias_error_mps2",
                       "min_cov_eigenvalue", "wall_clock_s"});
    for (const auto& t : result.trials) {
      const auto& m = t.metrics;
      w.writeRow({static_cast<double>(t.index), static_cast<double>(t.seed), t.ok ? 1.0 : 0.0,
                  t.mass, t.inertia_scale, t.cl_alpha, t.cm_alpha, t.wind_speed,
                  t.wind_direction, t.w20, t.density_scale, t.sensor_scale,
                  t.init_altitude_offset, t.init_airspeed_offset, t.init_yaw_offset,
                  m.simulated_time, m.rms_cross_track, m.max_cross_track,
                  m.rms_altitude_error, m.max_altitude_error, m.rms_airspeed_error,
                  m.max_bank * constants::kRadToDeg, m.max_alpha * constants::kRadToDeg,
                  static_cast<double>(m.laps_completed),
                  static_cast<double>(m.waypoints_reached), m.rmse_position, m.rmse_velocity,
                  m.rmse_attitude * constants::kRadToDeg, m.rmse_yaw * constants::kRadToDeg,
                  m.final_gyro_bias_error, m.final_accel_bias_error,
                  m.min_covariance_eigenvalue, m.wall_clock_seconds});
    }
  }

  // --- Recorded trajectories --------------------------------------------------------
  {
    util::CsvWriter w(cfg_.output_dir + "/trajectories.csv",
                      {"trial", "time_s", "north_m", "east_m", "altitude_m", "airspeed_mps",
                       "roll_rad", "pitch_rad", "yaw_rad", "cross_track_m",
                       "altitude_error_m", "est_pos_error_m", "est_att_error_rad"});
    for (const auto& t : result.trials) {
      for (const auto& s : t.trajectory)
        w.writeRow({static_cast<double>(t.index), s.time, s.north, s.east, s.altitude,
                    s.airspeed, s.roll, s.pitch, s.yaw, s.cross_track, s.altitude_error,
                    s.estimator_position_error, s.estimator_attitude_error});
    }
  }

  // --- Time-binned envelope ---------------------------------------------------------
  {
    // Collect per-time-bin samples across every recorded trajectory.
    std::map<long, std::vector<double>> cross_bins, alt_bins, esterr_bins, altitude_bins;
    const double bin = cfg_.trajectory_interval > 0.0 ? cfg_.trajectory_interval : 1.0;
    for (const auto& t : result.trials) {
      for (const auto& s : t.trajectory) {
        const long k = std::lround(s.time / bin);
        cross_bins[k].push_back(s.cross_track);
        alt_bins[k].push_back(s.altitude_error);
        esterr_bins[k].push_back(s.estimator_position_error);
        altitude_bins[k].push_back(s.altitude);
      }
    }
    util::CsvWriter w(cfg_.output_dir + "/envelope.csv",
                      {"time_s", "n", "cross_track_p05", "cross_track_p50", "cross_track_p95",
                       "cross_track_min", "cross_track_max", "altitude_error_p05",
                       "altitude_error_p50", "altitude_error_p95", "altitude_p05",
                       "altitude_p50", "altitude_p95", "est_pos_error_p50",
                       "est_pos_error_p95"});
    for (const auto& [k, values] : cross_bins) {
      const auto cs = computeStatistics("", "", values);
      const auto as = computeStatistics("", "", alt_bins[k]);
      const auto hs = computeStatistics("", "", altitude_bins[k]);
      const auto es = computeStatistics("", "", esterr_bins[k]);
      w.writeRow({static_cast<double>(k) * bin, static_cast<double>(cs.count), cs.p05,
                  cs.median, cs.p95, cs.min, cs.max, as.p05, as.median, as.p95, hs.p05,
                  hs.median, hs.p95, es.median, es.p95});
    }
  }

  // --- Summary ----------------------------------------------------------------------
  {
    std::ofstream f(cfg_.output_dir + "/summary.json");
    f << std::setprecision(10);
    f << "{\n";
    f << "  \"scenario\": \"" << base_.name << "\",\n";
    f << "  \"controller\": \"" << sim::toString(base_.controller) << "\",\n";
    f << "  \"feedback\": \"" << sim::toString(base_.feedback) << "\",\n";
    f << "  \"trials\": " << cfg_.trials << ",\n";
    f << "  \"master_seed\": " << cfg_.master_seed << ",\n";
    f << "  \"threads\": " << result.threads_used << ",\n";
    f << "  \"duration_per_trial_s\": " << base_.duration << ",\n";
    f << "  \"successes\": " << result.successes << ",\n";
    f << "  \"failures\": " << result.failures << ",\n";
    f << "  \"failure_rate\": " << result.failure_rate << ",\n";
    f << "  \"wall_clock_s\": " << result.wall_clock_seconds << ",\n";
    f << "  \"mean_trial_wall_clock_s\": " << result.mean_trial_seconds << ",\n";

    std::map<std::string, int> reasons;
    for (const auto& t : result.trials)
      if (!t.ok) ++reasons[t.termination_reason];
    f << "  \"failure_reasons\": {";
    bool first = true;
    for (const auto& [reason, count] : reasons) {
      if (!first) f << ", ";
      f << "\"" << reason << "\": " << count;
      first = false;
    }
    f << "},\n";

    f << "  \"statistics\": [\n";
    for (std::size_t i = 0; i < result.statistics.size(); ++i) {
      const auto& s = result.statistics[i];
      f << "    {\"name\": \"" << s.name << "\", \"unit\": \"" << s.unit << "\", \"count\": "
        << s.count << ", \"mean\": " << s.mean << ", \"stddev\": " << s.stddev
        << ", \"min\": " << s.min << ", \"p05\": " << s.p05 << ", \"median\": " << s.median
        << ", \"p95\": " << s.p95 << ", \"p99\": " << s.p99 << ", \"max\": " << s.max << "}";
      if (i + 1 < result.statistics.size()) f << ",";
      f << "\n";
    }
    f << "  ]\n}\n";
  }
}

}  // namespace aether::mc
