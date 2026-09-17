/// \file MonteCarlo.hpp
/// \brief Deterministic Monte-Carlo campaign over vehicle, environment and sensor dispersions.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "aether/sim/Config.hpp"
#include "aether/sim/Simulator.hpp"

namespace aether::mc {

/// \brief 1-sigma dispersions applied to each trial.
///
/// Multiplicative dispersions are applied as \f$x \leftarrow x\,(1 + \sigma\,\mathcal{N}(0,1))\f$
/// and are clipped to keep the airframe physically valid.
struct DispersionConfig {
  bool vary_mass_properties = true;    ///< Disperse mass and inertia.
  double mass_relative = 0.08;         ///< Relative 1-sigma on mass [-]
  double inertia_relative = 0.12;      ///< Relative 1-sigma on each inertia term [-]

  bool vary_aerodynamics = true;       ///< Disperse the aerodynamic coefficients.
  double aero_relative = 0.12;         ///< Relative 1-sigma on each aerodynamic derivative [-]
  double thrust_relative = 0.08;       ///< Relative 1-sigma on the motor constant [-]

  bool vary_initial_conditions = true; ///< Disperse the initial state.
  double initial_altitude = 10.0;      ///< 1-sigma altitude offset [m]
  double initial_airspeed = 1.5;       ///< 1-sigma airspeed offset [m/s]
  double initial_roll = 0.10;          ///< 1-sigma roll offset [rad]
  double initial_pitch = 0.07;         ///< 1-sigma pitch offset [rad]
  double initial_yaw = 0.35;           ///< 1-sigma heading offset [rad]

  bool vary_environment = true;        ///< Disperse the wind field and air density.
  double wind_speed_mean = 5.0;        ///< Mean steady wind magnitude [m/s]
  double wind_speed_sigma = 3.0;       ///< 1-sigma on the wind magnitude [m/s]
  double w20_mean = 7.0;               ///< Mean turbulence-driving wind at 6 m [m/s]
  double w20_sigma = 3.5;              ///< 1-sigma on that wind [m/s]
  double density_relative = 0.05;      ///< Relative 1-sigma on air density [-]

  bool vary_sensors = true;            ///< Disperse the sensor noise/bias magnitudes.
  double sensor_scale_sigma = 0.30;    ///< 1-sigma of log-normal scaling on sensor sigmas [-]
};

/// \brief Monte-Carlo campaign settings.
struct MonteCarloConfig {
  int trials = 200;                    ///< Number of trials.
  std::uint64_t master_seed = 987654321;  ///< Master seed; trial seeds derive from it.
  int threads = 0;                     ///< Worker threads (0 = hardware concurrency).
  double trajectory_interval = 1.0;    ///< In-memory trajectory sampling interval [s]
  int recorded_trajectories = 40;      ///< How many trials to keep full trajectories for.
  DispersionConfig dispersions;        ///< Dispersion definition.
  std::string output_dir = "results/monte_carlo";  ///< Output directory.
};

/// \brief Per-trial outcome.
struct TrialResult {
  int index = 0;                       ///< Trial index.
  std::uint64_t seed = 0;              ///< Seed used for the trial.
  bool ok = false;                     ///< True when the trial completed without a failure.
  std::string termination_reason;      ///< Why the trial ended.

  // Sampled dispersion values (recorded so results can be regressed against inputs).
  double mass = 0.0;                   ///< Dispersed mass [kg]
  double inertia_scale = 1.0;          ///< Mean inertia scale factor [-]
  double cl_alpha = 0.0;               ///< Dispersed lift-curve slope [1/rad]
  double cm_alpha = 0.0;               ///< Dispersed pitch stiffness [1/rad]
  double wind_speed = 0.0;             ///< Steady wind magnitude [m/s]
  double wind_direction = 0.0;         ///< Steady wind direction (from-north bearing) [rad]
  double w20 = 0.0;                    ///< Turbulence-driving wind speed [m/s]
  double density_scale = 1.0;          ///< Air-density scale factor [-]
  double sensor_scale = 1.0;           ///< Sensor noise scale factor [-]
  double init_altitude_offset = 0.0;   ///< Initial altitude offset [m]
  double init_airspeed_offset = 0.0;   ///< Initial airspeed offset [m/s]
  double init_yaw_offset = 0.0;        ///< Initial heading offset [rad]

  sim::SimulationMetrics metrics;      ///< Full metric set of the trial.
  std::vector<sim::TrajectorySample> trajectory;  ///< Recorded trajectory (may be empty).
};

/// \brief Aggregate campaign statistics for one scalar metric.
struct MetricStatistics {
  std::string name;   ///< Metric name.
  std::string unit;   ///< Physical unit.
  double mean = 0.0;  ///< Sample mean.
  double stddev = 0.0;///< Sample standard deviation (n-1).
  double min = 0.0;   ///< Minimum.
  double p05 = 0.0;   ///< 5th percentile.
  double median = 0.0;///< Median.
  double p95 = 0.0;   ///< 95th percentile.
  double p99 = 0.0;   ///< 99th percentile.
  double max = 0.0;   ///< Maximum.
  int count = 0;      ///< Number of samples contributing.
};

/// \brief Campaign result.
struct MonteCarloResult {
  std::vector<TrialResult> trials;          ///< Per-trial outcomes, ordered by index.
  std::vector<MetricStatistics> statistics; ///< Aggregate statistics over successful trials.
  int successes = 0;                        ///< Trials that completed without a failure.
  int failures = 0;                         ///< Trials that tripped a safety limit.
  double failure_rate = 0.0;                ///< failures / trials.
  double wall_clock_seconds = 0.0;          ///< Total campaign wall-clock time [s]
  double mean_trial_seconds = 0.0;          ///< Mean per-trial wall-clock time [s]
  int threads_used = 1;                     ///< Worker threads actually used.
};

/// \brief Deterministic Monte-Carlo runner.
///
/// Trial \a i draws every random quantity from streams derived with SplitMix64 from
/// `master_seed` and \a i, so the campaign is bit-for-bit reproducible and independent of the
/// number of worker threads or of the order in which trials complete.
class MonteCarloRunner {
 public:
  /// \param base Baseline scenario (its logging is forced off for the trials).
  /// \param aircraft Baseline airframe.
  /// \param config Campaign settings.
  MonteCarloRunner(sim::ScenarioConfig base, dynamics::AircraftParameters aircraft,
                   MonteCarloConfig config);

  /// Run the campaign. \param verbose Print progress to stdout.
  MonteCarloResult run(bool verbose = true);

  /// Build the dispersed airframe and scenario for a single trial (exposed for testing).
  void buildTrial(int index, dynamics::AircraftParameters* aircraft,
                  sim::ScenarioConfig* scenario, TrialResult* record) const;

  /// Write trials.csv, envelope.csv, trajectories.csv and summary.json into `output_dir`.
  void write(const MonteCarloResult& result) const;

 private:
  sim::ScenarioConfig base_;
  dynamics::AircraftParameters aircraft_;
  MonteCarloConfig cfg_;
};

/// \brief Compute mean/stddev/percentiles of a sample set.
MetricStatistics computeStatistics(std::string name, std::string unit, std::vector<double> data);

/// \brief Load the `monte_carlo:` section of a scenario file.
///
/// Any key that is absent keeps the value from \a defaults, so a scenario only needs to state
/// what it changes. A file with no `monte_carlo:` section returns \a defaults unchanged.
/// \throws std::runtime_error when the file cannot be read or is malformed.
MonteCarloConfig loadMonteCarloConfig(const std::string& scenario_path,
                                      const MonteCarloConfig& defaults = {});

}  // namespace aether::mc
