/// \file Simulator.hpp
/// \brief Closed-loop simulation driver: dynamics, environment, sensors, estimator,
///        guidance, control and logging.
#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "aether/analysis/Trim.hpp"
#include "aether/control/Autopilot.hpp"
#include "aether/dynamics/Actuators.hpp"
#include "aether/dynamics/RigidBody.hpp"
#include "aether/env/Atmosphere.hpp"
#include "aether/env/Wind.hpp"
#include "aether/estimation/ErrorStateEkf.hpp"
#include "aether/guidance/WaypointFollower.hpp"
#include "aether/integrate/Integrator.hpp"
#include "aether/sensors/Sensors.hpp"
#include "aether/sim/Config.hpp"

namespace aether::sim {

/// \brief Aggregate performance metrics of one closed-loop run.
struct SimulationMetrics {
  double simulated_time = 0.0;        ///< Simulated duration actually completed [s]
  bool completed = false;             ///< True if the run reached the requested duration
  bool stable = false;                ///< True if no safety limit was violated
  std::string termination_reason;     ///< "completed", "ground_contact", "airspeed_low", ...

  // Guidance / tracking (evaluated after a short settling window).
  double rms_cross_track = 0.0;       ///< RMS lateral path deviation [m]
  double max_cross_track = 0.0;       ///< Peak lateral path deviation [m]
  double rms_altitude_error = 0.0;    ///< RMS altitude tracking error [m]
  double max_altitude_error = 0.0;    ///< Peak altitude tracking error [m]
  double rms_airspeed_error = 0.0;    ///< RMS airspeed tracking error [m/s]
  double max_airspeed_error = 0.0;    ///< Peak airspeed tracking error [m/s]
  double max_bank = 0.0;              ///< Peak |roll| [rad]
  double max_alpha = 0.0;             ///< Peak |angle of attack| [rad]
  int waypoints_reached = 0;          ///< Waypoint switches performed
  int laps_completed = 0;             ///< Full circuits flown

  // Estimator accuracy (RMS over the analysis window).
  double rmse_position = 0.0;         ///< 3-D position error RMS [m]
  double rmse_position_horizontal = 0.0;  ///< Horizontal position error RMS [m]
  double rmse_altitude = 0.0;         ///< Altitude error RMS [m]
  double rmse_velocity = 0.0;         ///< 3-D velocity error RMS [m/s]
  double rmse_attitude = 0.0;         ///< Attitude error RMS (rotation angle) [rad]
  double rmse_yaw = 0.0;              ///< Yaw error RMS [rad]
  double final_gyro_bias_error = 0.0; ///< |b_g estimate - truth| at the end [rad/s]
  double final_accel_bias_error = 0.0;///< |b_a estimate - truth| at the end [m/s^2]
  double min_covariance_eigenvalue = 0.0;  ///< Smallest eigenvalue of P seen during the run

  // Numerics / performance.
  double wall_clock_seconds = 0.0;    ///< Wall-clock runtime of the run [s]
  long dynamics_evaluations = 0;      ///< Right-hand-side evaluations consumed
  int steps = 0;                      ///< Outer simulation steps taken
  int rejected_steps = 0;             ///< Rejected adaptive steps
  double real_time_factor = 0.0;      ///< simulated_time / wall_clock_seconds
};

/// \brief One recorded point along a trajectory, used by the Monte-Carlo runner and by any
/// caller that wants the trajectory in memory rather than on disk.
struct TrajectorySample {
  double time = 0.0;            ///< Simulation time [s]
  double north = 0.0;           ///< North position [m]
  double east = 0.0;            ///< East position [m]
  double altitude = 0.0;        ///< Altitude [m]
  double airspeed = 0.0;        ///< True airspeed [m/s]
  double roll = 0.0;            ///< Euler roll [rad]
  double pitch = 0.0;           ///< Euler pitch [rad]
  double yaw = 0.0;             ///< Euler yaw [rad]
  double cross_track = 0.0;     ///< Signed cross-track error [m]
  double altitude_error = 0.0;  ///< Altitude tracking error [m]
  double estimator_position_error = 0.0;  ///< |estimate - truth| position error [m]
  double estimator_attitude_error = 0.0;  ///< Attitude estimate error [rad]
};

/// Callback invoked at each recorded trajectory sample.
using TrajectoryRecorder = std::function<void(const TrajectorySample&)>;

/// \brief Result of a closed-loop run.
struct SimulationResult {
  SimulationMetrics metrics;          ///< Aggregate metrics.
  analysis::TrimResult trim;          ///< Trim point the run was initialised from.
  StateVec final_state = StateVec::Zero();  ///< Truth state at the end of the run.
};

/// \brief Closed-loop flight simulator.
///
/// The outer loop runs at a fixed step `config.integration.dt`. Within one step the
/// environment and actuator states are held constant (zero-order hold) while the selected
/// integrator advances the rigid-body state — for the adaptive integrator this means
/// arbitrarily many sub-steps inside each fixed frame, which keeps sensor and controller
/// scheduling deterministic regardless of the integrator.
///
/// Sensors, the estimator, guidance and the control laws each run on their own schedule, so
/// the filter sees genuinely asynchronous, multi-rate measurements.
class Simulator {
 public:
  /// \param config Scenario configuration.
  /// \param aircraft Airframe parameters.
  Simulator(const ScenarioConfig& config, const dynamics::AircraftParameters& aircraft);

  /// Run the scenario to completion (or until a safety limit trips).
  SimulationResult run();

  /// Trim point computed at construction.
  const analysis::TrimResult& trim() const { return trim_; }

  /// Linear model at the trim point (also used for the LQR design).
  const analysis::LinearModel& linearModel() const { return linear_; }

  /// The autopilot in use.
  const control::Autopilot& autopilot() const { return *autopilot_; }

  /// Airframe parameters in use.
  const dynamics::AircraftParameters& aircraft() const { return aircraft_; }

  /// Scenario configuration in use.
  const ScenarioConfig& config() const { return cfg_; }

  /// Write the mission waypoints to `<output_dir>/waypoints.csv`.
  void writeWaypoints(const std::string& dir) const;

  /// Record trajectory samples in memory at a fixed interval instead of (or in addition to)
  /// writing CSV logs.
  /// \param interval_s Sampling interval [s]; <= 0 disables recording.
  /// \param recorder Callback invoked for each sample.
  void setTrajectoryRecorder(double interval_s, TrajectoryRecorder recorder) {
    trajectory_interval_ = interval_s;
    trajectory_recorder_ = std::move(recorder);
  }

 private:
  StateVec buildInitialState(const Vec3& wind_ned) const;
  estimation::NavState buildInitialEstimate(const StateVec& truth, util::Rng& rng) const;
  control::VehicleFeedback truthFeedback(const StateVec& x,
                                         const dynamics::DynamicsDiagnostics& diag) const;
  control::VehicleFeedback estimateFeedback(const estimation::NavState& nav,
                                            double measured_airspeed) const;

  ScenarioConfig cfg_;
  dynamics::AircraftParameters aircraft_;
  dynamics::RigidBody6DOF model_;
  dynamics::ActuatorBank actuators_;
  env::Atmosphere atmosphere_;
  analysis::TrimResult trim_;
  analysis::LinearModel linear_;
  std::unique_ptr<control::Autopilot> autopilot_;
  guidance::WaypointFollower guidance_;
  double trajectory_interval_ = 0.0;
  TrajectoryRecorder trajectory_recorder_;
};

/// \brief Serialise a metrics structure to a JSON file.
void writeMetricsJson(const std::string& path, const SimulationResult& result,
                      const ScenarioConfig& cfg);

}  // namespace aether::sim
