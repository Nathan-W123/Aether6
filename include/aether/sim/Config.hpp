/// \file Config.hpp
/// \brief YAML-backed configuration for the airframe, the scenario and every subsystem.
#pragma once

#include <cstdint>
#include <string>

#include "aether/control/LqrAutopilot.hpp"
#include "aether/control/PidAutopilot.hpp"
#include "aether/dynamics/AircraftParameters.hpp"
#include "aether/env/Wind.hpp"
#include "aether/estimation/ErrorStateEkf.hpp"
#include "aether/guidance/WaypointFollower.hpp"
#include "aether/integrate/Integrator.hpp"
#include "aether/sensors/Sensors.hpp"

namespace aether::sim {

/// \brief Which controller is used.
enum class ControllerType { kPid, kLqr };

/// \brief Whether the control laws are fed truth or the navigation estimate.
enum class FeedbackSource { kTruth, kEstimate };

/// \brief Numerical integration settings for the simulation loop.
struct IntegrationConfig {
  std::string integrator = "rk4";        ///< "rk4" or "dopri54"
  double dt = 0.002;                     ///< Outer (fixed) simulation step [s]
  integrate::ToleranceSettings tolerances;  ///< Adaptive-integrator tolerances
};

/// \brief Initial-condition specification.
struct InitialConditionConfig {
  bool from_trim = true;      ///< Start from the trimmed straight-and-level condition.
  double airspeed = 25.0;     ///< Trim/initial airspeed [m/s]
  double altitude = 120.0;    ///< Initial altitude [m]
  double north = 0.0;         ///< Initial north position [m]
  double east = 0.0;          ///< Initial east position [m]
  double heading = 0.0;       ///< Initial heading [rad]
  double flight_path_angle = 0.0;  ///< Trim flight-path angle [rad]

  // Deterministic offsets applied on top of the trim state (used by scenarios and by
  // Monte-Carlo dispersion).
  double delta_altitude = 0.0;  ///< Additive altitude offset [m]
  double delta_airspeed = 0.0;  ///< Additive airspeed offset [m/s]
  double delta_roll = 0.0;      ///< Additive roll offset [rad]
  double delta_pitch = 0.0;     ///< Additive pitch offset [rad]
  double delta_yaw = 0.0;       ///< Additive yaw offset [rad]
};

/// \brief Logging options.
struct LoggingConfig {
  std::string output_dir = "results/nominal";  ///< Directory for CSV output
  int decimation = 10;       ///< Write one row every N simulation steps
  bool enabled = true;       ///< Disable to run without writing files (Monte Carlo)
  bool log_sensors = true;   ///< Include raw sensor columns
  bool log_estimator = true; ///< Include estimator and covariance columns
};

/// \brief Termination and safety limits.
struct SafetyConfig {
  double ground_altitude = 0.0;      ///< Altitude of the ground plane [m]
  double max_altitude = 3000.0;      ///< Abort above this altitude [m]
  double min_airspeed = 8.0;         ///< Abort below this airspeed [m/s]
  double max_airspeed = 60.0;        ///< Abort above this airspeed [m/s]
  double max_bank = 1.40;            ///< Abort beyond this |roll| [rad] (~80 deg)
  double max_pitch = 1.30;           ///< Abort beyond this |pitch| [rad] (~75 deg)
  bool stop_on_ground_contact = true;///< Stop the run when the ground plane is crossed
};

/// \brief Complete scenario definition.
struct ScenarioConfig {
  std::string name = "nominal";       ///< Scenario name (used in output paths)
  std::string aircraft_file;          ///< Path to the airframe YAML
  double duration = 240.0;            ///< Simulated duration [s]
  double control_rate = 100.0;        ///< Autopilot update rate [Hz]
  std::uint64_t seed = 20240917;      ///< Master random seed

  IntegrationConfig integration;
  InitialConditionConfig initial;
  env::WindConfig wind;
  double density_scale = 1.0;         ///< Multiplier on the ISA density [-]

  ControllerType controller = ControllerType::kLqr;
  FeedbackSource feedback = FeedbackSource::kEstimate;
  control::LqrWeights lqr_weights;
  control::LqrAutopilotLimits lqr_limits;
  control::PidAutopilotConfig pid_config;
  bool pid_config_explicit = false;   ///< True when the YAML overrode the default PID gains

  guidance::GuidanceConfig guidance;
  sensors::SensorConfig sensors;
  estimation::EkfConfig estimator;
  LoggingConfig logging;
  SafetyConfig safety;
};

/// \brief Load an airframe description from YAML.
/// \throws std::runtime_error when the file cannot be read or is malformed.
dynamics::AircraftParameters loadAircraft(const std::string& path);

/// \brief Load a scenario from YAML.
///
/// Relative `aircraft` paths are resolved against the scenario file's directory. Any key that
/// is absent keeps the default from the corresponding C++ struct, so configuration files only
/// need to state what they change.
/// \throws std::runtime_error when the file cannot be read or is malformed.
ScenarioConfig loadScenario(const std::string& path);

/// \brief Parse a controller name ("pid" or "lqr").
ControllerType parseControllerType(const std::string& name);
/// \brief Parse a feedback-source name ("truth" or "estimate").
FeedbackSource parseFeedbackSource(const std::string& name);
/// \brief Name of a controller type.
std::string toString(ControllerType t);
/// \brief Name of a feedback source.
std::string toString(FeedbackSource f);

}  // namespace aether::sim
