#include "aether/sim/Config.hpp"

#include "aether/mc/MonteCarlo.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <stdexcept>

#include "aether/core/Constants.hpp"

namespace aether::sim {

/// Shared YAML helpers. They live in a named namespace so the Monte-Carlo loader at the end
/// of this file can reuse them.
namespace detail {

/// Read an optional scalar, leaving the target untouched when the key is absent.
template <typename T>
void getScalar(const YAML::Node& node, const char* key, T& target) {
  if (node && node[key] && !node[key].IsNull()) target = node[key].as<T>();
}

/// Read an optional angle given in degrees into a radian-valued target.
inline void getDegrees(const YAML::Node& node, const char* key, double& target_rad) {
  if (node && node[key] && !node[key].IsNull())
    target_rad = node[key].as<double>() * constants::kDegToRad;
}

/// Load a YAML file, converting every failure into a std::runtime_error with context.
inline YAML::Node loadYamlFile(const std::string& path) {
  if (!std::filesystem::exists(path))
    throw std::runtime_error("configuration file not found: " + path);
  try {
    return YAML::LoadFile(path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("failed to parse '" + path + "': " + e.what());
  }
}

}  // namespace detail

namespace {

using detail::getDegrees;
using detail::getScalar;

/// Read an optional scalar, leaving the target untouched when the key is absent.
template <typename T>
void get(const YAML::Node& node, const char* key, T& target) {
  getScalar(node, key, target);
}

/// Read an optional angle given in degrees into a radian-valued target.
void getDeg(const YAML::Node& node, const char* key, double& target_rad) {
  getDegrees(node, key, target_rad);
}

/// Read an optional 3-vector.
void getVec3(const YAML::Node& node, const char* key, Vec3& target) {
  if (node && node[key] && node[key].IsSequence() && node[key].size() == 3) {
    target = Vec3(node[key][0].as<double>(), node[key][1].as<double>(),
                  node[key][2].as<double>());
  }
}

void loadPid(const YAML::Node& node, control::PidConfig& cfg) {
  if (!node) return;
  get(node, "kp", cfg.kp);
  get(node, "ki", cfg.ki);
  get(node, "kd", cfg.kd);
  get(node, "output_min", cfg.output_min);
  get(node, "output_max", cfg.output_max);
  get(node, "anti_windup_gain", cfg.anti_windup_gain);
  get(node, "derivative_filter_tau", cfg.derivative_filter_tau);
  get(node, "derivative_on_measurement", cfg.derivative_on_measurement);
}

YAML::Node loadFileOrThrow(const std::string& path) { return detail::loadYamlFile(path); }

}  // namespace

ControllerType parseControllerType(const std::string& name) {
  if (name == "pid") return ControllerType::kPid;
  if (name == "lqr") return ControllerType::kLqr;
  throw std::invalid_argument("unknown controller type '" + name + "' (expected pid or lqr)");
}

FeedbackSource parseFeedbackSource(const std::string& name) {
  if (name == "truth") return FeedbackSource::kTruth;
  if (name == "estimate") return FeedbackSource::kEstimate;
  throw std::invalid_argument("unknown feedback source '" + name +
                              "' (expected truth or estimate)");
}

std::string toString(ControllerType t) { return t == ControllerType::kPid ? "pid" : "lqr"; }
std::string toString(FeedbackSource f) {
  return f == FeedbackSource::kTruth ? "truth" : "estimate";
}

dynamics::AircraftParameters loadAircraft(const std::string& path) {
  const YAML::Node root = loadFileOrThrow(path);
  dynamics::AircraftParameters p = dynamics::defaultAircraft();

  get(root, "name", p.name);
  get(root, "synthetic", p.synthetic);

  if (const YAML::Node m = root["mass_properties"]) {
    get(m, "mass", p.mass);
    get(m, "Jx", p.Jx);
    get(m, "Jy", p.Jy);
    get(m, "Jz", p.Jz);
    get(m, "Jxz", p.Jxz);
  }
  if (const YAML::Node g = root["geometry"]) {
    get(g, "wing_area", p.wing_area);
    get(g, "wing_span", p.wing_span);
    get(g, "mean_chord", p.mean_chord);
  }
  if (const YAML::Node a = root["aerodynamics"]) {
    if (const YAML::Node l = a["longitudinal"]) {
      get(l, "CL0", p.lon.CL0);
      get(l, "CL_alpha", p.lon.CL_alpha);
      get(l, "CL_q", p.lon.CL_q);
      get(l, "CL_de", p.lon.CL_de);
      get(l, "CD0", p.lon.CD0);
      get(l, "CD_q", p.lon.CD_q);
      get(l, "CD_de", p.lon.CD_de);
      get(l, "oswald", p.lon.oswald);
      get(l, "Cm0", p.lon.Cm0);
      get(l, "Cm_alpha", p.lon.Cm_alpha);
      get(l, "Cm_q", p.lon.Cm_q);
      get(l, "Cm_de", p.lon.Cm_de);
      getDeg(l, "alpha_stall_deg", p.lon.alpha_stall);
      get(l, "stall_sharpness", p.lon.stall_sharpness);
      get(l, "enable_stall_model", p.lon.enable_stall_model);
    }
    if (const YAML::Node t = a["lateral"]) {
      get(t, "CY0", p.lat.CY0);
      get(t, "CY_beta", p.lat.CY_beta);
      get(t, "CY_p", p.lat.CY_p);
      get(t, "CY_r", p.lat.CY_r);
      get(t, "CY_da", p.lat.CY_da);
      get(t, "CY_dr", p.lat.CY_dr);
      get(t, "Cl0", p.lat.Cl0);
      get(t, "Cl_beta", p.lat.Cl_beta);
      get(t, "Cl_p", p.lat.Cl_p);
      get(t, "Cl_r", p.lat.Cl_r);
      get(t, "Cl_da", p.lat.Cl_da);
      get(t, "Cl_dr", p.lat.Cl_dr);
      get(t, "Cn0", p.lat.Cn0);
      get(t, "Cn_beta", p.lat.Cn_beta);
      get(t, "Cn_p", p.lat.Cn_p);
      get(t, "Cn_r", p.lat.Cn_r);
      get(t, "Cn_da", p.lat.Cn_da);
      get(t, "Cn_dr", p.lat.Cn_dr);
    }
  }
  if (const YAML::Node pr = root["propulsion"]) {
    get(pr, "disk_area", p.prop.disk_area);
    get(pr, "efficiency", p.prop.efficiency);
    get(pr, "k_motor", p.prop.k_motor);
    get(pr, "k_torque", p.prop.k_torque);
    get(pr, "k_omega", p.prop.k_omega);
    getVec3(pr, "thrust_offset", p.prop.thrust_offset);
  }
  if (const YAML::Node ac = root["actuators"]) {
    getDeg(ac, "elevator_min_deg", p.actuators.elevator_min);
    getDeg(ac, "elevator_max_deg", p.actuators.elevator_max);
    getDeg(ac, "aileron_min_deg", p.actuators.aileron_min);
    getDeg(ac, "aileron_max_deg", p.actuators.aileron_max);
    getDeg(ac, "rudder_min_deg", p.actuators.rudder_min);
    getDeg(ac, "rudder_max_deg", p.actuators.rudder_max);
    get(ac, "throttle_min", p.actuators.throttle_min);
    get(ac, "throttle_max", p.actuators.throttle_max);
    getDeg(ac, "surface_rate_limit_deg_s", p.actuators.surface_rate_limit);
    get(ac, "throttle_rate_limit", p.actuators.throttle_rate_limit);
    get(ac, "surface_time_constant", p.actuators.surface_time_constant);
    get(ac, "throttle_time_constant", p.actuators.throttle_time_constant);
  }

  p.validate();
  return p;
}

ScenarioConfig loadScenario(const std::string& path) {
  const YAML::Node root = loadFileOrThrow(path);
  ScenarioConfig c;
  const std::filesystem::path base = std::filesystem::path(path).parent_path();

  get(root, "name", c.name);
  get(root, "duration", c.duration);
  get(root, "control_rate", c.control_rate);
  get(root, "seed", c.seed);

  std::string aircraft = "../aircraft/aether6_uav.yaml";
  get(root, "aircraft", aircraft);
  std::filesystem::path ap(aircraft);
  c.aircraft_file = ap.is_absolute() ? ap.string() : (base / ap).lexically_normal().string();

  if (const YAML::Node n = root["integration"]) {
    get(n, "integrator", c.integration.integrator);
    get(n, "dt", c.integration.dt);
    if (const YAML::Node t = n["tolerances"]) {
      get(t, "rel_tol", c.integration.tolerances.rel_tol);
      get(t, "abs_tol", c.integration.tolerances.abs_tol);
      get(t, "min_step", c.integration.tolerances.min_step);
      get(t, "max_step", c.integration.tolerances.max_step);
      get(t, "safety", c.integration.tolerances.safety);
    }
  }

  if (const YAML::Node n = root["initial"]) {
    get(n, "from_trim", c.initial.from_trim);
    get(n, "airspeed", c.initial.airspeed);
    get(n, "altitude", c.initial.altitude);
    get(n, "north", c.initial.north);
    get(n, "east", c.initial.east);
    getDeg(n, "heading_deg", c.initial.heading);
    getDeg(n, "flight_path_angle_deg", c.initial.flight_path_angle);
    get(n, "delta_altitude", c.initial.delta_altitude);
    get(n, "delta_airspeed", c.initial.delta_airspeed);
    getDeg(n, "delta_roll_deg", c.initial.delta_roll);
    getDeg(n, "delta_pitch_deg", c.initial.delta_pitch);
    getDeg(n, "delta_yaw_deg", c.initial.delta_yaw);
  }

  if (const YAML::Node n = root["environment"]) {
    get(n, "density_scale", c.density_scale);
    if (const YAML::Node w = n["wind"]) {
      getVec3(w, "steady_ned", c.wind.steady_ned);
      get(w, "enable_shear", c.wind.enable_shear);
      get(w, "shear_reference_altitude", c.wind.shear_reference_altitude);
      get(w, "shear_roughness", c.wind.shear_roughness);
      get(w, "enable_turbulence", c.wind.enable_turbulence);
      getVec3(w, "sigma", c.wind.sigma);
      getVec3(w, "length_scale", c.wind.length_scale);
      get(w, "altitude_scaled", c.wind.altitude_scaled);
      get(w, "w20", c.wind.w20);
    }
  }

  if (const YAML::Node n = root["control"]) {
    std::string type = toString(c.controller);
    get(n, "type", type);
    c.controller = parseControllerType(type);
    std::string fb = toString(c.feedback);
    get(n, "feedback", fb);
    c.feedback = parseFeedbackSource(fb);

    if (const YAML::Node l = n["lqr"]) {
      if (const YAML::Node w = l["weights"]) {
        get(w, "q_u", c.lqr_weights.q_u);
        get(w, "q_w", c.lqr_weights.q_w);
        get(w, "q_pitch_rate", c.lqr_weights.q_pitch_rate);
        get(w, "q_pitch", c.lqr_weights.q_pitch);
        get(w, "q_altitude", c.lqr_weights.q_altitude);
        get(w, "q_int_altitude", c.lqr_weights.q_int_altitude);
        get(w, "q_int_airspeed", c.lqr_weights.q_int_airspeed);
        get(w, "r_elevator", c.lqr_weights.r_elevator);
        get(w, "r_throttle", c.lqr_weights.r_throttle);
        get(w, "q_v", c.lqr_weights.q_v);
        get(w, "q_roll_rate", c.lqr_weights.q_roll_rate);
        get(w, "q_yaw_rate", c.lqr_weights.q_yaw_rate);
        get(w, "q_roll", c.lqr_weights.q_roll);
        get(w, "q_course", c.lqr_weights.q_course);
        get(w, "q_int_course", c.lqr_weights.q_int_course);
        get(w, "r_aileron", c.lqr_weights.r_aileron);
        get(w, "r_rudder", c.lqr_weights.r_rudder);
      }
      if (const YAML::Node lim = l["limits"]) {
        getDeg(lim, "max_bank_deg", c.lqr_limits.max_bank);
        getDeg(lim, "max_pitch_deg", c.lqr_limits.max_pitch);
        get(lim, "derive_limits_from_gains", c.lqr_limits.derive_limits_from_gains);
        get(lim, "integrator_authority", c.lqr_limits.integrator_authority);
        get(lim, "protection_enabled", c.lqr_limits.protection.enabled);
        getDeg(lim, "alpha_max_deg", c.lqr_limits.protection.alpha_max);
        get(lim, "alpha_protection_gain", c.lqr_limits.protection.alpha_gain);
        get(lim, "alpha_protection_rate_gain", c.lqr_limits.protection.rate_gain);
        getDeg(lim, "alpha_protection_blend_deg", c.lqr_limits.protection.blend_width);
        get(lim, "max_climb_rate", c.lqr_limits.max_climb_rate);
        get(lim, "integrator_altitude_limit", c.lqr_limits.integrator_altitude_limit);
        get(lim, "integrator_airspeed_limit", c.lqr_limits.integrator_airspeed_limit);
        get(lim, "integrator_course_limit", c.lqr_limits.integrator_course_limit);
        getDeg(lim, "course_error_limit_deg", c.lqr_limits.course_error_limit);
        get(lim, "altitude_error_limit", c.lqr_limits.altitude_error_limit);
      }
    }
    if (const YAML::Node pnode = n["pid"]) {
      c.pid_config_explicit = true;
      loadPid(pnode["roll_to_aileron"], c.pid_config.roll_to_aileron);
      loadPid(pnode["pitch_to_elevator"], c.pid_config.pitch_to_elevator);
      loadPid(pnode["sideslip_to_rudder"], c.pid_config.sideslip_to_rudder);
      loadPid(pnode["course_to_roll"], c.pid_config.course_to_roll);
      loadPid(pnode["altitude_to_pitch"], c.pid_config.altitude_to_pitch);
      loadPid(pnode["airspeed_to_throttle"], c.pid_config.airspeed_to_throttle);
      getDeg(pnode, "max_bank_deg", c.pid_config.max_bank);
      getDeg(pnode, "max_pitch_deg", c.pid_config.max_pitch);
      get(pnode, "max_climb_rate", c.pid_config.max_climb_rate);
      get(pnode, "yaw_damper_gain", c.pid_config.yaw_damper_gain);
      get(pnode, "yaw_damper_tau", c.pid_config.yaw_damper_tau);
      get(pnode, "protection_enabled", c.pid_config.protection.enabled);
      getDeg(pnode, "alpha_max_deg", c.pid_config.protection.alpha_max);
      get(pnode, "alpha_protection_gain", c.pid_config.protection.alpha_gain);
      get(pnode, "alpha_protection_rate_gain", c.pid_config.protection.rate_gain);
      getDeg(pnode, "alpha_protection_blend_deg", c.pid_config.protection.blend_width);
    }
  }

  if (const YAML::Node n = root["guidance"]) {
    get(n, "path_gain", c.guidance.path_gain);
    getDeg(n, "chi_infinity_deg", c.guidance.chi_infinity);
    get(n, "capture_radius", c.guidance.capture_radius);
    get(n, "orbit_radius", c.guidance.orbit_radius);
    get(n, "orbit_gain", c.guidance.orbit_gain);
    get(n, "orbit_direction", c.guidance.orbit_direction);
    get(n, "use_half_plane_switching", c.guidance.use_half_plane_switching);
    std::string term = "loop";
    get(n, "terminal", term);
    c.guidance.terminal = guidance::parseTerminalBehaviour(term);
    if (const YAML::Node wps = n["waypoints"]) {
      c.guidance.waypoints.clear();
      for (const auto& w : wps) {
        guidance::Waypoint p;
        get(w, "north", p.north);
        get(w, "east", p.east);
        get(w, "altitude", p.altitude);
        p.airspeed = c.initial.airspeed;
        get(w, "airspeed", p.airspeed);
        c.guidance.waypoints.push_back(p);
      }
    }
  }

  if (const YAML::Node n = root["sensors"]) {
    get(n, "enable_gps", c.sensors.enable_gps);
    get(n, "gps_outage_start", c.sensors.gps_outage_start);
    get(n, "gps_outage_end", c.sensors.gps_outage_end);
    if (const YAML::Node s = n["imu"]) {
      get(s, "rate", c.sensors.imu.rate);
      get(s, "gyro_noise", c.sensors.imu.gyro_noise_density);
      get(s, "accel_noise", c.sensors.imu.accel_noise_density);
      get(s, "gyro_bias_initial", c.sensors.imu.gyro_bias_initial);
      get(s, "accel_bias_initial", c.sensors.imu.accel_bias_initial);
      get(s, "gyro_bias_walk", c.sensors.imu.gyro_bias_walk);
      get(s, "accel_bias_walk", c.sensors.imu.accel_bias_walk);
    }
    if (const YAML::Node s = n["gps"]) {
      get(s, "rate", c.sensors.gps.rate);
      get(s, "position_noise_ne", c.sensors.gps.position_noise_ne);
      get(s, "position_noise_d", c.sensors.gps.position_noise_d);
      get(s, "velocity_noise", c.sensors.gps.velocity_noise);
      get(s, "slow_error_sigma", c.sensors.gps.slow_error_sigma);
      get(s, "slow_error_tau", c.sensors.gps.slow_error_tau);
    }
    if (const YAML::Node s = n["baro"]) {
      get(s, "rate", c.sensors.baro.rate);
      get(s, "noise", c.sensors.baro.noise);
      get(s, "bias_sigma", c.sensors.baro.bias_sigma);
    }
    if (const YAML::Node s = n["magnetometer"]) {
      get(s, "rate", c.sensors.magnetometer.rate);
      get(s, "field_strength", c.sensors.magnetometer.field_strength);
      getDeg(s, "inclination_deg", c.sensors.magnetometer.inclination);
      getDeg(s, "declination_deg", c.sensors.magnetometer.declination);
      get(s, "noise", c.sensors.magnetometer.noise);
      get(s, "bias_sigma", c.sensors.magnetometer.bias_sigma);
    }
    if (const YAML::Node s = n["airspeed"]) {
      get(s, "rate", c.sensors.airspeed.rate);
      get(s, "noise", c.sensors.airspeed.noise);
      get(s, "bias_sigma", c.sensors.airspeed.bias_sigma);
    }
  }

  if (const YAML::Node n = root["estimator"]) {
    get(n, "gyro_noise", c.estimator.gyro_noise);
    get(n, "accel_noise", c.estimator.accel_noise);
    get(n, "gyro_bias_walk", c.estimator.gyro_bias_walk);
    get(n, "accel_bias_walk", c.estimator.accel_bias_walk);
    get(n, "gps_position_ne", c.estimator.gps_position_ne);
    get(n, "gps_position_d", c.estimator.gps_position_d);
    get(n, "gps_velocity", c.estimator.gps_velocity);
    get(n, "baro_noise", c.estimator.baro_noise);
    get(n, "mag_noise", c.estimator.mag_noise);
    get(n, "airspeed_noise", c.estimator.airspeed_noise);
    get(n, "wind_walk", c.estimator.wind_walk);
    get(n, "baro_bias_walk", c.estimator.baro_bias_walk);
    get(n, "init_position_sigma", c.estimator.init_position_sigma);
    get(n, "init_velocity_sigma", c.estimator.init_velocity_sigma);
    getDeg(n, "init_attitude_sigma_deg", c.estimator.init_attitude_sigma);
    get(n, "init_gyro_bias_sigma", c.estimator.init_gyro_bias_sigma);
    get(n, "init_accel_bias_sigma", c.estimator.init_accel_bias_sigma);
    get(n, "init_wind_sigma", c.estimator.init_wind_sigma);
    get(n, "init_baro_bias_sigma", c.estimator.init_baro_bias_sigma);
    get(n, "exact_discretisation", c.estimator.exact_discretisation);
    get(n, "innovation_gate", c.estimator.innovation_gate);
    get(n, "use_baro", c.estimator.use_baro);
    get(n, "use_magnetometer", c.estimator.use_magnetometer);
    get(n, "use_airspeed", c.estimator.use_airspeed);
    get(n, "initialise_wind_from_airspeed", c.estimator.initialise_wind_from_airspeed);
  }

  if (const YAML::Node n = root["logging"]) {
    get(n, "output_dir", c.logging.output_dir);
    get(n, "decimation", c.logging.decimation);
    get(n, "enabled", c.logging.enabled);
    get(n, "log_sensors", c.logging.log_sensors);
    get(n, "log_estimator", c.logging.log_estimator);
  }

  if (const YAML::Node n = root["safety"]) {
    get(n, "ground_altitude", c.safety.ground_altitude);
    get(n, "max_altitude", c.safety.max_altitude);
    get(n, "min_airspeed", c.safety.min_airspeed);
    get(n, "max_airspeed", c.safety.max_airspeed);
    getDeg(n, "max_bank_deg", c.safety.max_bank);
    getDeg(n, "max_pitch_deg", c.safety.max_pitch);
    get(n, "stop_on_ground_contact", c.safety.stop_on_ground_contact);
  }

  if (c.guidance.waypoints.size() < 2)
    throw std::runtime_error("scenario '" + path + "': at least two waypoints are required");
  if (!(c.integration.dt > 0.0))
    throw std::runtime_error("scenario '" + path + "': integration.dt must be positive");
  if (!(c.control_rate > 0.0))
    throw std::runtime_error("scenario '" + path + "': control_rate must be positive");

  return c;
}

}  // namespace aether::sim

namespace aether::mc {

MonteCarloConfig loadMonteCarloConfig(const std::string& scenario_path,
                                      const MonteCarloConfig& defaults) {
  using aether::sim::detail::getScalar;
  const YAML::Node root = aether::sim::detail::loadYamlFile(scenario_path);
  MonteCarloConfig c = defaults;
  const YAML::Node n = root["monte_carlo"];
  if (!n) return c;

  getScalar(n, "trials", c.trials);
  getScalar(n, "master_seed", c.master_seed);
  getScalar(n, "threads", c.threads);
  getScalar(n, "trajectory_interval", c.trajectory_interval);
  getScalar(n, "recorded_trajectories", c.recorded_trajectories);
  getScalar(n, "output_dir", c.output_dir);

  if (const YAML::Node d = n["dispersions"]) {
    auto& x = c.dispersions;
    getScalar(d, "vary_mass_properties", x.vary_mass_properties);
    getScalar(d, "mass_relative", x.mass_relative);
    getScalar(d, "inertia_relative", x.inertia_relative);
    getScalar(d, "vary_aerodynamics", x.vary_aerodynamics);
    getScalar(d, "aero_relative", x.aero_relative);
    getScalar(d, "thrust_relative", x.thrust_relative);
    getScalar(d, "vary_initial_conditions", x.vary_initial_conditions);
    getScalar(d, "initial_altitude", x.initial_altitude);
    getScalar(d, "initial_airspeed", x.initial_airspeed);
    aether::sim::detail::getDegrees(d, "initial_roll_deg", x.initial_roll);
    aether::sim::detail::getDegrees(d, "initial_pitch_deg", x.initial_pitch);
    aether::sim::detail::getDegrees(d, "initial_yaw_deg", x.initial_yaw);
    getScalar(d, "vary_environment", x.vary_environment);
    getScalar(d, "wind_speed_mean", x.wind_speed_mean);
    getScalar(d, "wind_speed_sigma", x.wind_speed_sigma);
    getScalar(d, "w20_mean", x.w20_mean);
    getScalar(d, "w20_sigma", x.w20_sigma);
    getScalar(d, "density_relative", x.density_relative);
    getScalar(d, "vary_sensors", x.vary_sensors);
    getScalar(d, "sensor_scale_sigma", x.sensor_scale_sigma);
  }
  return c;
}

}  // namespace aether::mc
