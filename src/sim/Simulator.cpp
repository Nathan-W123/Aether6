#include "aether/sim/Simulator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

#include "aether/control/LqrAutopilot.hpp"
#include "aether/control/PidAutopilot.hpp"
#include "aether/core/Constants.hpp"
#include "aether/integrate/RungeKutta.hpp"
#include "aether/math/Rotation.hpp"
#include "aether/util/Csv.hpp"
#include "aether/util/Random.hpp"

namespace aether::sim {

namespace {

constexpr std::uint64_t kStreamSensors = 101;
constexpr std::uint64_t kStreamWind = 102;
constexpr std::uint64_t kStreamEkfInit = 103;

/// Running mean-square accumulator.
struct RmsAccumulator {
  double sum = 0.0;
  double peak = 0.0;
  long n = 0;
  void add(double v) {
    sum += v * v;
    peak = std::max(peak, std::abs(v));
    ++n;
  }
  double rms() const { return n > 0 ? std::sqrt(sum / static_cast<double>(n)) : 0.0; }
};

}  // namespace

Simulator::Simulator(const ScenarioConfig& config, const dynamics::AircraftParameters& aircraft)
    : cfg_(config),
      aircraft_(aircraft),
      model_(aircraft),
      actuators_(aircraft),
      atmosphere_(config.density_scale),
      guidance_(config.guidance) {
  // --- Trim ------------------------------------------------------------------------
  analysis::TrimSpec spec;
  spec.airspeed = cfg_.initial.airspeed;
  spec.altitude = cfg_.initial.altitude;
  spec.flight_path_angle = cfg_.initial.flight_path_angle;
  spec.density_override = atmosphere_.density(cfg_.initial.altitude);
  trim_ = analysis::TrimSolver(model_).solve(spec);
  if (!trim_.converged)
    throw std::runtime_error("Simulator: trim did not converge (" + trim_.message + ")");

  // --- Linearise -------------------------------------------------------------------
  dynamics::EnvironmentSample env;
  env.density = trim_.density;
  env.wind_ned = Vec3::Zero();
  linear_ = analysis::linearize(model_, trim_.state, trim_.controls, env);

  // --- Controller ------------------------------------------------------------------
  if (cfg_.controller == ControllerType::kLqr) {
    autopilot_ = std::make_unique<control::LqrAutopilot>(trim_, linear_, cfg_.lqr_weights,
                                                         cfg_.lqr_limits, aircraft_);
  } else {
    control::PidAutopilotConfig pid = cfg_.pid_config_explicit
                                          ? cfg_.pid_config
                                          : control::PidAutopilot::defaultConfig(aircraft_);
    autopilot_ = std::make_unique<control::PidAutopilot>(pid, aircraft_);
  }
}

StateVec Simulator::buildInitialState(const Vec3& wind_ned) const {
  dynamics::AircraftState s = dynamics::AircraftState::fromVec(trim_.state);
  const Vec3 e = s.euler();

  // Scale the trimmed air-relative velocity by the requested airspeed offset.
  const double airspeed = cfg_.initial.airspeed + cfg_.initial.delta_airspeed;
  const double scale = cfg_.initial.airspeed > 1e-6 ? airspeed / cfg_.initial.airspeed : 1.0;
  const Vec3 v_air_body = s.velocity * scale;

  s.position = Vec3(cfg_.initial.north, cfg_.initial.east,
                    -(cfg_.initial.altitude + cfg_.initial.delta_altitude));
  s.quaternion = math::eulerToQuat(e.x() + cfg_.initial.delta_roll,
                                   e.y() + cfg_.initial.delta_pitch,
                                   cfg_.initial.heading + cfg_.initial.delta_yaw);
  // The vehicle starts trimmed *relative to the air mass*: the ground velocity is the trimmed
  // air-relative velocity plus the local wind, so alpha, beta and Va match the trim point
  // exactly even in a steady wind.
  s.velocity = v_air_body + math::rotateNedToBody(s.quaternion, wind_ned);
  return s.vec();
}

estimation::NavState Simulator::buildInitialEstimate(const StateVec& truth, util::Rng& rng) const {
  const dynamics::AircraftState s = dynamics::AircraftState::fromVec(truth);
  estimation::NavState nav;
  nav.position = s.position + cfg_.estimator.init_position_sigma * rng.gaussian3();
  nav.velocity = math::rotateBodyToNed(s.quaternion, s.velocity) +
                 cfg_.estimator.init_velocity_sigma * rng.gaussian3();
  nav.quaternion =
      math::boxPlus(s.quaternion, cfg_.estimator.init_attitude_sigma * rng.gaussian3());
  nav.gyro_bias = Vec3::Zero();
  nav.accel_bias = Vec3::Zero();
  return nav;
}

control::VehicleFeedback Simulator::truthFeedback(
    const StateVec& x, const dynamics::DynamicsDiagnostics& diag) const {
  const dynamics::AircraftState s = dynamics::AircraftState::fromVec(x);
  const Vec3 e = s.euler();
  const Vec3 vned = math::rotateBodyToNed(s.quaternion, s.velocity);

  control::VehicleFeedback fb;
  fb.position_ned = s.position;
  fb.velocity_ned = vned;
  fb.velocity_body = s.velocity;
  fb.velocity_air_body = diag.air.velocity_rel;
  fb.wind_ned = vned - math::rotateBodyToNed(s.quaternion, diag.air.velocity_rel);
  fb.altitude = s.altitude();
  fb.climb_rate = -vned.z();
  fb.airspeed = diag.air.airspeed;
  fb.alpha = diag.air.alpha;
  fb.beta = diag.air.beta;
  fb.roll = e.x();
  fb.pitch = e.y();
  fb.yaw = e.z();
  fb.course = std::atan2(vned.y(), vned.x());
  fb.rate = s.rate;
  return fb;
}

control::VehicleFeedback Simulator::estimateFeedback(const estimation::NavState& nav,
                                                     double measured_airspeed) const {
  const Vec3 e = nav.euler();
  const Vec3 vbody = nav.velocityBody();
  const Vec3 vair_body = nav.airVelocityBody();

  control::VehicleFeedback fb;
  fb.position_ned = nav.position;
  fb.velocity_ned = nav.velocity;
  fb.velocity_body = vbody;
  fb.velocity_air_body = vair_body;
  fb.wind_ned = nav.windNed();
  fb.altitude = nav.altitude();
  fb.climb_rate = -nav.velocity.z();
  fb.airspeed = measured_airspeed;
  fb.roll = e.x();
  fb.pitch = e.y();
  fb.yaw = e.z();
  fb.course = std::atan2(nav.velocity.y(), nav.velocity.x());
  fb.rate = Vec3::Zero();  // filled by the caller from the latest gyro sample
  // Angle of attack and sideslip come from the *estimated air-relative* velocity, so they
  // stay correct in a crosswind: this is what the wind states in the filter buy us.
  const double Va = std::max(vair_body.norm(), constants::kMinAirspeed);
  fb.alpha = std::atan2(vair_body.z(), std::max(vair_body.x(), constants::kMinAirspeed));
  fb.beta = std::asin(std::clamp(vair_body.y() / Va, -1.0, 1.0));
  return fb;
}

void Simulator::writeWaypoints(const std::string& dir) const {
  std::filesystem::create_directories(dir);
  util::CsvWriter w(dir + "/waypoints.csv",
                    {"index", "north_m", "east_m", "altitude_m", "airspeed_mps"});
  const auto& wps = guidance_.waypoints();
  for (std::size_t i = 0; i < wps.size(); ++i)
    w.writeRow({static_cast<double>(i), wps[i].north, wps[i].east, wps[i].altitude,
                wps[i].airspeed});
}

SimulationResult Simulator::run() {
  using Clock = std::chrono::steady_clock;
  const auto wall_start = Clock::now();

  const double dt = cfg_.integration.dt;
  const int control_every =
      std::max(1, static_cast<int>(std::lround(1.0 / (cfg_.control_rate * dt))));
  const int total_steps = static_cast<int>(std::llround(cfg_.duration / dt));

  // --- Subsystems ------------------------------------------------------------------
  sensors::SensorSuite sensor_suite(cfg_.sensors,
                                    util::deriveSeed(cfg_.seed, kStreamSensors, 0));
  env::WindModel wind(cfg_.wind, util::deriveSeed(cfg_.seed, kStreamWind, 0));
  estimation::ErrorStateEkf ekf(cfg_.estimator, sensor_suite.magneticFieldNed(),
                                constants::kGravity);
  util::Rng init_rng(util::deriveSeed(cfg_.seed, kStreamEkfInit, 0));

  guidance_.reset();
  control::TrimReference trim_ref;
  trim_ref.controls = trim_.controls;
  trim_ref.roll = trim_.phi;
  trim_ref.pitch = trim_.theta;
  trim_ref.airspeed = cfg_.initial.airspeed;
  trim_ref.altitude = cfg_.initial.altitude;
  autopilot_->reset(trim_ref);
  actuators_.reset(trim_.controls);

  const double initial_agl =
      cfg_.initial.altitude + cfg_.initial.delta_altitude - cfg_.safety.ground_altitude;
  StateVec x = buildInitialState(wind.steadyNed(initial_agl));
  ekf.initialise(buildInitialEstimate(x, init_rng));

  auto integrator = integrate::makeIntegrator(cfg_.integration.integrator,
                                              cfg_.integration.tolerances);
  // Renormalise the quaternion after every accepted step so the attitude stays on the
  // unit sphere regardless of the integrator's local error.
  integrator->setProjection([](VecX& state) {
    state.segment<4>(StateIndex::kQuatW) =
        math::quatNormalize(state.segment<4>(StateIndex::kQuatW));
  });

  // --- Logging ---------------------------------------------------------------------
  std::unique_ptr<util::CsvWriter> log_states, log_gps, log_imu, log_baro, log_mag, log_air;
  if (cfg_.logging.enabled) {
    std::filesystem::create_directories(cfg_.logging.output_dir);
    std::vector<std::string> header = {
        "time_s",
        // truth state
        "pn_m", "pe_m", "pd_m", "altitude_m", "u_mps", "v_mps", "w_mps",
        "vn_mps", "ve_mps", "vd_mps", "qw", "qx", "qy", "qz",
        "roll_rad", "pitch_rad", "yaw_rad", "p_radps", "q_radps", "r_radps",
        "airspeed_mps", "alpha_rad", "beta_rad", "course_rad", "climb_rate_mps",
        // commands and actuators
        "cmd_elevator_rad", "cmd_aileron_rad", "cmd_rudder_rad", "cmd_throttle",
        "act_elevator_rad", "act_aileron_rad", "act_rudder_rad", "act_throttle",
        // guidance
        "wp_index", "wp_laps", "cmd_altitude_m", "cmd_airspeed_mps", "cmd_course_rad",
        "cross_track_m", "distance_to_wp_m", "orbiting",
        // autopilot diagnostics
        "roll_cmd_rad", "pitch_cmd_rad", "altitude_cmd_filtered_m", "course_error_rad",
        "altitude_error_m", "airspeed_error_mps",
        // environment
        "wind_n_mps", "wind_e_mps", "wind_d_mps", "gust_u_mps", "gust_v_mps", "gust_w_mps",
        "density_kgm3",
        // forces and moments (body axes)
        "fx_N", "fy_N", "fz_N", "mx_Nm", "my_Nm", "mz_Nm", "thrust_N",
        // estimator
        "est_pn_m", "est_pe_m", "est_pd_m", "est_vn_mps", "est_ve_mps", "est_vd_mps",
        "est_roll_rad", "est_pitch_rad", "est_yaw_rad",
        "est_bgx", "est_bgy", "est_bgz", "est_bax", "est_bay", "est_baz",
        "true_bgx", "true_bgy", "true_bgz", "true_bax", "true_bay", "true_baz",
        "est_wind_n_mps", "est_wind_e_mps", "est_airspeed_mps", "est_alpha_rad",
        "est_beta_rad", "est_baro_bias_m", "true_baro_bias_m",
        "sig_pn", "sig_pe", "sig_pd", "sig_vn", "sig_ve", "sig_vd",
        "sig_att_x", "sig_att_y", "sig_att_z",
        "sig_bgx", "sig_bgy", "sig_bgz", "sig_bax", "sig_bay", "sig_baz",
        "sig_wind_n", "sig_wind_e", "sig_baro_bias",
        "est_pos_err_m", "est_vel_err_mps", "est_att_err_rad"};
    log_states = std::make_unique<util::CsvWriter>(cfg_.logging.output_dir + "/states.csv",
                                                   header);
    if (cfg_.logging.log_sensors) {
      log_gps = std::make_unique<util::CsvWriter>(
          cfg_.logging.output_dir + "/sensor_gps.csv",
          std::vector<std::string>{"time_s", "pn_m", "pe_m", "pd_m", "vn_mps", "ve_mps",
                                   "vd_mps"});
      log_imu = std::make_unique<util::CsvWriter>(
          cfg_.logging.output_dir + "/sensor_imu.csv",
          std::vector<std::string>{"time_s", "gx_radps", "gy_radps", "gz_radps", "ax_mps2",
                                   "ay_mps2", "az_mps2"});
      log_baro = std::make_unique<util::CsvWriter>(
          cfg_.logging.output_dir + "/sensor_baro.csv",
          std::vector<std::string>{"time_s", "altitude_m"});
      log_mag = std::make_unique<util::CsvWriter>(
          cfg_.logging.output_dir + "/sensor_mag.csv",
          std::vector<std::string>{"time_s", "mx_T", "my_T", "mz_T"});
      log_air = std::make_unique<util::CsvWriter>(
          cfg_.logging.output_dir + "/sensor_airspeed.csv",
          std::vector<std::string>{"time_s", "airspeed_mps"});
    }
    writeWaypoints(cfg_.logging.output_dir);
  }

  // --- Run state -------------------------------------------------------------------
  SimulationResult result;
  result.trim = trim_;
  SimulationMetrics& m = result.metrics;
  m.termination_reason = "completed";
  m.completed = true;
  m.stable = true;

  RmsAccumulator acc_cross, acc_alt, acc_va, acc_pos, acc_posh, acc_altest, acc_vel,
      acc_att, acc_yaw;
  double min_cov_eig = std::numeric_limits<double>::infinity();
  // Ignore an initial settling window when scoring tracking and estimator accuracy.
  const double settle_time = std::min(20.0, 0.15 * cfg_.duration);

  ControlInput command = trim_.controls;
  control::AutopilotCommand guidance_cmd;
  Vec3 last_gyro = Vec3::Zero();
  double last_airspeed_meas = cfg_.initial.airspeed;
  double last_imu_time = 0.0;
  bool imu_seen = false;
  int last_wp_index = 1;
  int wp_switches = 0;
  long fevals = 0;
  int rejected = 0;

  double next_trajectory = 0.0;
  double time = 0.0;
  int step = 0;
  for (; step < total_steps; ++step) {
    time = static_cast<double>(step) * dt;

    const dynamics::AircraftState truth = dynamics::AircraftState::fromVec(x);
    const double altitude_agl = truth.altitude() - cfg_.safety.ground_altitude;

    // --- Environment ---------------------------------------------------------------
    const double density = atmosphere_.density(truth.altitude());
    dynamics::EnvironmentSample env;
    env.density = density;
    env.gravity = constants::kGravity;
    env.wind_ned = wind.totalNed(truth.quaternion, altitude_agl);

    // Diagnostics at the current state (needed by the sensors and the log).
    dynamics::DynamicsDiagnostics diag;
    model_.derivative(x, actuators_.state(), env, &diag);

    // --- Sensors and estimator -----------------------------------------------------
    const sensors::SensorBundle meas = sensor_suite.sample(time, dt, x, diag);
    if (meas.imu_valid) {
      const double imu_dt = imu_seen ? (meas.imu.time - last_imu_time) : (1.0 / cfg_.sensors.imu.rate);
      ekf.predict(meas.imu, imu_dt);
      last_imu_time = meas.imu.time;
      imu_seen = true;
      last_gyro = meas.imu.gyro;
      if (log_imu)
        log_imu->writeRow({meas.imu.time, meas.imu.gyro.x(), meas.imu.gyro.y(),
                           meas.imu.gyro.z(), meas.imu.accel.x(), meas.imu.accel.y(),
                           meas.imu.accel.z()});
    }
    if (meas.gps_valid) {
      ekf.updateGps(meas.gps);
      if (log_gps)
        log_gps->writeRow({meas.gps.time, meas.gps.position_ned.x(), meas.gps.position_ned.y(),
                           meas.gps.position_ned.z(), meas.gps.velocity_ned.x(),
                           meas.gps.velocity_ned.y(), meas.gps.velocity_ned.z()});
    }
    if (meas.baro_valid) {
      ekf.updateBaro(meas.baro);
      if (log_baro) log_baro->writeRow({meas.baro.time, meas.baro.altitude});
    }
    if (meas.mag_valid) {
      ekf.updateMagnetometer(meas.mag);
      if (log_mag)
        log_mag->writeRow({meas.mag.time, meas.mag.field_body.x(), meas.mag.field_body.y(),
                           meas.mag.field_body.z()});
    }
    if (meas.airspeed_valid) {
      ekf.updateAirspeed(meas.airspeed);
      last_airspeed_meas = meas.airspeed.airspeed;
      if (log_air) log_air->writeRow({meas.airspeed.time, meas.airspeed.airspeed});
    }
    min_cov_eig = std::min(min_cov_eig, ekf.minimumEigenvalue());

    // --- Guidance and control ------------------------------------------------------
    control::VehicleFeedback fb;
    if (cfg_.feedback == FeedbackSource::kTruth) {
      fb = truthFeedback(x, diag);
    } else {
      fb = estimateFeedback(ekf.state(), last_airspeed_meas);
      // Rate feedback comes straight from the (bias-corrected) gyro, as on a real vehicle.
      fb.rate = last_gyro - ekf.state().gyro_bias;
    }

    if (step % control_every == 0) {
      const double control_dt = control_every * dt;
      guidance_cmd = guidance_.update(fb.position_ned, fb.course);
      command = autopilot_->update(guidance_cmd, fb, control_dt);
      const int idx = guidance_.diagnostics().active_index;
      if (idx != last_wp_index) {
        ++wp_switches;
        last_wp_index = idx;
      }
    }

    // --- Actuators -----------------------------------------------------------------
    const ControlInput& surfaces = actuators_.update(command, dt);

    // --- Metrics -------------------------------------------------------------------
    const control::VehicleFeedback truth_fb = truthFeedback(x, diag);
    if (time >= settle_time) {
      acc_cross.add(guidance_.diagnostics().cross_track_error);
      acc_alt.add(truth_fb.altitude - guidance_cmd.altitude);
      acc_va.add(truth_fb.airspeed - guidance_cmd.airspeed);
      const Vec3 pos_err = ekf.state().position - truth.position;
      acc_pos.add(pos_err.norm());
      acc_posh.add(std::hypot(pos_err.x(), pos_err.y()));
      acc_altest.add(pos_err.z());
      acc_vel.add((ekf.state().velocity - truth_fb.velocity_ned).norm());
      acc_att.add(math::boxMinus(ekf.state().quaternion, truth.quaternion).norm());
      acc_yaw.add(math::wrapPi(ekf.state().euler().z() - truth_fb.yaw));
    }
    m.max_bank = std::max(m.max_bank, std::abs(truth_fb.roll));
    m.max_alpha = std::max(m.max_alpha, std::abs(truth_fb.alpha));

    // --- In-memory trajectory recording ---------------------------------------------
    if (trajectory_recorder_ && trajectory_interval_ > 0.0 && time + 1e-12 >= next_trajectory) {
      TrajectorySample sample;
      const Vec3 e = truth.euler();
      sample.time = time;
      sample.north = truth.position.x();
      sample.east = truth.position.y();
      sample.altitude = truth.altitude();
      sample.airspeed = diag.air.airspeed;
      sample.roll = e.x();
      sample.pitch = e.y();
      sample.yaw = e.z();
      sample.cross_track = guidance_.diagnostics().cross_track_error;
      sample.altitude_error = truth.altitude() - guidance_cmd.altitude;
      sample.estimator_position_error = (ekf.state().position - truth.position).norm();
      sample.estimator_attitude_error =
          math::boxMinus(ekf.state().quaternion, truth.quaternion).norm();
      trajectory_recorder_(sample);
      next_trajectory = time + trajectory_interval_;
    }

    // --- Logging -------------------------------------------------------------------
    if (log_states && (step % std::max(1, cfg_.logging.decimation) == 0)) {
      const Vec3 e = truth.euler();
      const Vec3 vned = truth_fb.velocity_ned;
      const estimation::NavState& nav = ekf.state();
      const Vec3 est_e = nav.euler();
      const VecX sig = ekf.sigma();
      const Vec3 pos_err = nav.position - truth.position;
      const Wrench total = model_.totalWrench(x, surfaces, env);
      const double thrust = model_.propulsion().thrust(surfaces.throttle, diag.air.airspeed,
                                                       env.density);
      std::vector<double> row = {
          time,
          truth.position.x(), truth.position.y(), truth.position.z(), truth.altitude(),
          truth.velocity.x(), truth.velocity.y(), truth.velocity.z(),
          vned.x(), vned.y(), vned.z(),
          truth.quaternion(0), truth.quaternion(1), truth.quaternion(2), truth.quaternion(3),
          e.x(), e.y(), e.z(), truth.rate.x(), truth.rate.y(), truth.rate.z(),
          diag.air.airspeed, diag.air.alpha, diag.air.beta, truth_fb.course,
          truth_fb.climb_rate,
          command.elevator, command.aileron, command.rudder, command.throttle,
          surfaces.elevator, surfaces.aileron, surfaces.rudder, surfaces.throttle,
          static_cast<double>(guidance_.diagnostics().active_index),
          static_cast<double>(guidance_.diagnostics().laps_completed),
          guidance_cmd.altitude, guidance_cmd.airspeed, guidance_cmd.course,
          guidance_.diagnostics().cross_track_error,
          guidance_.diagnostics().distance_to_waypoint,
          guidance_.diagnostics().orbiting ? 1.0 : 0.0,
          autopilot_->diagnostics().roll_command, autopilot_->diagnostics().pitch_command,
          autopilot_->diagnostics().altitude_command_filtered,
          autopilot_->diagnostics().course_error, autopilot_->diagnostics().altitude_error,
          autopilot_->diagnostics().airspeed_error,
          env.wind_ned.x(), env.wind_ned.y(), env.wind_ned.z(),
          wind.gustBody().x(), wind.gustBody().y(), wind.gustBody().z(), env.density,
          total.force.x(), total.force.y(), total.force.z(),
          total.moment.x(), total.moment.y(), total.moment.z(), thrust,
          nav.position.x(), nav.position.y(), nav.position.z(),
          nav.velocity.x(), nav.velocity.y(), nav.velocity.z(),
          est_e.x(), est_e.y(), est_e.z(),
          nav.gyro_bias.x(), nav.gyro_bias.y(), nav.gyro_bias.z(),
          nav.accel_bias.x(), nav.accel_bias.y(), nav.accel_bias.z(),
          sensor_suite.gyroBias().x(), sensor_suite.gyroBias().y(), sensor_suite.gyroBias().z(),
          sensor_suite.accelBias().x(), sensor_suite.accelBias().y(),
          sensor_suite.accelBias().z()};
      const Vec3 est_air_body = nav.airVelocityBody();
      row.push_back(nav.wind.x());
      row.push_back(nav.wind.y());
      row.push_back(nav.airspeed());
      row.push_back(std::atan2(est_air_body.z(),
                               std::max(est_air_body.x(), constants::kMinAirspeed)));
      row.push_back(std::asin(std::clamp(
          est_air_body.y() / std::max(est_air_body.norm(), constants::kMinAirspeed), -1.0,
          1.0)));
      row.push_back(nav.baro_bias);
      row.push_back(sensor_suite.baroBias());
      for (int i = 0; i < estimation::ErrorStateEkf::kStateDim; ++i) row.push_back(sig(i));
      row.push_back(pos_err.norm());
      row.push_back((nav.velocity - vned).norm());
      row.push_back(math::boxMinus(nav.quaternion, truth.quaternion).norm());
      log_states->writeRow(row);
    }

    // --- Safety checks -------------------------------------------------------------
    auto abort = [&](const char* reason) {
      m.termination_reason = reason;
      m.completed = false;
      m.stable = false;
    };
    if (truth.altitude() <= cfg_.safety.ground_altitude && cfg_.safety.stop_on_ground_contact) {
      abort("ground_contact");
    } else if (truth.altitude() > cfg_.safety.max_altitude) {
      abort("altitude_high");
    } else if (diag.air.airspeed < cfg_.safety.min_airspeed) {
      abort("airspeed_low");
    } else if (diag.air.airspeed > cfg_.safety.max_airspeed) {
      abort("airspeed_high");
    } else if (std::abs(truth_fb.roll) > cfg_.safety.max_bank) {
      abort("bank_exceeded");
    } else if (std::abs(truth_fb.pitch) > cfg_.safety.max_pitch) {
      abort("pitch_exceeded");
    } else if (!x.allFinite()) {
      abort("numerical_divergence");
    }
    if (!m.stable) break;

    // --- Advance the environment and the rigid-body state --------------------------
    wind.step(dt, diag.air.airspeed, altitude_agl);

    integrate::OdeFunction f = [&](double, const VecX& state) -> VecX {
      const StateVec xs = state;
      return model_.derivative(xs, surfaces, env);
    };
    integrate::IntegrationStats stats;
    const VecX xn = integrate::integrateTo(f, *integrator, time, x, time + dt, dt, &stats);
    x = xn;
    x.segment<4>(StateIndex::kQuatW) = math::quatNormalize(x.segment<4>(StateIndex::kQuatW));
    fevals += stats.function_evals;
    rejected += stats.rejected_steps;
  }

  const auto wall_end = Clock::now();
  const double wall =
      std::chrono::duration<double>(wall_end - wall_start).count();

  m.simulated_time = time;
  m.metrics_window_s = std::max(0.0, m.simulated_time - settle_time);
  m.rms_cross_track = acc_cross.rms();
  m.max_cross_track = acc_cross.peak;
  m.rms_altitude_error = acc_alt.rms();
  m.max_altitude_error = acc_alt.peak;
  m.rms_airspeed_error = acc_va.rms();
  m.max_airspeed_error = acc_va.peak;
  m.waypoints_reached = wp_switches;
  m.laps_completed = guidance_.diagnostics().laps_completed;
  m.rmse_position = acc_pos.rms();
  m.rmse_position_horizontal = acc_posh.rms();
  m.rmse_altitude = acc_altest.rms();
  m.rmse_velocity = acc_vel.rms();
  m.rmse_attitude = acc_att.rms();
  m.rmse_yaw = acc_yaw.rms();
  m.final_gyro_bias_error = (ekf.state().gyro_bias - sensor_suite.gyroBias()).norm();
  m.final_accel_bias_error = (ekf.state().accel_bias - sensor_suite.accelBias()).norm();
  m.min_covariance_eigenvalue = std::isfinite(min_cov_eig) ? min_cov_eig : 0.0;
  m.wall_clock_seconds = wall;
  m.dynamics_evaluations = fevals;
  m.steps = step;
  m.rejected_steps = rejected;
  m.real_time_factor = wall > 0.0 ? m.simulated_time / wall : 0.0;

  result.final_state = x;
  return result;
}

void writeMetricsJson(const std::string& path, const SimulationResult& result,
                      const ScenarioConfig& cfg) {
  std::filesystem::create_directories(std::filesystem::path(path).parent_path());
  std::ofstream f(path);
  if (!f) throw std::runtime_error("writeMetricsJson: cannot open " + path);
  f << std::setprecision(10);
  const auto& m = result.metrics;
  f << "{\n";
  f << "  \"scenario\": \"" << cfg.name << "\",\n";
  f << "  \"controller\": \"" << toString(cfg.controller) << "\",\n";
  f << "  \"feedback\": \"" << toString(cfg.feedback) << "\",\n";
  f << "  \"integrator\": \"" << cfg.integration.integrator << "\",\n";
  f << "  \"dt\": " << cfg.integration.dt << ",\n";
  f << "  \"seed\": " << cfg.seed << ",\n";
  f << "  \"simulated_time_s\": " << m.simulated_time << ",\n";
  f << "  \"completed\": " << (m.completed ? "true" : "false") << ",\n";
  f << "  \"stable\": " << (m.stable ? "true" : "false") << ",\n";
  f << "  \"termination_reason\": \"" << m.termination_reason << "\",\n";
  f << "  \"metrics_window_s\": " << m.metrics_window_s << ",\n";
  f << "  \"trim\": {\n";
  f << "    \"alpha_deg\": " << result.trim.alpha * constants::kRadToDeg << ",\n";
  f << "    \"theta_deg\": " << result.trim.theta * constants::kRadToDeg << ",\n";
  f << "    \"elevator_deg\": " << result.trim.controls.elevator * constants::kRadToDeg << ",\n";
  f << "    \"throttle\": " << result.trim.controls.throttle << ",\n";
  f << "    \"residual_inf_norm\": " << result.trim.residual_inf << "\n";
  f << "  },\n";
  f << "  \"tracking\": {\n";
  f << "    \"rms_cross_track_m\": " << m.rms_cross_track << ",\n";
  f << "    \"max_cross_track_m\": " << m.max_cross_track << ",\n";
  f << "    \"rms_altitude_error_m\": " << m.rms_altitude_error << ",\n";
  f << "    \"max_altitude_error_m\": " << m.max_altitude_error << ",\n";
  f << "    \"rms_airspeed_error_mps\": " << m.rms_airspeed_error << ",\n";
  f << "    \"max_airspeed_error_mps\": " << m.max_airspeed_error << ",\n";
  f << "    \"max_bank_deg\": " << m.max_bank * constants::kRadToDeg << ",\n";
  f << "    \"max_alpha_deg\": " << m.max_alpha * constants::kRadToDeg << ",\n";
  f << "    \"waypoints_reached\": " << m.waypoints_reached << ",\n";
  f << "    \"laps_completed\": " << m.laps_completed << "\n";
  f << "  },\n";
  f << "  \"estimator\": {\n";
  f << "    \"rmse_position_m\": " << m.rmse_position << ",\n";
  f << "    \"rmse_position_horizontal_m\": " << m.rmse_position_horizontal << ",\n";
  f << "    \"rmse_altitude_m\": " << m.rmse_altitude << ",\n";
  f << "    \"rmse_velocity_mps\": " << m.rmse_velocity << ",\n";
  f << "    \"rmse_attitude_deg\": " << m.rmse_attitude * constants::kRadToDeg << ",\n";
  f << "    \"rmse_yaw_deg\": " << m.rmse_yaw * constants::kRadToDeg << ",\n";
  f << "    \"final_gyro_bias_error_radps\": " << m.final_gyro_bias_error << ",\n";
  f << "    \"final_accel_bias_error_mps2\": " << m.final_accel_bias_error << ",\n";
  f << "    \"min_covariance_eigenvalue\": " << m.min_covariance_eigenvalue << "\n";
  f << "  },\n";
  f << "  \"performance\": {\n";
  f << "    \"wall_clock_s\": " << m.wall_clock_seconds << ",\n";
  f << "    \"real_time_factor\": " << m.real_time_factor << ",\n";
  f << "    \"dynamics_evaluations\": " << m.dynamics_evaluations << ",\n";
  f << "    \"steps\": " << m.steps << ",\n";
  f << "    \"rejected_steps\": " << m.rejected_steps << "\n";
  f << "  }\n";
  f << "}\n";
}

}  // namespace aether::sim
