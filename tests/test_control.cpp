/// \file test_control.cpp
/// \brief PID behaviour and anti-windup, LQR synthesis, guidance logic and a closed-loop
///        regression on the nominal scenario.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "aether/analysis/Linearize.hpp"
#include "aether/analysis/Trim.hpp"
#include "aether/control/LqrAutopilot.hpp"
#include "aether/control/Pid.hpp"
#include "aether/control/PidAutopilot.hpp"
#include "aether/core/Constants.hpp"
#include "aether/env/Atmosphere.hpp"
#include "aether/guidance/WaypointFollower.hpp"
#include "aether/math/Rotation.hpp"
#include "aether/sim/Config.hpp"
#include "aether/sim/Simulator.hpp"

using namespace aether;
using Catch::Approx;

TEST_CASE("PID reproduces the parallel control law", "[control][pid]") {
  control::PidConfig cfg;
  cfg.kp = 2.0;
  cfg.ki = 0.0;
  cfg.kd = 0.0;
  control::Pid pid(cfg);
  REQUIRE(pid.update(1.0, 0.0, 0.01) == Approx(2.0));
  REQUIRE(pid.update(1.0, 0.5, 0.01) == Approx(1.0));
  REQUIRE(pid.update(0.0, 1.0, 0.01) == Approx(-2.0));

  cfg.kp = 0.0;
  cfg.ki = 1.0;
  control::Pid integrator(cfg);
  double last = 0.0;
  for (int i = 0; i < 100; ++i) last = integrator.updateWithError(1.0, 0.01);
  // The integrator is advanced *after* the output is formed (the usual discrete PID
  // convention), so after N steps the output carries N-1 increments and the internal state
  // carries N.
  REQUIRE(integrator.integrator() == Approx(1.0).epsilon(1e-9));  // integral of 1 over 1 s
  REQUIRE(last == Approx(0.99).epsilon(1e-9));
}

TEST_CASE("PID output saturation and anti-windup", "[control][pid][antiwindup]") {
  control::PidConfig cfg;
  cfg.kp = 1.0;
  cfg.ki = 5.0;
  cfg.kd = 0.0;
  cfg.output_min = -1.0;
  cfg.output_max = 1.0;
  cfg.anti_windup_gain = 2.0;

  control::Pid pid(cfg);
  // Drive hard into the positive saturation for a long time.
  for (int i = 0; i < 1000; ++i) pid.updateWithError(10.0, 0.01);
  REQUIRE(pid.output() == Approx(1.0));
  REQUIRE(pid.saturated());
  // Without anti-windup the integrator would be at 5*10*10 = 500. It must stay bounded.
  REQUIRE(std::abs(pid.integrator()) < 10.0);

  // On sign reversal the output must leave saturation almost immediately.
  int steps_to_recover = 0;
  for (int i = 0; i < 1000; ++i) {
    const double u = pid.updateWithError(-10.0, 0.01);
    ++steps_to_recover;
    if (u < 0.99) break;
  }
  REQUIRE(steps_to_recover < 20);

  // Conditional integration alone already bounds the integrator: without it the value after
  // 10 s of a constant error of 10 with ki = 5 would be 500.
  control::PidConfig conditional_only = cfg;
  conditional_only.anti_windup_gain = 0.0;
  control::Pid conditional(conditional_only);
  for (int i = 0; i < 1000; ++i) conditional.updateWithError(10.0, 0.01);
  REQUIRE(std::abs(conditional.integrator()) < 10.0);

  // Back-calculation does something conditional integration cannot: it actively unwinds an
  // integrator that is already past the achievable output (for example after a gain change
  // or a hand-over). Seed both with a large integrator and compare.
  control::Pid with_bc(cfg);
  control::Pid without_bc(conditional_only);
  with_bc.setIntegrator(50.0);
  without_bc.setIntegrator(50.0);
  for (int i = 0; i < 100; ++i) {  // 1 s of zero error while saturated
    with_bc.updateWithError(0.0, 0.01);
    without_bc.updateWithError(0.0, 0.01);
  }
  REQUIRE(without_bc.integrator() == Approx(50.0));   // frozen, still wound up
  REQUIRE(with_bc.integrator() < 50.0);               // actively bled back
  // Back-calculation gives dI/dt = k_aw * (u_sat - u_raw) = k_aw * (u_max - I), i.e. a
  // first-order decay towards the achievable output.
  const double expected =
      cfg.output_max + (50.0 - cfg.output_max) * std::exp(-cfg.anti_windup_gain * 1.0);
  REQUIRE(with_bc.integrator() == Approx(expected).epsilon(0.05));
}

TEST_CASE("PID derivative acts on the measurement and can be supplied externally",
          "[control][pid]") {
  control::PidConfig cfg;
  cfg.kp = 0.0;
  cfg.ki = 0.0;
  cfg.kd = 3.0;
  cfg.derivative_filter_tau = 0.0;
  control::Pid pid(cfg);

  // Externally supplied rate: d(error)/dt = -d(measurement)/dt.
  pid.update(0.0, 0.0, 0.01, 2.0);
  REQUIRE(pid.output() == Approx(-6.0));

  // A setpoint step must not produce a derivative kick when acting on the measurement.
  control::Pid kickless(cfg);
  kickless.update(0.0, 0.0, 0.01);
  const double u = kickless.update(100.0, 0.0, 0.01);
  REQUIRE(u == Approx(0.0).margin(1e-12));
}

TEST_CASE("PID reset clears all internal state", "[control][pid]") {
  control::PidConfig cfg;
  cfg.kp = 1.0;
  cfg.ki = 1.0;
  control::Pid pid(cfg);
  for (int i = 0; i < 100; ++i) pid.updateWithError(1.0, 0.01);
  REQUIRE(pid.integrator() != Approx(0.0));
  pid.reset();
  REQUIRE(pid.integrator() == Approx(0.0));
  REQUIRE(pid.output() == Approx(0.0));
  pid.setIntegrator(0.25);
  REQUIRE(pid.integrator() == Approx(0.25));
}

namespace {

struct DesignFixture {
  dynamics::AircraftParameters params = dynamics::defaultAircraft();
  dynamics::RigidBody6DOF model{params};
  analysis::TrimResult trim;
  analysis::LinearModel linear;

  DesignFixture() {
    analysis::TrimSpec spec;
    spec.airspeed = 25.0;
    spec.altitude = 120.0;
    spec.density_override = env::Atmosphere().density(spec.altitude);
    trim = analysis::TrimSolver(model).solve(spec);
    dynamics::EnvironmentSample env;
    env.density = trim.density;
    linear = analysis::linearize(model, trim.state, trim.controls, env);
  }
};

}  // namespace

TEST_CASE("LQR synthesis produces a stabilising and well-conditioned design", "[control][lqr]") {
  DesignFixture f;
  REQUIRE(f.trim.converged);

  control::LqrWeights w;
  control::LqrAutopilotLimits limits;
  control::LqrAutopilot ap(f.trim, f.linear, w, limits, f.params);

  REQUIRE(ap.name() == "lqr");
  REQUIRE(ap.longitudinalGain().rows() == 2);
  REQUIRE(ap.longitudinalGain().cols() == 7);
  REQUIRE(ap.lateralGain().rows() == 2);
  REQUIRE(ap.lateralGain().cols() == 6);
  REQUIRE(ap.longitudinalGain().allFinite());
  REQUIRE(ap.lateralGain().allFinite());

  // The Riccati equations were solved accurately.
  REQUIRE(ap.careResiduals()(0) < 1e-8);
  REQUIRE(ap.careResiduals()(1) < 1e-8);

  // Both augmented closed loops are asymptotically stable.
  for (Eigen::Index i = 0; i < ap.longitudinalClosedLoop().size(); ++i)
    REQUIRE(ap.longitudinalClosedLoop()(i).real() < 0.0);
  for (Eigen::Index i = 0; i < ap.lateralClosedLoop().size(); ++i)
    REQUIRE(ap.lateralClosedLoop()(i).real() < 0.0);

  // The gain-derived limits must be tighter than the configured fallbacks.
  REQUIRE(ap.effectiveLimits().course_error_limit < limits.course_error_limit);
  REQUIRE(ap.effectiveLimits().course_error_limit > 0.0);
  REQUIRE(ap.effectiveLimits().altitude_error_limit < limits.altitude_error_limit);
}

TEST_CASE("LQR command limiting keeps the steady bank demand within the limit",
          "[control][lqr][limits]") {
  DesignFixture f;
  control::LqrWeights w;
  control::LqrAutopilotLimits limits;
  limits.max_bank = 35.0 * constants::kDegToRad;
  control::LqrAutopilot ap(f.trim, f.linear, w, limits, f.params);

  // The bank at which the aileron demand from the (saturated) course terms is cancelled by
  // the bank feedback must not exceed max_bank.
  const double k_phi = std::abs(ap.lateralGain()(0, 3));
  const double k_chi = std::abs(ap.lateralGain()(0, 4));
  const double k_int = std::abs(ap.lateralGain()(0, 5));
  const double equilibrium_bank =
      (k_chi * ap.effectiveLimits().course_error_limit +
       k_int * ap.effectiveLimits().integrator_course_limit) / k_phi;
  REQUIRE(equilibrium_bank <= limits.max_bank * 1.001);
}

TEST_CASE("LQR and PID autopilots hold trim when the vehicle is already on condition",
          "[control][autopilot]") {
  DesignFixture f;
  const dynamics::AircraftState s = dynamics::AircraftState::fromVec(f.trim.state);
  const Vec3 e = s.euler();

  control::VehicleFeedback fb;
  fb.position_ned = s.position;
  fb.velocity_body = s.velocity;
  fb.velocity_air_body = s.velocity;
  fb.velocity_ned = math::rotateBodyToNed(s.quaternion, s.velocity);
  fb.altitude = s.altitude();
  fb.airspeed = s.velocity.norm();
  fb.roll = e.x();
  fb.pitch = e.y();
  fb.yaw = e.z();
  fb.course = 0.0;
  fb.rate = s.rate;
  fb.climb_rate = 0.0;

  control::AutopilotCommand cmd;
  cmd.altitude = fb.altitude;
  cmd.airspeed = fb.airspeed;
  cmd.course = 0.0;

  for (const std::string kind : {"lqr", "pid"}) {
    std::unique_ptr<control::Autopilot> ap;
    if (kind == "lqr")
      ap = std::make_unique<control::LqrAutopilot>(f.trim, f.linear, control::LqrWeights{},
                                                   control::LqrAutopilotLimits{}, f.params);
    else
      ap = std::make_unique<control::PidAutopilot>(
          control::PidAutopilot::defaultConfig(f.params), f.params);
    control::TrimReference ref;
    ref.controls = f.trim.controls;
    ref.roll = f.trim.phi;
    ref.pitch = f.trim.theta;
    ref.airspeed = 25.0;
    ap->reset(ref);
    ControlInput u;
    for (int i = 0; i < 200; ++i) u = ap->update(cmd, fb, 0.01);
    INFO("controller " << kind);
    REQUIRE(u.elevator == Approx(f.trim.controls.elevator).margin(2e-3));
    REQUIRE(u.throttle == Approx(f.trim.controls.throttle).margin(2e-2));
    REQUIRE(std::abs(u.aileron) < 0.05);
    REQUIRE(std::abs(u.rudder) < 0.05);
  }
}

TEST_CASE("autopilot commands respect the actuator limits", "[control][autopilot]") {
  DesignFixture f;
  control::PidAutopilot pid(control::PidAutopilot::defaultConfig(f.params), f.params);
  control::TrimReference ref;
  ref.controls = f.trim.controls;
  ref.pitch = f.trim.theta;
  pid.reset(ref);

  control::VehicleFeedback fb;
  fb.airspeed = 25.0;
  fb.altitude = 100.0;
  fb.velocity_air_body = Vec3(25, 0, 0);
  fb.velocity_body = fb.velocity_air_body;

  control::AutopilotCommand cmd;
  cmd.altitude = 5000.0;   // absurd demands in every channel
  cmd.airspeed = 200.0;
  cmd.course = constants::kPi;

  for (int i = 0; i < 2000; ++i) {
    const ControlInput u = pid.update(cmd, fb, 0.01);
    REQUIRE(u.elevator >= f.params.actuators.elevator_min - 1e-12);
    REQUIRE(u.elevator <= f.params.actuators.elevator_max + 1e-12);
    REQUIRE(u.aileron >= f.params.actuators.aileron_min - 1e-12);
    REQUIRE(u.aileron <= f.params.actuators.aileron_max + 1e-12);
    REQUIRE(u.rudder >= f.params.actuators.rudder_min - 1e-12);
    REQUIRE(u.rudder <= f.params.actuators.rudder_max + 1e-12);
    REQUIRE(u.throttle >= 0.0);
    REQUIRE(u.throttle <= 1.0);
  }
}

TEST_CASE("waypoint guidance produces the expected vector field", "[guidance]") {
  guidance::GuidanceConfig cfg;
  cfg.waypoints = {{0.0, 0.0, 100.0, 25.0}, {1000.0, 0.0, 120.0, 25.0}};
  cfg.terminal = guidance::TerminalBehaviour::kHold;
  cfg.path_gain = 0.05;
  cfg.chi_infinity = constants::kPi / 3;
  guidance::WaypointFollower g(cfg);

  // On the path: fly the path course (due north).
  auto cmd = g.update(Vec3(200.0, 0.0, -110.0), 0.0);
  REQUIRE(cmd.course == Approx(0.0).margin(1e-9));
  REQUIRE(g.diagnostics().cross_track_error == Approx(0.0).margin(1e-9));

  // To the right of the path (positive east): steer left (negative course).
  cmd = g.update(Vec3(200.0, 50.0, -110.0), 0.0);
  REQUIRE(g.diagnostics().cross_track_error == Approx(50.0).margin(1e-9));
  REQUIRE(cmd.course < 0.0);
  REQUIRE(cmd.course > -cfg.chi_infinity);

  // Far off the path the command saturates at chi_infinity.
  cmd = g.update(Vec3(200.0, 5000.0, -110.0), 0.0);
  REQUIRE(cmd.course == Approx(-cfg.chi_infinity).margin(0.02));

  // Altitude is interpolated along the leg.
  cmd = g.update(Vec3(500.0, 0.0, -110.0), 0.0);
  REQUIRE(cmd.altitude == Approx(110.0).margin(1e-9));
  cmd = g.update(Vec3(0.0, 0.0, -100.0), 0.0);
  REQUIRE(cmd.altitude == Approx(100.0).margin(1e-9));
}

TEST_CASE("half-plane switching advances the active waypoint and closes the circuit",
          "[guidance][switching]") {
  guidance::GuidanceConfig cfg;
  cfg.waypoints = {{0.0, 0.0, 100.0, 25.0},
                   {500.0, 0.0, 100.0, 25.0},
                   {500.0, 500.0, 100.0, 25.0},
                   {0.0, 500.0, 100.0, 25.0}};
  cfg.terminal = guidance::TerminalBehaviour::kLoop;
  cfg.capture_radius = 1.0;  // force the half-plane rule to do the work
  guidance::WaypointFollower g(cfg);

  REQUIRE(g.diagnostics().active_index == 1);
  g.update(Vec3(100.0, 0.0, -100.0), 0.0);
  REQUIRE(g.diagnostics().active_index == 1);

  // Crossing the bisector half-plane at waypoint 1 advances to waypoint 2 immediately,
  // i.e. the command returned by this very call already belongs to the new leg.
  const auto after_switch = g.update(Vec3(520.0, 10.0, -100.0), 0.0);
  REQUIRE(g.diagnostics().active_index == 2);
  REQUIRE(std::abs(math::wrapPi(after_switch.course - constants::kPi / 2)) < 0.8);

  g.update(Vec3(510.0, 520.0, -100.0), 0.0);
  REQUIRE(g.diagnostics().active_index == 3);

  // Passing the last waypoint wraps to index 0, i.e. the closing leg 3 -> 0.
  g.update(Vec3(-20.0, 510.0, -100.0), 0.0);
  REQUIRE(g.diagnostics().active_index == 0);
  REQUIRE(g.diagnostics().laps_completed == 1);
  // The closing leg runs from waypoint 3 (0, 500) to waypoint 0 (0, 0), i.e. due west.
  // On the path the commanded course is exactly the path course.
  const auto cmd = g.update(Vec3(0.0, 300.0, -100.0), 0.0);
  REQUIRE(g.diagnostics().cross_track_error == Approx(0.0).margin(1e-9));
  REQUIRE(cmd.course == Approx(-constants::kPi / 2).margin(1e-9));
  // Left of that westward path (negative cross-track), the command steers back to the right.
  const auto off_path = g.update(Vec3(-80.0, 300.0, -100.0), 0.0);
  REQUIRE(g.diagnostics().cross_track_error < 0.0);
  REQUIRE(off_path.course > -constants::kPi / 2);

  // ... and then back to leg 0 -> 1.
  g.update(Vec3(20.0, 10.0, -100.0), 0.0);
  REQUIRE(g.diagnostics().laps_completed == 1);
  REQUIRE(g.diagnostics().active_index == 1);

  g.reset();
  REQUIRE(g.diagnostics().active_index == 1);
  REQUIRE(g.diagnostics().laps_completed == 0);
}

TEST_CASE("terminal orbit mode converges onto the loiter circle", "[guidance][orbit]") {
  guidance::GuidanceConfig cfg;
  cfg.waypoints = {{0.0, 0.0, 100.0, 25.0}, {200.0, 0.0, 100.0, 25.0}};
  cfg.terminal = guidance::TerminalBehaviour::kOrbit;
  cfg.orbit_radius = 100.0;
  cfg.capture_radius = 30.0;
  guidance::WaypointFollower g(cfg);

  g.update(Vec3(190.0, 0.0, -100.0), 0.0);  // inside the capture radius -> start orbiting
  const auto cmd = g.update(Vec3(300.0, 0.0, -100.0), 0.0);
  REQUIRE(g.diagnostics().orbiting);
  // Exactly on the circle due north of the centre, a clockwise orbit commands due east.
  REQUIRE(cmd.course == Approx(constants::kPi / 2).margin(0.02));
  REQUIRE(g.diagnostics().cross_track_error == Approx(0.0).margin(1e-9));

  // Outside the circle, the command turns inwards.
  const auto outside = g.update(Vec3(400.0, 0.0, -100.0), 0.0);
  REQUIRE(outside.course > constants::kPi / 2);
}

TEST_CASE("guidance rejects a degenerate mission", "[guidance][api]") {
  guidance::GuidanceConfig cfg;
  cfg.waypoints = {{0.0, 0.0, 100.0, 25.0}};
  REQUIRE_THROWS_AS(guidance::WaypointFollower(cfg), std::invalid_argument);
  REQUIRE(guidance::parseTerminalBehaviour("loop") == guidance::TerminalBehaviour::kLoop);
  REQUIRE(guidance::parseTerminalBehaviour("orbit") == guidance::TerminalBehaviour::kOrbit);
  REQUIRE(guidance::parseTerminalBehaviour("hold") == guidance::TerminalBehaviour::kHold);
  REQUIRE_THROWS_AS(guidance::parseTerminalBehaviour("spiral"), std::invalid_argument);
}

TEST_CASE("closed-loop regression: both controllers fly the nominal course",
          "[control][regression][slow]") {
  const std::string scenario = std::string(AETHER_CONFIG_DIR) + "/scenarios/ideal.yaml";
  sim::ScenarioConfig cfg = sim::loadScenario(scenario);
  cfg.logging.enabled = false;
  cfg.duration = 200.0;
  const auto aircraft = sim::loadAircraft(cfg.aircraft_file);

  for (const auto controller : {sim::ControllerType::kLqr, sim::ControllerType::kPid}) {
    sim::ScenarioConfig c = cfg;
    c.controller = controller;
    sim::Simulator simulator(c, aircraft);
    const sim::SimulationResult r = simulator.run();
    INFO("controller " << sim::toString(controller) << ": " << r.metrics.termination_reason);

    REQUIRE(r.metrics.completed);
    REQUIRE(r.metrics.stable);
    REQUIRE(r.metrics.simulated_time == Approx(c.duration).margin(c.integration.dt * 2));
    // The vehicle must actually follow the course, not just stay airborne.
    REQUIRE(r.metrics.waypoints_reached >= 5);
    REQUIRE(r.metrics.rms_cross_track < 30.0);
    REQUIRE(r.metrics.max_cross_track < 90.0);
    REQUIRE(r.metrics.rms_altitude_error < 8.0);
    REQUIRE(r.metrics.rms_airspeed_error < 2.0);
    // Attitude stays in the normal flight envelope.
    REQUIRE(r.metrics.max_bank < 50.0 * constants::kDegToRad);
    REQUIRE(r.metrics.max_alpha < 20.0 * constants::kDegToRad);
    REQUIRE(r.final_state.allFinite());
  }
}

TEST_CASE("closed-loop regression: the estimator drives the loop in wind",
          "[control][regression][slow]") {
  const std::string scenario = std::string(AETHER_CONFIG_DIR) + "/scenarios/nominal.yaml";
  sim::ScenarioConfig cfg = sim::loadScenario(scenario);
  cfg.logging.enabled = false;
  cfg.duration = 200.0;
  const auto aircraft = sim::loadAircraft(cfg.aircraft_file);

  sim::Simulator simulator(cfg, aircraft);
  const sim::SimulationResult r = simulator.run();
  INFO(r.metrics.termination_reason);

  REQUIRE(cfg.feedback == sim::FeedbackSource::kEstimate);
  REQUIRE(r.metrics.completed);
  REQUIRE(r.metrics.stable);
  REQUIRE(r.metrics.waypoints_reached >= 5);
  REQUIRE(r.metrics.rms_cross_track < 40.0);
  REQUIRE(r.metrics.rms_altitude_error < 10.0);
  // Estimator accuracy while flying closed loop on its own output.
  REQUIRE(r.metrics.rmse_position < 10.0);
  REQUIRE(r.metrics.rmse_velocity < 1.0);
  REQUIRE(r.metrics.rmse_attitude < 5.0 * constants::kDegToRad);
  REQUIRE(r.metrics.min_covariance_eigenvalue > 0.0);
}

TEST_CASE("the simulation is bit-for-bit reproducible for a fixed seed",
          "[control][determinism]") {
  const std::string scenario = std::string(AETHER_CONFIG_DIR) + "/scenarios/nominal.yaml";
  sim::ScenarioConfig cfg = sim::loadScenario(scenario);
  cfg.logging.enabled = false;
  cfg.duration = 40.0;
  const auto aircraft = sim::loadAircraft(cfg.aircraft_file);

  sim::Simulator a(cfg, aircraft), b(cfg, aircraft);
  const auto ra = a.run();
  const auto rb = b.run();
  REQUIRE((ra.final_state - rb.final_state).norm() == Approx(0.0));
  REQUIRE(ra.metrics.rms_cross_track == Approx(rb.metrics.rms_cross_track));
  REQUIRE(ra.metrics.rmse_position == Approx(rb.metrics.rmse_position));

  sim::ScenarioConfig other = cfg;
  other.seed = cfg.seed + 1;
  sim::Simulator c(other, aircraft);
  const auto rc = c.run();
  REQUIRE((ra.final_state - rc.final_state).norm() > 1e-9);
}

TEST_CASE("angle-of-attack envelope protection", "[control][protection]") {
  control::EnvelopeProtection p;
  p.enabled = true;
  p.alpha_max = 0.24;
  p.alpha_gain = 2.5;
  p.rate_gain = 0.35;
  p.blend_width = 0.09;

  SECTION("inactive below the limit") {
    bool active = true;
    REQUIRE(control::applyEnvelopeProtection(-0.3, 0.10, 0.0, p, &active) == Approx(-0.3));
    REQUIRE_FALSE(active);
    REQUIRE(control::envelopeProtectionAuthority(0.10, p) == Approx(0.0));
    REQUIRE(control::envelopeProtectionAuthority(0.24, p) == Approx(0.0));
  }

  SECTION("authority ramps linearly over the blend width") {
    REQUIRE(control::envelopeProtectionAuthority(0.24 + 0.045, p) == Approx(0.5));
    REQUIRE(control::envelopeProtectionAuthority(0.24 + 0.09, p) == Approx(1.0));
    REQUIRE(control::envelopeProtectionAuthority(0.24 + 0.5, p) == Approx(1.0));
  }

  SECTION("pushes nose-down above the limit and is continuous at the threshold") {
    bool active = false;
    // Just above the threshold the blend weight is ~0, so the command is barely changed.
    const double just_above = control::applyEnvelopeProtection(-0.3, 0.2401, 0.0, p, &active);
    REQUIRE(just_above == Approx(-0.3).margin(1e-3));
    // Far above it, the protective demand takes over (positive elevator = nose down).
    const double deep = control::applyEnvelopeProtection(-0.3, 0.45, 0.0, p, &active);
    REQUIRE(active);
    REQUIRE(deep > 0.0);
    REQUIRE(deep == Approx(p.alpha_gain * (0.45 - p.alpha_max)).epsilon(1e-9));
  }

  SECTION("pitch-rate damping is included") {
    const double no_rate = control::applyEnvelopeProtection(-0.4, 0.40, 0.0, p);
    const double nose_up_rate = control::applyEnvelopeProtection(-0.4, 0.40, 0.5, p);
    REQUIRE(nose_up_rate > no_rate);  // a nose-up rate demands more nose-down protection
  }

  SECTION("never reduces a stronger nose-down command from the control law") {
    // The protective demand here is 2.5*(0.45-0.24) = 0.525 rad, so a command of 0.6 rad
    // already exceeds it and must pass through untouched, while 0.4 rad is raised to 0.525.
    bool active = false;
    REQUIRE(control::applyEnvelopeProtection(0.6, 0.45, 0.0, p, &active) == Approx(0.6));
    REQUIRE_FALSE(active);
    REQUIRE(control::applyEnvelopeProtection(0.4, 0.45, 0.0, p, &active) == Approx(0.525));
    REQUIRE(active);
  }

  SECTION("can be disabled") {
    control::EnvelopeProtection off = p;
    off.enabled = false;
    bool active = true;
    REQUIRE(control::applyEnvelopeProtection(-0.4, 1.0, 0.0, off, &active) == Approx(-0.4));
    REQUIRE_FALSE(active);
    REQUIRE(control::envelopeProtectionAuthority(1.0, off) == Approx(0.0));
  }
}

TEST_CASE("envelope protection stops a commanded stall", "[control][protection][slow]") {
  // A 30% overweight airframe asked to fly the circuit at 23 m/s has almost no stall margin
  // in the 35 deg turns. Without protection the altitude loop trades the last of it for
  // pitch attitude and the aircraft departs; with protection it widens the turns instead.
  const std::string scenario = std::string(AETHER_CONFIG_DIR) + "/scenarios/ideal.yaml";
  sim::ScenarioConfig cfg = sim::loadScenario(scenario);
  cfg.logging.enabled = false;
  cfg.duration = 120.0;
  cfg.initial.airspeed = 23.0;
  for (auto& wp : cfg.guidance.waypoints) wp.airspeed = 23.0;

  auto aircraft = sim::loadAircraft(cfg.aircraft_file);
  aircraft.mass *= 1.30;

  sim::ScenarioConfig protected_cfg = cfg;
  protected_cfg.lqr_limits.protection.enabled = true;
  sim::ScenarioConfig unprotected_cfg = cfg;
  unprotected_cfg.lqr_limits.protection.enabled = false;

  const auto guarded = sim::Simulator(protected_cfg, aircraft).run();
  const auto bare = sim::Simulator(unprotected_cfg, aircraft).run();

  INFO("protected max alpha " << guarded.metrics.max_alpha * constants::kRadToDeg
                              << " deg (" << guarded.metrics.termination_reason
                              << "), unprotected " << bare.metrics.max_alpha * constants::kRadToDeg
                              << " deg (" << bare.metrics.termination_reason << ")");
  // With protection the aircraft stays inside the envelope and completes the mission.
  REQUIRE(guarded.metrics.stable);
  REQUIRE(guarded.metrics.completed);
  REQUIRE(guarded.metrics.max_alpha < aircraft.lon.alpha_stall);
  // Without it, it is driven well past the stall and is lost.
  REQUIRE(bare.metrics.max_alpha > aircraft.lon.alpha_stall);
  REQUIRE_FALSE(bare.metrics.stable);
  REQUIRE(bare.metrics.max_alpha > guarded.metrics.max_alpha + 0.15);
}
