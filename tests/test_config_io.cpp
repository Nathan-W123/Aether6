/// \file test_config_io.cpp
/// \brief YAML configuration loading, CSV output and the shipped configuration files.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "aether/core/Constants.hpp"
#include "aether/sim/Config.hpp"
#include "aether/util/Csv.hpp"
#include "aether/util/Random.hpp"

using namespace aether;
using Catch::Approx;

namespace {

std::string configDir() { return std::string(AETHER_CONFIG_DIR); }

/// Write a temporary YAML file and return its path.
std::string writeTemp(const std::string& name, const std::string& content) {
  const auto dir = std::filesystem::temp_directory_path() / "aether_tests";
  std::filesystem::create_directories(dir);
  const auto path = dir / name;
  std::ofstream f(path);
  f << content;
  f.close();
  return path.string();
}

}  // namespace

TEST_CASE("the shipped airframe file loads and matches the built-in defaults",
          "[config][aircraft]") {
  const auto p = sim::loadAircraft(configDir() + "/aircraft/aether6_uav.yaml");
  const auto d = dynamics::defaultAircraft();

  REQUIRE(p.synthetic);
  REQUIRE(p.mass == Approx(d.mass));
  REQUIRE(p.Jx == Approx(d.Jx));
  REQUIRE(p.Jy == Approx(d.Jy));
  REQUIRE(p.Jz == Approx(d.Jz));
  REQUIRE(p.Jxz == Approx(d.Jxz));
  REQUIRE(p.wing_area == Approx(d.wing_area));
  REQUIRE(p.wing_span == Approx(d.wing_span));
  REQUIRE(p.mean_chord == Approx(d.mean_chord));
  REQUIRE(p.lon.CL_alpha == Approx(d.lon.CL_alpha));
  REQUIRE(p.lon.Cm_alpha == Approx(d.lon.Cm_alpha));
  REQUIRE(p.lon.Cm_de == Approx(d.lon.Cm_de));
  REQUIRE(p.lat.Cl_da == Approx(d.lat.Cl_da));
  REQUIRE(p.lat.Cn_dr == Approx(d.lat.Cn_dr));
  REQUIRE(p.prop.k_motor == Approx(d.prop.k_motor));
  // Angles in the YAML are degrees and must arrive in radians.
  REQUIRE(p.actuators.elevator_max == Approx(25.0 * constants::kDegToRad));
  REQUIRE(p.actuators.aileron_min == Approx(-20.0 * constants::kDegToRad));
  REQUIRE(p.actuators.surface_rate_limit == Approx(300.0 * constants::kDegToRad));
  REQUIRE(p.lon.alpha_stall == Approx(17.2 * constants::kDegToRad));
  REQUIRE_NOTHROW(p.validate());
}

TEST_CASE("every shipped scenario loads and is self-consistent", "[config][scenario]") {
  for (const auto& entry : std::filesystem::directory_iterator(configDir() + "/scenarios")) {
    if (entry.path().extension() != ".yaml") continue;
    INFO("scenario file " << entry.path().string());
    const auto cfg = sim::loadScenario(entry.path().string());
    REQUIRE_FALSE(cfg.name.empty());
    REQUIRE(cfg.duration > 0.0);
    REQUIRE(cfg.integration.dt > 0.0);
    REQUIRE(cfg.control_rate > 0.0);
    REQUIRE(cfg.guidance.waypoints.size() >= 2);
    REQUIRE(std::filesystem::exists(cfg.aircraft_file));
    REQUIRE_NOTHROW(sim::loadAircraft(cfg.aircraft_file));
    // The control rate must be an achievable multiple of the integration step.
    REQUIRE(1.0 / cfg.control_rate >= cfg.integration.dt);
  }
}

TEST_CASE("the nominal scenario has the documented settings", "[config][scenario]") {
  const auto cfg = sim::loadScenario(configDir() + "/scenarios/nominal.yaml");
  REQUIRE(cfg.name == "nominal");
  REQUIRE(cfg.controller == sim::ControllerType::kLqr);
  REQUIRE(cfg.feedback == sim::FeedbackSource::kEstimate);
  REQUIRE(cfg.integration.integrator == "rk4");
  REQUIRE(cfg.guidance.terminal == guidance::TerminalBehaviour::kLoop);
  REQUIRE(cfg.guidance.waypoints.size() == 6);
  REQUIRE(cfg.guidance.chi_infinity == Approx(60.0 * constants::kDegToRad));
  REQUIRE(cfg.wind.enable_turbulence);
  REQUIRE(cfg.wind.steady_ned.norm() > 0.0);
  REQUIRE(cfg.sensors.imu.rate == Approx(200.0));
  REQUIRE(cfg.sensors.gps.rate == Approx(5.0));
  REQUIRE(cfg.estimator.use_airspeed);
  REQUIRE(cfg.lqr_limits.max_bank == Approx(35.0 * constants::kDegToRad));

  const auto pid_cfg = sim::loadScenario(configDir() + "/scenarios/nominal_pid.yaml");
  REQUIRE(pid_cfg.controller == sim::ControllerType::kPid);
  REQUIRE(pid_cfg.pid_config_explicit);
  REQUIRE(pid_cfg.pid_config.pitch_to_elevator.kp < 0.0);
  REQUIRE(pid_cfg.pid_config.sideslip_to_rudder.kp < 0.0);

  const auto ideal = sim::loadScenario(configDir() + "/scenarios/ideal.yaml");
  REQUIRE(ideal.feedback == sim::FeedbackSource::kTruth);
  REQUIRE_FALSE(ideal.wind.enable_turbulence);
}

TEST_CASE("absent keys keep their C++ defaults", "[config][scenario]") {
  const std::string yaml = R"(
name: minimal
aircraft: )" + configDir() + R"(/aircraft/aether6_uav.yaml
guidance:
  waypoints:
    - {north: 0.0, east: 0.0, altitude: 100.0}
    - {north: 500.0, east: 0.0, altitude: 100.0}
)";
  const auto cfg = sim::loadScenario(writeTemp("minimal.yaml", yaml));
  const sim::ScenarioConfig defaults;
  REQUIRE(cfg.name == "minimal");
  REQUIRE(cfg.duration == Approx(defaults.duration));
  REQUIRE(cfg.control_rate == Approx(defaults.control_rate));
  REQUIRE(cfg.integration.dt == Approx(defaults.integration.dt));
  REQUIRE(cfg.sensors.imu.rate == Approx(defaults.sensors.imu.rate));
  REQUIRE(cfg.estimator.gps_position_ne == Approx(defaults.estimator.gps_position_ne));
  REQUIRE_FALSE(cfg.pid_config_explicit);
  // Waypoint airspeed defaults to the scenario's initial airspeed.
  REQUIRE(cfg.guidance.waypoints[0].airspeed == Approx(cfg.initial.airspeed));
}

TEST_CASE("malformed configuration is rejected with a clear error", "[config][errors]") {
  REQUIRE_THROWS_AS(sim::loadScenario("/definitely/not/here.yaml"), std::runtime_error);
  REQUIRE_THROWS_AS(sim::loadAircraft("/definitely/not/here.yaml"), std::runtime_error);

  // Fewer than two waypoints.
  const std::string one_wp = R"(
name: bad
aircraft: )" + configDir() + R"(/aircraft/aether6_uav.yaml
guidance:
  waypoints:
    - {north: 0.0, east: 0.0, altitude: 100.0}
)";
  REQUIRE_THROWS_AS(sim::loadScenario(writeTemp("one_wp.yaml", one_wp)), std::runtime_error);

  // Negative integration step.
  const std::string bad_dt = R"(
name: bad
aircraft: )" + configDir() + R"(/aircraft/aether6_uav.yaml
integration: {dt: -0.001}
guidance:
  waypoints:
    - {north: 0.0, east: 0.0, altitude: 100.0}
    - {north: 100.0, east: 0.0, altitude: 100.0}
)";
  REQUIRE_THROWS_AS(sim::loadScenario(writeTemp("bad_dt.yaml", bad_dt)), std::runtime_error);

  // Unknown enumeration values.
  REQUIRE_THROWS_AS(sim::parseControllerType("mpc"), std::invalid_argument);
  REQUIRE_THROWS_AS(sim::parseFeedbackSource("oracle"), std::invalid_argument);

  // An airframe with an invalid inertia tensor.
  const std::string bad_air = R"(
name: broken
mass_properties: {mass: 13.5, Jx: 0.8, Jy: 1.1, Jz: 1.7, Jxz: 9.0}
)";
  REQUIRE_THROWS_AS(sim::loadAircraft(writeTemp("bad_air.yaml", bad_air)),
                    std::invalid_argument);

  // Invalid YAML syntax.
  REQUIRE_THROWS_AS(sim::loadScenario(writeTemp("syntax.yaml", "name: [unclosed\n")),
                    std::runtime_error);
}

TEST_CASE("controller and feedback names round-trip", "[config]") {
  REQUIRE(sim::toString(sim::ControllerType::kPid) == "pid");
  REQUIRE(sim::toString(sim::ControllerType::kLqr) == "lqr");
  REQUIRE(sim::parseControllerType(sim::toString(sim::ControllerType::kLqr)) ==
          sim::ControllerType::kLqr);
  REQUIRE(sim::parseFeedbackSource(sim::toString(sim::FeedbackSource::kTruth)) ==
          sim::FeedbackSource::kTruth);
}

TEST_CASE("CSV writer produces a well-formed file", "[util][csv]") {
  const auto dir = std::filesystem::temp_directory_path() / "aether_tests";
  std::filesystem::create_directories(dir);
  const auto path = (dir / "csv_test.csv").string();
  {
    util::CsvWriter w(path, {"a", "b", "c"});
    w.writeRow({1.0, 2.5, -3.25});
    w.writeRow({0.0, 1e-12, 1e12});
    REQUIRE(w.rows() == 2);
    REQUIRE(w.header().size() == 3);
    REQUIRE_THROWS_AS(w.writeRow({1.0}), std::invalid_argument);
  }
  std::ifstream in(path);
  std::string line;
  std::getline(in, line);
  REQUIRE(line == "a,b,c");
  std::getline(in, line);
  REQUIRE(line == "1,2.5,-3.25");
  std::getline(in, line);
  REQUIRE(line == "0,1e-12,1e+12");

  REQUIRE_THROWS_AS(util::CsvWriter("/nonexistent_dir_xyz/out.csv", {"a"}),
                    std::runtime_error);
}

TEST_CASE("seed derivation is deterministic and well separated", "[util][random]") {
  REQUIRE(util::deriveSeed(1, 2, 3) == util::deriveSeed(1, 2, 3));
  REQUIRE(util::deriveSeed(1, 2, 3) != util::deriveSeed(1, 2, 4));
  REQUIRE(util::deriveSeed(1, 2, 3) != util::deriveSeed(1, 3, 3));
  REQUIRE(util::deriveSeed(1, 2, 3) != util::deriveSeed(2, 2, 3));

  // Consecutive indices must not produce correlated streams.
  double correlation = 0.0;
  const int n = 4000;
  for (int i = 0; i < n; ++i) {
    util::Rng a(util::deriveSeed(42, 0, i));
    util::Rng b(util::deriveSeed(42, 0, i + 1));
    correlation += a.gaussian() * b.gaussian();
  }
  REQUIRE(std::abs(correlation / n) < 0.06);

  util::Rng r(7);
  REQUIRE(r.seed() == 7);
  double sum = 0.0, sumsq = 0.0;
  const int m = 200000;
  for (int i = 0; i < m; ++i) {
    const double v = r.gaussian();
    sum += v;
    sumsq += v * v;
  }
  REQUIRE(sum / m == Approx(0.0).margin(0.01));
  REQUIRE(std::sqrt(sumsq / m) == Approx(1.0).epsilon(0.01));

  util::Rng u(9);
  double lo = 1e9, hi = -1e9;
  for (int i = 0; i < 10000; ++i) {
    const double v = u.uniform(-2.0, 5.0);
    lo = std::min(lo, v);
    hi = std::max(hi, v);
  }
  REQUIRE(lo >= -2.0);
  REQUIRE(hi <= 5.0);
  REQUIRE(lo < -1.9);
  REQUIRE(hi > 4.9);
}
