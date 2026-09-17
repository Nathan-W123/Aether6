/// \file test_montecarlo.cpp
/// \brief Monte-Carlo determinism, dispersion behaviour and statistics.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <vector>

#include "aether/mc/MonteCarlo.hpp"
#include "aether/sim/Config.hpp"

using namespace aether;
using Catch::Approx;

namespace {

sim::ScenarioConfig baseScenario() {
  sim::ScenarioConfig cfg =
      sim::loadScenario(std::string(AETHER_CONFIG_DIR) + "/scenarios/nominal.yaml");
  cfg.logging.enabled = false;
  cfg.duration = 30.0;              // keep the unit test fast
  cfg.integration.dt = 0.005;
  return cfg;
}

mc::MonteCarloConfig quickConfig(int trials, std::uint64_t seed = 4242) {
  mc::MonteCarloConfig c;
  c.trials = trials;
  c.master_seed = seed;
  c.threads = 1;
  c.trajectory_interval = 2.0;
  c.recorded_trajectories = trials;
  c.output_dir = (std::filesystem::temp_directory_path() / "aether_mc_test").string();
  return c;
}

}  // namespace

TEST_CASE("statistics helper computes correct moments and percentiles", "[mc][statistics]") {
  std::vector<double> data;
  for (int i = 1; i <= 100; ++i) data.push_back(static_cast<double>(i));
  const auto s = mc::computeStatistics("test", "-", data);
  REQUIRE(s.count == 100);
  REQUIRE(s.mean == Approx(50.5));
  REQUIRE(s.min == Approx(1.0));
  REQUIRE(s.max == Approx(100.0));
  REQUIRE(s.median == Approx(50.5));
  REQUIRE(s.p05 == Approx(5.95));
  REQUIRE(s.p95 == Approx(95.05));
  // Sample standard deviation of 1..100 is sqrt(100*101/12) = 29.011...
  REQUIRE(s.stddev == Approx(29.0115).epsilon(1e-4));

  const auto empty = mc::computeStatistics("e", "-", {});
  REQUIRE(empty.count == 0);
  REQUIRE(empty.mean == Approx(0.0));

  const auto single = mc::computeStatistics("s", "-", {7.0});
  REQUIRE(single.mean == Approx(7.0));
  REQUIRE(single.stddev == Approx(0.0));
  REQUIRE(single.median == Approx(7.0));
}

TEST_CASE("trial construction is deterministic and actually disperses", "[mc][determinism]") {
  const auto base = baseScenario();
  const auto aircraft = sim::loadAircraft(base.aircraft_file);
  mc::MonteCarloRunner runner(base, aircraft, quickConfig(8));

  dynamics::AircraftParameters a1, a2, a3;
  sim::ScenarioConfig s1, s2, s3;
  mc::TrialResult r1, r2, r3;
  runner.buildTrial(3, &a1, &s1, &r1);
  runner.buildTrial(3, &a2, &s2, &r2);
  runner.buildTrial(4, &a3, &s3, &r3);

  // Same index -> identical dispersion.
  REQUIRE(a1.mass == Approx(a2.mass));
  REQUIRE(a1.lon.CL_alpha == Approx(a2.lon.CL_alpha));
  REQUIRE(r1.seed == r2.seed);
  REQUIRE(s1.wind.steady_ned.isApprox(s2.wind.steady_ned));
  REQUIRE(s1.initial.delta_yaw == Approx(s2.initial.delta_yaw));

  // Different index -> different dispersion.
  REQUIRE(a1.mass != Approx(a3.mass));
  REQUIRE(r1.seed != r3.seed);

  // The dispersed airframe is still physically valid and actually different from nominal.
  REQUIRE_NOTHROW(a1.validate());
  REQUIRE(a1.mass != Approx(aircraft.mass));
  REQUIRE(a1.Jx > 0.0);
  REQUIRE(a1.lon.oswald > 0.0);
  REQUIRE(a1.lon.oswald <= 1.0);
  REQUIRE(a1.lon.Cm_alpha < 0.0);   // dispersion must not destroy static stability
  REQUIRE(a1.lat.Cn_beta > 0.0);
  REQUIRE(a1.lat.Cl_p < 0.0);

  // The dispersed inertia tensor must remain a physically realisable rigid body.
  Eigen::SelfAdjointEigenSolver<Mat3> es(a1.inertia());
  const Vec3 principal = es.eigenvalues();
  REQUIRE(principal.minCoeff() > 0.0);
  REQUIRE(principal(0) + principal(1) >= principal(2));
}

TEST_CASE("dispersion magnitudes match the configured sigmas", "[mc][statistics]") {
  const auto base = baseScenario();
  const auto aircraft = sim::loadAircraft(base.aircraft_file);
  auto cfg = quickConfig(600);
  mc::MonteCarloRunner runner(base, aircraft, cfg);

  std::vector<double> mass, clalpha, wind, yaw;
  for (int i = 0; i < cfg.trials; ++i) {
    dynamics::AircraftParameters a;
    sim::ScenarioConfig s;
    mc::TrialResult r;
    REQUIRE_NOTHROW(runner.buildTrial(i, &a, &s, &r));
    REQUIRE_NOTHROW(a.validate());
    mass.push_back(a.mass);
    clalpha.push_back(a.lon.CL_alpha);
    wind.push_back(r.wind_speed);
    yaw.push_back(s.initial.delta_yaw);
  }
  const auto ms = mc::computeStatistics("mass", "kg", mass);
  REQUIRE(ms.mean == Approx(aircraft.mass).epsilon(0.02));
  REQUIRE(ms.stddev ==
          Approx(cfg.dispersions.mass_relative * aircraft.mass).epsilon(0.15));

  const auto cs = mc::computeStatistics("cl", "1/rad", clalpha);
  REQUIRE(cs.mean == Approx(aircraft.lon.CL_alpha).epsilon(0.03));
  REQUIRE(cs.stddev ==
          Approx(cfg.dispersions.aero_relative * aircraft.lon.CL_alpha).epsilon(0.2));

  const auto ws = mc::computeStatistics("wind", "m/s", wind);
  REQUIRE(ws.mean == Approx(cfg.dispersions.wind_speed_mean).epsilon(0.2));
  REQUIRE(ws.min >= 0.0);

  const auto ys = mc::computeStatistics("yaw", "rad", yaw);
  REQUIRE(ys.mean == Approx(0.0).margin(0.06));
  REQUIRE(ys.stddev == Approx(cfg.dispersions.initial_yaw).epsilon(0.15));
}

TEST_CASE("dispersion can be switched off entirely", "[mc]") {
  auto base = baseScenario();
  const auto aircraft = sim::loadAircraft(base.aircraft_file);
  auto cfg = quickConfig(4);
  cfg.dispersions.vary_mass_properties = false;
  cfg.dispersions.vary_aerodynamics = false;
  cfg.dispersions.vary_initial_conditions = false;
  cfg.dispersions.vary_environment = false;
  cfg.dispersions.vary_sensors = false;
  mc::MonteCarloRunner runner(base, aircraft, cfg);

  dynamics::AircraftParameters a;
  sim::ScenarioConfig s;
  mc::TrialResult r;
  runner.buildTrial(2, &a, &s, &r);
  REQUIRE(a.mass == Approx(aircraft.mass));
  REQUIRE(a.lon.CL_alpha == Approx(aircraft.lon.CL_alpha));
  REQUIRE(s.wind.steady_ned.isApprox(base.wind.steady_ned));
  REQUIRE(s.initial.delta_yaw == Approx(0.0));
  REQUIRE(r.sensor_scale == Approx(1.0));
  // Only the seed still varies between trials.
  REQUIRE(s.seed != base.seed);
}

TEST_CASE("a seeded campaign reproduces identical results", "[mc][determinism][slow]") {
  const auto base = baseScenario();
  const auto aircraft = sim::loadAircraft(base.aircraft_file);

  mc::MonteCarloRunner a(base, aircraft, quickConfig(6, 20240917));
  mc::MonteCarloRunner b(base, aircraft, quickConfig(6, 20240917));
  const auto ra = a.run(false);
  const auto rb = b.run(false);

  REQUIRE(ra.trials.size() == rb.trials.size());
  REQUIRE(ra.successes == rb.successes);
  REQUIRE(ra.failures == rb.failures);
  for (std::size_t i = 0; i < ra.trials.size(); ++i) {
    INFO("trial " << i);
    REQUIRE(ra.trials[i].seed == rb.trials[i].seed);
    REQUIRE(ra.trials[i].ok == rb.trials[i].ok);
    REQUIRE(ra.trials[i].metrics.rms_cross_track ==
            Approx(rb.trials[i].metrics.rms_cross_track));
    REQUIRE(ra.trials[i].metrics.rmse_position == Approx(rb.trials[i].metrics.rmse_position));
    REQUIRE(ra.trials[i].trajectory.size() == rb.trials[i].trajectory.size());
  }
  for (std::size_t i = 0; i < ra.statistics.size(); ++i) {
    INFO("statistic " << ra.statistics[i].name);
    REQUIRE(ra.statistics[i].name == rb.statistics[i].name);
    // Wall-clock timing is measured, not computed, so it is the one statistic that is
    // legitimately not reproducible.
    if (ra.statistics[i].name == "trial_wall_clock") continue;
    REQUIRE(ra.statistics[i].mean == Approx(rb.statistics[i].mean));
    REQUIRE(ra.statistics[i].p95 == Approx(rb.statistics[i].p95));
  }

  // A different master seed must change the outcome.
  mc::MonteCarloRunner c(base, aircraft, quickConfig(6, 987));
  const auto rc = c.run(false);
  REQUIRE(rc.trials[0].seed != ra.trials[0].seed);
}

TEST_CASE("multithreaded execution matches single-threaded execution",
          "[mc][determinism][slow]") {
  const auto base = baseScenario();
  const auto aircraft = sim::loadAircraft(base.aircraft_file);

  auto single = quickConfig(8, 5150);
  single.threads = 1;
  auto multi = quickConfig(8, 5150);
  multi.threads = 4;

  const auto rs = mc::MonteCarloRunner(base, aircraft, single).run(false);
  const auto rm = mc::MonteCarloRunner(base, aircraft, multi).run(false);

  REQUIRE(rm.threads_used == 4);
  REQUIRE(rs.threads_used == 1);
  for (std::size_t i = 0; i < rs.trials.size(); ++i) {
    INFO("trial " << i);
    REQUIRE(rs.trials[i].seed == rm.trials[i].seed);
    REQUIRE(rs.trials[i].metrics.rms_cross_track ==
            Approx(rm.trials[i].metrics.rms_cross_track));
    REQUIRE(rs.trials[i].metrics.rmse_attitude == Approx(rm.trials[i].metrics.rmse_attitude));
  }
}

TEST_CASE("a campaign produces usable statistics and output files", "[mc][slow]") {
  const auto base = baseScenario();
  const auto aircraft = sim::loadAircraft(base.aircraft_file);
  auto cfg = quickConfig(12, 31337);
  cfg.threads = 2;
  mc::MonteCarloRunner runner(base, aircraft, cfg);
  const auto result = runner.run(false);

  REQUIRE(result.successes + result.failures == cfg.trials);
  REQUIRE(result.failure_rate == Approx(static_cast<double>(result.failures) / cfg.trials));
  REQUIRE(result.wall_clock_seconds > 0.0);
  REQUIRE_FALSE(result.statistics.empty());

  bool found_cross = false, found_attitude = false;
  for (const auto& s : result.statistics) {
    if (s.name == "rms_cross_track") {
      found_cross = true;
      REQUIRE(s.unit == "m");
      REQUIRE(s.count == result.successes);
      REQUIRE(s.min <= s.median);
      REQUIRE(s.median <= s.p95);
      REQUIRE(s.p95 <= s.max);
      REQUIRE(s.mean >= 0.0);
    }
    if (s.name == "estimator_attitude_rmse") {
      found_attitude = true;
      REQUIRE(s.unit == "deg");
    }
  }
  REQUIRE(found_cross);
  REQUIRE(found_attitude);

  // Trajectories were recorded at the configured interval.
  REQUIRE_FALSE(result.trials[0].trajectory.empty());
  const auto& traj = result.trials[0].trajectory;
  if (traj.size() > 1)
    REQUIRE(traj[1].time - traj[0].time == Approx(cfg.trajectory_interval).epsilon(0.05));

  runner.write(result);
  for (const char* file : {"trials.csv", "trajectories.csv", "envelope.csv", "summary.json"}) {
    const auto path = std::filesystem::path(cfg.output_dir) / file;
    INFO("expected output " << path.string());
    REQUIRE(std::filesystem::exists(path));
    REQUIRE(std::filesystem::file_size(path) > 0);
  }

  // trials.csv must have one header line plus one row per trial.
  std::ifstream trials(std::filesystem::path(cfg.output_dir) / "trials.csv");
  std::string line;
  int lines = 0;
  while (std::getline(trials, line)) ++lines;
  REQUIRE(lines == cfg.trials + 1);

  // summary.json must be parseable enough to contain the headline numbers.
  std::ifstream summary(std::filesystem::path(cfg.output_dir) / "summary.json");
  const std::string text((std::istreambuf_iterator<char>(summary)),
                         std::istreambuf_iterator<char>());
  REQUIRE(text.find("\"trials\": 12") != std::string::npos);
  REQUIRE(text.find("\"failure_rate\"") != std::string::npos);
  REQUIRE(text.find("\"statistics\"") != std::string::npos);
  REQUIRE(text.find("rms_cross_track") != std::string::npos);
}
