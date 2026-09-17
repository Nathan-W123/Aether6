/// \file aether_sim.cpp
/// \brief Run a single closed-loop scenario and write its logs, trim and linear model.
///
/// Usage: aether_sim [scenario.yaml] [--output DIR] [--duration S] [--integrator NAME]
///                   [--controller pid|lqr] [--feedback truth|estimate] [--seed N] [--quiet]
#include <cstdlib>
#include <iostream>
#include <string>

#include "aether/analysis/Linearize.hpp"
#include "aether/core/Constants.hpp"
#include "aether/sim/Config.hpp"
#include "aether/sim/Simulator.hpp"

using namespace aether;

namespace {

void printUsage() {
  std::cout <<
      "Usage: aether_sim [SCENARIO.yaml] [options]\n"
      "\n"
      "Options:\n"
      "  --output DIR        override logging.output_dir\n"
      "  --duration S        override the simulated duration [s]\n"
      "  --integrator NAME   rk4 | dopri54\n"
      "  --dt S              override the outer integration step [s]\n"
      "  --controller NAME   pid | lqr\n"
      "  --feedback NAME     truth | estimate\n"
      "  --seed N            override the master random seed\n"
      "  --no-log            disable all file output\n"
      "  --quiet             suppress the console report\n"
      "  -h, --help          show this message\n"
      "\n"
      "Default scenario: configs/scenarios/nominal.yaml\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string scenario_path = "configs/scenarios/nominal.yaml";
  std::string output_override;
  std::string integrator_override;
  std::string controller_override;
  std::string feedback_override;
  double duration_override = -1.0;
  double dt_override = -1.0;
  long long seed_override = -1;
  bool quiet = false;
  bool no_log = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char* what) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "aether_sim: missing value for " << what << "\n";
        std::exit(2);
      }
      return argv[++i];
    };
    if (arg == "-h" || arg == "--help") {
      printUsage();
      return 0;
    } else if (arg == "--output") {
      output_override = next("--output");
    } else if (arg == "--duration") {
      duration_override = std::stod(next("--duration"));
    } else if (arg == "--dt") {
      dt_override = std::stod(next("--dt"));
    } else if (arg == "--integrator") {
      integrator_override = next("--integrator");
    } else if (arg == "--controller") {
      controller_override = next("--controller");
    } else if (arg == "--feedback") {
      feedback_override = next("--feedback");
    } else if (arg == "--seed") {
      seed_override = std::stoll(next("--seed"));
    } else if (arg == "--quiet") {
      quiet = true;
    } else if (arg == "--no-log") {
      no_log = true;
    } else if (!arg.empty() && arg[0] == '-') {
      std::cerr << "aether_sim: unknown option '" << arg << "'\n";
      printUsage();
      return 2;
    } else {
      scenario_path = arg;
    }
  }

  try {
    sim::ScenarioConfig cfg = sim::loadScenario(scenario_path);
    if (!output_override.empty()) cfg.logging.output_dir = output_override;
    if (duration_override > 0.0) cfg.duration = duration_override;
    if (dt_override > 0.0) cfg.integration.dt = dt_override;
    if (!integrator_override.empty()) cfg.integration.integrator = integrator_override;
    if (!controller_override.empty())
      cfg.controller = sim::parseControllerType(controller_override);
    if (!feedback_override.empty())
      cfg.feedback = sim::parseFeedbackSource(feedback_override);
    if (seed_override >= 0) cfg.seed = static_cast<std::uint64_t>(seed_override);
    if (no_log) cfg.logging.enabled = false;

    const dynamics::AircraftParameters aircraft = sim::loadAircraft(cfg.aircraft_file);
    sim::Simulator simulator(cfg, aircraft);
    const sim::SimulationResult result = simulator.run();

    if (cfg.logging.enabled) {
      sim::writeMetricsJson(cfg.logging.output_dir + "/summary.json", result, cfg);
      analysis::exportLinearModel(simulator.linearModel(), cfg.logging.output_dir + "/linear");
    }

    if (!quiet) {
      const auto& m = result.metrics;
      const auto& t = result.trim;
      std::cout << "=== Aether-6 closed-loop simulation =====================================\n";
      std::cout << "scenario        : " << cfg.name << "  (" << scenario_path << ")\n";
      std::cout << "aircraft        : " << aircraft.name
                << (aircraft.synthetic ? "  [SYNTHETIC PARAMETERS]" : "") << "\n";
      std::cout << "controller      : " << sim::toString(cfg.controller)
                << "   feedback: " << sim::toString(cfg.feedback) << "\n";
      std::cout << "integrator      : " << cfg.integration.integrator
                << "   dt = " << cfg.integration.dt << " s\n";
      std::cout << "seed            : " << cfg.seed << "\n";
      std::cout << "-- trim ----------------------------------------------------------------\n";
      std::cout << "  alpha = " << t.alpha * constants::kRadToDeg
                << " deg, theta = " << t.theta * constants::kRadToDeg
                << " deg, elevator = " << t.controls.elevator * constants::kRadToDeg
                << " deg, throttle = " << t.controls.throttle << "\n";
      std::cout << "  residual (inf-norm) = " << t.residual_inf << "\n";
      std::cout << "-- mission -------------------------------------------------------------\n";
      std::cout << "  simulated       : " << m.simulated_time << " s  (" << m.termination_reason
                << ")\n";
      std::cout << "  waypoints hit   : " << m.waypoints_reached << "   laps: "
                << m.laps_completed << "\n";
      std::cout << "  cross-track     : rms " << m.rms_cross_track << " m, max "
                << m.max_cross_track << " m\n";
      std::cout << "  altitude error  : rms " << m.rms_altitude_error << " m, max "
                << m.max_altitude_error << " m\n";
      std::cout << "  airspeed error  : rms " << m.rms_airspeed_error << " m/s, max "
                << m.max_airspeed_error << " m/s\n";
      std::cout << "  peak bank/alpha : " << m.max_bank * constants::kRadToDeg << " deg / "
                << m.max_alpha * constants::kRadToDeg << " deg\n";
      std::cout << "-- estimator -----------------------------------------------------------\n";
      std::cout << "  position RMSE   : " << m.rmse_position << " m (horizontal "
                << m.rmse_position_horizontal << " m, vertical " << m.rmse_altitude << " m)\n";
      std::cout << "  velocity RMSE   : " << m.rmse_velocity << " m/s\n";
      std::cout << "  attitude RMSE   : " << m.rmse_attitude * constants::kRadToDeg
                << " deg (yaw " << m.rmse_yaw * constants::kRadToDeg << " deg)\n";
      std::cout << "  bias error      : gyro " << m.final_gyro_bias_error << " rad/s, accel "
                << m.final_accel_bias_error << " m/s^2\n";
      std::cout << "  min eig(P)      : " << m.min_covariance_eigenvalue << "\n";
      std::cout << "-- performance ---------------------------------------------------------\n";
      std::cout << "  wall clock      : " << m.wall_clock_seconds << " s  (real-time factor "
                << m.real_time_factor << "x)\n";
      std::cout << "  rhs evaluations : " << m.dynamics_evaluations << "  steps: " << m.steps
                << "  rejected: " << m.rejected_steps << "\n";
      if (cfg.logging.enabled)
        std::cout << "output          : " << cfg.logging.output_dir << "\n";
      std::cout << "========================================================================\n";
    }
    return result.metrics.stable ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "aether_sim: " << e.what() << "\n";
    return 2;
  }
}
