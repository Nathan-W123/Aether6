/// \file aether_mc.cpp
/// \brief Run a deterministic Monte-Carlo campaign over vehicle, environment, initial-condition
///        and sensor dispersions.
///
/// Usage: aether_mc [scenario.yaml] [--trials N] [--seed N] [--threads N] [--output DIR]
///                  [--duration S] [--controller pid|lqr]
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

#include "aether/mc/MonteCarlo.hpp"
#include "aether/sim/Config.hpp"

using namespace aether;

int main(int argc, char** argv) {
  std::string scenario_path = "configs/scenarios/monte_carlo.yaml";
  mc::MonteCarloConfig mcfg;
  bool trials_set = false, seed_set = false, threads_set = false, output_set = false,
       trajectories_set = false;
  double duration_override = -1.0;
  double dt_override = -1.0;
  std::string controller_override;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char* what) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "aether_mc: missing value for " << what << "\n";
        std::exit(2);
      }
      return argv[++i];
    };
    if (arg == "-h" || arg == "--help") {
      std::cout << "Usage: aether_mc [SCENARIO.yaml] [--trials N] [--seed N] [--threads N]\n"
                   "                 [--output DIR] [--duration S] [--dt S]\n"
                   "                 [--controller pid|lqr] [--trajectories N]\n";
      return 0;
    } else if (arg == "--trials") {
      mcfg.trials = std::stoi(next("--trials"));
      trials_set = true;
    } else if (arg == "--seed") {
      mcfg.master_seed = static_cast<std::uint64_t>(std::stoull(next("--seed")));
      seed_set = true;
    } else if (arg == "--threads") {
      mcfg.threads = std::stoi(next("--threads"));
      threads_set = true;
    } else if (arg == "--output") {
      mcfg.output_dir = next("--output");
      output_set = true;
    } else if (arg == "--trajectories") {
      mcfg.recorded_trajectories = std::stoi(next("--trajectories"));
      trajectories_set = true;
    } else if (arg == "--duration") {
      duration_override = std::stod(next("--duration"));
    } else if (arg == "--dt") {
      dt_override = std::stod(next("--dt"));
    } else if (arg == "--controller") {
      controller_override = next("--controller");
    } else if (!arg.empty() && arg[0] == '-') {
      std::cerr << "aether_mc: unknown option '" << arg << "'\n";
      return 2;
    } else {
      scenario_path = arg;
    }
  }

  try {
    sim::ScenarioConfig base = sim::loadScenario(scenario_path);
    // The scenario file supplies the campaign definition; explicit command-line flags win.
    const mc::MonteCarloConfig file_cfg = mc::loadMonteCarloConfig(scenario_path);
    const mc::MonteCarloConfig cli = mcfg;
    mcfg = file_cfg;
    if (trials_set) mcfg.trials = cli.trials;
    if (seed_set) mcfg.master_seed = cli.master_seed;
    if (threads_set) mcfg.threads = cli.threads;
    if (output_set) mcfg.output_dir = cli.output_dir;
    if (trajectories_set) mcfg.recorded_trajectories = cli.recorded_trajectories;
    if (duration_override > 0.0) base.duration = duration_override;
    if (dt_override > 0.0) base.integration.dt = dt_override;
    if (!controller_override.empty())
      base.controller = sim::parseControllerType(controller_override);
    const dynamics::AircraftParameters aircraft = sim::loadAircraft(base.aircraft_file);

    std::cout << "=== Aether-6 Monte-Carlo campaign =======================================\n";
    std::cout << "scenario   : " << base.name << " (" << scenario_path << ")\n";
    std::cout << "controller : " << sim::toString(base.controller)
              << "   feedback: " << sim::toString(base.feedback) << "\n";
    std::cout << "trials     : " << mcfg.trials << "   master seed: " << mcfg.master_seed
              << "   duration/trial: " << base.duration << " s\n";

    mc::MonteCarloRunner runner(base, aircraft, mcfg);
    const mc::MonteCarloResult result = runner.run(true);
    runner.write(result);

    std::cout << "-- outcome --------------------------------------------------------------\n";
    std::cout << "  successes   : " << result.successes << "/" << mcfg.trials << "\n";
    std::cout << "  failures    : " << result.failures << "  (rate "
              << std::fixed << std::setprecision(4) << result.failure_rate << ")\n";
    std::cout << "  threads     : " << result.threads_used << "\n";
    std::cout << "  wall clock  : " << std::setprecision(2) << result.wall_clock_seconds
              << " s total, " << result.mean_trial_seconds << " s mean per trial\n";
    std::cout << "-- statistics over successful trials -------------------------------------\n";
    std::cout << std::left << std::setw(30) << "metric" << std::setw(8) << "unit"
              << std::right << std::setw(12) << "mean" << std::setw(12) << "stddev"
              << std::setw(12) << "p50" << std::setw(12) << "p95" << std::setw(12) << "max"
              << "\n";
    for (const auto& s : result.statistics) {
      std::cout << std::left << std::setw(30) << s.name << std::setw(8) << s.unit << std::right
                << std::setprecision(4) << std::setw(12) << s.mean << std::setw(12) << s.stddev
                << std::setw(12) << s.median << std::setw(12) << s.p95 << std::setw(12)
                << s.max << "\n";
    }
    std::cout << "results written to " << mcfg.output_dir << "\n";
    std::cout << "=========================================================================\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "aether_mc: " << e.what() << "\n";
    return 2;
  }
}
