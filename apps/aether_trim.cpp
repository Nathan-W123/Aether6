/// \file aether_trim.cpp
/// \brief Compute trim points, linearise around them, report the classical modes and export
///        the linear model for the Python tooling.
///
/// Usage: aether_trim [scenario.yaml] [--output DIR] [--airspeed V] [--altitude H]
///                    [--gamma DEG] [--turn-rate DEG_S] [--sweep V0:V1:N]
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "aether/analysis/Linearize.hpp"
#include "aether/analysis/Trim.hpp"
#include "aether/core/Constants.hpp"
#include "aether/env/Atmosphere.hpp"
#include "aether/math/LinearAlgebra.hpp"
#include "aether/sim/Config.hpp"
#include "aether/util/Csv.hpp"

using namespace aether;

namespace {

void printModes(const char* title, const std::vector<analysis::ModeInfo>& modes) {
  std::cout << "  " << title << "\n";
  std::cout << "    " << std::left << std::setw(18) << "mode" << std::right << std::setw(12)
            << "real" << std::setw(12) << "imag" << std::setw(12) << "wn[rad/s]"
            << std::setw(10) << "zeta" << std::setw(11) << "T[s]" << std::setw(13)
            << "t_half[s]" << "\n";
  for (const auto& m : modes) {
    if (m.eigenvalue.imag() < -1e-9) continue;  // print one representative per pair
    std::cout << "    " << std::left << std::setw(18) << m.name << std::right << std::fixed
              << std::setprecision(5) << std::setw(12) << m.eigenvalue.real() << std::setw(12)
              << m.eigenvalue.imag() << std::setw(12) << m.natural_frequency << std::setw(10)
              << m.damping_ratio << std::setw(11) << m.period << std::setw(13)
              << m.time_to_half << (m.stable ? "" : "   UNSTABLE") << "\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::string scenario_path = "configs/scenarios/nominal.yaml";
  std::string output_dir = "results/linear";
  double airspeed = -1.0, altitude = -1.0, gamma_deg = 0.0, turn_rate_deg = 0.0;
  double sweep_lo = 0.0, sweep_hi = 0.0;
  int sweep_n = 0;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char* what) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "aether_trim: missing value for " << what << "\n";
        std::exit(2);
      }
      return argv[++i];
    };
    if (arg == "-h" || arg == "--help") {
      std::cout << "Usage: aether_trim [SCENARIO.yaml] [--output DIR] [--airspeed V]\n"
                   "                   [--altitude H] [--gamma DEG] [--turn-rate DEG_S]\n"
                   "                   [--sweep V0:V1:N]\n";
      return 0;
    } else if (arg == "--output") {
      output_dir = next("--output");
    } else if (arg == "--airspeed") {
      airspeed = std::stod(next("--airspeed"));
    } else if (arg == "--altitude") {
      altitude = std::stod(next("--altitude"));
    } else if (arg == "--gamma") {
      gamma_deg = std::stod(next("--gamma"));
    } else if (arg == "--turn-rate") {
      turn_rate_deg = std::stod(next("--turn-rate"));
    } else if (arg == "--sweep") {
      const std::string spec = next("--sweep");
      const auto a = spec.find(':');
      const auto b = spec.find(':', a + 1);
      if (a == std::string::npos || b == std::string::npos) {
        std::cerr << "aether_trim: --sweep expects V0:V1:N\n";
        return 2;
      }
      sweep_lo = std::stod(spec.substr(0, a));
      sweep_hi = std::stod(spec.substr(a + 1, b - a - 1));
      sweep_n = std::stoi(spec.substr(b + 1));
    } else if (!arg.empty() && arg[0] == '-') {
      std::cerr << "aether_trim: unknown option '" << arg << "'\n";
      return 2;
    } else {
      scenario_path = arg;
    }
  }

  try {
    const sim::ScenarioConfig cfg = sim::loadScenario(scenario_path);
    const dynamics::AircraftParameters aircraft = sim::loadAircraft(cfg.aircraft_file);
    const dynamics::RigidBody6DOF model(aircraft);

    analysis::TrimSpec spec;
    spec.airspeed = airspeed > 0.0 ? airspeed : cfg.initial.airspeed;
    spec.altitude = altitude > 0.0 ? altitude : cfg.initial.altitude;
    spec.flight_path_angle = gamma_deg * constants::kDegToRad;
    spec.turn_rate = turn_rate_deg * constants::kDegToRad;
    spec.density_override = env::Atmosphere(cfg.density_scale).density(spec.altitude);

    const analysis::TrimSolver solver(model);
    const analysis::TrimResult trim = solver.solve(spec);

    std::cout << "=== Aether-6 trim and linearisation =====================================\n";
    std::cout << "aircraft   : " << aircraft.name
              << (aircraft.synthetic ? "  [SYNTHETIC PARAMETERS]" : "") << "\n";
    std::cout << "condition  : Va = " << spec.airspeed << " m/s, h = " << spec.altitude
              << " m, gamma = " << gamma_deg << " deg, turn rate = " << turn_rate_deg
              << " deg/s\n";
    std::cout << "density    : " << trim.density << " kg/m^3\n";
    std::cout << "-- trim solution ---------------------------------------------------------\n";
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "  alpha    = " << trim.alpha * constants::kRadToDeg << " deg\n";
    std::cout << "  beta     = " << trim.beta * constants::kRadToDeg << " deg\n";
    std::cout << "  phi      = " << trim.phi * constants::kRadToDeg << " deg\n";
    std::cout << "  theta    = " << trim.theta * constants::kRadToDeg << " deg\n";
    std::cout << "  elevator = " << trim.controls.elevator * constants::kRadToDeg << " deg\n";
    std::cout << "  aileron  = " << trim.controls.aileron * constants::kRadToDeg << " deg\n";
    std::cout << "  rudder   = " << trim.controls.rudder * constants::kRadToDeg << " deg\n";
    std::cout << "  throttle = " << trim.controls.throttle << "\n";
    std::cout << "  converged = " << (trim.converged ? "yes" : "NO") << "  (" << trim.message
              << "), " << trim.iterations << " iterations\n";
    std::cout << "-- trim residuals --------------------------------------------------------\n";
    std::cout << std::scientific << std::setprecision(6);
    for (int i = 0; i < trim.residual.size(); ++i)
      std::cout << "  " << std::left << std::setw(12) << analysis::TrimResult::residualNames()[i]
                << std::right << std::setw(16) << trim.residual(i) << "  ["
                << analysis::TrimResult::residualUnits()[i] << "]\n";
    std::cout << "  inf-norm = " << trim.residual_inf << "\n";

    dynamics::EnvironmentSample env;
    env.density = trim.density;
    const analysis::LinearModel lm = analysis::linearize(model, trim.state, trim.controls, env);
    const analysis::ReducedModels red = analysis::extractReducedModels(lm);

    std::cout << "-- stability analysis ----------------------------------------------------\n";
    printModes("longitudinal (states: du, dw, dq, dtheta)",
               analysis::classifyLongitudinal(red.A_lon));
    printModes("lateral      (states: dv, dp, dr, dphi)",
               analysis::classifyLateral(red.A_lat));

    const int rank_lon = math::numericalRank(math::controllabilityMatrix(red.A_lon, red.B_lon));
    const int rank_lat = math::numericalRank(math::controllabilityMatrix(red.A_lat, red.B_lat));
    std::cout << "  controllability rank: longitudinal " << rank_lon << "/4, lateral "
              << rank_lat << "/4\n";

    analysis::exportLinearModel(lm, output_dir);
    std::cout << "linear model exported to " << output_dir << "\n";

    // Optional airspeed sweep: trim at a range of speeds and record the trim controls.
    if (sweep_n > 1) {
      std::filesystem::create_directories(output_dir);
      util::CsvWriter sweep(output_dir + "/trim_sweep.csv",
                            {"airspeed_mps", "alpha_deg", "theta_deg", "elevator_deg",
                             "throttle", "residual_inf", "converged", "short_period_wn",
                             "short_period_zeta", "phugoid_wn", "phugoid_zeta",
                             "dutch_roll_wn", "dutch_roll_zeta", "roll_subsidence",
                             "spiral"});
      for (int i = 0; i < sweep_n; ++i) {
        analysis::TrimSpec s = spec;
        s.airspeed = sweep_lo + (sweep_hi - sweep_lo) * i / (sweep_n - 1);
        const analysis::TrimResult t = solver.solve(s);
        double sp_wn = 0, sp_z = 0, ph_wn = 0, ph_z = 0, dr_wn = 0, dr_z = 0, rs = 0, sp_r = 0;
        if (t.converged) {
          dynamics::EnvironmentSample e2;
          e2.density = t.density;
          const analysis::LinearModel l2 = analysis::linearize(model, t.state, t.controls, e2);
          const analysis::ReducedModels r2 = analysis::extractReducedModels(l2);
          for (const auto& mm : analysis::classifyLongitudinal(r2.A_lon)) {
            if (mm.name == "short_period" && mm.eigenvalue.imag() >= 0) {
              sp_wn = mm.natural_frequency;
              sp_z = mm.damping_ratio;
            }
            if (mm.name == "phugoid" && mm.eigenvalue.imag() >= 0) {
              ph_wn = mm.natural_frequency;
              ph_z = mm.damping_ratio;
            }
          }
          for (const auto& mm : analysis::classifyLateral(r2.A_lat)) {
            if (mm.name == "dutch_roll" && mm.eigenvalue.imag() >= 0) {
              dr_wn = mm.natural_frequency;
              dr_z = mm.damping_ratio;
            }
            if (mm.name == "roll_subsidence") rs = mm.eigenvalue.real();
            if (mm.name == "spiral") sp_r = mm.eigenvalue.real();
          }
        }
        sweep.writeRow({s.airspeed, t.alpha * constants::kRadToDeg,
                        t.theta * constants::kRadToDeg,
                        t.controls.elevator * constants::kRadToDeg, t.controls.throttle,
                        t.residual_inf, t.converged ? 1.0 : 0.0, sp_wn, sp_z, ph_wn, ph_z,
                        dr_wn, dr_z, rs, sp_r});
      }
      std::cout << "airspeed sweep written to " << output_dir << "/trim_sweep.csv\n";
    }
    std::cout << "=========================================================================\n";
    return trim.converged ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "aether_trim: " << e.what() << "\n";
    return 2;
  }
}
