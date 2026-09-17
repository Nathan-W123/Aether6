/// \file aether_integrators.cpp
/// \brief Integrator accuracy / cost study: order verification on an analytic problem and a
///        reference-solution comparison on the full nonlinear aircraft model.
///
/// Usage: aether_integrators [scenario.yaml] [--output DIR] [--horizon S]
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "aether/analysis/Trim.hpp"
#include "aether/core/Constants.hpp"
#include "aether/env/Atmosphere.hpp"
#include "aether/integrate/RungeKutta.hpp"
#include "aether/math/Rotation.hpp"
#include "aether/sim/Config.hpp"
#include "aether/util/Csv.hpp"

using namespace aether;

namespace {

/// Damped harmonic oscillator with a closed-form solution, used for order verification.
struct Oscillator {
  double omega = 2.0;  ///< Undamped natural frequency [rad/s]
  double zeta = 0.05;  ///< Damping ratio [-]

  VecX derivative(double, const VecX& x) const {
    VecX d(2);
    d(0) = x(1);
    d(1) = -omega * omega * x(0) - 2.0 * zeta * omega * x(1);
    return d;
  }

  VecX exact(double t, const VecX& x0) const {
    const double wd = omega * std::sqrt(1.0 - zeta * zeta);
    const double s = std::exp(-zeta * omega * t);
    const double A = x0(0);
    const double B = (x0(1) + zeta * omega * x0(0)) / wd;
    VecX x(2);
    x(0) = s * (A * std::cos(wd * t) + B * std::sin(wd * t));
    x(1) = s * (-zeta * omega * (A * std::cos(wd * t) + B * std::sin(wd * t)) +
                wd * (-A * std::sin(wd * t) + B * std::cos(wd * t)));
    return x;
  }
};

/// Weighted error norm over the 13-element aircraft state: positions in m, velocities in m/s,
/// the attitude as a rotation angle in rad, and body rates in rad/s, combined as an RMS.
double aircraftStateError(const VecX& a, const VecX& b) {
  const Vec3 dp = a.segment<3>(StateIndex::kPosN) - b.segment<3>(StateIndex::kPosN);
  const Vec3 dv = a.segment<3>(StateIndex::kVelU) - b.segment<3>(StateIndex::kVelU);
  const Vec3 dth = math::boxMinus(a.segment<4>(StateIndex::kQuatW), b.segment<4>(StateIndex::kQuatW));
  const Vec3 dw = a.segment<3>(StateIndex::kRateP) - b.segment<3>(StateIndex::kRateP);
  const double s = dp.squaredNorm() + dv.squaredNorm() + dth.squaredNorm() + dw.squaredNorm();
  return std::sqrt(s / 12.0);
}

}  // namespace

int main(int argc, char** argv) {
  std::string scenario_path = "configs/scenarios/nominal.yaml";
  std::string output_dir = "results/integrators";
  double horizon = 20.0;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char* what) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "aether_integrators: missing value for " << what << "\n";
        std::exit(2);
      }
      return argv[++i];
    };
    if (arg == "-h" || arg == "--help") {
      std::cout << "Usage: aether_integrators [SCENARIO.yaml] [--output DIR] [--horizon S]\n";
      return 0;
    } else if (arg == "--output") {
      output_dir = next("--output");
    } else if (arg == "--horizon") {
      horizon = std::stod(next("--horizon"));
    } else if (!arg.empty() && arg[0] == '-') {
      std::cerr << "aether_integrators: unknown option '" << arg << "'\n";
      return 2;
    } else {
      scenario_path = arg;
    }
  }

  try {
    std::filesystem::create_directories(output_dir);

    // =====================================================================================
    // Part 1: order verification on the damped harmonic oscillator.
    // =====================================================================================
    const Oscillator osc;
    VecX x0(2);
    x0 << 1.0, 0.0;
    const double T = 10.0;

    util::CsvWriter conv(output_dir + "/convergence.csv",
                         {"integrator", "dt_s", "global_error", "function_evals",
                          "wall_clock_s", "observed_order"});
    std::cout << "=== Integrator study =====================================================\n";
    std::cout << "-- Part 1: order verification, damped harmonic oscillator (analytic) -----\n";
    std::cout << "   xddot + 2*zeta*omega*xdot + omega^2 x = 0, omega = " << osc.omega
              << " rad/s, zeta = " << osc.zeta << ", horizon " << T << " s\n";

    const VecX exact = osc.exact(T, x0);
    const std::vector<double> steps = {0.2, 0.1, 0.05, 0.025, 0.0125, 0.00625, 0.003125};
    double prev_err = 0.0, prev_dt = 0.0;
    std::cout << std::setw(12) << "dt[s]" << std::setw(16) << "error" << std::setw(12)
              << "order" << std::setw(12) << "evals" << std::setw(14) << "wall[s]" << "\n";
    for (double dt : steps) {
      integrate::RungeKutta4 rk4;
      integrate::IntegrationStats st;
      const auto t0 = std::chrono::steady_clock::now();
      VecX x = x0;
      // Repeat to get a measurable wall-clock time for the coarse steps.
      const int repeats = std::max(1, static_cast<int>(2000 * dt / 0.003125 / 100));
      for (int r = 0; r < repeats; ++r)
        x = integrate::integrateTo([&](double t, const VecX& s) { return osc.derivative(t, s); },
                                   rk4, 0.0, x0, T, dt, &st);
      const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
                              .count() / repeats;
      const double err = (x - exact).norm();
      double order = 0.0;
      if (prev_err > 0.0) order = std::log(prev_err / err) / std::log(prev_dt / dt);
      conv.writeRow({0.0, dt, err, static_cast<double>(st.function_evals), wall, order});
      std::cout << std::scientific << std::setprecision(4) << std::setw(12) << dt
                << std::setw(16) << err << std::fixed << std::setprecision(3) << std::setw(12)
                << order << std::setw(12) << st.function_evals << std::scientific
                << std::setw(14) << wall << "\n";
      prev_err = err;
      prev_dt = dt;
    }

    std::cout << "-- Part 1b: Dormand-Prince 5(4) tolerance sweep --------------------------\n";
    std::cout << std::setw(12) << "rel_tol" << std::setw(16) << "error" << std::setw(12)
              << "accepted" << std::setw(12) << "rejected" << std::setw(12) << "evals"
              << std::setw(14) << "wall[s]" << "\n";
    util::CsvWriter tolsweep(output_dir + "/dopri_tolerance.csv",
                             {"rel_tol", "global_error", "accepted_steps", "rejected_steps",
                              "function_evals", "wall_clock_s"});
    for (double rtol : {1e-4, 1e-6, 1e-8, 1e-10, 1e-12}) {
      integrate::ToleranceSettings tol;
      tol.rel_tol = rtol;
      tol.abs_tol = rtol * 1e-3;
      tol.max_step = 1.0;
      integrate::DormandPrince54 dp(tol);
      integrate::IntegrationStats st;
      const auto t0 = std::chrono::steady_clock::now();
      const int repeats = 50;
      VecX x = x0;
      for (int r = 0; r < repeats; ++r) {
        dp.reset();
        x = integrate::integrateTo([&](double t, const VecX& s) { return osc.derivative(t, s); },
                                   dp, 0.0, x0, T, 0.05, &st);
      }
      const double wall =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() / repeats;
      const double err = (x - exact).norm();
      tolsweep.writeRow({rtol, err, static_cast<double>(st.accepted_steps),
                         static_cast<double>(st.rejected_steps),
                         static_cast<double>(st.function_evals), wall});
      std::cout << std::scientific << std::setprecision(4) << std::setw(12) << rtol
                << std::setw(16) << err << std::fixed << std::setw(12) << st.accepted_steps
                << std::setw(12) << st.rejected_steps << std::setw(12) << st.function_evals
                << std::scientific << std::setw(14) << wall << "\n";
    }

    // =====================================================================================
    // Part 2: full nonlinear aircraft model against a high-accuracy reference solution.
    // =====================================================================================
    const sim::ScenarioConfig cfg = sim::loadScenario(scenario_path);
    const dynamics::AircraftParameters aircraft = sim::loadAircraft(cfg.aircraft_file);
    const dynamics::RigidBody6DOF model(aircraft);

    analysis::TrimSpec spec;
    spec.airspeed = cfg.initial.airspeed;
    spec.altitude = cfg.initial.altitude;
    spec.density_override = env::Atmosphere().density(spec.altitude);
    const analysis::TrimResult trim = analysis::TrimSolver(model).solve(spec);

    dynamics::EnvironmentSample env;
    env.density = trim.density;
    env.wind_ned = Vec3(2.0, -3.0, 0.0);  // a constant wind keeps the response non-trivial

    // Open-loop excitation: an elevator doublet followed by an aileron doublet. As in the
    // closed-loop simulator, the control is held constant over each fixed frame (zero-order
    // hold), so the right-hand side is smooth *within* a step and the observed convergence
    // order is not corrupted by a discontinuity straddled by the Runge-Kutta stages.
    constexpr double kFrame = 0.5;  // s, the fixed frame every integrator is driven over
    auto controls = [&](double t) {
      ControlInput u = trim.controls;
      if (t >= 1.0 && t < 2.0) u.elevator += 4.0 * constants::kDegToRad;
      if (t >= 2.0 && t < 3.0) u.elevator -= 4.0 * constants::kDegToRad;
      if (t >= 5.0 && t < 6.0) u.aileron += 5.0 * constants::kDegToRad;
      if (t >= 6.0 && t < 7.0) u.aileron -= 5.0 * constants::kDegToRad;
      return u;
    };
    ControlInput frame_control = trim.controls;
    integrate::OdeFunction f = [&](double, const VecX& state) -> VecX {
      const StateVec xs = state;
      return model.derivative(xs, frame_control, env);
    };
    auto project = [](VecX& state) {
      state.segment<4>(StateIndex::kQuatW) =
          math::quatNormalize(state.segment<4>(StateIndex::kQuatW));
    };

    // Reference: DOPRI 5(4) at a very tight tolerance. Integrated over the same 1 s frames
    // used by the candidates so the control discontinuities land on frame boundaries.
    integrate::ToleranceSettings ref_tol;
    // 1e-12 relative on states of order 10^2 is an absolute accuracy of ~10^-10, which is
    // still ~10^6 times tighter than the tightest candidate below but stays comfortably
    // above the double-precision round-off floor of the right-hand side.
    ref_tol.rel_tol = 1e-12;
    ref_tol.abs_tol = 1e-14;
    ref_tol.min_step = 1e-6;
    ref_tol.max_step = 0.01;
    integrate::DormandPrince54 ref(ref_tol);
    ref.setProjection(project);
    VecX reference = trim.state;
    {
      const int frames = static_cast<int>(std::llround(horizon / kFrame));
      for (int i = 0; i < frames; ++i) {
        frame_control = controls(i * kFrame);
        ref.reset();
        reference = integrate::integrateTo(f, ref, i * kFrame, reference, (i + 1) * kFrame, 1e-3);
      }
    }

    std::cout << "-- Part 2: nonlinear 6-DOF aircraft vs a 1e-12 reference solution --------\n";
    std::cout << "   open-loop elevator/aileron doublets, horizon " << horizon << " s\n";
    util::CsvWriter air(output_dir + "/aircraft_comparison.csv",
                        {"integrator", "control_parameter", "dt_or_rtol", "state_error",
                         "position_error_m", "attitude_error_rad", "function_evals",
                         "wall_clock_s", "accepted_steps", "rejected_steps"});
    std::cout << std::left << std::setw(12) << "method" << std::right << std::setw(14)
              << "dt/rtol" << std::setw(16) << "state error" << std::setw(16) << "pos err[m]"
              << std::setw(16) << "att err[rad]" << std::setw(12) << "evals" << std::setw(14)
              << "wall[s]" << "\n";

    auto runAircraft = [&](integrate::Integrator& integ, double dt_init, const char* label,
                           double param) {
      integ.setProjection(project);
      integrate::IntegrationStats total;
      const auto t0 = std::chrono::steady_clock::now();
      VecX x = trim.state;
      const int frames = static_cast<int>(std::llround(horizon / kFrame));
      for (int i = 0; i < frames; ++i) {
        frame_control = controls(i * kFrame);
        integ.reset();
        integrate::IntegrationStats st;
        x = integrate::integrateTo(f, integ, i * kFrame, x, (i + 1) * kFrame, dt_init, &st);
        total.function_evals += st.function_evals;
        total.accepted_steps += st.accepted_steps;
        total.rejected_steps += st.rejected_steps;
      }
      const double wall =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      const double err = aircraftStateError(x, reference);
      const double perr =
          (x.segment<3>(StateIndex::kPosN) - reference.segment<3>(StateIndex::kPosN)).norm();
      const double aerr = math::boxMinus(x.segment<4>(StateIndex::kQuatW),
                                         reference.segment<4>(StateIndex::kQuatW)).norm();
      air.writeRow({0.0, 0.0, param, err, perr, aerr,
                    static_cast<double>(total.function_evals), wall,
                    static_cast<double>(total.accepted_steps),
                    static_cast<double>(total.rejected_steps)});
      std::cout << std::left << std::setw(12) << label << std::right << std::scientific
                << std::setprecision(4) << std::setw(14) << param << std::setw(16) << err
                << std::setw(16) << perr << std::setw(16) << aerr << std::fixed << std::setw(12)
                << total.function_evals << std::scientific << std::setw(14) << wall << "\n";
    };

    for (double dt : {0.05, 0.02, 0.01, 0.005, 0.002, 0.001}) {
      integrate::RungeKutta4 rk4;
      runAircraft(rk4, dt, "rk4", dt);
    }
    for (double rtol : {1e-4, 1e-6, 1e-8, 1e-10}) {
      integrate::ToleranceSettings tol;
      tol.rel_tol = rtol;
      tol.abs_tol = rtol * 1e-2;
      tol.min_step = 1e-6;
      tol.max_step = 0.05;
      integrate::DormandPrince54 dp(tol);
      runAircraft(dp, 0.01, "dopri54", rtol);
    }

    // Fix up the integrator name column (CsvWriter stores doubles; write a companion key file).
    {
      std::ofstream key(output_dir + "/README.txt");
      key << "convergence.csv        : RK4 order verification on the damped harmonic "
             "oscillator.\n"
             "                         Column 'integrator' is 0 (rk4). 'observed_order' is\n"
             "                         log(e_{k-1}/e_k)/log(h_{k-1}/h_k).\n"
             "dopri_tolerance.csv    : Dormand-Prince 5(4) error/cost vs requested rel_tol on\n"
             "                         the same problem.\n"
             "aircraft_comparison.csv: nonlinear 6-DOF errors against a rel_tol = 1e-13\n"
             "                         reference. Rows 1-6 are RK4 (dt_or_rtol = step size),\n"
             "                         rows 7-10 are DOPRI 5(4) (dt_or_rtol = rel_tol).\n";
    }

    std::cout << "results written to " << output_dir << "\n";
    std::cout << "=========================================================================\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "aether_integrators: " << e.what() << "\n";
    return 2;
  }
}
