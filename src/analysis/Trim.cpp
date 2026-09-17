#include "aether/analysis/Trim.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "aether/env/Atmosphere.hpp"
#include "aether/math/Optimize.hpp"
#include "aether/math/Rotation.hpp"

namespace aether::analysis {

namespace {
constexpr int kNumUnknowns = 8;
const char* const kResidualNames[] = {"udot", "vdot", "wdot", "pdot",
                                      "qdot", "rdot", "climb_rate", "sideslip"};
const char* const kResidualUnits[] = {"m/s^2",   "m/s^2",   "m/s^2", "rad/s^2",
                                      "rad/s^2", "rad/s^2", "m/s",   "rad"};
}  // namespace

const char* const* TrimResult::residualNames() { return kResidualNames; }
const char* const* TrimResult::residualUnits() { return kResidualUnits; }

void TrimSolver::unpack(const TrimSpec& spec, const VecX& z, StateVec* x,
                        ControlInput* u) const {
  const double alpha = z(0);
  const double beta = z(1);
  const double phi = z(2);
  const double theta = z(3);

  dynamics::AircraftState s;
  s.position = Vec3(0.0, 0.0, -spec.altitude);
  s.velocity = spec.airspeed * Vec3(std::cos(alpha) * std::cos(beta), std::sin(beta),
                                    std::sin(alpha) * std::cos(beta));
  s.quaternion = math::eulerToQuat(phi, theta, 0.0);
  // Steady coordinated turn: body rates implied by a constant heading rate with
  // phi_dot = theta_dot = 0.
  const double psidot = spec.turn_rate;
  s.rate = Vec3(-psidot * std::sin(theta), psidot * std::cos(theta) * std::sin(phi),
                psidot * std::cos(theta) * std::cos(phi));
  *x = s.vec();

  u->elevator = z(4);
  u->aileron = z(5);
  u->rudder = z(6);
  u->throttle = z(7);
}

VecX TrimSolver::residual(const TrimSpec& spec, const VecX& z) const {
  StateVec x;
  ControlInput u;
  unpack(spec, z, &x, &u);

  dynamics::EnvironmentSample env;
  env.wind_ned = Vec3::Zero();  // trim is defined in still air
  env.density = spec.density_override > 0.0
                    ? spec.density_override
                    : env::Atmosphere().density(spec.altitude);

  const StateVec dx = model_.derivative(x, u, env);

  VecX r(kNumUnknowns);
  r.segment<3>(0) = dx.segment<3>(StateIndex::kVelU);   // udot, vdot, wdot
  r.segment<3>(3) = dx.segment<3>(StateIndex::kRateP);  // pdot, qdot, rdot
  // Climb-rate residual: h_dot = -p_D_dot must equal Va*sin(gamma) in still air.
  r(6) = -dx(StateIndex::kPosD) - spec.airspeed * std::sin(spec.flight_path_angle);
  r(7) = z(1) - spec.sideslip;
  return r;
}

VecX TrimSolver::penalisedResidual(const TrimSpec& spec, const VecX& z) const {
  const VecX physical = residual(spec, z);
  const auto& a = model_.params().actuators;

  // One-sided quadratic-free (linear) penalties, weighted well above the physical residual
  // scale so a bound violation always dominates.
  constexpr double kWeight = 50.0;
  auto penalty = [](double value, double lo, double hi) {
    return std::max(0.0, value - hi) + std::max(0.0, lo - value);
  };

  VecX out(kNumUnknowns + 6);
  out.head(kNumUnknowns) = physical;
  out(kNumUnknowns + 0) = kWeight * penalty(z(4), a.elevator_min, a.elevator_max);
  out(kNumUnknowns + 1) = kWeight * penalty(z(5), a.aileron_min, a.aileron_max);
  out(kNumUnknowns + 2) = kWeight * penalty(z(6), a.rudder_min, a.rudder_max);
  out(kNumUnknowns + 3) = kWeight * penalty(z(7), a.throttle_min, a.throttle_max);
  // Keep the search inside a sensible aerodynamic envelope as well.
  out(kNumUnknowns + 4) = kWeight * penalty(z(0), -0.35, 0.35);  // alpha, +-20 deg
  out(kNumUnknowns + 5) = kWeight * penalty(z(1), -0.26, 0.26);  // beta, +-15 deg
  return out;
}

TrimResult TrimSolver::solve(const TrimSpec& spec, double tolerance) const {
  // Initial guess: small positive alpha, wings level, pitch = alpha + gamma, mid throttle.
  VecX z0(kNumUnknowns);
  z0 << 0.05, spec.sideslip, 0.0, 0.05 + spec.flight_path_angle, 0.0, 0.0, 0.0, 0.5;

  math::LmOptions opts;
  opts.max_iterations = 400;
  opts.cost_tol = 1e-24;
  opts.jacobian_epsilon = 1e-7;
  opts.jacobian_scale = 1e-4;

  const auto lm = math::levenbergMarquardt(
      [&](const VecX& z) { return penalisedResidual(spec, z); }, z0, opts);

  TrimResult out;
  out.alpha = lm.x(0);
  out.beta = lm.x(1);
  out.phi = lm.x(2);
  out.theta = lm.x(3);
  unpack(spec, lm.x, &out.state, &out.controls);
  out.density = spec.density_override > 0.0
                    ? spec.density_override
                    : env::Atmosphere().density(spec.altitude);
  // Report only the physical residuals; the penalty terms are a solver device.
  out.residual = lm.residual.head(kNumUnknowns);
  out.residual_norm = out.residual.norm();
  out.residual_inf = out.residual.lpNorm<Eigen::Infinity>();
  out.iterations = lm.iterations;
  out.converged = lm.residual_inf < tolerance;
  out.message = out.converged ? "converged" : ("not converged: " + lm.message);

  // A trim point with a saturated actuator is not a usable equilibrium; flag it.
  const auto& a = model_.params().actuators;
  const bool saturated =
      out.controls.elevator <= a.elevator_min + 1e-9 ||
      out.controls.elevator >= a.elevator_max - 1e-9 ||
      out.controls.throttle <= a.throttle_min + 1e-9 ||
      out.controls.throttle >= a.throttle_max - 1e-9;
  if (saturated) {
    out.converged = false;
    out.message = "trim requires a saturated actuator (flight condition not achievable)";
  }
  return out;
}

}  // namespace aether::analysis
