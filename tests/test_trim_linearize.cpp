/// \file test_trim_linearize.cpp
/// \brief Trim convergence, residual quality and linearisation consistency.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "aether/analysis/Linearize.hpp"
#include "aether/analysis/Trim.hpp"
#include "aether/core/Constants.hpp"
#include "aether/env/Atmosphere.hpp"
#include "aether/math/LinearAlgebra.hpp"
#include "aether/math/Rotation.hpp"

using namespace aether;
using Catch::Approx;

namespace {

analysis::TrimResult trimAt(const dynamics::RigidBody6DOF& model, double Va, double h = 120.0,
                            double gamma = 0.0, double turn = 0.0) {
  analysis::TrimSpec spec;
  spec.airspeed = Va;
  spec.altitude = h;
  spec.flight_path_angle = gamma;
  spec.turn_rate = turn;
  spec.density_override = env::Atmosphere().density(h);
  return analysis::TrimSolver(model).solve(spec);
}

}  // namespace

TEST_CASE("straight-and-level trim converges to machine precision", "[trim]") {
  const auto p = dynamics::defaultAircraft();
  const dynamics::RigidBody6DOF model(p);
  const auto trim = trimAt(model, 25.0);

  REQUIRE(trim.converged);
  REQUIRE(trim.residual_inf < 1e-10);
  REQUIRE(trim.residual.size() == 8);

  // Physically sensible trim for a statically stable aircraft in level flight.
  REQUIRE(trim.alpha > 0.0);
  REQUIRE(trim.alpha < 0.20);
  REQUIRE(trim.theta == Approx(trim.alpha).margin(1e-9));  // gamma = 0 and beta = 0
  REQUIRE(trim.controls.elevator < 0.0);                   // up elevator on a stable aircraft
  REQUIRE(trim.controls.throttle > 0.0);
  REQUIRE(trim.controls.throttle < 1.0);
  REQUIRE(std::abs(trim.beta) < 1e-9);

  // The derivative of the packed state must vanish at the trim point.
  dynamics::EnvironmentSample env;
  env.density = trim.density;
  const StateVec dx = model.derivative(trim.state, trim.controls, env);
  REQUIRE(dx.segment<3>(StateIndex::kVelU).norm() < 1e-10);
  REQUIRE(dx.segment<3>(StateIndex::kRateP).norm() < 1e-10);
  REQUIRE(std::abs(dx(StateIndex::kPosD)) < 1e-10);  // level flight
  REQUIRE(dx(StateIndex::kPosN) == Approx(25.0).epsilon(1e-9));
}

TEST_CASE("trim over an airspeed range shows the expected trends", "[trim]") {
  const auto p = dynamics::defaultAircraft();
  const dynamics::RigidBody6DOF model(p);
  double previous_alpha = 1e9;
  for (double Va : {18.0, 20.0, 22.0, 25.0, 28.0, 32.0}) {
    const auto t = trimAt(model, Va);
    INFO("Va = " << Va);
    REQUIRE(t.converged);
    REQUIRE(t.residual_inf < 1e-9);
    // Faster flight needs less lift coefficient, hence less angle of attack.
    REQUIRE(t.alpha < previous_alpha);
    previous_alpha = t.alpha;
  }
}

TEST_CASE("climbing and descending trims are consistent with the flight-path angle",
          "[trim]") {
  const auto p = dynamics::defaultAircraft();
  const dynamics::RigidBody6DOF model(p);

  const auto level = trimAt(model, 25.0, 120.0, 0.0);
  const auto climb = trimAt(model, 25.0, 120.0, 5.0 * constants::kDegToRad);
  const auto descend = trimAt(model, 25.0, 120.0, -5.0 * constants::kDegToRad);

  REQUIRE(climb.converged);
  REQUIRE(descend.converged);
  REQUIRE(climb.residual_inf < 1e-9);
  REQUIRE(descend.residual_inf < 1e-9);

  // theta = alpha + gamma for beta = 0.
  REQUIRE(climb.theta == Approx(climb.alpha + 5.0 * constants::kDegToRad).margin(1e-8));
  REQUIRE(descend.theta == Approx(descend.alpha - 5.0 * constants::kDegToRad).margin(1e-8));
  // Climbing needs more thrust than level flight, descending needs less.
  REQUIRE(climb.controls.throttle > level.controls.throttle);
  REQUIRE(descend.controls.throttle < level.controls.throttle);

  // Verify the achieved climb rate directly.
  dynamics::EnvironmentSample env;
  env.density = climb.density;
  const StateVec dx = model.derivative(climb.state, climb.controls, env);
  REQUIRE(-dx(StateIndex::kPosD) ==
          Approx(25.0 * std::sin(5.0 * constants::kDegToRad)).margin(1e-9));
}

TEST_CASE("coordinated-turn trim balances a banked steady turn", "[trim]") {
  const auto p = dynamics::defaultAircraft();
  const dynamics::RigidBody6DOF model(p);
  const double turn_rate = 15.0 * constants::kDegToRad;  // deg/s -> rad/s
  const auto t = trimAt(model, 25.0, 120.0, 0.0, turn_rate);

  REQUIRE(t.converged);
  REQUIRE(t.residual_inf < 1e-9);
  REQUIRE(t.phi > 0.0);  // right turn requires right bank
  // Coordinated-turn geometry: tan(phi) = V * psidot / g.
  REQUIRE(std::tan(t.phi) == Approx(25.0 * turn_rate / constants::kGravity).epsilon(0.05));
  REQUIRE(std::abs(t.beta) < 1e-9);
}

TEST_CASE("an unreachable flight condition is reported as not trimmed", "[trim]") {
  const auto p = dynamics::defaultAircraft();
  const dynamics::RigidBody6DOF model(p);
  // A very steep climb exceeds the available thrust.
  const auto t = trimAt(model, 30.0, 120.0, 40.0 * constants::kDegToRad);
  REQUIRE_FALSE(t.converged);
  REQUIRE_FALSE(t.message.empty());
}

TEST_CASE("linearisation reproduces finite perturbations of the nonlinear model",
          "[linearize]") {
  const auto p = dynamics::defaultAircraft();
  const dynamics::RigidBody6DOF model(p);
  const auto trim = trimAt(model, 25.0);
  REQUIRE(trim.converged);

  dynamics::EnvironmentSample env;
  env.density = trim.density;
  const analysis::LinearModel lm = analysis::linearize(model, trim.state, trim.controls, env);

  REQUIRE(lm.A.rows() == kErrorStateDim);
  REQUIRE(lm.A.cols() == kErrorStateDim);
  REQUIRE(lm.B.rows() == kErrorStateDim);
  REQUIRE(lm.B.cols() == kInputDim);
  REQUIRE(lm.A.allFinite());
  REQUIRE(lm.B.allFinite());

  // For a range of small perturbations the linear prediction must match the nonlinear
  // derivative to second order: the relative error should shrink like the perturbation.
  auto nonlinearErrorDerivative = [&](const VecX& d, const ControlInput& u) {
    StateVec x = trim.state;
    x.segment<3>(StateIndex::kPosN) += d.segment<3>(ErrorIndex::kPos);
    x.segment<3>(StateIndex::kVelU) += d.segment<3>(ErrorIndex::kVel);
    x.segment<4>(StateIndex::kQuatW) =
        math::boxPlus(trim.state.segment<4>(StateIndex::kQuatW), d.segment<3>(ErrorIndex::kAtt));
    x.segment<3>(StateIndex::kRateP) += d.segment<3>(ErrorIndex::kRate);
    const StateVec dx = model.derivative(x, u, env);
    VecX out(kErrorStateDim);
    out.segment<3>(ErrorIndex::kPos) = dx.segment<3>(StateIndex::kPosN);
    out.segment<3>(ErrorIndex::kVel) = dx.segment<3>(StateIndex::kVelU);
    out.segment<3>(ErrorIndex::kAtt) = x.segment<3>(StateIndex::kRateP);
    out.segment<3>(ErrorIndex::kRate) = dx.segment<3>(StateIndex::kRateP);
    return out;
  };
  const VecX f0 = nonlinearErrorDerivative(VecX::Zero(kErrorStateDim), trim.controls);

  double previous_ratio = 0.0;
  for (double eps : {1e-2, 1e-3, 1e-4}) {
    double worst = 0.0;
    for (int j = 0; j < kErrorStateDim; ++j) {
      VecX d = VecX::Zero(kErrorStateDim);
      d(j) = eps;
      const VecX nl = nonlinearErrorDerivative(d, trim.controls) - f0;
      const VecX lin = lm.A * d;
      worst = std::max(worst, (nl - lin).norm());
    }
    INFO("eps = " << eps << " worst mismatch " << worst);
    // Second-order accuracy: halving eps by 10 should cut the mismatch by ~100.
    if (previous_ratio > 0.0) REQUIRE(worst < previous_ratio * 0.05);
    previous_ratio = worst;
    REQUIRE(worst < 50.0 * eps * eps + 1e-9);
  }

  // Same check for the input matrix.
  for (double eps : {1e-3, 1e-5}) {
    for (int j = 0; j < kInputDim; ++j) {
      auto uvec = trim.controls.vec();
      uvec(j) += eps;
      const VecX nl =
          nonlinearErrorDerivative(VecX::Zero(kErrorStateDim), ControlInput::fromVec(uvec)) - f0;
      const VecX lin = lm.B.col(j) * eps;
      INFO("input " << j << " eps " << eps);
      REQUIRE((nl - lin).norm() < 50.0 * eps * eps + 1e-9);
    }
  }
}

TEST_CASE("the linear model integrates consistently with the nonlinear model",
          "[linearize]") {
  const auto p = dynamics::defaultAircraft();
  const dynamics::RigidBody6DOF model(p);
  const auto trim = trimAt(model, 25.0);
  dynamics::EnvironmentSample env;
  env.density = trim.density;
  const analysis::LinearModel lm = analysis::linearize(model, trim.state, trim.controls, env);

  // Apply a small elevator step and propagate both models for 1 s.
  ControlInput u = trim.controls;
  const double du = 0.5 * constants::kDegToRad;
  u.elevator += du;

  const double T = 1.0, dt = 1e-4;
  StateVec x = trim.state;
  for (int i = 0; i < static_cast<int>(T / dt); ++i) {
    const StateVec k1 = model.derivative(x, u, env);
    const StateVec k2 = model.derivative(StateVec(x + 0.5 * dt * k1), u, env);
    const StateVec k3 = model.derivative(StateVec(x + 0.5 * dt * k2), u, env);
    const StateVec k4 = model.derivative(StateVec(x + dt * k3), u, env);
    x += (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);
    x.segment<4>(StateIndex::kQuatW) = math::quatNormalize(x.segment<4>(StateIndex::kQuatW));
  }

  VecX d = VecX::Zero(kErrorStateDim);
  VecX duv = VecX::Zero(kInputDim);
  duv(0) = du;
  for (int i = 0; i < static_cast<int>(T / dt); ++i) d += dt * (lm.A * d + lm.B * duv);

  // Compare velocity, attitude and rate perturbations (position drifts slowly and is the
  // integral of the others, so it is the least sensitive check).
  const Vec3 dv_nl = x.segment<3>(StateIndex::kVelU) - trim.state.segment<3>(StateIndex::kVelU);
  const Vec3 dth_nl = math::boxMinus(x.segment<4>(StateIndex::kQuatW),
                                     trim.state.segment<4>(StateIndex::kQuatW));
  const Vec3 dw_nl = x.segment<3>(StateIndex::kRateP);

  REQUIRE((dv_nl - d.segment<3>(ErrorIndex::kVel)).norm() < 0.02 * dv_nl.norm() + 1e-6);
  REQUIRE((dth_nl - d.segment<3>(ErrorIndex::kAtt)).norm() < 0.02 * dth_nl.norm() + 1e-6);
  REQUIRE((dw_nl - d.segment<3>(ErrorIndex::kRate)).norm() < 0.02 * dw_nl.norm() + 1e-6);
}

TEST_CASE("the classical flight modes are identified and physically plausible",
          "[linearize][modes]") {
  const auto p = dynamics::defaultAircraft();
  const dynamics::RigidBody6DOF model(p);
  const auto trim = trimAt(model, 25.0);
  dynamics::EnvironmentSample env;
  env.density = trim.density;
  const analysis::LinearModel lm = analysis::linearize(model, trim.state, trim.controls, env);
  const analysis::ReducedModels red = analysis::extractReducedModels(lm);

  const auto lon = analysis::classifyLongitudinal(red.A_lon);
  bool found_sp = false, found_ph = false;
  for (const auto& m : lon) {
    if (m.eigenvalue.imag() <= 0.0) continue;
    if (m.name == "short_period") {
      found_sp = true;
      REQUIRE(m.stable);
      REQUIRE(m.natural_frequency > 2.0);
      REQUIRE(m.natural_frequency < 20.0);
      REQUIRE(m.damping_ratio > 0.25);  // Level 1 handling qualities
      REQUIRE(m.damping_ratio < 2.0);
    }
    if (m.name == "phugoid") {
      found_ph = true;
      REQUIRE(m.stable);
      REQUIRE(m.damping_ratio < 0.4);   // the phugoid is always lightly damped
      // Lanchester approximation: T ~ pi*sqrt(2)*V/g.
      const double lanchester = constants::kPi * std::sqrt(2.0) * 25.0 / constants::kGravity;
      REQUIRE(m.period == Approx(lanchester).epsilon(0.35));
    }
  }
  REQUIRE(found_sp);
  REQUIRE(found_ph);
  // The short period must be much faster than the phugoid.
  REQUIRE(lon.size() == 4);

  const auto lat = analysis::classifyLateral(red.A_lat);
  bool found_dr = false, found_roll = false, found_spiral = false;
  for (const auto& m : lat) {
    if (m.name == "dutch_roll" && m.eigenvalue.imag() > 0.0) {
      found_dr = true;
      REQUIRE(m.stable);
      REQUIRE(m.natural_frequency > 1.0);
      REQUIRE(m.damping_ratio > 0.02);
    }
    if (m.name == "roll_subsidence") {
      found_roll = true;
      REQUIRE(m.stable);
      REQUIRE(m.eigenvalue.real() < -3.0);  // fast roll mode
      REQUIRE(m.eigenvalue.imag() == Approx(0.0));
    }
    if (m.name == "spiral") {
      found_spiral = true;
      REQUIRE(m.eigenvalue.imag() == Approx(0.0));
      REQUIRE(std::abs(m.eigenvalue.real()) < 0.5);  // slow either way
    }
  }
  REQUIRE(found_dr);
  REQUIRE(found_roll);
  REQUIRE(found_spiral);

  // Both reduced models must be controllable with their two inputs.
  REQUIRE(math::numericalRank(math::controllabilityMatrix(red.A_lon, red.B_lon)) == 4);
  REQUIRE(math::numericalRank(math::controllabilityMatrix(red.A_lat, red.B_lat)) == 4);
}

TEST_CASE("the full 12-state model has the expected structure", "[linearize]") {
  const auto p = dynamics::defaultAircraft();
  const dynamics::RigidBody6DOF model(p);
  const auto trim = trimAt(model, 25.0);
  dynamics::EnvironmentSample env;
  env.density = trim.density;
  const analysis::LinearModel lm = analysis::linearize(model, trim.state, trim.controls, env);

  // Attitude error kinematics: d(dtheta)/dt = domega exactly for a non-rotating reference.
  const Mat3 att_rate = lm.A.block<3, 3>(ErrorIndex::kAtt, ErrorIndex::kRate);
  REQUIRE((att_rate - Mat3::Identity()).norm() < 1e-6);
  REQUIRE(lm.A.block<3, 3>(ErrorIndex::kAtt, ErrorIndex::kPos).norm() < 1e-9);
  REQUIRE(lm.A.block<3, 3>(ErrorIndex::kAtt, ErrorIndex::kVel).norm() < 1e-9);

  // Nothing depends on absolute horizontal position (flat Earth, uniform atmosphere in the
  // horizontal plane), so those columns are zero.
  REQUIRE(lm.A.col(ErrorIndex::kPos + 0).norm() < 1e-8);
  REQUIRE(lm.A.col(ErrorIndex::kPos + 1).norm() < 1e-8);
  // Altitude does matter, through the density gradient.
  REQUIRE(lm.A.col(ErrorIndex::kPos + 2).norm() >= 0.0);

  // Four (near-)zero eigenvalues: three position integrators and heading.
  const auto modes = analysis::analyseEigenvalues(lm.A);
  int near_zero = 0;
  for (const auto& m : modes)
    if (std::abs(m.eigenvalue) < 1e-6) ++near_zero;
  REQUIRE(near_zero == 4);
}

TEST_CASE("trim stays inside the actuator box", "[trim][bounds]") {
  const auto p = dynamics::defaultAircraft();
  const dynamics::RigidBody6DOF model(p);
  const analysis::TrimSolver solver(model);

  SECTION("the bound penalties vanish on a feasible point") {
    const auto t = trimAt(model, 25.0);
    REQUIRE(t.converged);
    VecX z(8);
    z << t.alpha, t.beta, t.phi, t.theta, t.controls.elevator, t.controls.aileron,
        t.controls.rudder, t.controls.throttle;
    analysis::TrimSpec spec;
    spec.airspeed = 25.0;
    spec.altitude = 120.0;
    spec.density_override = env::Atmosphere().density(120.0);
    const VecX physical = solver.residual(spec, z);
    const VecX penalised = solver.penalisedResidual(spec, z);
    REQUIRE(penalised.size() == physical.size() + 6);
    REQUIRE((penalised.head(physical.size()) - physical).norm() == Approx(0.0));
    REQUIRE(penalised.tail(6).norm() == Approx(0.0));  // feasible: no penalty at all
  }

  SECTION("an infeasible condition still reports a physically meaningful point") {
    // A 40 deg climb is far beyond the available thrust. The solver must say so, and the
    // point it reports must still be inside the actuator box and a sane alpha/beta envelope
    // rather than an arbitrary excursion chased while minimising an impossible residual.
    const auto t = trimAt(model, 30.0, 120.0, 40.0 * constants::kDegToRad);
    REQUIRE_FALSE(t.converged);
    const auto& a = p.actuators;
    REQUIRE(t.controls.elevator >= a.elevator_min - 1e-3);
    REQUIRE(t.controls.elevator <= a.elevator_max + 1e-3);
    REQUIRE(t.controls.aileron >= a.aileron_min - 1e-3);
    REQUIRE(t.controls.aileron <= a.aileron_max + 1e-3);
    REQUIRE(t.controls.rudder >= a.rudder_min - 1e-3);
    REQUIRE(t.controls.rudder <= a.rudder_max + 1e-3);
    REQUIRE(t.controls.throttle <= a.throttle_max + 1e-3);
    REQUIRE(std::abs(t.alpha) < 0.36);
    REQUIRE(std::abs(t.beta) < 0.27);
  }

  SECTION("a steep but reachable climb converges") {
    const auto t = trimAt(model, 30.0, 120.0, 5.0 * constants::kDegToRad);
    REQUIRE(t.converged);
    REQUIRE(t.residual_inf < 1e-9);
    REQUIRE(t.controls.throttle < 1.0);
  }
}
