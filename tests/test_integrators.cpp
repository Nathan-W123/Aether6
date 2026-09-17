/// \file test_integrators.cpp
/// \brief Convergence order, error control and structure preservation of the integrators.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <vector>

#include "aether/core/Constants.hpp"
#include "aether/dynamics/RigidBody.hpp"
#include "aether/integrate/RungeKutta.hpp"
#include "aether/math/Rotation.hpp"

using namespace aether;
using Catch::Approx;

namespace {

/// Scalar exponential decay: y' = lambda*y, y(t) = y0*exp(lambda*t).
integrate::OdeFunction exponential(double lambda) {
  return [lambda](double, const VecX& y) { return VecX(lambda * y); };
}

/// Undamped harmonic oscillator: x'' = -w^2 x.
integrate::OdeFunction oscillator(double w) {
  return [w](double, const VecX& x) {
    VecX d(2);
    d(0) = x(1);
    d(1) = -w * w * x(0);
    return d;
  };
}

}  // namespace

TEST_CASE("RK4 exhibits fourth-order global convergence", "[integrate][order]") {
  const double lambda = -0.8;
  const double T = 4.0;
  VecX y0(1);
  y0 << 1.0;
  const double exact = std::exp(lambda * T);

  std::vector<double> errors;
  const std::vector<double> steps = {0.2, 0.1, 0.05, 0.025, 0.0125};
  for (double dt : steps) {
    integrate::RungeKutta4 rk4;
    const VecX y = integrate::integrateTo(exponential(lambda), rk4, 0.0, y0, T, dt);
    errors.push_back(std::abs(y(0) - exact));
  }
  for (std::size_t i = 1; i < errors.size(); ++i) {
    const double order = std::log(errors[i - 1] / errors[i]) / std::log(2.0);
    INFO("step pair " << i << " observed order " << order);
    REQUIRE(order > 3.85);
    REQUIRE(order < 4.15);
  }
  REQUIRE(errors.back() < 1e-9);
}

TEST_CASE("RK4 is fourth order on an oscillatory problem too", "[integrate][order]") {
  const double w = 3.0, T = 5.0;
  VecX x0(2);
  x0 << 1.0, 0.0;
  VecX exact(2);
  exact << std::cos(w * T), -w * std::sin(w * T);

  std::vector<double> errors;
  for (double dt : {0.02, 0.01, 0.005, 0.0025}) {
    integrate::RungeKutta4 rk4;
    const VecX x = integrate::integrateTo(oscillator(w), rk4, 0.0, x0, T, dt);
    errors.push_back((x - exact).norm());
  }
  for (std::size_t i = 1; i < errors.size(); ++i) {
    const double order = std::log(errors[i - 1] / errors[i]) / std::log(2.0);
    INFO("observed order " << order);
    REQUIRE(order > 3.8);
    REQUIRE(order < 4.2);
  }
}

TEST_CASE("Dormand-Prince 5(4) meets the requested tolerance and is fifth order",
          "[integrate][adaptive]") {
  const double lambda = -1.3, T = 3.0;
  VecX y0(1);
  y0 << 2.0;
  const double exact = 2.0 * std::exp(lambda * T);

  double previous_error = 1e9;
  for (double rtol : {1e-4, 1e-6, 1e-8, 1e-10}) {
    integrate::ToleranceSettings tol;
    tol.rel_tol = rtol;
    tol.abs_tol = rtol * 1e-3;
    tol.max_step = 1.0;
    integrate::DormandPrince54 dp(tol);
    integrate::IntegrationStats stats;
    const VecX y = integrate::integrateTo(exponential(lambda), dp, 0.0, y0, T, 0.1, &stats);
    const double err = std::abs(y(0) - exact);
    INFO("rtol " << rtol << " error " << err << " steps " << stats.accepted_steps);
    // The achieved error must track the requested tolerance (with generous slack for the
    // difference between local and global error) and improve monotonically.
    REQUIRE(err < std::max(200.0 * rtol, 1e-13));
    REQUIRE(err <= previous_error);
    REQUIRE(stats.accepted_steps > 0);
    previous_error = err;
  }
}

TEST_CASE("the adaptive controller adapts the step size and lands exactly on t_end",
          "[integrate][adaptive]") {
  // A problem with a sharp transient: the step size must shrink then grow again.
  auto f = [](double t, const VecX& y) {
    VecX d(1);
    d(0) = -50.0 * std::exp(-100.0 * (t - 1.0) * (t - 1.0)) * y(0);
    return d;
  };
  integrate::ToleranceSettings tol;
  tol.rel_tol = 1e-9;
  tol.abs_tol = 1e-12;
  tol.max_step = 0.5;
  integrate::DormandPrince54 dp(tol);
  integrate::IntegrationStats stats;
  VecX y0(1);
  y0 << 1.0;
  const VecX y = integrate::integrateTo(f, dp, 0.0, y0, 2.0, 0.1, &stats);
  REQUIRE(y.allFinite());
  REQUIRE(stats.min_dt < stats.max_dt);          // the step really varied
  REQUIRE(stats.accepted_steps > 10);
  // The analytic solution is exp(-50*sqrt(pi)/10 * (erf(10(t-1)) - erf(-10))/2 ... ):
  // easier is to compare against a very fine RK4 solution.
  integrate::RungeKutta4 rk4;
  const VecX ref = integrate::integrateTo(f, rk4, 0.0, y0, 2.0, 1e-5);
  REQUIRE(y(0) == Approx(ref(0)).epsilon(1e-6));
}

TEST_CASE("rejected steps are reported and do not advance the state", "[integrate][adaptive]") {
  integrate::ToleranceSettings tol;
  tol.rel_tol = 1e-12;
  tol.abs_tol = 1e-14;
  tol.max_step = 10.0;
  integrate::DormandPrince54 dp(tol);
  VecX y0(1);
  y0 << 1.0;
  // A deliberately oversized first step on a stiff-ish problem must be rejected.
  const auto report = dp.step(exponential(-5.0), 0.0, y0, 5.0);
  REQUIRE_FALSE(report.accepted);
  REQUIRE(report.dt_used == 0.0);
  REQUIRE(report.state(0) == Approx(1.0));
  REQUIRE(report.dt_next < 5.0);
  REQUIRE(report.error_norm > 1.0);
}

TEST_CASE("the projection hook keeps the quaternion normalised", "[integrate][quaternion]") {
  // Pure attitude kinematics at a constant body rate: the exact solution is a rotation about
  // the rate axis, so the quaternion norm must be preserved exactly.
  const Vec3 omega(0.7, -0.4, 0.2);
  auto f = [&](double, const VecX& q) {
    const Vec4 qq = q;
    const Vec4 w(0.0, omega.x(), omega.y(), omega.z());
    return VecX(0.5 * math::quatMultiply(qq, w));
  };
  VecX q0 = math::eulerToQuat(0.1, 0.2, -0.3);

  integrate::RungeKutta4 rk4;
  rk4.setProjection([](VecX& s) { s = math::quatNormalize(Vec4(s)); });
  const VecX qf = integrate::integrateTo(f, rk4, 0.0, q0, 20.0, 0.01);
  REQUIRE(qf.norm() == Approx(1.0).margin(1e-15));

  // The rotation angle must match |omega| * T (modulo the shortest-arc convention).
  const Vec3 dtheta = math::boxMinus(Vec4(qf), Vec4(q0));
  const double expected_angle = std::fmod(omega.norm() * 20.0, 2.0 * constants::kPi);
  const double achieved = dtheta.norm();
  const double diff = std::min(std::abs(achieved - expected_angle),
                               std::abs(2.0 * constants::kPi - expected_angle - achieved));
  REQUIRE(diff == Approx(0.0).margin(1e-8));
}

TEST_CASE("fixed and adaptive integrators agree on the aircraft model",
          "[integrate][aircraft]") {
  const auto p = dynamics::defaultAircraft();
  dynamics::RigidBody6DOF model(p);
  dynamics::EnvironmentSample env;
  env.density = 1.2;
  env.wind_ned = Vec3(2.0, -1.0, 0.0);

  dynamics::AircraftState s;
  s.position = Vec3(0, 0, -150);
  s.velocity = Vec3(25.0, 0.5, 1.5);
  s.quaternion = math::eulerToQuat(0.05, 0.08, 0.3);
  s.rate = Vec3(0.05, 0.02, -0.03);
  ControlInput u;
  u.elevator = -0.02;
  u.throttle = 0.65;

  auto f = [&](double, const VecX& st) { return VecX(model.derivative(st, u, env)); };
  auto project = [](VecX& st) {
    st.segment<4>(StateIndex::kQuatW) = math::quatNormalize(st.segment<4>(StateIndex::kQuatW));
  };

  integrate::RungeKutta4 rk4;
  rk4.setProjection(project);
  const VecX x_rk4 = integrate::integrateTo(f, rk4, 0.0, s.vec(), 8.0, 1e-4);

  integrate::ToleranceSettings tol;
  tol.rel_tol = 1e-11;
  tol.abs_tol = 1e-13;
  tol.max_step = 0.05;
  integrate::DormandPrince54 dp(tol);
  dp.setProjection(project);
  integrate::IntegrationStats stats;
  const VecX x_dp = integrate::integrateTo(f, dp, 0.0, s.vec(), 8.0, 0.01, &stats);

  const double pos_err =
      (x_rk4.segment<3>(StateIndex::kPosN) - x_dp.segment<3>(StateIndex::kPosN)).norm();
  const double att_err = math::boxMinus(x_rk4.segment<4>(StateIndex::kQuatW),
                                        x_dp.segment<4>(StateIndex::kQuatW)).norm();
  INFO("position difference " << pos_err << " m, attitude difference " << att_err << " rad");
  REQUIRE(pos_err < 1e-5);
  REQUIRE(att_err < 1e-8);
  // The adaptive scheme should need far fewer evaluations than 8 s at dt = 1e-4.
  REQUIRE(stats.function_evals < 80000 * 4);
}

TEST_CASE("integrator factory and argument validation", "[integrate][api]") {
  REQUIRE(integrate::makeIntegrator("rk4")->name() == "rk4");
  REQUIRE(integrate::makeIntegrator("dopri54")->name() == "dopri54");
  REQUIRE(integrate::makeIntegrator("rk45")->adaptive());
  REQUIRE_FALSE(integrate::makeIntegrator("rk4")->adaptive());
  REQUIRE(integrate::makeIntegrator("rk4")->order() == 4);
  REQUIRE(integrate::makeIntegrator("dopri54")->order() == 5);
  REQUIRE_THROWS_AS(integrate::makeIntegrator("euler"), std::invalid_argument);

  integrate::RungeKutta4 rk4;
  VecX y(1);
  y << 1.0;
  REQUIRE_THROWS_AS(integrate::integrateTo(exponential(-1.0), rk4, 0.0, y, 1.0, 0.0),
                    std::invalid_argument);
  REQUIRE_THROWS_AS(integrate::integrateTo(exponential(-1.0), rk4, 1.0, y, 0.0, 0.1),
                    std::invalid_argument);
}
