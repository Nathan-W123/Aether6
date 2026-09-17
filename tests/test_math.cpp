/// \file test_math.cpp
/// \brief Quaternion algebra, rotation identities, matrix exponential and Riccati/LQR tests.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <cmath>
#include <random>

#include "aether/core/Constants.hpp"
#include "aether/math/LinearAlgebra.hpp"
#include "aether/math/NumericalDiff.hpp"
#include "aether/math/Optimize.hpp"
#include "aether/math/Rotation.hpp"

using namespace aether;
using Catch::Approx;

namespace {
Vec4 randomQuat(std::mt19937_64& rng) {
  std::normal_distribution<double> n(0.0, 1.0);
  return math::quatNormalize(Vec4(n(rng), n(rng), n(rng), n(rng)));
}
}  // namespace

TEST_CASE("quaternion normalisation and norm preservation", "[math][quaternion]") {
  std::mt19937_64 rng(12345);
  for (int i = 0; i < 200; ++i) {
    const Vec4 q = randomQuat(rng);
    REQUIRE(q.norm() == Approx(1.0).margin(1e-14));
    REQUIRE(q(0) >= 0.0);  // canonical sign convention
  }
  // A degenerate input must fall back to identity rather than produce NaNs.
  const Vec4 degenerate = math::quatNormalize(Vec4::Zero());
  REQUIRE(degenerate(0) == Approx(1.0));
  REQUIRE(degenerate.tail<3>().norm() == Approx(0.0));
}

TEST_CASE("quaternion product is associative and the conjugate inverts it",
          "[math][quaternion]") {
  std::mt19937_64 rng(999);
  for (int i = 0; i < 100; ++i) {
    const Vec4 a = randomQuat(rng), b = randomQuat(rng), c = randomQuat(rng);
    const Vec4 lhs = math::quatMultiply(math::quatMultiply(a, b), c);
    const Vec4 rhs = math::quatMultiply(a, math::quatMultiply(b, c));
    REQUIRE((lhs - rhs).norm() == Approx(0.0).margin(1e-13));

    const Vec4 identity = math::quatMultiply(a, math::quatConjugate(a));
    REQUIRE(identity(0) == Approx(1.0).margin(1e-13));
    REQUIRE(identity.tail<3>().norm() == Approx(0.0).margin(1e-13));
  }
}

TEST_CASE("rotation matrix from a quaternion is a proper orthogonal matrix",
          "[math][rotation]") {
  std::mt19937_64 rng(777);
  for (int i = 0; i < 200; ++i) {
    const Mat3 R = math::quatToRotation(randomQuat(rng));
    REQUIRE((R * R.transpose() - Mat3::Identity()).norm() == Approx(0.0).margin(1e-13));
    REQUIRE(R.determinant() == Approx(1.0).margin(1e-13));
  }
}

TEST_CASE("rotating with the sandwich product matches the rotation matrix",
          "[math][rotation]") {
  std::mt19937_64 rng(31337);
  std::normal_distribution<double> n(0.0, 5.0);
  for (int i = 0; i < 200; ++i) {
    const Vec4 q = randomQuat(rng);
    const Vec3 v(n(rng), n(rng), n(rng));
    const Vec3 by_quat = math::rotateBodyToNed(q, v);
    const Vec3 by_matrix = math::quatToRotation(q) * v;
    REQUIRE((by_quat - by_matrix).norm() == Approx(0.0).margin(1e-12));
    // Rotation preserves length, and the inverse rotation returns the original vector.
    REQUIRE(by_quat.norm() == Approx(v.norm()).margin(1e-12));
    REQUIRE((math::rotateNedToBody(q, by_quat) - v).norm() == Approx(0.0).margin(1e-12));
  }
}

TEST_CASE("Euler <-> quaternion round trip", "[math][rotation]") {
  std::mt19937_64 rng(4242);
  std::uniform_real_distribution<double> roll(-constants::kPi, constants::kPi);
  std::uniform_real_distribution<double> pitch(-1.5, 1.5);  // avoid the gimbal singularity
  for (int i = 0; i < 300; ++i) {
    const double r = roll(rng), p = pitch(rng), y = roll(rng);
    const Vec3 e = math::quatToEuler(math::eulerToQuat(r, p, y));
    REQUIRE(math::wrapPi(e.x() - r) == Approx(0.0).margin(1e-11));
    REQUIRE(e.y() == Approx(p).margin(1e-11));
    REQUIRE(math::wrapPi(e.z() - y) == Approx(0.0).margin(1e-11));
  }
}

TEST_CASE("known rotations have the documented sign convention", "[math][rotation]") {
  // Yaw +90 deg: the body x-axis (nose) points East in NED.
  const Vec4 q_yaw = math::eulerToQuat(0.0, 0.0, constants::kPi / 2);
  const Vec3 nose = math::rotateBodyToNed(q_yaw, Vec3(1, 0, 0));
  REQUIRE(nose.x() == Approx(0.0).margin(1e-12));
  REQUIRE(nose.y() == Approx(1.0).margin(1e-12));

  // Pitch +90 deg (nose up): the body x-axis points straight up, i.e. -Down.
  const Vec4 q_pitch = math::eulerToQuat(0.0, constants::kPi / 2, 0.0);
  const Vec3 up = math::rotateBodyToNed(q_pitch, Vec3(1, 0, 0));
  REQUIRE(up.z() == Approx(-1.0).margin(1e-12));

  // Roll +90 deg (right wing down): the body y-axis points straight down.
  const Vec4 q_roll = math::eulerToQuat(constants::kPi / 2, 0.0, 0.0);
  const Vec3 wing = math::rotateBodyToNed(q_roll, Vec3(0, 1, 0));
  REQUIRE(wing.z() == Approx(1.0).margin(1e-12));
}

TEST_CASE("exp/log maps and boxplus/boxminus are consistent", "[math][quaternion]") {
  std::mt19937_64 rng(2024);
  std::normal_distribution<double> n(0.0, 0.7);
  for (int i = 0; i < 300; ++i) {
    const Vec3 dtheta(n(rng), n(rng), n(rng));
    if (dtheta.norm() > constants::kPi * 0.95) continue;  // log is only unique inside the ball
    const Vec3 recovered = math::logMapQuat(math::expMapQuat(dtheta));
    REQUIRE((recovered - dtheta).norm() == Approx(0.0).margin(1e-11));

    const Vec4 q = randomQuat(rng);
    const Vec4 q2 = math::boxPlus(q, dtheta);
    REQUIRE(q2.norm() == Approx(1.0).margin(1e-14));
    REQUIRE((math::boxMinus(q2, q) - dtheta).norm() == Approx(0.0).margin(1e-11));
  }
  // The small-angle branch must match the general branch.
  const Vec3 tiny(1e-10, -2e-10, 3e-11);
  REQUIRE((math::logMapQuat(math::expMapQuat(tiny)) - tiny).norm() == Approx(0.0).margin(1e-18));
}

TEST_CASE("skew matrix reproduces the cross product", "[math]") {
  std::mt19937_64 rng(55);
  std::normal_distribution<double> n(0.0, 2.0);
  for (int i = 0; i < 100; ++i) {
    const Vec3 a(n(rng), n(rng), n(rng)), b(n(rng), n(rng), n(rng));
    REQUIRE((math::skew(a) * b - a.cross(b)).norm() == Approx(0.0).margin(1e-14));
    REQUIRE((math::skew(a) + math::skew(a).transpose()).norm() == Approx(0.0).margin(1e-15));
  }
}

TEST_CASE("wrapPi maps angles into the half-open interval -pi to pi", "[math]") {
  REQUIRE(math::wrapPi(0.0) == Approx(0.0));
  REQUIRE(math::wrapPi(constants::kPi) == Approx(constants::kPi));
  REQUIRE(math::wrapPi(-constants::kPi) == Approx(constants::kPi));
  REQUIRE(math::wrapPi(3.0 * constants::kPi) == Approx(constants::kPi));
  REQUIRE(math::wrapPi(2.0 * constants::kPi + 0.3) == Approx(0.3).margin(1e-12));
  REQUIRE(math::wrapPi(-2.0 * constants::kPi - 0.3) == Approx(-0.3).margin(1e-12));
}

TEST_CASE("matrix exponential matches analytic results", "[math][linalg]") {
  // Diagonal case.
  MatX D(3, 3);
  D.setZero();
  D.diagonal() << 1.0, -2.0, 0.5;
  const MatX E = math::expm(D);
  REQUIRE(E(0, 0) == Approx(std::exp(1.0)).epsilon(1e-12));
  REQUIRE(E(1, 1) == Approx(std::exp(-2.0)).epsilon(1e-12));
  REQUIRE(E(2, 2) == Approx(std::exp(0.5)).epsilon(1e-12));

  // Rotation generator: expm([[0,-w],[w,0]]*t) is a planar rotation by w*t.
  const double w = 1.7, t = 0.9;
  MatX S(2, 2);
  S << 0.0, -w * t, w * t, 0.0;
  const MatX R = math::expm(S);
  REQUIRE(R(0, 0) == Approx(std::cos(w * t)).epsilon(1e-12));
  REQUIRE(R(0, 1) == Approx(-std::sin(w * t)).epsilon(1e-12));

  // Nilpotent case has an exactly truncating series.
  MatX N(3, 3);
  N.setZero();
  N(0, 1) = 2.0;
  N(1, 2) = 3.0;
  const MatX EN = math::expm(N);
  REQUIRE(EN(0, 1) == Approx(2.0).epsilon(1e-12));
  REQUIRE(EN(0, 2) == Approx(3.0).epsilon(1e-12));  // 0.5 * 2 * 3
  REQUIRE(EN(1, 2) == Approx(3.0).epsilon(1e-12));

  // Large-norm input still works thanks to scaling and squaring.
  MatX big(2, 2);
  big << 0.0, 30.0, -30.0, 0.0;
  const MatX Ebig = math::expm(big);
  REQUIRE(Ebig(0, 0) == Approx(std::cos(30.0)).margin(1e-10));
  REQUIRE((Ebig * Ebig.transpose() - MatX::Identity(2, 2)).norm() == Approx(0.0).margin(1e-10));
}

TEST_CASE("Van Loan discretisation reproduces a scalar Ornstein-Uhlenbeck process",
          "[math][linalg]") {
  // dx = -a x dt + dw with intensity q: stationary variance q/(2a), and the exact discrete
  // process noise is (q/(2a)) * (1 - exp(-2 a dt)).
  const double a = 0.8, q = 2.5, dt = 0.37;
  MatX A(1, 1), Qc(1, 1);
  A(0, 0) = -a;
  Qc(0, 0) = q;
  const auto d = math::vanLoanDiscretize(A, Qc, dt);
  REQUIRE(d.Ad(0, 0) == Approx(std::exp(-a * dt)).epsilon(1e-12));
  REQUIRE(d.Qd(0, 0) ==
          Approx(q / (2.0 * a) * (1.0 - std::exp(-2.0 * a * dt))).epsilon(1e-11));
}

TEST_CASE("CARE solution is symmetric and positive semi-definite and stabilising",
          "[math][linalg][lqr]") {
  MatX A(3, 3);
  A << 0.0, 1.0, 0.0,
       0.0, 0.0, 1.0,
       -2.0, -3.0, -4.0;
  MatX B(3, 2);
  B << 0.0, 0.0,
       1.0, 0.0,
       0.0, 1.0;
  MatX Q = MatX::Identity(3, 3) * 3.0;
  MatX R = MatX::Identity(2, 2) * 0.7;

  const auto sol = math::solveCare(A, B, Q, R);
  REQUIRE((sol.P - sol.P.transpose()).norm() == Approx(0.0).margin(1e-12));
  Eigen::SelfAdjointEigenSolver<MatX> es(sol.P);
  REQUIRE(es.eigenvalues().minCoeff() > -1e-10);
  REQUIRE(sol.residual < 1e-9);

  const auto k = math::lqr(A, B, Q, R);
  for (Eigen::Index i = 0; i < k.closed_loop.size(); ++i)
    REQUIRE(k.closed_loop(i).real() < 0.0);
}

TEST_CASE("LQR on an open-loop unstable plant stabilises it", "[math][lqr]") {
  MatX A(2, 2);
  A << 1.0, 2.0, 0.0, 3.0;  // both eigenvalues in the right half plane
  MatX B(2, 1);
  B << 0.0, 1.0;
  MatX Q = MatX::Identity(2, 2);
  MatX R(1, 1);
  R(0, 0) = 0.1;
  const auto k = math::lqr(A, B, Q, R);
  REQUIRE(k.care_residual < 1e-9);
  for (Eigen::Index i = 0; i < k.closed_loop.size(); ++i)
    REQUIRE(k.closed_loop(i).real() < 0.0);
}

TEST_CASE("controllability rank detects an uncontrollable mode", "[math][linalg]") {
  MatX A(2, 2);
  A << -1.0, 0.0, 0.0, -2.0;
  MatX B(2, 1);
  B << 1.0, 0.0;  // the second mode cannot be reached
  REQUIRE(math::numericalRank(math::controllabilityMatrix(A, B)) == 1);
  MatX B2(2, 1);
  B2 << 1.0, 1.0;
  REQUIRE(math::numericalRank(math::controllabilityMatrix(A, B2)) == 2);
}

TEST_CASE("numerical Jacobian matches an analytic one", "[math][diff]") {
  auto f = [](const VecX& x) {
    VecX y(3);
    y(0) = x(0) * x(1) + std::sin(x(2));
    y(1) = std::exp(0.3 * x(0)) - x(2) * x(2);
    y(2) = x(0) + 2.0 * x(1) - 0.5 * x(2);
    return y;
  };
  VecX x(3);
  x << 0.7, -1.3, 0.45;
  MatX J(3, 3);
  J << x(1), x(0), std::cos(x(2)),
       0.3 * std::exp(0.3 * x(0)), 0.0, -2.0 * x(2),
       1.0, 2.0, -0.5;
  const MatX Jn = math::numericalJacobian(f, x);
  REQUIRE((Jn - J).cwiseAbs().maxCoeff() < 1e-8);
}

TEST_CASE("Levenberg-Marquardt solves a nonlinear least-squares problem", "[math][optimise]") {
  // Rosenbrock in residual form: r = [10(x1 - x0^2), 1 - x0], minimum at (1, 1).
  auto residual = [](const VecX& x) {
    VecX r(2);
    r(0) = 10.0 * (x(1) - x(0) * x(0));
    r(1) = 1.0 - x(0);
    return r;
  };
  VecX x0(2);
  x0 << -1.2, 1.0;
  const auto result = math::levenbergMarquardt(residual, x0);
  REQUIRE(result.x(0) == Approx(1.0).margin(1e-6));
  REQUIRE(result.x(1) == Approx(1.0).margin(1e-6));
  REQUIRE(result.residual_norm < 1e-6);
}
