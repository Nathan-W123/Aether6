#include "aether/env/Wind.hpp"

#include <algorithm>
#include <cmath>

#include "aether/core/Constants.hpp"
#include "aether/math/LinearAlgebra.hpp"
#include "aether/math/Rotation.hpp"

namespace aether::env {
namespace {

constexpr double kMetersPerFoot = 0.3048;

/// Draw a zero-mean sample with covariance Qd using an eigen-decomposition
/// (robust when Qd is only positive *semi*-definite, which happens for the
/// second-order Dryden filters where the noise enters only one state).
template <int N>
Eigen::Matrix<double, N, 1> sampleCovariance(const Eigen::Matrix<double, N, N>& Qd,
                                             util::Rng& rng) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, N, N>> es(
      0.5 * (Qd + Qd.transpose()));
  Eigen::Matrix<double, N, 1> lambda = es.eigenvalues();
  for (int i = 0; i < N; ++i) lambda(i) = lambda(i) > 0.0 ? std::sqrt(lambda(i)) : 0.0;
  Eigen::Matrix<double, N, 1> z;
  for (int i = 0; i < N; ++i) z(i) = rng.gaussian();
  return es.eigenvectors() * lambda.asDiagonal() * z;
}

}  // namespace

WindModel::WindModel(const WindConfig& config, std::uint64_t seed)
    : cfg_(config), rng_(seed) {
  sigma_eff_ = cfg_.sigma;
  length_eff_ = cfg_.length_scale;
}

void WindModel::reset() {
  xu_.setZero();
  xv_.setZero();
  xw_.setZero();
  gust_body_.setZero();
}

void WindModel::updateScaling(double altitude_agl) {
  if (!cfg_.altitude_scaled) {
    sigma_eff_ = cfg_.sigma;
    length_eff_ = cfg_.length_scale;
    return;
  }
  // MIL-F-8785C low-altitude (h < 1000 ft) turbulence model. Formulas are stated in
  // feet, so convert in and out. The height is clamped to keep the fit in its valid range.
  const double h_ft = std::clamp(altitude_agl / kMetersPerFoot, 10.0, 1000.0);
  const double w20_fps = cfg_.w20 / kMetersPerFoot;
  const double denom = std::pow(0.177 + 0.000823 * h_ft, 1.2);
  const double Lw_ft = h_ft;
  const double Lu_ft = h_ft / denom;
  const double sigma_w_fps = 0.1 * w20_fps;
  const double sigma_u_fps = sigma_w_fps / std::pow(0.177 + 0.000823 * h_ft, 0.4);

  length_eff_ = Vec3(Lu_ft, Lu_ft, Lw_ft) * kMetersPerFoot;
  sigma_eff_ = Vec3(sigma_u_fps, sigma_u_fps, sigma_w_fps) * kMetersPerFoot;
}

void WindModel::step(double dt, double airspeed, double altitude_agl) {
  if (!cfg_.enable_turbulence || dt <= 0.0) {
    gust_body_.setZero();
    return;
  }
  updateScaling(altitude_agl);
  const double Va = std::max(airspeed, constants::kMinAirspeed);

  // --- u axis: first-order filter -------------------------------------------------
  {
    const double L = std::max(length_eff_.x(), 1.0);
    const double a = Va / L;
    MatX A(1, 1);
    A(0, 0) = -a;
    MatX Qc(1, 1);
    Qc(0, 0) = 1.0;
    const auto d = math::vanLoanDiscretize(A, Qc, dt);
    Eigen::Matrix<double, 1, 1> Qd;
    Qd(0, 0) = d.Qd(0, 0);
    xu_ = d.Ad(0, 0) * xu_ + sampleCovariance<1>(Qd, rng_);
    gust_body_.x() = sigma_eff_.x() * std::sqrt(2.0 * a) * xu_(0);
  }

  // --- v and w axes: second-order filters ------------------------------------------
  auto secondOrder = [&](double L, double sigma, Eigen::Vector2d& x) {
    const double Ls = std::max(L, 1.0);
    const double a = Va / Ls;
    MatX A(2, 2);
    A << 0.0, 1.0, -a * a, -2.0 * a;
    MatX Qc = MatX::Zero(2, 2);
    Qc(1, 1) = 1.0;
    const auto d = math::vanLoanDiscretize(A, Qc, dt);
    Eigen::Matrix2d Ad = d.Ad;
    Eigen::Matrix2d Qd = d.Qd;
    x = Ad * x + sampleCovariance<2>(Qd, rng_);
    const double K = sigma * std::sqrt(3.0 * a);
    return K * (a / std::sqrt(3.0) * x(0) + x(1));
  };
  gust_body_.y() = secondOrder(length_eff_.y(), sigma_eff_.y(), xv_);
  gust_body_.z() = secondOrder(length_eff_.z(), sigma_eff_.z(), xw_);
}

Vec3 WindModel::steadyNed(double altitude_agl) const {
  if (!cfg_.enable_shear) return cfg_.steady_ned;
  const double z0 = std::max(cfg_.shear_roughness, 1e-3);
  const double h_ref = std::max(cfg_.shear_reference_altitude, 2.0 * z0);
  const double h = std::max(altitude_agl, 2.0 * z0);
  const double scale = std::log(h / z0) / std::log(h_ref / z0);
  return cfg_.steady_ned * std::max(scale, 0.0);
}

Vec3 WindModel::totalNed(const Vec4& q_nb, double altitude_agl) const {
  return steadyNed(altitude_agl) + math::rotateBodyToNed(q_nb, gust_body_);
}

}  // namespace aether::env
