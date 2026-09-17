#include "aether/estimation/ErrorStateEkf.hpp"

#include <algorithm>
#include <cmath>

#include "aether/math/LinearAlgebra.hpp"
#include "aether/math/Rotation.hpp"

namespace aether::estimation {

namespace {
// The block indices themselves are public members of ErrorStateEkf (see the header), so the
// member functions below refer to them unqualified; only the dimension needs an alias.
constexpr int kNx = ErrorStateEkf::kStateDim;  // 18
}  // namespace

Vec3 NavState::euler() const { return math::quatToEuler(quaternion); }
Vec3 NavState::velocityBody() const { return math::rotateNedToBody(quaternion, velocity); }
Vec3 NavState::airVelocityBody() const {
  return math::rotateNedToBody(quaternion, airVelocityNed());
}

ErrorStateEkf::ErrorStateEkf(const EkfConfig& config, const Vec3& magnetic_field_ned,
                             double gravity)
    : cfg_(config), mag_ned_(magnetic_field_ned), gravity_(gravity) {
  P_ = MatX::Zero(kNx, kNx);
  Qc_ = MatX::Zero(kNx, kNx);
  initialise(NavState{});
}

void ErrorStateEkf::initialise(const NavState& state) {
  nominal_ = state;
  nominal_.quaternion = math::quatNormalize(nominal_.quaternion);
  P_ = MatX::Zero(kNx, kNx);
  P_.block<3, 3>(kIdxPosition, kIdxPosition) =
      Mat3::Identity() * cfg_.init_position_sigma * cfg_.init_position_sigma;
  P_.block<3, 3>(kIdxVelocity, kIdxVelocity) =
      Mat3::Identity() * cfg_.init_velocity_sigma * cfg_.init_velocity_sigma;
  P_.block<3, 3>(kIdxAttitude, kIdxAttitude) =
      Mat3::Identity() * cfg_.init_attitude_sigma * cfg_.init_attitude_sigma;
  P_.block<3, 3>(kIdxGyroBias, kIdxGyroBias) =
      Mat3::Identity() * cfg_.init_gyro_bias_sigma * cfg_.init_gyro_bias_sigma;
  P_.block<3, 3>(kIdxAccelBias, kIdxAccelBias) =
      Mat3::Identity() * cfg_.init_accel_bias_sigma * cfg_.init_accel_bias_sigma;
  P_.block<2, 2>(kIdxWind, kIdxWind) =
      Eigen::Matrix2d::Identity() * cfg_.init_wind_sigma * cfg_.init_wind_sigma;
  P_(kIdxBaroBias, kIdxBaroBias) = cfg_.init_baro_bias_sigma * cfg_.init_baro_bias_sigma;
  diag_ = EkfDiagnostics{};
  wind_initialised_ = false;
}

void ErrorStateEkf::predict(const sensors::ImuSample& imu, double dt) {
  if (!(dt > 0.0)) return;

  const Vec3 omega = imu.gyro - nominal_.gyro_bias;
  const Vec3 accel = imu.accel - nominal_.accel_bias;
  const Mat3 R = math::quatToRotation(nominal_.quaternion);

  // --- Nominal state propagation (first-order hold on the IMU sample) ---------------
  const Vec3 accel_ned = R * accel + Vec3(0.0, 0.0, gravity_);
  nominal_.position += nominal_.velocity * dt + 0.5 * accel_ned * dt * dt;
  nominal_.velocity += accel_ned * dt;
  nominal_.quaternion = math::boxPlus(nominal_.quaternion, omega * dt);
  // Biases are modelled as random walks, so their means are unchanged by propagation.

  // --- Error-state dynamics ---------------------------------------------------------
  MatX F = MatX::Zero(kNx, kNx);
  F.block<3, 3>(kIdxPosition, kIdxVelocity) = Mat3::Identity();
  F.block<3, 3>(kIdxVelocity, kIdxAttitude) = -R * math::skew(accel);
  F.block<3, 3>(kIdxVelocity, kIdxAccelBias) = -R;
  F.block<3, 3>(kIdxAttitude, kIdxAttitude) = -math::skew(omega);
  F.block<3, 3>(kIdxAttitude, kIdxGyroBias) = -Mat3::Identity();

  // Continuous process-noise power spectral density. The rotation matrices in the noise
  // input map cancel because R R^T = I.
  Qc_.setZero();
  Qc_.block<3, 3>(kIdxVelocity, kIdxVelocity) = Mat3::Identity() * cfg_.accel_noise * cfg_.accel_noise;
  Qc_.block<3, 3>(kIdxAttitude, kIdxAttitude) = Mat3::Identity() * cfg_.gyro_noise * cfg_.gyro_noise;
  Qc_.block<3, 3>(kIdxGyroBias, kIdxGyroBias) =
      Mat3::Identity() * cfg_.gyro_bias_walk * cfg_.gyro_bias_walk;
  Qc_.block<3, 3>(kIdxAccelBias, kIdxAccelBias) =
      Mat3::Identity() * cfg_.accel_bias_walk * cfg_.accel_bias_walk;
  Qc_.block<2, 2>(kIdxWind, kIdxWind) =
      Eigen::Matrix2d::Identity() * cfg_.wind_walk * cfg_.wind_walk;
  Qc_(kIdxBaroBias, kIdxBaroBias) = cfg_.baro_bias_walk * cfg_.baro_bias_walk;

  MatX Phi, Qd;
  if (cfg_.exact_discretisation) {
    const auto d = math::vanLoanDiscretize(F, Qc_, dt);
    Phi = d.Ad;
    Qd = d.Qd;
  } else {
    // Second-order series for the transition matrix and the trapezoidal (van-Loan
    // first-order) form for the discrete process noise. At 200 Hz the neglected terms are
    // O(dt^3) ~ 1e-7 relative, which the unit tests verify against the exact form.
    const MatX Fdt = F * dt;
    Phi = MatX::Identity(kNx, kNx) + Fdt + 0.5 * Fdt * Fdt;
    Qd = Qc_ * dt + 0.5 * dt * dt * (F * Qc_ + Qc_ * F.transpose());
  }

  P_ = Phi * P_ * Phi.transpose() + Qd;
  P_ = math::symmetrize(P_);
}

bool ErrorStateEkf::applyUpdate(const VecX& innovation, const MatX& H, const MatX& R,
                                double* nis) {
  const MatX S = H * P_ * H.transpose() + R;
  const Eigen::LDLT<MatX> ldlt(S);
  if (ldlt.info() != Eigen::Success) return false;

  const double d2 = innovation.dot(ldlt.solve(innovation));
  if (nis) *nis = d2;
  if (cfg_.innovation_gate > 0.0 && d2 > cfg_.innovation_gate * static_cast<double>(innovation.size()))
    return false;

  const MatX K = P_ * H.transpose() * ldlt.solve(MatX::Identity(S.rows(), S.cols()));
  const VecX dx = K * innovation;

  // Joseph form keeps P symmetric and positive semi-definite even with a suboptimal gain.
  const MatX I = MatX::Identity(kNx, kNx);
  const MatX IKH = I - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
  P_ = math::symmetrize(P_);

  injectAndReset(dx);
  return true;
}

void ErrorStateEkf::injectAndReset(const VecX& dx) {
  nominal_.position += dx.segment<3>(kIdxPosition);
  nominal_.velocity += dx.segment<3>(kIdxVelocity);
  const Vec3 dtheta = dx.segment<3>(kIdxAttitude);
  nominal_.quaternion = math::boxPlus(nominal_.quaternion, dtheta);
  nominal_.gyro_bias += dx.segment<3>(kIdxGyroBias);
  nominal_.accel_bias += dx.segment<3>(kIdxAccelBias);
  nominal_.wind += dx.segment<2>(kIdxWind);
  nominal_.baro_bias += dx(kIdxBaroBias);

  // Covariance reset for the multiplicative attitude error (Sola 2017, eq. 285-286).
  MatX G = MatX::Identity(kNx, kNx);
  G.block<3, 3>(kIdxAttitude, kIdxAttitude) = Mat3::Identity() - 0.5 * math::skew(dtheta);
  P_ = math::symmetrize(G * P_ * G.transpose());
}

void ErrorStateEkf::updateGps(const sensors::GpsSample& gps) {
  MatX H = MatX::Zero(6, kNx);
  H.block<3, 3>(0, kIdxPosition) = Mat3::Identity();
  H.block<3, 3>(3, kIdxVelocity) = Mat3::Identity();

  MatX R = MatX::Zero(6, 6);
  R(0, 0) = cfg_.gps_position_ne * cfg_.gps_position_ne;
  R(1, 1) = cfg_.gps_position_ne * cfg_.gps_position_ne;
  R(2, 2) = cfg_.gps_position_d * cfg_.gps_position_d;
  R(3, 3) = R(4, 4) = R(5, 5) = cfg_.gps_velocity * cfg_.gps_velocity;

  VecX y(6);
  y.head<3>() = gps.position_ned - nominal_.position;
  y.tail<3>() = gps.velocity_ned - nominal_.velocity;

  double nis = 0.0;
  if (applyUpdate(y, H, R, &nis))
    ++diag_.gps_updates;
  else
    ++diag_.gps_rejected;
  diag_.last_gps_nis = nis;
}

void ErrorStateEkf::updateBaro(const sensors::BaroSample& baro) {
  if (!cfg_.use_baro) return;
  // z = -p_D + b_baro: the pressure-offset bias is estimated alongside altitude, so the
  // GNSS vertical channel and the barometer together pin down the offset.
  MatX H = MatX::Zero(1, kNx);
  H(0, kIdxPosition + 2) = -1.0;
  H(0, kIdxBaroBias) = 1.0;

  MatX R(1, 1);
  R(0, 0) = cfg_.baro_noise * cfg_.baro_noise;

  VecX y(1);
  y(0) = baro.altitude - (-nominal_.position.z() + nominal_.baro_bias);

  double nis = 0.0;
  if (applyUpdate(y, H, R, &nis))
    ++diag_.baro_updates;
  else
    ++diag_.baro_rejected;
}

void ErrorStateEkf::updateMagnetometer(const sensors::MagSample& mag) {
  if (!cfg_.use_magnetometer) return;
  const Mat3 R_nb = math::quatToRotation(nominal_.quaternion);
  const Vec3 predicted = R_nb.transpose() * mag_ned_;

  // z = R(q)^T m_ned, with q = qhat (x) exp(dtheta/2) giving z ~ predicted + [predicted]x dtheta.
  MatX H = MatX::Zero(3, kNx);
  H.block<3, 3>(0, kIdxAttitude) = math::skew(predicted);

  MatX Rm = Mat3::Identity() * cfg_.mag_noise * cfg_.mag_noise;
  VecX y = mag.field_body - predicted;

  double nis = 0.0;
  if (applyUpdate(y, H, Rm, &nis))
    ++diag_.mag_updates;
  else
    ++diag_.mag_rejected;
  diag_.last_mag_nis = nis;
}

void ErrorStateEkf::updateAirspeed(const sensors::AirspeedSample& airspeed) {
  if (!cfg_.use_airspeed) return;

  // One-shot wind-triangle seeding: assume the air-relative velocity lies along the body
  // x-axis (alpha = beta = 0) and take the difference from the ground velocity as the wind.
  if (!wind_initialised_ && cfg_.initialise_wind_from_airspeed && airspeed.airspeed > 1.0) {
    const Vec3 v_air_ned =
        math::rotateBodyToNed(nominal_.quaternion, Vec3(airspeed.airspeed, 0.0, 0.0));
    nominal_.wind = (nominal_.velocity - v_air_ned).head<2>();
    wind_initialised_ = true;
  }
  // z = || v^n - w^n ||. Jacobian: dz/dv = u^T, dz/dw = -u^T restricted to the horizontal
  // components, with u the unit air-relative velocity vector.
  const Vec3 air = nominal_.airVelocityNed();
  const double Va = air.norm();
  if (Va < 1.0) return;  // the direction is ill-conditioned at very low airspeed
  const Vec3 u = air / Va;

  MatX H = MatX::Zero(1, kNx);
  H.block<1, 3>(0, kIdxVelocity) = u.transpose();
  H(0, kIdxWind + 0) = -u.x();
  H(0, kIdxWind + 1) = -u.y();

  MatX R(1, 1);
  R(0, 0) = cfg_.airspeed_noise * cfg_.airspeed_noise;

  VecX y(1);
  y(0) = airspeed.airspeed - Va;

  double nis = 0.0;
  if (applyUpdate(y, H, R, &nis))
    ++diag_.airspeed_updates;
  else
    ++diag_.airspeed_rejected;
}

VecX ErrorStateEkf::sigma() const {
  VecX s(kNx);
  for (int i = 0; i < kNx; ++i) s(i) = std::sqrt(std::max(P_(i, i), 0.0));
  return s;
}

double ErrorStateEkf::minimumEigenvalue() const {
  Eigen::SelfAdjointEigenSolver<MatX> es(P_);
  return es.eigenvalues().minCoeff();
}

}  // namespace aether::estimation
