/// \file test_estimation.cpp
/// \brief Error-state EKF: covariance properties, convergence and wind observability.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "aether/core/Constants.hpp"
#include "aether/dynamics/RigidBody.hpp"
#include "aether/estimation/ErrorStateEkf.hpp"
#include "aether/math/Rotation.hpp"
#include "aether/sensors/Sensors.hpp"
#include "aether/util/Random.hpp"

using namespace aether;
using Catch::Approx;

namespace {

constexpr int kNx = estimation::ErrorStateEkf::kStateDim;

Vec3 magneticFieldNed() {
  sensors::SensorConfig c;
  return sensors::SensorSuite(c, 0).magneticFieldNed();
}

bool isSymmetric(const MatX& P, double tol = 1e-12) {
  return (P - P.transpose()).cwiseAbs().maxCoeff() <= tol * std::max(1.0, P.norm());
}

double minEigenvalue(const MatX& P) {
  Eigen::SelfAdjointEigenSolver<MatX> es(P);
  return es.eigenvalues().minCoeff();
}

/// Closed-form truth generator: a coordinated level turn at constant airspeed. The IMU
/// measurements are computed analytically, so the filter can be exercised without the full
/// aircraft model.
struct TurnTruth {
  double speed = 25.0;
  double turn_rate = 0.05;  ///< rad/s
  double altitude = 150.0;
  Vec3 wind = Vec3(4.0, -3.0, 0.0);

  double heading(double t) const { return turn_rate * t; }
  Vec3 velocityNed(double t) const {
    return Vec3(speed * std::cos(heading(t)), speed * std::sin(heading(t)), 0.0) + wind;
  }
  Vec3 positionNed(double t) const {
    if (std::abs(turn_rate) < 1e-9)
      return Vec3(speed * t, 0.0, -altitude) + Vec3(wind.x() * t, wind.y() * t, 0.0);
    return Vec3(speed / turn_rate * std::sin(turn_rate * t),
                speed / turn_rate * (1.0 - std::cos(turn_rate * t)), -altitude) +
           Vec3(wind.x() * t, wind.y() * t, 0.0);
  }
  /// Bank angle of a coordinated turn.
  double bank() const { return std::atan2(speed * turn_rate, constants::kGravity); }
  Vec4 quaternion(double t) const { return math::eulerToQuat(bank(), 0.0, heading(t)); }
  Vec3 bodyRate() const {
    // phi_dot = theta_dot = 0, psi_dot = turn_rate, theta = 0.
    return Vec3(0.0, turn_rate * std::sin(bank()), turn_rate * std::cos(bank()));
  }
  /// Specific force measured by an ideal accelerometer.
  Vec3 specificForce(double t) const {
    // a_ned = d/dt(v_ned); f_body = R^T (a_ned - g_ned)
    const Vec3 a_ned(-speed * turn_rate * std::sin(heading(t)),
                     speed * turn_rate * std::cos(heading(t)), 0.0);
    const Vec3 g_ned(0.0, 0.0, constants::kGravity);
    return math::rotateNedToBody(quaternion(t), a_ned - g_ned);
  }
  double airspeed() const { return speed; }
};

}  // namespace

TEST_CASE("the covariance stays symmetric and positive semi-definite", "[ekf][covariance]") {
  estimation::EkfConfig cfg;
  estimation::ErrorStateEkf ekf(cfg, magneticFieldNed());

  estimation::NavState init;
  init.position = Vec3(0, 0, -150);
  init.velocity = Vec3(25, 0, 0);
  ekf.initialise(init);

  REQUIRE(ekf.covariance().rows() == kNx);
  REQUIRE(isSymmetric(ekf.covariance()));
  REQUIRE(minEigenvalue(ekf.covariance()) > 0.0);

  TurnTruth truth;
  util::Rng rng(4242);
  const double dt = 0.005;
  for (int i = 0; i < 4000; ++i) {
    const double t = i * dt;
    sensors::ImuSample imu;
    imu.time = t;
    imu.gyro = truth.bodyRate() + 0.002 * rng.gaussian3();
    imu.accel = truth.specificForce(t) + 0.05 * rng.gaussian3();
    ekf.predict(imu, dt);

    REQUIRE(isSymmetric(ekf.covariance()));
    REQUIRE(minEigenvalue(ekf.covariance()) > -1e-12);

    if (i % 40 == 0) {  // 5 Hz GNSS
      sensors::GpsSample gps;
      gps.time = t;
      gps.position_ned = truth.positionNed(t) + 1.2 * rng.gaussian3();
      gps.velocity_ned = truth.velocityNed(t) + 0.15 * rng.gaussian3();
      ekf.updateGps(gps);
      REQUIRE(isSymmetric(ekf.covariance()));
      REQUIRE(minEigenvalue(ekf.covariance()) > -1e-12);
    }
    if (i % 10 == 0) {  // 20 Hz barometer
      sensors::BaroSample baro;
      baro.time = t;
      baro.altitude = truth.altitude + 0.6 * rng.gaussian();
      ekf.updateBaro(baro);
      REQUIRE(isSymmetric(ekf.covariance()));
    }
    if (i % 4 == 0) {  // 50 Hz magnetometer
      sensors::MagSample mag;
      mag.time = t;
      mag.field_body =
          math::rotateNedToBody(truth.quaternion(t), magneticFieldNed()) + 4e-7 * rng.gaussian3();
      ekf.updateMagnetometer(mag);
      REQUIRE(isSymmetric(ekf.covariance()));
      REQUIRE(minEigenvalue(ekf.covariance()) > -1e-12);
    }
  }
  REQUIRE(ekf.diagnostics().gps_updates > 50);
  REQUIRE(ekf.diagnostics().mag_updates > 500);
}

TEST_CASE("the filter converges to the truth on a coordinated turn", "[ekf][convergence]") {
  estimation::EkfConfig cfg;
  cfg.airspeed_noise = 0.5;
  estimation::ErrorStateEkf ekf(cfg, magneticFieldNed());

  TurnTruth truth;
  util::Rng rng(20240917);

  // Deliberately wrong initial estimate.
  estimation::NavState init;
  init.position = truth.positionNed(0.0) + Vec3(6.0, -4.0, 3.0);
  init.velocity = truth.velocityNed(0.0) + Vec3(1.0, -1.0, 0.5);
  init.quaternion = math::boxPlus(truth.quaternion(0.0), Vec3(0.05, -0.04, 0.08));
  ekf.initialise(init);

  const double dt = 0.005;
  const double T = 120.0;  // two full turns, so the wind becomes fully observable
  const Vec3 gyro_bias(0.006, -0.004, 0.003);
  const Vec3 accel_bias(0.05, -0.04, 0.06);

  double final_pos_err = 0.0, final_vel_err = 0.0, final_att_err = 0.0;
  for (int i = 0; i < static_cast<int>(T / dt); ++i) {
    const double t = i * dt;
    sensors::ImuSample imu;
    imu.time = t;
    imu.gyro = truth.bodyRate() + gyro_bias + 0.0035 * rng.gaussian3();
    imu.accel = truth.specificForce(t) + accel_bias + 0.05 * rng.gaussian3();
    ekf.predict(imu, dt);

    if (i % 40 == 0) {
      sensors::GpsSample gps;
      gps.time = t;
      gps.position_ned = truth.positionNed(t) + 1.2 * rng.gaussian3();
      gps.velocity_ned = truth.velocityNed(t) + 0.15 * rng.gaussian3();
      ekf.updateGps(gps);
    }
    if (i % 10 == 0) {
      sensors::BaroSample baro;
      baro.time = t;
      baro.altitude = truth.altitude + 0.6 * rng.gaussian();
      ekf.updateBaro(baro);
    }
    if (i % 4 == 0) {
      sensors::MagSample mag;
      mag.time = t;
      mag.field_body =
          math::rotateNedToBody(truth.quaternion(t), magneticFieldNed()) + 4e-7 * rng.gaussian3();
      ekf.updateMagnetometer(mag);
      sensors::AirspeedSample as;
      as.time = t;
      as.airspeed = truth.airspeed() + 0.35 * rng.gaussian();
      ekf.updateAirspeed(as);
    }

    if (t > T - 10.0) {
      final_pos_err = std::max(final_pos_err, (ekf.state().position - truth.positionNed(t)).norm());
      final_vel_err = std::max(final_vel_err, (ekf.state().velocity - truth.velocityNed(t)).norm());
      final_att_err =
          std::max(final_att_err, math::boxMinus(ekf.state().quaternion, truth.quaternion(t)).norm());
    }
  }

  INFO("position error " << final_pos_err << " m, velocity error " << final_vel_err
                         << " m/s, attitude error " << final_att_err * constants::kRadToDeg
                         << " deg");
  REQUIRE(final_pos_err < 4.0);
  REQUIRE(final_vel_err < 0.6);
  REQUIRE(final_att_err < 4.0 * constants::kDegToRad);

  // The inertial biases are observable through the motion and must be largely recovered.
  REQUIRE((ekf.state().gyro_bias - gyro_bias).norm() < 0.5 * gyro_bias.norm() + 2e-3);

  // The wind becomes observable once the heading has swept a full circle.
  INFO("wind estimate " << ekf.state().wind.transpose());
  REQUIRE(ekf.state().wind.x() == Approx(truth.wind.x()).margin(1.5));
  REQUIRE(ekf.state().wind.y() == Approx(truth.wind.y()).margin(1.5));

  // Covariance must still be healthy at the end.
  REQUIRE(isSymmetric(ekf.covariance()));
  REQUIRE(ekf.minimumEigenvalue() > 0.0);
  const VecX sigma = ekf.sigma();
  REQUIRE(sigma.size() == kNx);
  REQUIRE((sigma.array() > 0.0).all());
  REQUIRE(sigma.segment<3>(estimation::ErrorStateEkf::kIdxPosition).maxCoeff() <
          cfg.init_position_sigma);  // covariance shrank
}

TEST_CASE("the innovation gate rejects a gross outlier", "[ekf][robustness]") {
  estimation::EkfConfig cfg;
  cfg.innovation_gate = 9.0;
  estimation::ErrorStateEkf ekf(cfg, magneticFieldNed());

  estimation::NavState init;
  init.position = Vec3(0, 0, -150);
  init.velocity = Vec3(25, 0, 0);
  ekf.initialise(init);

  // Shrink the covariance first so the gate is meaningful.
  util::Rng rng(3);
  for (int i = 0; i < 2000; ++i) {
    sensors::ImuSample imu;
    imu.time = i * 0.005;
    imu.accel = Vec3(0, 0, -constants::kGravity);
    ekf.predict(imu, 0.005);
    if (i % 40 == 0) {
      sensors::GpsSample gps;
      gps.time = i * 0.005;
      gps.position_ned = Vec3(25.0 * i * 0.005, 0, -150) + 1.2 * rng.gaussian3();
      gps.velocity_ned = Vec3(25, 0, 0) + 0.15 * rng.gaussian3();
      ekf.updateGps(gps);
    }
  }
  const int accepted_before = ekf.diagnostics().gps_updates;
  const int rejected_before = ekf.diagnostics().gps_rejected;
  const Vec3 pos_before = ekf.state().position;

  sensors::GpsSample bad;
  bad.time = 10.0;
  bad.position_ned = ekf.state().position + Vec3(5000.0, -3000.0, 800.0);
  bad.velocity_ned = ekf.state().velocity;
  ekf.updateGps(bad);

  REQUIRE(ekf.diagnostics().gps_rejected == rejected_before + 1);
  REQUIRE(ekf.diagnostics().gps_updates == accepted_before);
  REQUIRE((ekf.state().position - pos_before).norm() == Approx(0.0).margin(1e-12));
}

TEST_CASE("exact and approximate covariance discretisation agree at typical IMU rates",
          "[ekf][numerics]") {
  estimation::EkfConfig approx_cfg;
  approx_cfg.exact_discretisation = false;
  estimation::EkfConfig exact_cfg = approx_cfg;
  exact_cfg.exact_discretisation = true;

  estimation::ErrorStateEkf a(approx_cfg, magneticFieldNed());
  estimation::ErrorStateEkf e(exact_cfg, magneticFieldNed());

  estimation::NavState init;
  init.position = Vec3(0, 0, -150);
  init.velocity = Vec3(25, 2, -1);
  init.quaternion = math::eulerToQuat(0.2, 0.1, 0.4);
  a.initialise(init);
  e.initialise(init);

  sensors::ImuSample imu;
  imu.gyro = Vec3(0.3, -0.2, 0.15);
  imu.accel = Vec3(1.2, 0.4, -9.0);
  const double dt = 1.0 / 200.0;
  for (int i = 0; i < 400; ++i) {
    imu.time = i * dt;
    a.predict(imu, dt);
    e.predict(imu, dt);
  }
  const double rel = (a.covariance() - e.covariance()).norm() / e.covariance().norm();
  INFO("relative covariance difference " << rel);
  REQUIRE(rel < 1e-4);
  // The nominal states are propagated identically.
  REQUIRE((a.state().position - e.state().position).norm() == Approx(0.0).margin(1e-12));
}

TEST_CASE("the wind state is only observable once the heading changes", "[ekf][observability]") {
  estimation::EkfConfig cfg;
  cfg.airspeed_noise = 0.5;
  cfg.wind_walk = 0.0;  // isolate observability from the random walk

  auto runStraightOrTurning = [&](double turn_rate) {
    estimation::ErrorStateEkf ekf(cfg, magneticFieldNed());
    TurnTruth truth;
    truth.turn_rate = turn_rate;
    util::Rng rng(31337);
    estimation::NavState init;
    init.position = truth.positionNed(0.0);
    init.velocity = truth.velocityNed(0.0);
    init.quaternion = truth.quaternion(0.0);
    ekf.initialise(init);

    const double dt = 0.005;
    for (int i = 0; i < static_cast<int>(120.0 / dt); ++i) {
      const double t = i * dt;
      sensors::ImuSample imu;
      imu.time = t;
      imu.gyro = truth.bodyRate();
      imu.accel = truth.specificForce(t);
      ekf.predict(imu, dt);
      if (i % 40 == 0) {
        sensors::GpsSample gps;
        gps.time = t;
        gps.position_ned = truth.positionNed(t) + 0.5 * rng.gaussian3();
        gps.velocity_ned = truth.velocityNed(t) + 0.05 * rng.gaussian3();
        ekf.updateGps(gps);
      }
      if (i % 4 == 0) {
        sensors::MagSample mag;
        mag.time = t;
        mag.field_body = math::rotateNedToBody(truth.quaternion(t), magneticFieldNed());
        ekf.updateMagnetometer(mag);
        sensors::AirspeedSample as;
        as.time = t;
        as.airspeed = truth.airspeed();
        ekf.updateAirspeed(as);
      }
    }
    return ekf;
  };

  const auto turning = runStraightOrTurning(0.05);
  const auto straight = runStraightOrTurning(0.0);

  const Vec3 wind_truth = TurnTruth{}.wind;
  const double turn_err =
      (turning.state().wind - wind_truth.head<2>()).norm();
  const double straight_err =
      (straight.state().wind - wind_truth.head<2>()).norm();
  INFO("turning wind error " << turn_err << " m/s, straight-line wind error " << straight_err);
  REQUIRE(turn_err < 1.0);
  // Flying straight leaves the cross-track wind component unobservable, so the estimate is
  // worse and its covariance stays large in that direction.
  REQUIRE(straight_err > turn_err);
  constexpr int kWind = estimation::ErrorStateEkf::kIdxWind;
  const VecX s_turn = turning.sigma();
  const VecX s_straight = straight.sigma();
  REQUIRE(s_straight.segment<2>(kWind).maxCoeff() > s_turn.segment<2>(kWind).maxCoeff());
}

TEST_CASE("filter API guards", "[ekf][api]") {
  estimation::EkfConfig cfg;
  estimation::ErrorStateEkf ekf(cfg, magneticFieldNed());
  const MatX P0 = ekf.covariance();

  sensors::ImuSample imu;
  imu.accel = Vec3(0, 0, -constants::kGravity);
  ekf.predict(imu, 0.0);       // a non-positive step must be a no-op
  ekf.predict(imu, -0.01);
  REQUIRE((ekf.covariance() - P0).norm() == Approx(0.0));

  cfg.use_baro = false;
  cfg.use_magnetometer = false;
  cfg.use_airspeed = false;
  estimation::ErrorStateEkf off(cfg, magneticFieldNed());
  const MatX Q0 = off.covariance();
  sensors::BaroSample b;
  b.altitude = 1000.0;
  off.updateBaro(b);
  sensors::MagSample m;
  m.field_body = Vec3(1, 1, 1);
  off.updateMagnetometer(m);
  sensors::AirspeedSample a;
  a.airspeed = 25.0;
  off.updateAirspeed(a);
  REQUIRE((off.covariance() - Q0).norm() == Approx(0.0));
  REQUIRE(off.diagnostics().baro_updates == 0);
}

TEST_CASE("the wind states are seeded from the first airspeed measurement",
          "[ekf][wind][initialisation]") {
  estimation::EkfConfig cfg;
  cfg.initialise_wind_from_airspeed = true;
  estimation::ErrorStateEkf ekf(cfg, magneticFieldNed());

  // Flying due north at 25 m/s airspeed into a 10 m/s wind from the south-west: the ground
  // velocity is the vector sum, and the wind triangle must recover the wind immediately.
  const Vec3 wind(6.0, 8.0, 0.0);
  const Vec3 air_ned(25.0, 0.0, 0.0);
  estimation::NavState init;
  init.position = Vec3(0, 0, -150);
  init.velocity = air_ned + wind;
  init.quaternion = math::eulerToQuat(0.0, 0.0, 0.0);  // nose north
  ekf.initialise(init);

  REQUIRE_FALSE(ekf.windInitialised());
  REQUIRE(ekf.state().wind.norm() == Approx(0.0));
  // Before seeding, the reconstructed sideslip is badly wrong.
  REQUIRE(std::abs(ekf.state().airVelocityBody().y()) > 5.0);

  sensors::AirspeedSample as;
  as.time = 0.0;
  as.airspeed = 25.0;
  ekf.updateAirspeed(as);

  REQUIRE(ekf.windInitialised());
  REQUIRE(ekf.state().wind.x() == Approx(wind.x()).margin(0.2));
  REQUIRE(ekf.state().wind.y() == Approx(wind.y()).margin(0.2));
  // ... and the air-relative velocity is now along the body x-axis, i.e. beta ~ 0.
  REQUIRE(std::abs(ekf.state().airVelocityBody().y()) < 0.3);
  REQUIRE(ekf.state().airspeed() == Approx(25.0).margin(0.2));

  // Seeding happens exactly once.
  sensors::AirspeedSample as2 = as;
  as2.airspeed = 40.0;
  ekf.updateAirspeed(as2);
  REQUIRE(ekf.state().wind.x() == Approx(wind.x()).margin(1.0));

  const double seeded_error = (ekf.state().wind - wind.head<2>()).norm();

  // Without seeding, a single scalar airspeed update can only correct the wind along the
  // current ground track, so the estimate is much worse after the same measurement.
  estimation::EkfConfig off = cfg;
  off.initialise_wind_from_airspeed = false;
  estimation::ErrorStateEkf no_seed(off, magneticFieldNed());
  no_seed.initialise(init);
  no_seed.updateAirspeed(as);
  REQUIRE_FALSE(no_seed.windInitialised());
  const double unseeded_error = (no_seed.state().wind - wind.head<2>()).norm();
  INFO("seeded error " << seeded_error << " m/s, unseeded error " << unseeded_error);
  REQUIRE(unseeded_error > 5.0 * seeded_error);
  REQUIRE(unseeded_error > 2.0);
}
