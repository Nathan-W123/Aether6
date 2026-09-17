/// \file test_env_sensors.cpp
/// \brief Wind/turbulence statistics and simulated-sensor behaviour.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <vector>

#include "aether/core/Constants.hpp"
#include "aether/dynamics/RigidBody.hpp"
#include "aether/env/Wind.hpp"
#include "aether/math/Rotation.hpp"
#include "aether/sensors/Sensors.hpp"

using namespace aether;
using Catch::Approx;

TEST_CASE("steady wind and the logarithmic shear profile", "[env][wind]") {
  env::WindConfig cfg;
  cfg.steady_ned = Vec3(5.0, -3.0, 0.0);
  cfg.enable_turbulence = false;
  cfg.enable_shear = false;
  cfg.shear_reference_altitude = 100.0;

  env::WindModel no_shear(cfg, 1);
  REQUIRE((no_shear.steadyNed(10.0) - cfg.steady_ned).norm() == Approx(0.0));
  REQUIRE((no_shear.steadyNed(500.0) - cfg.steady_ned).norm() == Approx(0.0));

  cfg.enable_shear = true;
  env::WindModel shear(cfg, 1);
  // At the reference altitude the profile returns the configured wind exactly.
  REQUIRE((shear.steadyNed(100.0) - cfg.steady_ned).norm() == Approx(0.0).margin(1e-12));
  // Wind speed increases monotonically with height and is weaker near the ground.
  REQUIRE(shear.steadyNed(10.0).norm() < shear.steadyNed(100.0).norm());
  REQUIRE(shear.steadyNed(200.0).norm() > shear.steadyNed(100.0).norm());
  // The direction is unchanged.
  REQUIRE(shear.steadyNed(10.0).normalized().dot(cfg.steady_ned.normalized()) ==
          Approx(1.0).margin(1e-12));
  // The profile stays finite at (and below) the ground.
  REQUIRE(shear.steadyNed(0.0).allFinite());
}

TEST_CASE("Dryden gusts have the prescribed stationary variance", "[env][wind][statistics]") {
  env::WindConfig cfg;
  cfg.steady_ned = Vec3::Zero();
  cfg.enable_shear = false;
  cfg.enable_turbulence = true;
  cfg.altitude_scaled = false;            // use the configured sigma/L directly
  cfg.sigma = Vec3(1.5, 1.5, 1.0);
  cfg.length_scale = Vec3(200.0, 200.0, 50.0);

  env::WindModel wind(cfg, 4242);
  const double dt = 0.01, Va = 25.0;
  // Burn in a few correlation times so the filters reach their stationary distribution.
  for (int i = 0; i < 20000; ++i) wind.step(dt, Va, 100.0);

  const int n = 400000;
  Vec3 sum = Vec3::Zero(), sumsq = Vec3::Zero();
  for (int i = 0; i < n; ++i) {
    wind.step(dt, Va, 100.0);
    const Vec3 g = wind.gustBody();
    sum += g;
    sumsq += g.cwiseProduct(g);
  }
  const Vec3 mean = sum / n;
  const Vec3 var = sumsq / n - mean.cwiseProduct(mean);

  INFO("gust std = " << std::sqrt(var.x()) << ", " << std::sqrt(var.y()) << ", "
                     << std::sqrt(var.z()));
  // The Dryden filters are scaled so the stationary standard deviation equals sigma.
  REQUIRE(std::sqrt(var.x()) == Approx(cfg.sigma.x()).epsilon(0.06));
  REQUIRE(std::sqrt(var.y()) == Approx(cfg.sigma.y()).epsilon(0.06));
  REQUIRE(std::sqrt(var.z()) == Approx(cfg.sigma.z()).epsilon(0.06));
  // Zero mean. The gust is strongly autocorrelated (correlation time tau = L/Va), so the
  // standard error of the sample mean is sigma*sqrt(2*tau/T) rather than sigma/sqrt(n);
  // the bound below is four of those standard errors.
  const double T_total = n * dt;
  auto meanBound = [&](double sigma, double L) {
    return 4.0 * sigma * std::sqrt(2.0 * (L / Va) / T_total);
  };
  REQUIRE(std::abs(mean.x()) < meanBound(cfg.sigma.x(), cfg.length_scale.x()));
  REQUIRE(std::abs(mean.y()) < meanBound(cfg.sigma.y(), cfg.length_scale.y()));
  REQUIRE(std::abs(mean.z()) < meanBound(cfg.sigma.z(), cfg.length_scale.z()));
}

TEST_CASE("Dryden statistics are independent of the integration step",
          "[env][wind][statistics]") {
  env::WindConfig cfg;
  cfg.enable_shear = false;
  cfg.altitude_scaled = false;
  cfg.sigma = Vec3(1.0, 1.0, 1.0);
  cfg.length_scale = Vec3(200.0, 200.0, 200.0);

  auto stdOf = [&](double dt, int n) {
    env::WindModel w(cfg, 7);
    for (int i = 0; i < 20000; ++i) w.step(dt, 25.0, 100.0);
    double s = 0.0;
    for (int i = 0; i < n; ++i) {
      w.step(dt, 25.0, 100.0);
      s += w.gustBody().x() * w.gustBody().x();
    }
    return std::sqrt(s / n);
  };
  // Exact (Van Loan) discretisation means the variance does not depend on dt.
  REQUIRE(stdOf(0.002, 200000) == Approx(1.0).epsilon(0.08));
  REQUIRE(stdOf(0.05, 200000) == Approx(1.0).epsilon(0.08));
}

TEST_CASE("turbulence is reproducible and can be disabled", "[env][wind]") {
  env::WindConfig cfg;
  cfg.enable_shear = false;
  cfg.altitude_scaled = false;

  env::WindModel a(cfg, 99), b(cfg, 99), c(cfg, 100);
  for (int i = 0; i < 500; ++i) {
    a.step(0.01, 25.0, 100.0);
    b.step(0.01, 25.0, 100.0);
    c.step(0.01, 25.0, 100.0);
  }
  REQUIRE((a.gustBody() - b.gustBody()).norm() == Approx(0.0));
  REQUIRE((a.gustBody() - c.gustBody()).norm() > 1e-6);

  a.reset();
  REQUIRE(a.gustBody().norm() == Approx(0.0));

  cfg.enable_turbulence = false;
  env::WindModel off(cfg, 1);
  for (int i = 0; i < 100; ++i) off.step(0.01, 25.0, 100.0);
  REQUIRE(off.gustBody().norm() == Approx(0.0));
}

TEST_CASE("MIL-F-8785C altitude scaling reduces turbulence near the ground",
          "[env][wind]") {
  env::WindConfig cfg;
  cfg.enable_shear = false;
  cfg.altitude_scaled = true;
  cfg.w20 = 10.0;

  auto rms = [&](double altitude) {
    env::WindModel w(cfg, 31);
    for (int i = 0; i < 20000; ++i) w.step(0.01, 25.0, altitude);
    double s = 0.0;
    const int n = 100000;
    for (int i = 0; i < n; ++i) {
      w.step(0.01, 25.0, altitude);
      s += w.gustBody().z() * w.gustBody().z();
    }
    return std::sqrt(s / n);
  };
  // sigma_w = 0.1 * W20 independently of height, but sigma_u/sigma_v grow towards the ground.
  REQUIRE(rms(200.0) == Approx(0.1 * cfg.w20).epsilon(0.15));

  auto rmsU = [&](double altitude) {
    env::WindModel w(cfg, 32);
    for (int i = 0; i < 20000; ++i) w.step(0.01, 25.0, altitude);
    double s = 0.0;
    const int n = 100000;
    for (int i = 0; i < n; ++i) {
      w.step(0.01, 25.0, altitude);
      s += w.gustBody().x() * w.gustBody().x();
    }
    return std::sqrt(s / n);
  };
  REQUIRE(rmsU(10.0) > rmsU(250.0));
}

TEST_CASE("total wind combines the steady field with the body-frame gust", "[env][wind]") {
  env::WindConfig cfg;
  cfg.steady_ned = Vec3(4.0, 0.0, 0.0);
  cfg.enable_shear = false;
  cfg.enable_turbulence = false;
  env::WindModel w(cfg, 5);
  const Vec4 q = math::eulerToQuat(0.0, 0.0, constants::kPi / 2);
  REQUIRE((w.totalNed(q, 100.0) - cfg.steady_ned).norm() == Approx(0.0));
}

namespace {

/// A steady level state used to exercise the sensors.
StateVec sensorState() {
  dynamics::AircraftState s;
  s.position = Vec3(100.0, -50.0, -150.0);
  s.velocity = Vec3(25.0, 0.0, 1.0);
  s.quaternion = math::eulerToQuat(0.1, 0.05, 0.6);
  s.rate = Vec3(0.02, -0.01, 0.03);
  return s.vec();
}

}  // namespace

TEST_CASE("sensors fire at their configured rates", "[sensors]") {
  sensors::SensorConfig cfg;
  cfg.imu.rate = 200.0;
  cfg.gps.rate = 5.0;
  cfg.baro.rate = 20.0;
  cfg.magnetometer.rate = 50.0;
  cfg.airspeed.rate = 50.0;

  sensors::SensorSuite suite(cfg, 1234);
  dynamics::DynamicsDiagnostics diag;
  diag.air.airspeed = 25.0;
  diag.specific_force = Vec3(0.0, 0.0, -constants::kGravity);

  const double dt = 0.001;
  const double duration = 10.0;
  int imu = 0, gps = 0, baro = 0, mag = 0, air = 0;
  for (int i = 0; i < static_cast<int>(duration / dt); ++i) {
    const auto m = suite.sample(i * dt, dt, sensorState(), diag);
    imu += m.imu_valid;
    gps += m.gps_valid;
    baro += m.baro_valid;
    mag += m.mag_valid;
    air += m.airspeed_valid;
  }
  REQUIRE(imu == Approx(2000).margin(2));
  REQUIRE(gps == Approx(50).margin(2));
  REQUIRE(baro == Approx(200).margin(2));
  REQUIRE(mag == Approx(500).margin(2));
  REQUIRE(air == Approx(500).margin(2));
}

TEST_CASE("sampling rates survive a step that is not a multiple of the period",
          "[sensors][scheduling]") {
  // 200 Hz on a 500 Hz frame: the period (5 ms) is not a multiple of the step (2 ms), so a
  // schedule advanced from the *fire* time would drift to an effective 167 Hz. Advancing from
  // the scheduled time keeps the average rate exact, with at most one step of jitter.
  sensors::SensorConfig cfg;
  cfg.imu.rate = 200.0;
  cfg.gps.rate = 5.0;
  cfg.baro.rate = 20.0;
  cfg.magnetometer.rate = 50.0;
  cfg.airspeed.rate = 50.0;

  sensors::SensorSuite suite(cfg, 2024);
  dynamics::DynamicsDiagnostics diag;
  diag.air.airspeed = 25.0;

  const double dt = 0.002;   // 500 Hz frame
  const double duration = 20.0;
  int imu = 0, gps = 0, baro = 0, mag = 0, air = 0;
  double previous_imu = -1.0, max_gap = 0.0, min_gap = 1e9;
  for (int i = 0; i < static_cast<int>(duration / dt); ++i) {
    const double t = i * dt;
    const auto m = suite.sample(t, dt, sensorState(), diag);
    if (m.imu_valid) {
      if (previous_imu >= 0.0) {
        max_gap = std::max(max_gap, m.imu.time - previous_imu);
        min_gap = std::min(min_gap, m.imu.time - previous_imu);
      }
      previous_imu = m.imu.time;
    }
    imu += m.imu_valid;
    gps += m.gps_valid;
    baro += m.baro_valid;
    mag += m.mag_valid;
    air += m.airspeed_valid;
  }
  INFO("imu samples " << imu << " over " << duration << " s");
  REQUIRE(imu == Approx(duration * cfg.imu.rate).epsilon(0.005));
  REQUIRE(gps == Approx(duration * cfg.gps.rate).epsilon(0.02));
  REQUIRE(baro == Approx(duration * cfg.baro.rate).epsilon(0.02));
  REQUIRE(mag == Approx(duration * cfg.magnetometer.rate).epsilon(0.02));
  REQUIRE(air == Approx(duration * cfg.airspeed.rate).epsilon(0.02));
  // Jitter is bounded by one simulation step either side of the nominal period.
  REQUIRE(min_gap >= 1.0 / cfg.imu.rate - dt - 1e-9);
  REQUIRE(max_gap <= 1.0 / cfg.imu.rate + dt + 1e-9);
}

TEST_CASE("a step longer than the sensor period resynchronises the schedule",
          "[sensors][scheduling]") {
  // 200 Hz requested but a 100 Hz frame: the sensor can only fire once per step, and the
  // schedule must not accumulate an unbounded backlog.
  sensors::SensorConfig cfg;
  cfg.imu.rate = 200.0;
  cfg.gps.rate = cfg.baro.rate = cfg.magnetometer.rate = cfg.airspeed.rate = 0.0;
  sensors::SensorSuite suite(cfg, 5);
  dynamics::DynamicsDiagnostics diag;

  const double dt = 0.01;
  int imu = 0;
  for (int i = 0; i < 1000; ++i) imu += suite.sample(i * dt, dt, sensorState(), diag).imu_valid;
  REQUIRE(imu == 1000);  // one per step, not a runaway backlog
}

TEST_CASE("sensor noise and bias have the configured statistics", "[sensors][statistics]") {
  sensors::SensorConfig cfg;
  cfg.imu.rate = 100.0;
  cfg.imu.gyro_noise_density = 0.01;
  cfg.imu.gyro_bias_initial = 0.0;   // isolate the white noise
  cfg.imu.gyro_bias_walk = 0.0;
  cfg.imu.accel_noise_density = 0.2;
  cfg.imu.accel_bias_initial = 0.0;
  cfg.imu.accel_bias_walk = 0.0;
  cfg.gps.rate = 0.0;
  cfg.baro.rate = 0.0;
  cfg.magnetometer.rate = 0.0;
  cfg.airspeed.rate = 0.0;

  sensors::SensorSuite suite(cfg, 20240917);
  const StateVec x = sensorState();
  const Vec3 truth_rate = dynamics::AircraftState::fromVec(x).rate;
  dynamics::DynamicsDiagnostics diag;
  diag.air.airspeed = 25.0;
  diag.specific_force = Vec3(0.5, -0.2, -9.5);

  const int n = 200000;
  Vec3 sum = Vec3::Zero(), sumsq = Vec3::Zero();
  int count = 0;
  for (int i = 0; i < n; ++i) {
    const auto m = suite.sample(i * 0.01, 0.01, x, diag);
    if (!m.imu_valid) continue;
    const Vec3 e = m.imu.gyro - truth_rate;
    sum += e;
    sumsq += e.cwiseProduct(e);
    ++count;
  }
  const Vec3 mean = sum / count;
  const Vec3 var = sumsq / count - mean.cwiseProduct(mean);
  REQUIRE(std::sqrt(var.x()) == Approx(cfg.imu.gyro_noise_density).epsilon(0.03));
  REQUIRE(std::abs(mean.x()) < 0.05 * cfg.imu.gyro_noise_density);
}

TEST_CASE("gyro bias random walk grows like sqrt(t)", "[sensors][statistics]") {
  sensors::SensorConfig cfg;
  cfg.imu.rate = 100.0;
  cfg.imu.gyro_bias_initial = 0.0;
  cfg.imu.gyro_bias_walk = 0.01;
  cfg.gps.rate = cfg.baro.rate = cfg.magnetometer.rate = cfg.airspeed.rate = 0.0;

  const double T = 100.0;
  const int trials = 400;
  double sumsq = 0.0;
  for (int k = 0; k < trials; ++k) {
    sensors::SensorSuite suite(cfg, 1000 + k);
    dynamics::DynamicsDiagnostics diag;
    for (int i = 0; i < static_cast<int>(T / 0.01); ++i)
      suite.sample(i * 0.01, 0.01, sensorState(), diag);
    sumsq += suite.gyroBias().x() * suite.gyroBias().x();
  }
  const double rms = std::sqrt(sumsq / trials);
  // Random-walk standard deviation after T seconds is walk * sqrt(T).
  REQUIRE(rms == Approx(cfg.imu.gyro_bias_walk * std::sqrt(T)).epsilon(0.15));
}

TEST_CASE("magnetometer reads the rotated reference field", "[sensors]") {
  sensors::SensorConfig cfg;
  cfg.magnetometer.rate = 10.0;
  cfg.magnetometer.noise = 0.0;
  cfg.magnetometer.bias_sigma = 0.0;
  cfg.imu.rate = cfg.gps.rate = cfg.baro.rate = cfg.airspeed.rate = 0.0;

  sensors::SensorSuite suite(cfg, 7);
  const StateVec x = sensorState();
  dynamics::DynamicsDiagnostics diag;
  const auto m = suite.sample(0.0, 0.01, x, diag);
  REQUIRE(m.mag_valid);

  const Vec3 expected = math::rotateNedToBody(
      dynamics::AircraftState::fromVec(x).quaternion, suite.magneticFieldNed());
  REQUIRE((m.mag.field_body - expected).norm() == Approx(0.0).margin(1e-15));
  REQUIRE(suite.magneticFieldNed().norm() == Approx(cfg.magnetometer.field_strength));
  // Northern-hemisphere field: the down component is positive.
  REQUIRE(suite.magneticFieldNed().z() > 0.0);
}

TEST_CASE("a scripted GNSS outage suppresses fixes", "[sensors]") {
  sensors::SensorConfig cfg;
  cfg.gps.rate = 5.0;
  cfg.imu.rate = cfg.baro.rate = cfg.magnetometer.rate = cfg.airspeed.rate = 0.0;
  cfg.gps_outage_start = 3.0;
  cfg.gps_outage_end = 6.0;

  sensors::SensorSuite suite(cfg, 11);
  dynamics::DynamicsDiagnostics diag;
  int before = 0, during = 0, after = 0;
  for (int i = 0; i < 1000; ++i) {
    const double t = i * 0.01;
    const auto m = suite.sample(t, 0.01, sensorState(), diag);
    if (!m.gps_valid) continue;
    if (t < 3.0) ++before;
    else if (t <= 6.0) ++during;
    else ++after;
  }
  REQUIRE(before > 10);
  REQUIRE(during == 0);
  REQUIRE(after > 10);

  cfg.enable_gps = false;
  cfg.gps_outage_start = -1.0;
  sensors::SensorSuite disabled(cfg, 11);
  int any = 0;
  for (int i = 0; i < 1000; ++i) any += disabled.sample(i * 0.01, 0.01, sensorState(), diag).gps_valid;
  REQUIRE(any == 0);
}

TEST_CASE("sensor suites are reproducible from the seed", "[sensors][determinism]") {
  sensors::SensorConfig cfg;
  sensors::SensorSuite a(cfg, 555), b(cfg, 555), c(cfg, 556);
  dynamics::DynamicsDiagnostics diag;
  diag.air.airspeed = 25.0;

  bool differed = false;
  for (int i = 0; i < 500; ++i) {
    const double t = i * 0.005;
    const auto ma = a.sample(t, 0.005, sensorState(), diag);
    const auto mb = b.sample(t, 0.005, sensorState(), diag);
    const auto mc = c.sample(t, 0.005, sensorState(), diag);
    REQUIRE(ma.imu_valid == mb.imu_valid);
    if (ma.imu_valid) {
      REQUIRE((ma.imu.gyro - mb.imu.gyro).norm() == Approx(0.0));
      REQUIRE((ma.imu.accel - mb.imu.accel).norm() == Approx(0.0));
      if ((ma.imu.gyro - mc.imu.gyro).norm() > 1e-9) differed = true;
    }
  }
  REQUIRE(differed);
}
