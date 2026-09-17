#include "aether/sensors/Sensors.hpp"

#include <algorithm>
#include <cmath>

#include "aether/math/Rotation.hpp"

namespace aether::sensors {

namespace {
constexpr std::uint64_t kStreamImu = 1;
constexpr std::uint64_t kStreamGps = 2;
constexpr std::uint64_t kStreamBaro = 3;
constexpr std::uint64_t kStreamMag = 4;
constexpr std::uint64_t kStreamAir = 5;
}  // namespace

SensorSuite::SensorSuite(const SensorConfig& config, std::uint64_t seed)
    : cfg_(config),
      rng_imu_(util::deriveSeed(seed, kStreamImu, 0)),
      rng_gps_(util::deriveSeed(seed, kStreamGps, 0)),
      rng_baro_(util::deriveSeed(seed, kStreamBaro, 0)),
      rng_mag_(util::deriveSeed(seed, kStreamMag, 0)),
      rng_air_(util::deriveSeed(seed, kStreamAir, 0)) {
  for (int i = 0; i < 3; ++i) {
    gyro_bias_(i) = rng_imu_.gaussian(0.0, cfg_.imu.gyro_bias_initial);
    accel_bias_(i) = rng_imu_.gaussian(0.0, cfg_.imu.accel_bias_initial);
    mag_bias_(i) = rng_mag_.gaussian(0.0, cfg_.magnetometer.bias_sigma);
  }
  baro_bias_ = rng_baro_.gaussian(0.0, cfg_.baro.bias_sigma);
  airspeed_bias_ = rng_air_.gaussian(0.0, cfg_.airspeed.bias_sigma);
}

Vec3 SensorSuite::magneticFieldNed() const {
  const auto& m = cfg_.magnetometer;
  const double ch = std::cos(m.inclination);
  const double sh = std::sin(m.inclination);
  return m.field_strength * Vec3(ch * std::cos(m.declination), ch * std::sin(m.declination), sh);
}

SensorBundle SensorSuite::sample(double time, double dt, const StateVec& truth,
                                 const dynamics::DynamicsDiagnostics& diag) {
  SensorBundle out;
  const dynamics::AircraftState s = dynamics::AircraftState::fromVec(truth);
  const Vec3 vel_ned = math::rotateBodyToNed(s.quaternion, s.velocity);

  // --- IMU --------------------------------------------------------------------------
  if (cfg_.imu.rate > 0.0 && time + 1e-12 >= next_imu_) {
    const double period = 1.0 / cfg_.imu.rate;
    // Bias random walk advanced over the elapsed sampling interval.
    const double walk_dt = period;
    for (int i = 0; i < 3; ++i) {
      gyro_bias_(i) += cfg_.imu.gyro_bias_walk * std::sqrt(walk_dt) * rng_imu_.gaussian();
      accel_bias_(i) += cfg_.imu.accel_bias_walk * std::sqrt(walk_dt) * rng_imu_.gaussian();
    }
    out.imu_valid = true;
    out.imu.time = time;
    for (int i = 0; i < 3; ++i) {
      out.imu.gyro(i) = s.rate(i) + gyro_bias_(i) +
                        cfg_.imu.gyro_noise_density * rng_imu_.gaussian();
      out.imu.accel(i) = diag.specific_force(i) + accel_bias_(i) +
                         cfg_.imu.accel_noise_density * rng_imu_.gaussian();
    }
    next_imu_ = time + period;
  }

  // --- GNSS -------------------------------------------------------------------------
  const bool outage = cfg_.gps_outage_start >= 0.0 && cfg_.gps_outage_end > cfg_.gps_outage_start &&
                      time >= cfg_.gps_outage_start && time <= cfg_.gps_outage_end;
  if (cfg_.enable_gps && cfg_.gps.rate > 0.0 && time + 1e-12 >= next_gps_) {
    const double period = 1.0 / cfg_.gps.rate;
    // First-order Gauss-Markov correlated position error.
    const double tau = std::max(cfg_.gps.slow_error_tau, 1e-3);
    const double a = std::exp(-period / tau);
    const double sd = cfg_.gps.slow_error_sigma * std::sqrt(std::max(1.0 - a * a, 0.0));
    for (int i = 0; i < 3; ++i) gps_slow_error_(i) = a * gps_slow_error_(i) + sd * rng_gps_.gaussian();

    if (!outage) {
      out.gps_valid = true;
      out.gps.time = time;
      out.gps.position_ned =
          s.position + gps_slow_error_ +
          Vec3(cfg_.gps.position_noise_ne * rng_gps_.gaussian(),
               cfg_.gps.position_noise_ne * rng_gps_.gaussian(),
               cfg_.gps.position_noise_d * rng_gps_.gaussian());
      out.gps.velocity_ned = vel_ned + cfg_.gps.velocity_noise * rng_gps_.gaussian3();
    }
    next_gps_ = time + period;
  }

  // --- Barometer --------------------------------------------------------------------
  if (cfg_.baro.rate > 0.0 && time + 1e-12 >= next_baro_) {
    out.baro_valid = true;
    out.baro.time = time;
    out.baro.altitude = -s.position.z() + baro_bias_ + cfg_.baro.noise * rng_baro_.gaussian();
    next_baro_ = time + 1.0 / cfg_.baro.rate;
  }

  // --- Magnetometer -----------------------------------------------------------------
  if (cfg_.magnetometer.rate > 0.0 && time + 1e-12 >= next_mag_) {
    out.mag_valid = true;
    out.mag.time = time;
    const Vec3 field_body = math::rotateNedToBody(s.quaternion, magneticFieldNed());
    out.mag.field_body = field_body + mag_bias_ + cfg_.magnetometer.noise * rng_mag_.gaussian3();
    next_mag_ = time + 1.0 / cfg_.magnetometer.rate;
  }

  // --- Pitot ------------------------------------------------------------------------
  if (cfg_.airspeed.rate > 0.0 && time + 1e-12 >= next_air_) {
    out.airspeed_valid = true;
    out.airspeed.time = time;
    out.airspeed.airspeed = std::max(
        0.0, diag.air.airspeed + airspeed_bias_ + cfg_.airspeed.noise * rng_air_.gaussian());
    next_air_ = time + 1.0 / cfg_.airspeed.rate;
  }

  (void)dt;
  return out;
}

}  // namespace aether::sensors
