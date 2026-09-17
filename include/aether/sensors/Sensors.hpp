/// \file Sensors.hpp
/// \brief Simulated avionics sensors with independent sample rates, noise and bias.
#pragma once

#include <cstdint>

#include "aether/core/Types.hpp"
#include "aether/dynamics/RigidBody.hpp"
#include "aether/util/Random.hpp"

namespace aether::sensors {

/// \brief Inertial measurement unit configuration.
struct ImuConfig {
  double rate = 200.0;               ///< Sample rate [Hz]
  double gyro_noise_density = 0.0035;///< Gyro white noise \f$\sigma\f$ at the sample rate [rad/s]
  double accel_noise_density = 0.05; ///< Accelerometer white noise \f$\sigma\f$ [m/s^2]
  double gyro_bias_initial = 0.010;  ///< 1-sigma of the initial gyro bias [rad/s]
  double accel_bias_initial = 0.08;  ///< 1-sigma of the initial accelerometer bias [m/s^2]
  double gyro_bias_walk = 3.0e-4;    ///< Gyro bias random-walk intensity [rad/s/sqrt(s)]
  double accel_bias_walk = 3.0e-3;   ///< Accel bias random-walk intensity [m/s^2/sqrt(s)]
};

/// \brief GNSS receiver configuration.
struct GpsConfig {
  double rate = 5.0;              ///< Sample rate [Hz]
  double position_noise_ne = 1.2; ///< White noise on horizontal position [m]
  double position_noise_d = 2.5;  ///< White noise on vertical position [m]
  double velocity_noise = 0.15;   ///< White noise on each NED velocity component [m/s]
  double slow_error_sigma = 1.5;  ///< Stationary sigma of the correlated position error [m]
  double slow_error_tau = 300.0;  ///< Correlation time of that error (Gauss-Markov) [s]
};

/// \brief Barometric altimeter configuration.
struct BaroConfig {
  double rate = 20.0;          ///< Sample rate [Hz]
  double noise = 0.6;          ///< White noise on altitude [m]
  double bias_sigma = 2.0;     ///< 1-sigma of the (constant per run) pressure-offset bias [m]
};

/// \brief Three-axis magnetometer configuration.
struct MagnetometerConfig {
  double rate = 50.0;            ///< Sample rate [Hz]
  double field_strength = 50.0e-6; ///< Local field magnitude [T]
  double inclination = 1.134;     ///< Field inclination below horizontal [rad] (65 deg)
  double declination = 0.192;     ///< Field declination east of true north [rad] (11 deg)
  double noise = 0.4e-6;          ///< White noise per axis [T]
  double bias_sigma = 0.6e-6;     ///< 1-sigma of the (constant per run) hard-iron bias [T]
};

/// \brief Pitot-static airspeed sensor configuration.
struct AirspeedConfig {
  double rate = 50.0;   ///< Sample rate [Hz]
  double noise = 0.35;  ///< White noise on true airspeed [m/s]
  double bias_sigma = 0.25;  ///< 1-sigma of the constant scale/offset error [m/s]
};

/// \brief Full sensor-suite configuration.
struct SensorConfig {
  ImuConfig imu;
  GpsConfig gps;
  BaroConfig baro;
  MagnetometerConfig magnetometer;
  AirspeedConfig airspeed;
  bool enable_gps = true;          ///< Disable to simulate a GNSS outage.
  double gps_outage_start = -1.0;  ///< Start of a scripted GNSS outage [s] (<0 disables)
  double gps_outage_end = -1.0;    ///< End of the scripted GNSS outage [s]
};

/// \brief IMU sample: body angular rate and specific force.
struct ImuSample {
  double time = 0.0;          ///< Sample time [s]
  Vec3 gyro = Vec3::Zero();   ///< Measured body rate [rad/s]
  Vec3 accel = Vec3::Zero();  ///< Measured specific force in body axes [m/s^2]
};

/// \brief GNSS sample.
struct GpsSample {
  double time = 0.0;                  ///< Sample time [s]
  Vec3 position_ned = Vec3::Zero();   ///< Measured NED position [m]
  Vec3 velocity_ned = Vec3::Zero();   ///< Measured NED velocity [m/s]
};

/// \brief Barometric altitude sample.
struct BaroSample {
  double time = 0.0;      ///< Sample time [s]
  double altitude = 0.0;  ///< Measured altitude [m]
};

/// \brief Magnetometer sample.
struct MagSample {
  double time = 0.0;              ///< Sample time [s]
  Vec3 field_body = Vec3::Zero(); ///< Measured field in body axes [T]
};

/// \brief Airspeed sample.
struct AirspeedSample {
  double time = 0.0;      ///< Sample time [s]
  double airspeed = 0.0;  ///< Measured true airspeed [m/s]
};

/// \brief All measurements produced at one simulation step, with validity flags.
struct SensorBundle {
  bool imu_valid = false;   ImuSample imu;
  bool gps_valid = false;   GpsSample gps;
  bool baro_valid = false;  BaroSample baro;
  bool mag_valid = false;   MagSample mag;
  bool airspeed_valid = false;  AirspeedSample airspeed;
};

/// \brief Deterministic simulated sensor suite.
///
/// Every sensor keeps its own phase accumulator and emits a sample only when its period has
/// elapsed, so the filter genuinely sees asynchronous, multi-rate measurements. All biases
/// are drawn once at construction from the configured 1-sigma values; the inertial biases
/// then evolve as random walks. The random streams are derived from the master seed with
/// SplitMix64 so each sensor is independent and the whole suite is reproducible.
class SensorSuite {
 public:
  /// \param config Sensor configuration.
  /// \param seed   Master seed for all sensor noise streams.
  SensorSuite(const SensorConfig& config, std::uint64_t seed);

  /// Produce the measurements due in \f$[t, t+dt)\f$.
  /// \param time Current simulation time [s].
  /// \param dt Simulation step [s].
  /// \param truth Truth state of the vehicle.
  /// \param diag Dynamics diagnostics at the same instant (supplies the specific force).
  SensorBundle sample(double time, double dt, const StateVec& truth,
                      const dynamics::DynamicsDiagnostics& diag);

  /// Reference magnetic field in NED [T], used by both the sensor and the filter.
  Vec3 magneticFieldNed() const;

  /// Truth value of the current gyro bias [rad/s] (for estimator error plots).
  const Vec3& gyroBias() const { return gyro_bias_; }
  /// Truth value of the current accelerometer bias [m/s^2].
  const Vec3& accelBias() const { return accel_bias_; }
  /// Truth value of the barometer bias [m].
  double baroBias() const { return baro_bias_; }

  /// Configuration in use.
  const SensorConfig& config() const { return cfg_; }

 private:
  SensorConfig cfg_;
  util::Rng rng_imu_, rng_gps_, rng_baro_, rng_mag_, rng_air_;

  Vec3 gyro_bias_ = Vec3::Zero();
  Vec3 accel_bias_ = Vec3::Zero();
  double baro_bias_ = 0.0;
  Vec3 mag_bias_ = Vec3::Zero();
  double airspeed_bias_ = 0.0;
  Vec3 gps_slow_error_ = Vec3::Zero();

  double next_imu_ = 0.0;
  double next_gps_ = 0.0;
  double next_baro_ = 0.0;
  double next_mag_ = 0.0;
  double next_air_ = 0.0;
};

}  // namespace aether::sensors
