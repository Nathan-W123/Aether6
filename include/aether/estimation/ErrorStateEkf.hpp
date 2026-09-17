/// \file ErrorStateEkf.hpp
/// \brief Error-state (indirect) extended Kalman filter for the navigation solution.
#pragma once

#include "aether/core/Types.hpp"
#include "aether/sensors/Sensors.hpp"

namespace aether::estimation {

/// \brief Nominal (full) navigation state maintained by the filter.
struct NavState {
  Vec3 position = Vec3::Zero();      ///< NED position [m]
  Vec3 velocity = Vec3::Zero();      ///< NED velocity [m/s]
  Vec4 quaternion = Vec4(1, 0, 0, 0);///< Attitude \f$q_{nb}\f$ [-]
  Vec3 gyro_bias = Vec3::Zero();     ///< Gyro bias estimate [rad/s]
  Vec3 accel_bias = Vec3::Zero();    ///< Accelerometer bias estimate [m/s^2]
  Eigen::Vector2d wind = Eigen::Vector2d::Zero();  ///< Horizontal wind (north, east) [m/s]
  double baro_bias = 0.0;            ///< Barometric pressure-offset bias [m]

  /// Euler angles \f$(\phi,\theta,\psi)\f$ [rad].
  Vec3 euler() const;
  /// Body-frame ground velocity [m/s].
  Vec3 velocityBody() const;
  /// Estimated wind vector in NED (vertical component assumed zero) [m/s].
  Vec3 windNed() const { return Vec3(wind.x(), wind.y(), 0.0); }
  /// Air-relative velocity in NED [m/s].
  Vec3 airVelocityNed() const { return velocity - windNed(); }
  /// Air-relative velocity in body axes [m/s].
  Vec3 airVelocityBody() const;
  /// Estimated true airspeed [m/s].
  double airspeed() const { return airVelocityNed().norm(); }
  /// Altitude above the NED origin [m].
  double altitude() const { return -position.z(); }
};

/// \brief Tuning of the error-state EKF.
struct EkfConfig {
  // Process noise (continuous-time spectral densities).
  double gyro_noise = 0.0035;      ///< Gyro white noise \f$\sigma\f$ [rad/s]
  double accel_noise = 0.05;       ///< Accelerometer white noise \f$\sigma\f$ [m/s^2]
  double gyro_bias_walk = 3.0e-4;  ///< Gyro bias random walk [rad/s/sqrt(s)]
  double accel_bias_walk = 3.0e-3; ///< Accel bias random walk [m/s^2/sqrt(s)]. Tune this
                                   ///< well above the sensor truth: the x accelerometer
                                   ///< bias is only weakly separable from pitch attitude
                                   ///< in near-level flight, and an optimistic value makes
                                   ///< the covariance confidently wrong.

  // Measurement noise.
  double gps_position_ne = 1.2;    ///< GNSS horizontal position \f$\sigma\f$ [m]
  double gps_position_d = 2.5;     ///< GNSS vertical position \f$\sigma\f$ [m]
  double gps_velocity = 0.2;       ///< GNSS velocity \f$\sigma\f$ [m/s]
  double baro_noise = 0.8;         ///< Barometric altitude \f$\sigma\f$ [m]
  double mag_noise = 0.5e-6;       ///< Magnetometer \f$\sigma\f$ per axis [T]
  double airspeed_noise = 0.6;     ///< Pitot true-airspeed \f$\sigma\f$ [m/s]
  double wind_walk = 0.05;         ///< Wind random-walk intensity [m/s/sqrt(s)]
  double baro_bias_walk = 0.01;    ///< Barometric bias random walk [m/sqrt(s)]

  // Initial covariance (1-sigma).
  double init_position_sigma = 5.0;   ///< [m]
  double init_velocity_sigma = 1.0;   ///< [m/s]
  double init_attitude_sigma = 0.09;  ///< [rad] (~5 deg)
  double init_gyro_bias_sigma = 0.02; ///< [rad/s]
  double init_accel_bias_sigma = 0.2; ///< [m/s^2]
  double init_wind_sigma = 6.0;       ///< [m/s]
  double init_baro_bias_sigma = 3.0;  ///< [m]

  /// Use an exact Van Loan discretisation instead of the second-order series expansion.
  /// Exact is slower but is used by the unit tests to bound the approximation error.
  bool exact_discretisation = false;

  /// Chi-squared gate on innovations; measurements above this normalised distance are
  /// rejected. Set <= 0 to disable gating.
  double innovation_gate = 25.0;

  /// Fuse barometric altitude in addition to the GNSS vertical channel.
  bool use_baro = true;
  /// Fuse the magnetometer vector.
  bool use_magnetometer = true;
  /// Fuse the pitot true-airspeed measurement, which is what makes the wind observable.
  bool use_airspeed = true;
  /// Seed the wind states from the first airspeed measurement using the "wind triangle"
  /// \f$ w = v^n - R(\hat q)[V_a, 0, 0]^\top \f$ (exact when alpha = beta = 0). Without this
  /// the wind estimate starts at zero, so in a strong wind the reconstructed sideslip is
  /// wrong by tens of degrees for the first second — long enough for a control law that
  /// feeds back air-relative velocity to upset the aircraft.
  bool initialise_wind_from_airspeed = true;
};

/// \brief Filter health/diagnostic counters.
struct EkfDiagnostics {
  int gps_updates = 0;       ///< Accepted GNSS updates.
  int gps_rejected = 0;      ///< GNSS updates rejected by the innovation gate.
  int baro_updates = 0;      ///< Accepted barometer updates.
  int baro_rejected = 0;     ///< Barometer updates rejected.
  int mag_updates = 0;       ///< Accepted magnetometer updates.
  int mag_rejected = 0;      ///< Magnetometer updates rejected.
  int airspeed_updates = 0;  ///< Accepted airspeed updates.
  int airspeed_rejected = 0; ///< Airspeed updates rejected.
  double last_gps_nis = 0.0; ///< Normalised innovation squared of the last GNSS update.
  double last_mag_nis = 0.0; ///< Normalised innovation squared of the last magnetometer update.
};

/// \brief 18-state error-state extended Kalman filter with wind and barometer-bias states.
///
/// Nominal state: \f$[p^n, v^n, q_{nb}, b_g, b_a, w_{ne}, b_{baro}]\f$ (19 scalars).
/// Error state:
/// \f$\delta x = [\delta p, \delta v, \delta\theta, \delta b_g, \delta b_a, \delta w,
/// \delta b_{baro}]\f$
/// (18 scalars), with the attitude error defined multiplicatively in the **body** frame,
/// \f$q = \hat q \otimes \exp(\delta\theta/2)\f$.
///
/// The two horizontal wind states are driven by the scalar pitot measurement
/// \f$V_a = \|v^n - w^n\|\f$, which makes the wind component along the current ground track
/// observable immediately and the cross-track component observable as soon as the heading
/// changes — the classical "wind triangle" estimator. Estimating the wind is what lets the
/// control laws work in genuinely air-relative quantities, so a crosswind crab is not
/// mistaken for a sideslip.
///
/// The barometer-bias state absorbs the (unknown, slowly drifting) pressure offset of the
/// static system. Without it the altitude estimate inherits that offset as an unmodelled
/// error while the covariance keeps shrinking, so the filter is *confidently wrong* about
/// altitude; the state makes the estimate consistent because the GNSS vertical channel and
/// the barometer together observe the offset.
///
/// Propagation is driven by the IMU (the inertial measurements are *inputs*, not
/// measurements), and the error dynamics are
/// \f{align*}{
///   \delta\dot p &= \delta v
///   \\ \delta\dot v &= -\hat R\,[a_m-\hat b_a]_\times\,\delta\theta - \hat R\,\delta b_a - \hat R n_a
///   \\ \delta\dot\theta &= -[\omega_m-\hat b_g]_\times\,\delta\theta - \delta b_g - n_g
///   \\ \delta\dot b_g &= n_{bg},\qquad \delta\dot b_a = n_{ba}.
/// \f}
/// Updates use the Joseph form so the covariance stays symmetric and positive semi-definite,
/// and every update is followed by an explicit symmetrisation. After each update the error
/// state is injected into the nominal state and reset, including the attitude-error reset
/// Jacobian \f$G=\mathrm{blkdiag}(I,I,I-\tfrac12[\delta\theta]_\times,I,I)\f$.
class ErrorStateEkf {
 public:
  /// \param config Filter tuning.
  /// \param magnetic_field_ned Reference magnetic field in NED [T].
  /// \param gravity Gravitational acceleration [m/s^2].
  ErrorStateEkf(const EkfConfig& config, const Vec3& magnetic_field_ned,
                double gravity = 9.80665);

  /// Initialise the nominal state and covariance.
  void initialise(const NavState& state);

  /// Propagate the nominal state and covariance with one IMU sample.
  /// \param imu IMU sample (gyro + specific force).
  /// \param dt Propagation interval [s], must be > 0.
  void predict(const sensors::ImuSample& imu, double dt);

  /// Fuse a GNSS position+velocity measurement.
  void updateGps(const sensors::GpsSample& gps);
  /// Fuse a barometric altitude measurement.
  void updateBaro(const sensors::BaroSample& baro);
  /// Fuse a three-axis magnetometer measurement.
  void updateMagnetometer(const sensors::MagSample& mag);
  /// Fuse a pitot true-airspeed measurement (drives the wind states).
  void updateAirspeed(const sensors::AirspeedSample& airspeed);

  /// True once the wind states have been seeded from an airspeed measurement.
  bool windInitialised() const { return wind_initialised_; }

  /// Current nominal state estimate.
  const NavState& state() const { return nominal_; }
  /// Current 15x15 error covariance.
  const MatX& covariance() const { return P_; }
  /// Diagnostic counters.
  const EkfDiagnostics& diagnostics() const { return diag_; }
  /// Filter configuration.
  const EkfConfig& config() const { return cfg_; }

  /// 1-sigma standard deviations of the error states (square roots of the diagonal).
  VecX sigma() const;

  /// Number of error states (18).
  static constexpr int kStateDim = 18;

  /// \name Error-state block indices
  /// Offsets of each block inside the 18-element error state and inside `sigma()`.
  ///@{
  static constexpr int kIdxPosition = 0;    ///< 3 elements, [m]
  static constexpr int kIdxVelocity = 3;    ///< 3 elements, [m/s]
  static constexpr int kIdxAttitude = 6;    ///< 3 elements, body rotation vector [rad]
  static constexpr int kIdxGyroBias = 9;    ///< 3 elements, [rad/s]
  static constexpr int kIdxAccelBias = 12;  ///< 3 elements, [m/s^2]
  static constexpr int kIdxWind = 15;       ///< 2 elements, horizontal wind [m/s]
  static constexpr int kIdxBaroBias = 17;   ///< 1 element, barometric offset [m]
  ///@}

  /// Smallest eigenvalue of the covariance; a health indicator that must stay >= 0.
  double minimumEigenvalue() const;

 private:
  /// Apply a generic linear-Gaussian update with Joseph-form covariance propagation.
  /// Returns false when the innovation gate rejects the measurement.
  bool applyUpdate(const VecX& innovation, const MatX& H, const MatX& R, double* nis);
  /// Inject the error state into the nominal state and reset it.
  void injectAndReset(const VecX& dx);

  EkfConfig cfg_;
  Vec3 mag_ned_;
  double gravity_;

  NavState nominal_;
  MatX P_;
  MatX Qc_;
  EkfDiagnostics diag_;
  bool wind_initialised_ = false;
};

}  // namespace aether::estimation
