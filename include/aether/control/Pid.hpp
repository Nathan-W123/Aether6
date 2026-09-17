/// \file Pid.hpp
/// \brief PID controller with output saturation, derivative filtering and anti-windup.
#pragma once

#include <limits>

namespace aether::control {

/// \brief Tuning and limits for a single PID loop.
struct PidConfig {
  double kp = 0.0;  ///< Proportional gain [output unit / error unit]
  double ki = 0.0;  ///< Integral gain [output unit / (error unit * s)]
  double kd = 0.0;  ///< Derivative gain [output unit * s / error unit]

  double output_min = -std::numeric_limits<double>::infinity();  ///< Lower output saturation
  double output_max = std::numeric_limits<double>::infinity();   ///< Upper output saturation

  /// Back-calculation anti-windup gain [1/s]. The integrator is unwound at a rate
  /// proportional to the amount the output is saturated; 0 disables back-calculation and
  /// falls back to pure conditional integration.
  double anti_windup_gain = 1.0;

  /// Time constant of the first-order filter applied to the derivative term [s].
  /// 0 disables filtering.
  double derivative_filter_tau = 0.02;

  /// When true the derivative acts on the measurement instead of the error, which avoids
  /// a derivative kick when the setpoint steps.
  bool derivative_on_measurement = true;
};

/// \brief Scalar PID controller.
///
/// Update law (parallel form):
/// \f[ u_{raw} = k_p e + k_i\!\int\! e\,dt + k_d \dot e,\qquad u = \mathrm{sat}(u_{raw}) \f]
/// with the integrator advanced as
/// \f$ I \mathrel{+}= \left(k_i e + k_{aw}(u - u_{raw})\right)\Delta t \f$.
/// The back-calculation term stops the integrator winding up while the output is clipped,
/// and additionally the integrator is frozen when the error would push further into the
/// same saturation.
class Pid {
 public:
  Pid() = default;
  explicit Pid(const PidConfig& cfg) : cfg_(cfg) {}

  /// Advance the loop one step and return the saturated output.
  /// \param setpoint Desired value.
  /// \param measurement Measured value.
  /// \param dt Time step [s], must be > 0.
  /// \param measurement_rate Optional externally supplied derivative of the measurement
  ///        (e.g. a gyro rate for an attitude loop). If NaN, the derivative is estimated
  ///        by finite differences and filtered.
  double update(double setpoint, double measurement, double dt,
                double measurement_rate = std::numeric_limits<double>::quiet_NaN());

  /// Advance the loop given the error directly (for wrapped angular errors).
  double updateWithError(double error, double dt,
                         double error_rate = std::numeric_limits<double>::quiet_NaN());

  /// Clear the integrator and derivative memory.
  void reset();

  /// Preload the integrator, e.g. to start from a trimmed actuator position.
  void setIntegrator(double value) { integral_ = value; }

  /// Current integrator value.
  double integrator() const { return integral_; }

  /// Last saturated output.
  double output() const { return last_output_; }

  /// True when the last output hit a saturation limit.
  bool saturated() const { return saturated_; }

  /// Mutable access to the configuration (gains may be retuned at runtime).
  PidConfig& config() { return cfg_; }
  /// Read-only access to the configuration.
  const PidConfig& config() const { return cfg_; }

 private:
  PidConfig cfg_;
  double integral_ = 0.0;
  double prev_error_ = 0.0;
  double prev_measurement_ = 0.0;
  double derivative_state_ = 0.0;
  bool initialised_ = false;
  double last_output_ = 0.0;
  bool saturated_ = false;
};

}  // namespace aether::control
