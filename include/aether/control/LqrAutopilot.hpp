/// \file LqrAutopilot.hpp
/// \brief Servo-LQR autopilot synthesised from the numerically linearised model.
#pragma once

#include "aether/analysis/Linearize.hpp"
#include "aether/analysis/Trim.hpp"
#include "aether/control/Autopilot.hpp"

namespace aether::control {

/// \brief LQR weight selection (Bryson's rule: weights are 1/(maximum acceptable value)^2).
struct LqrWeights {
  // Longitudinal: states [du, dw, dq, dtheta, h_err, int_h_err, int_Va_err].
  double q_u = 2.0;          ///< Weight on body-x velocity error [s^2/m^2]
  double q_w = 2.0;          ///< Weight on body-z velocity error [s^2/m^2]
  double q_pitch_rate = 8.0; ///< Weight on pitch-rate error [s^2/rad^2]
  double q_pitch = 60.0;     ///< Weight on pitch-attitude error [1/rad^2]
  double q_altitude = 3.0;   ///< Weight on altitude error [1/m^2]
  double q_int_altitude = 0.20;  ///< Weight on integrated altitude error [1/(m*s)^2]
  double q_int_airspeed = 3.0;   ///< Weight on integrated airspeed error [1/(m/s*s)^2]
  double r_elevator = 300.0; ///< Elevator effort weight [1/rad^2]
  double r_throttle = 6.0;   ///< Throttle effort weight [-]

  // Lateral: states [dv, dp, dr, dphi, chi_err, int_chi_err].
  double q_v = 1.0;          ///< Weight on body-y velocity error [s^2/m^2]
  double q_roll_rate = 3.0;  ///< Weight on roll-rate error [s^2/rad^2]
  double q_yaw_rate = 12.0;  ///< Weight on yaw-rate error [s^2/rad^2]
  double q_roll = 25.0;      ///< Weight on bank-angle error [1/rad^2]
  double q_course = 90.0;    ///< Weight on course error [1/rad^2]
  double q_int_course = 6.0; ///< Weight on integrated course error [1/(rad*s)^2]
  double r_aileron = 200.0;  ///< Aileron effort weight [1/rad^2]
  double r_rudder = 200.0;   ///< Rudder effort weight [1/rad^2]
};

/// \brief Limits applied on top of the LQR feedback law.
///
/// The error states are saturated so the feedback law cannot demand an attitude outside the
/// vehicle's handling limits. With `derive_limits_from_gains` (the default) the saturations
/// are computed from the synthesised gains rather than guessed. With the aileron near its
/// balance point the closed loop settles where the bank term cancels the course terms,
/// \f$K_\phi\,\phi = K_\chi\,\chi_{err} + K_{\int}\!\int\chi_{err}\f$, so clamping
/// \f[ |\chi_{err}| \le (1-a)\,\phi_{max} K_\phi/K_\chi, \qquad
///     \left|\int\chi_{err}\right| \le a\,\phi_{max} K_\phi/K_{\int} \f]
/// makes that balance point exactly \f$\phi_{max}\f$, with the fraction \a a of the bank
/// budget reserved for integral (trim) authority. The altitude channel uses the same split
/// against the pitch budget. The configured values act as upper bounds.
/// See docs/model.md, "LQR command limiting".
struct LqrAutopilotLimits {
  double max_bank = 0.61;          ///< Bank limit used to saturate the course error [rad]
  double max_pitch = 0.35;         ///< Pitch limit used to saturate the altitude error [rad]
  double max_climb_rate = 4.0;     ///< Slew limit on the altitude command [m/s]
  double integrator_altitude_limit = 400.0;  ///< |int_h_err| clamp [m*s]
  double integrator_airspeed_limit = 60.0;   ///< |int_Va_err| clamp [(m/s)*s]
  double integrator_course_limit = 8.0;      ///< |int_chi_err| clamp [rad*s]
  double course_error_limit = 1.2;           ///< Saturation on the course error state [rad]
  double altitude_error_limit = 60.0;        ///< Saturation on the altitude error state [m]
  /// Tighten the four clamps above using the synthesised gains (recommended).
  bool derive_limits_from_gains = true;
  EnvelopeProtection protection;  ///< Angle-of-attack envelope protection.
  /// Fraction \a a of the attitude budget reserved for the integral states [-]. The
  /// proportional error state gets the remaining (1 - a), so the two together can never
  /// demand more than the configured bank / pitch limit.
  double integrator_authority = 0.25;
};

/// \brief Full-state LQR with integral augmentation, designed offline from a trim point.
///
/// Two decoupled designs are performed on the reduced models produced by
/// aether::analysis::extractReducedModels:
///
/// **Longitudinal** — state
/// \f$[\delta u,\ \delta w,\ \delta q,\ \delta\theta,\ h-h_c,\ \int(h-h_c),\ \int(V_a-V_{a,c})]\f$,
/// inputs \f$[\delta_e,\ \delta_t]\f$. The altitude row uses the exact linearisation of
/// \f$\dot h = u\sin\theta - w\cos\theta\f$ about the trim attitude, and the airspeed row uses
/// \f$\delta V_a = (u_0\delta u + w_0\delta w)/V_{a0}\f$.
///
/// **Lateral** — state
/// \f$[\delta v,\ \delta p,\ \delta r,\ \delta\phi,\ \chi-\chi_c,\ \int(\chi-\chi_c)]\f$,
/// inputs \f$[\delta_a,\ \delta_r]\f$, with
/// \f$\dot\chi \approx \delta r/\cos\theta_0\f$.
///
/// The control law is \f$u = u_{trim} - K x_{aug}\f$. Integrator states are clamped and use
/// conditional integration whenever the corresponding actuator is saturated.
class LqrAutopilot final : public Autopilot {
 public:
  /// \param trim   Converged trim point the design is based on.
  /// \param linear Linearised model at that trim point.
  /// \param weights LQR weights.
  /// \param limits Command and integrator limits.
  /// \param params Airframe parameters (actuator saturations).
  LqrAutopilot(const analysis::TrimResult& trim, const analysis::LinearModel& linear,
               const LqrWeights& weights, const LqrAutopilotLimits& limits,
               const dynamics::AircraftParameters& params);

  std::string name() const override { return "lqr"; }
  ControlInput update(const AutopilotCommand& cmd, const VehicleFeedback& fb,
                      double dt) override;
  void reset(const TrimReference& trim) override;
  const AutopilotDiagnostics& diagnostics() const override { return diag_; }

  /// Longitudinal gain matrix, 2x7.
  const MatX& longitudinalGain() const { return K_lon_; }
  /// Lateral gain matrix, 2x6.
  const MatX& lateralGain() const { return K_lat_; }
  /// Closed-loop eigenvalues of the augmented longitudinal design.
  const Eigen::VectorXcd& longitudinalClosedLoop() const { return lon_cl_; }
  /// Closed-loop eigenvalues of the augmented lateral design.
  const Eigen::VectorXcd& lateralClosedLoop() const { return lat_cl_; }
  /// Riccati residual norms (longitudinal, lateral) as a sanity check on the CARE solve.
  Eigen::Vector2d careResiduals() const { return care_residuals_; }

  /// Effective (possibly gain-derived) error-state and integrator saturations actually used.
  const LqrAutopilotLimits& effectiveLimits() const { return limits_; }

 private:
  dynamics::AircraftParameters params_;
  LqrAutopilotLimits limits_;

  MatX K_lon_, K_lat_;
  Eigen::VectorXcd lon_cl_, lat_cl_;
  Eigen::Vector2d care_residuals_ = Eigen::Vector2d::Zero();

  // Trim reference values.
  double u0_ = 0.0, v0_ = 0.0, w0_ = 0.0, theta0_ = 0.0, phi0_ = 0.0, Va0_ = 0.0;
  ControlInput trim_{};

  // Integrator states.
  double int_altitude_ = 0.0;
  double int_airspeed_ = 0.0;
  double int_course_ = 0.0;

  double altitude_cmd_filtered_ = 0.0;
  bool altitude_cmd_initialised_ = false;

  AutopilotDiagnostics diag_;
};

}  // namespace aether::control
