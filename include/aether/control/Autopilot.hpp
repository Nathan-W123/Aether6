/// \file Autopilot.hpp
/// \brief Common interface and feedback types for the flight-control laws.
#pragma once

#include <algorithm>
#include <memory>
#include <string>

#include "aether/core/Types.hpp"
#include "aether/dynamics/AircraftParameters.hpp"

namespace aether::control {

/// \brief The vehicle information a controller is allowed to use.
///
/// This is deliberately a *separate* type from the truth state: the simulator fills it
/// either from the truth model or from the navigation filter, so the control laws cannot
/// accidentally reach into quantities a real autopilot would not have.
struct VehicleFeedback {
  Vec3 position_ned = Vec3::Zero();  ///< NED position [m]
  Vec3 velocity_ned = Vec3::Zero();  ///< NED (ground) velocity [m/s]
  Vec3 velocity_body = Vec3::Zero(); ///< Body-frame ground velocity [m/s]
  Vec3 velocity_air_body = Vec3::Zero();  ///< Body-frame **air-relative** velocity [m/s]
  Vec3 wind_ned = Vec3::Zero();      ///< Wind estimate (or truth) in NED [m/s]
  double altitude = 0.0;             ///< Altitude above the NED origin [m]
  double climb_rate = 0.0;           ///< Positive-up climb rate [m/s]
  double airspeed = 0.0;             ///< True airspeed [m/s]
  double alpha = 0.0;                ///< Angle of attack [rad]
  double beta = 0.0;                 ///< Sideslip [rad]
  double roll = 0.0;                 ///< Euler roll \f$\phi\f$ [rad]
  double pitch = 0.0;                ///< Euler pitch \f$\theta\f$ [rad]
  double yaw = 0.0;                  ///< Euler yaw \f$\psi\f$ [rad]
  double course = 0.0;               ///< Ground-track course \f$\chi\f$ [rad]
  Vec3 rate = Vec3::Zero();          ///< Body angular rate \f$(p,q,r)\f$ [rad/s]
};

/// \brief Outer-loop commands produced by the guidance layer.
struct AutopilotCommand {
  double altitude = 0.0;   ///< Commanded altitude [m]
  double airspeed = 25.0;  ///< Commanded true airspeed [m/s]
  double course = 0.0;     ///< Commanded ground-track course [rad]
};

/// \brief The trimmed condition a controller is initialised from.
///
/// Controllers preload their integrators from this so the loop starts out holding the
/// trimmed attitude and actuator positions instead of ramping up to them.
struct TrimReference {
  ControlInput controls;   ///< Trimmed control deflections.
  double roll = 0.0;       ///< Trimmed bank angle [rad]
  double pitch = 0.0;      ///< Trimmed pitch attitude [rad]
  double airspeed = 25.0;  ///< Trim airspeed [m/s]
  double altitude = 0.0;   ///< Trim altitude [m]
};

/// \brief Angle-of-attack envelope protection shared by every control law.
///
/// Without it, an aggressive altitude or pitch demand can drive the aircraft past the stall
/// and then *hold* it there: the altitude error keeps growing as the vehicle sinks, which
/// commands yet more nose-up elevator. The protection blends in nose-down authority above
/// `alpha_max` and takes priority over the outer loops, which is what a real UAV autopilot
/// does. Its authority is proportional to the exceedance, so it does nothing in normal flight.
struct EnvelopeProtection {
  bool enabled = true;      ///< Enable the protection.
  double alpha_max = 0.24;  ///< Protection threshold on angle of attack [rad] (13.8 deg)
  double alpha_gain = 2.5;  ///< Nose-down elevator per radian of exceedance [rad/rad]
  double rate_gain = 0.35;  ///< Pitch-rate damping inside the protection [rad/(rad/s)]
  double blend_width = 0.09;///< Exceedance over which the protection fades in [rad] (5 deg)
};

/// \brief Fraction of protection authority currently demanded, in [0, 1].
///
/// Zero below `alpha_max`, ramping linearly to one over `blend_width` of exceedance. The
/// control laws use it both to blend the protective elevator in and to fade the *bank*
/// command out: in a turn it is the load factor that drives angle of attack, so giving up
/// course tracking is the cheapest way to buy back stall margin.
inline double envelopeProtectionAuthority(double alpha, const EnvelopeProtection& protection) {
  if (!protection.enabled) return 0.0;
  const double excess = alpha - protection.alpha_max;
  if (excess <= 0.0) return 0.0;
  const double width = protection.blend_width > 1e-9 ? protection.blend_width : 1e-9;
  return std::min(1.0, excess / width);
}

/// \brief Blend nose-down elevator in above the angle-of-attack limit.
///
/// Positive elevator is trailing-edge down (nose-down). The protective demand is
/// \f$ \delta_{e,prot} = k_\alpha (\alpha - \alpha_{max}) + k_q q \f$ — the pitch-rate term
/// matters: a pure proportional override switches on and off at the short-period frequency
/// and sets up a bang-bang limit cycle. The protection is faded in linearly over
/// `blend_width` of exceedance rather than switched, for the same reason, and it only ever
/// pushes nose-down (it never reduces a nose-down command from the control law).
///
/// \param elevator Elevator demanded by the control law [rad].
/// \param alpha Measured or estimated angle of attack [rad].
/// \param pitch_rate Body pitch rate q [rad/s].
/// \param protection Protection settings.
/// \param active Optional output flag set when the protection is contributing.
/// \return The protected elevator command [rad].
inline double applyEnvelopeProtection(double elevator, double alpha, double pitch_rate,
                                      const EnvelopeProtection& protection,
                                      bool* active = nullptr) {
  if (active) *active = false;
  if (!protection.enabled) return elevator;
  const double excess = alpha - protection.alpha_max;
  if (excess <= 0.0) return elevator;
  const double protective = protection.alpha_gain * excess + protection.rate_gain * pitch_rate;
  if (protective <= elevator) return elevator;
  const double weight = envelopeProtectionAuthority(alpha, protection);
  if (active) *active = true;
  return elevator + weight * (protective - elevator);
}

/// \brief Internal autopilot signals exposed for logging and diagnosis.
struct AutopilotDiagnostics {
  double roll_command = 0.0;   ///< Commanded bank angle [rad]
  double pitch_command = 0.0;  ///< Commanded pitch attitude [rad]
  double altitude_command_filtered = 0.0;  ///< Rate-limited altitude command [m]
  double course_error = 0.0;   ///< Wrapped course error [rad]
  double altitude_error = 0.0; ///< Altitude error [m]
  double airspeed_error = 0.0; ///< Airspeed error [m/s]
  bool envelope_protection_active = false;  ///< True while alpha protection is contributing
};

/// \brief Abstract flight-control law.
class Autopilot {
 public:
  virtual ~Autopilot() = default;

  /// Short identifier, e.g. "pid" or "lqr".
  virtual std::string name() const = 0;

  /// Compute the commanded control deflections.
  /// \param cmd Outer-loop commands.
  /// \param fb Vehicle feedback (truth or estimate).
  /// \param dt Controller time step [s].
  virtual ControlInput update(const AutopilotCommand& cmd, const VehicleFeedback& fb,
                              double dt) = 0;

  /// Reset all internal states, seeding the integrators from the trimmed condition.
  virtual void reset(const TrimReference& trim) = 0;

  /// Diagnostics from the most recent update.
  virtual const AutopilotDiagnostics& diagnostics() const = 0;
};

}  // namespace aether::control
