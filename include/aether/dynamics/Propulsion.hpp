/// \file Propulsion.hpp
/// \brief Momentum-theory propeller/motor model.
#pragma once

#include "aether/core/Types.hpp"
#include "aether/dynamics/AircraftParameters.hpp"

namespace aether::dynamics {

/// \brief Single tractor propeller aligned with the body x-axis.
///
/// Thrust follows the actuator-disk (momentum theory) form used for small UAVs
/// (Beard & McLain 2012, eq. 4.19):
/// \f[ T = \tfrac12 \rho S_{prop} C_{prop}\left[(k_{motor}\delta_t)^2 - V_a^2\right] \f]
/// so the propeller produces negative (windmilling) thrust when the airspeed exceeds the
/// slipstream exit velocity. The motor reaction torque about the body x-axis is
/// \f$ M_x = -k_{T_p}(k_\Omega \delta_t)^2 \f$, i.e. a right-handed propeller rolls the
/// airframe to the left.
class Propulsion {
 public:
  explicit Propulsion(const AircraftParameters& params) : p_(params) {}

  /// Thrust along body x [N].
  /// \param throttle Throttle setting [-], clamped to the configured limits.
  /// \param airspeed True airspeed [m/s].
  /// \param density Air density [kg/m^3].
  double thrust(double throttle, double airspeed, double density) const;

  /// Propeller reaction torque about body x [N*m] (negative for positive throttle).
  /// \param throttle Throttle setting [-].
  double reactionTorque(double throttle) const;

  /// Force and moment contribution of the propulsion system in body axes.
  Wrench wrench(double throttle, double airspeed, double density) const;

 private:
  AircraftParameters p_;
};

}  // namespace aether::dynamics
