#include "aether/dynamics/Propulsion.hpp"

#include <algorithm>

#include "aether/math/Rotation.hpp"

namespace aether::dynamics {

double Propulsion::thrust(double throttle, double airspeed, double density) const {
  const double dt = std::clamp(throttle, p_.actuators.throttle_min, p_.actuators.throttle_max);
  const double ve = p_.prop.k_motor * dt;
  return 0.5 * density * p_.prop.disk_area * p_.prop.efficiency *
         (ve * ve - airspeed * airspeed);
}

double Propulsion::reactionTorque(double throttle) const {
  const double dt = std::clamp(throttle, p_.actuators.throttle_min, p_.actuators.throttle_max);
  const double omega = p_.prop.k_omega * dt;
  return -p_.prop.k_torque * omega * omega;
}

Wrench Propulsion::wrench(double throttle, double airspeed, double density) const {
  Wrench w;
  const double T = thrust(throttle, airspeed, density);
  w.force = Vec3(T, 0.0, 0.0);
  w.moment = Vec3(reactionTorque(throttle), 0.0, 0.0);
  // Moment from a thrust line offset from the CG: M = r x F.
  if (!p_.prop.thrust_offset.isZero(0.0)) {
    w.moment += p_.prop.thrust_offset.cross(w.force);
  }
  return w;
}

}  // namespace aether::dynamics
