#include "aether/dynamics/RigidBody.hpp"

#include "aether/math/Rotation.hpp"

namespace aether::dynamics {

StateVec AircraftState::vec() const {
  StateVec x;
  x.segment<3>(StateIndex::kPosN) = position;
  x.segment<3>(StateIndex::kVelU) = velocity;
  x.segment<4>(StateIndex::kQuatW) = quaternion;
  x.segment<3>(StateIndex::kRateP) = rate;
  return x;
}

AircraftState AircraftState::fromVec(const StateVec& x) {
  AircraftState s;
  s.position = x.segment<3>(StateIndex::kPosN);
  s.velocity = x.segment<3>(StateIndex::kVelU);
  s.quaternion = x.segment<4>(StateIndex::kQuatW);
  s.rate = x.segment<3>(StateIndex::kRateP);
  return s;
}

Vec3 AircraftState::euler() const { return math::quatToEuler(quaternion); }

RigidBody6DOF::RigidBody6DOF(const AircraftParameters& params)
    : p_(params), aero_(params), prop_(params) {
  p_.validate();
  J_ = p_.inertia();
  Jinv_ = p_.inertiaInverse();
}

Vec3 RigidBody6DOF::relativeVelocityBody(const StateVec& x, const Vec3& wind_ned) {
  const Vec4 q = x.segment<4>(StateIndex::kQuatW);
  const Vec3 vb = x.segment<3>(StateIndex::kVelU);
  return vb - math::rotateNedToBody(q, wind_ned);
}

Wrench RigidBody6DOF::totalWrench(const StateVec& x, const ControlInput& u,
                                  const EnvironmentSample& env,
                                  DynamicsDiagnostics* diag) const {
  const Vec4 q = x.segment<4>(StateIndex::kQuatW);
  const Vec3 omega = x.segment<3>(StateIndex::kRateP);

  const Vec3 v_rel = relativeVelocityBody(x, env.wind_ned);
  const AirData air = computeAirData(v_rel, env.density);

  const Wrench w_aero = aero_.wrench(air, omega, u);
  const Wrench w_prop = prop_.wrench(u.throttle, air.airspeed, env.density);

  // Gravity: constant [0,0,mg] in NED, rotated into body axes.
  const Vec3 g_ned(0.0, 0.0, p_.mass * env.gravity);
  const Vec3 g_body = math::rotateNedToBody(q, g_ned);

  Wrench total;
  total.force = w_aero.force + w_prop.force + g_body;
  total.moment = w_aero.moment + w_prop.moment;

  if (diag) {
    diag->air = air;
    diag->aero = w_aero;
    diag->propulsion = w_prop;
    diag->gravity_body = g_body;
    diag->specific_force = (w_aero.force + w_prop.force) / p_.mass;
  }
  return total;
}

StateVec RigidBody6DOF::derivative(const StateVec& x, const ControlInput& u,
                                   const EnvironmentSample& env,
                                   DynamicsDiagnostics* diag) const {
  const Vec4 q_raw = x.segment<4>(StateIndex::kQuatW);
  const Vec4 q = math::quatNormalize(q_raw);
  const Vec3 vb = x.segment<3>(StateIndex::kVelU);
  const Vec3 omega = x.segment<3>(StateIndex::kRateP);

  const Wrench w = totalWrench(x, u, env, diag);

  StateVec dx = StateVec::Zero();

  // Translational kinematics: velocity of the CG in NED.
  dx.segment<3>(StateIndex::kPosN) = math::rotateBodyToNed(q, vb);

  // Translational dynamics in the rotating body frame (transport theorem).
  const Vec3 vdot = w.force / p_.mass - omega.cross(vb);
  dx.segment<3>(StateIndex::kVelU) = vdot;

  // Quaternion kinematics with Baumgarte norm stabilisation.
  const Vec4 omega_quat(0.0, omega.x(), omega.y(), omega.z());
  const double norm_err = 1.0 - q_raw.squaredNorm();
  dx.segment<4>(StateIndex::kQuatW) =
      0.5 * math::quatMultiply(q_raw, omega_quat) + k_quat_ * norm_err * q_raw;

  // Rotational dynamics: Euler's equation with the full inertia tensor (Jxz coupling).
  dx.segment<3>(StateIndex::kRateP) = Jinv_ * (w.moment - omega.cross(J_ * omega));

  if (diag) diag->accel_body = vdot;
  return dx;
}

}  // namespace aether::dynamics
