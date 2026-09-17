/// \file RigidBody.hpp
/// \brief Nonlinear 6-DOF rigid-body flight dynamics of the airframe.
#pragma once

#include "aether/core/Types.hpp"
#include "aether/dynamics/Aerodynamics.hpp"
#include "aether/dynamics/AircraftParameters.hpp"
#include "aether/dynamics/Propulsion.hpp"

namespace aether::dynamics {

/// \brief Convenience view of the packed 13-element state vector.
struct AircraftState {
  Vec3 position = Vec3::Zero();  ///< NED position \f$p^n\f$ [m] (down positive)
  Vec3 velocity = Vec3::Zero();  ///< Body-frame velocity \f$v^b=(u,v,w)\f$ [m/s]
  Vec4 quaternion = Vec4(1, 0, 0, 0);  ///< Attitude \f$q_{nb}\f$ [-]
  Vec3 rate = Vec3::Zero();      ///< Body angular rate \f$\omega^b=(p,q,r)\f$ [rad/s]

  /// Pack into the 13x1 state vector.
  StateVec vec() const;
  /// Unpack from the 13x1 state vector.
  static AircraftState fromVec(const StateVec& x);
  /// Altitude above the NED origin [m], \f$h = -p_D\f$.
  double altitude() const { return -position.z(); }
  /// Euler angles \f$(\phi,\theta,\psi)\f$ [rad].
  Vec3 euler() const;
};

/// \brief Environmental sample seen by the airframe at one instant.
struct EnvironmentSample {
  Vec3 wind_ned = Vec3::Zero();  ///< Total wind velocity in NED [m/s]
  double density = 1.225;        ///< Air density [kg/m^3]
  double gravity = 9.80665;      ///< Gravitational acceleration [m/s^2]
};

/// \brief Diagnostic quantities produced as a by-product of a derivative evaluation.
struct DynamicsDiagnostics {
  AirData air;                      ///< Air data at the evaluation point.
  Wrench aero;                      ///< Aerodynamic wrench in body axes.
  Wrench propulsion;                ///< Propulsive wrench in body axes.
  Vec3 gravity_body = Vec3::Zero(); ///< Gravity force in body axes [N]
  Vec3 specific_force = Vec3::Zero(); ///< Non-gravitational specific force in body axes [m/s^2]
                                      ///< (this is what an ideal accelerometer measures)
  Vec3 accel_body = Vec3::Zero();   ///< \f$\dot v^b\f$ [m/s^2]
};

/// \brief Nonlinear six-degree-of-freedom rigid-body model.
///
/// State \f$x = [p^n, v^b, q_{nb}, \omega^b]\f$ (13 elements). Equations of motion:
/// \f{align*}{
///   \dot p^n &= R(q_{nb})\, v^b
///   \\ \dot v^b &= \frac{1}{m} F^b - \omega^b \times v^b
///   \\ \dot q_{nb} &= \tfrac12\, q_{nb} \otimes [0,\ \omega^b]^\top
///                  + k_q (1-\|q\|^2)\, q_{nb}
///   \\ \dot\omega^b &= J^{-1}\left(M^b - \omega^b \times (J\omega^b)\right)
/// \f}
/// The last term of the quaternion kinematics is Baumgarte-style norm stabilisation: it is
/// zero on the unit sphere and pulls numerical drift back towards it between the explicit
/// renormalisations performed by the integrator.
class RigidBody6DOF {
 public:
  explicit RigidBody6DOF(const AircraftParameters& params);

  /// Evaluate \f$\dot x = f(x,u)\f$.
  /// \param x State vector (13).
  /// \param u Control deflections (already at the actuator, not commanded).
  /// \param env Environmental sample (wind, density, gravity).
  /// \param diag Optional output for forces, air data and specific force.
  StateVec derivative(const StateVec& x, const ControlInput& u, const EnvironmentSample& env,
                      DynamicsDiagnostics* diag = nullptr) const;

  /// Total body-axis force and moment acting on the airframe (including gravity).
  Wrench totalWrench(const StateVec& x, const ControlInput& u, const EnvironmentSample& env,
                     DynamicsDiagnostics* diag = nullptr) const;

  /// Air-relative velocity in body axes [m/s].
  static Vec3 relativeVelocityBody(const StateVec& x, const Vec3& wind_ned);

  /// Airframe parameters.
  const AircraftParameters& params() const { return p_; }
  /// Aerodynamic sub-model.
  const Aerodynamics& aero() const { return aero_; }
  /// Propulsion sub-model.
  const Propulsion& propulsion() const { return prop_; }

  /// Gain of the quaternion norm-stabilisation term [1/s]. Default 1.0.
  void setQuaternionStabilisationGain(double k) { k_quat_ = k; }

 private:
  AircraftParameters p_;
  Aerodynamics aero_;
  Propulsion prop_;
  Mat3 J_;
  Mat3 Jinv_;
  double k_quat_ = 1.0;
};

}  // namespace aether::dynamics
