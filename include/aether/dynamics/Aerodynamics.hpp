/// \file Aerodynamics.hpp
/// \brief Aerodynamic force and moment model with a blended post-stall extension.
#pragma once

#include "aether/core/Types.hpp"
#include "aether/dynamics/AircraftParameters.hpp"

namespace aether::dynamics {

/// \brief Air-relative flow state derived from the body-frame relative velocity.
struct AirData {
  double airspeed = 0.0;    ///< True airspeed \f$V_a = \|v^b_{rel}\|\f$ [m/s]
  double alpha = 0.0;       ///< Angle of attack \f$\alpha=\mathrm{atan2}(w_r,u_r)\f$ [rad]
  double beta = 0.0;        ///< Sideslip angle \f$\beta=\arcsin(v_r/V_a)\f$ [rad]
  double dynamic_pressure = 0.0;  ///< \f$\bar q = \tfrac12\rho V_a^2\f$ [Pa]
  double density = 0.0;     ///< Air density used [kg/m^3]
  Vec3 velocity_rel = Vec3::Zero();  ///< Air-relative velocity in body axes [m/s]
};

/// \brief Compute air data from the body-frame air-relative velocity.
///
/// Numerically safe at very low airspeed: \f$\alpha\f$ uses `atan2` (well defined except at
/// the origin, where it returns 0) and \f$\beta\f$ clamps its argument to \f$[-1,1]\f$.
/// \param v_rel_body Air-relative velocity in body axes [m/s].
/// \param density Air density [kg/m^3].
AirData computeAirData(const Vec3& v_rel_body, double density);

/// \brief Dimensionless aerodynamic coefficients at one flight condition.
struct AeroCoefficients {
  double CL = 0.0;  ///< Lift coefficient (stability axes) [-]
  double CD = 0.0;  ///< Drag coefficient (stability axes) [-]
  double CY = 0.0;  ///< Side-force coefficient (body y) [-]
  double Cl = 0.0;  ///< Rolling-moment coefficient [-]
  double Cm = 0.0;  ///< Pitching-moment coefficient [-]
  double Cn = 0.0;  ///< Yawing-moment coefficient [-]
};

/// \brief Aerodynamic model of the airframe.
///
/// Forces are built in stability axes (lift/drag) and rotated into the body frame through the
/// angle of attack, with the side force acting directly along body \a y — the standard
/// small-UAV formulation of Beard & McLain (2012, eq. 4.18-4.19). Rate derivatives are
/// non-dimensionalised with \f$c/(2V_a)\f$ (longitudinal) and \f$b/(2V_a)\f$ (lateral).
class Aerodynamics {
 public:
  explicit Aerodynamics(const AircraftParameters& params) : p_(params) {}

  /// Lift coefficient including the optional flat-plate post-stall blend.
  /// \param alpha Angle of attack [rad].
  double liftCoefficient(double alpha) const;

  /// Drag coefficient from the quadratic polar \f$C_{D0}+C_L^2/(\pi e AR)\f$.
  /// \param alpha Angle of attack [rad].
  /// \param elevator Elevator deflection [rad].
  double dragCoefficient(double alpha, double elevator) const;

  /// Full non-dimensional coefficient set at the given condition.
  /// \param air Air data (airspeed, alpha, beta).
  /// \param omega_body Body angular rate \f$(p,q,r)\f$ [rad/s].
  /// \param u Control deflections.
  AeroCoefficients coefficients(const AirData& air, const Vec3& omega_body,
                                const ControlInput& u) const;

  /// Aerodynamic force and moment about the CG, in body axes.
  /// \param air Air data.
  /// \param omega_body Body angular rate [rad/s].
  /// \param u Control deflections.
  /// \return Force [N] and moment [N*m] in body axes.
  Wrench wrench(const AirData& air, const Vec3& omega_body, const ControlInput& u) const;

  /// Airframe parameters in use.
  const AircraftParameters& params() const { return p_; }

 private:
  AircraftParameters p_;
};

}  // namespace aether::dynamics
