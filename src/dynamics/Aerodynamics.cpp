#include "aether/dynamics/Aerodynamics.hpp"

#include <algorithm>
#include <cmath>

#include "aether/core/Constants.hpp"

namespace aether::dynamics {

AirData computeAirData(const Vec3& v_rel_body, double density) {
  AirData a;
  a.velocity_rel = v_rel_body;
  a.density = density;
  a.airspeed = v_rel_body.norm();

  const double ur = v_rel_body.x();
  const double vr = v_rel_body.y();
  const double wr = v_rel_body.z();

  // atan2(0,0) is defined as 0 by IEEE-754, so this is safe at rest.
  a.alpha = std::atan2(wr, ur);

  const double denom = std::max(a.airspeed, constants::kMinAirspeed);
  a.beta = std::asin(std::clamp(vr / denom, -1.0, 1.0));

  a.dynamic_pressure = 0.5 * density * a.airspeed * a.airspeed;
  return a;
}

double Aerodynamics::liftCoefficient(double alpha) const {
  const double cl_linear = p_.lon.CL0 + p_.lon.CL_alpha * alpha;
  if (!p_.lon.enable_stall_model) return cl_linear;

  // Sigmoid blending function (Beard & McLain 2012, eq. 4.10). Exponentials are
  // clamped to avoid overflow for large |alpha|.
  const double M = p_.lon.stall_sharpness;
  const double a0 = p_.lon.alpha_stall;
  const double e1 = std::exp(std::clamp(-M * (alpha - a0), -500.0, 500.0));
  const double e2 = std::exp(std::clamp(M * (alpha + a0), -500.0, 500.0));
  const double sigma = (1.0 + e1 + e2) / ((1.0 + e1) * (1.0 + e2));

  const double sa = std::sin(alpha);
  const double cl_plate = 2.0 * (alpha >= 0.0 ? 1.0 : -1.0) * sa * sa * std::cos(alpha);
  return (1.0 - sigma) * cl_linear + sigma * cl_plate;
}

double Aerodynamics::dragCoefficient(double alpha, double elevator) const {
  // Quadratic polar driven by the *linear* lift coefficient (Beard & McLain 2012, eq. 4.11):
  // past stall this keeps drag growing smoothly instead of collapsing with the blended CL.
  const double cl_linear = p_.lon.CL0 + p_.lon.CL_alpha * alpha;
  const double k = 1.0 / (constants::kPi * p_.lon.oswald * p_.aspectRatio());
  return p_.lon.CD0 + k * cl_linear * cl_linear + p_.lon.CD_de * std::abs(elevator);
}

AeroCoefficients Aerodynamics::coefficients(const AirData& air, const Vec3& omega,
                                            const ControlInput& u) const {
  AeroCoefficients c;

  // Non-dimensionalising factors. Va is clamped so the rate terms stay finite at very
  // low airspeed; the dynamic pressure still uses the true Va, so forces vanish at rest.
  const double Va = std::max(air.airspeed, constants::kMinAirspeed);
  const double lon_hat = p_.mean_chord / (2.0 * Va);
  const double lat_hat = p_.wing_span / (2.0 * Va);

  const double p_hat = omega.x() * lat_hat;
  const double q_hat = omega.y() * lon_hat;
  const double r_hat = omega.z() * lat_hat;

  c.CL = liftCoefficient(air.alpha) + p_.lon.CL_q * q_hat + p_.lon.CL_de * u.elevator;
  c.CD = dragCoefficient(air.alpha, u.elevator) + p_.lon.CD_q * q_hat;
  c.Cm = p_.lon.Cm0 + p_.lon.Cm_alpha * air.alpha + p_.lon.Cm_q * q_hat +
         p_.lon.Cm_de * u.elevator;

  c.CY = p_.lat.CY0 + p_.lat.CY_beta * air.beta + p_.lat.CY_p * p_hat +
         p_.lat.CY_r * r_hat + p_.lat.CY_da * u.aileron + p_.lat.CY_dr * u.rudder;
  c.Cl = p_.lat.Cl0 + p_.lat.Cl_beta * air.beta + p_.lat.Cl_p * p_hat +
         p_.lat.Cl_r * r_hat + p_.lat.Cl_da * u.aileron + p_.lat.Cl_dr * u.rudder;
  c.Cn = p_.lat.Cn0 + p_.lat.Cn_beta * air.beta + p_.lat.Cn_p * p_hat +
         p_.lat.Cn_r * r_hat + p_.lat.Cn_da * u.aileron + p_.lat.Cn_dr * u.rudder;
  return c;
}

Wrench Aerodynamics::wrench(const AirData& air, const Vec3& omega,
                            const ControlInput& u) const {
  const AeroCoefficients c = coefficients(air, omega, u);
  const double qS = air.dynamic_pressure * p_.wing_area;

  const double ca = std::cos(air.alpha);
  const double sa = std::sin(air.alpha);

  Wrench w;
  // Rotate the stability-axis (-D, -L) pair into body axes through alpha.
  w.force.x() = qS * (-c.CD * ca + c.CL * sa);
  w.force.y() = qS * c.CY;
  w.force.z() = qS * (-c.CD * sa - c.CL * ca);

  w.moment.x() = qS * p_.wing_span * c.Cl;
  w.moment.y() = qS * p_.mean_chord * c.Cm;
  w.moment.z() = qS * p_.wing_span * c.Cn;
  return w;
}

}  // namespace aether::dynamics
