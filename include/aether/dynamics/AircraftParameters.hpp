/// \file AircraftParameters.hpp
/// \brief Mass, geometry, aerodynamic and propulsion parameters of the simulated airframe.
///
/// **All values are SI.** The shipped parameter set (`configs/aircraft/aether6_uav.yaml`)
/// describes the *Aether-6*, a **synthetic** 13.5 kg fixed-wing UAV. The model *structure*
/// follows the standard textbook formulation (Beard & McLain, *Small Unmanned Aircraft:
/// Theory and Practice*, Princeton University Press, 2012, Ch. 4; Stevens, Lewis & Johnson,
/// *Aircraft Control and Simulation*, 3rd ed., Wiley, 2016, Ch. 2), but the numerical
/// coefficients are **not** taken from any published aircraft. They were derived from
/// classical thin-aerofoil / tail-volume relations for a plausible airframe geometry and are
/// documented in `docs/model.md`. Do not treat them as representing a real vehicle.
#pragma once

#include <string>

#include "aether/core/Types.hpp"

namespace aether::dynamics {

/// \brief Longitudinal aerodynamic coefficients (per radian unless noted).
struct LongitudinalCoefficients {
  double CL0 = 0.0;      ///< Lift coefficient at zero angle of attack [-]
  double CL_alpha = 0.0; ///< Lift-curve slope \f$\partial C_L/\partial\alpha\f$ [1/rad]
  double CL_q = 0.0;     ///< Lift due to pitch rate, non-dim by \f$qc/2V_a\f$ [1/rad]
  double CL_de = 0.0;    ///< Lift due to elevator [1/rad]

  double CD0 = 0.0;      ///< Parasite drag coefficient [-]
  double CD_q = 0.0;     ///< Drag due to pitch rate [1/rad]
  double CD_de = 0.0;    ///< Drag due to elevator (quadratic term uses |de|) [1/rad]
  double oswald = 0.9;   ///< Oswald span efficiency factor \a e [-]

  double Cm0 = 0.0;      ///< Pitching moment at zero angle of attack [-]
  double Cm_alpha = 0.0; ///< Pitch stiffness \f$\partial C_m/\partial\alpha\f$ [1/rad] (negative = stable)
  double Cm_q = 0.0;     ///< Pitch damping, non-dim by \f$qc/2V_a\f$ [1/rad] (negative = damped)
  double Cm_de = 0.0;    ///< Pitch control power [1/rad] (negative with the TE-down-positive convention)

  /// Blending parameters for the flat-plate post-stall extension (Beard & McLain eq. 4.10).
  double alpha_stall = 0.30;  ///< Stall angle of attack \f$\alpha_0\f$ [rad]
  double stall_sharpness = 50.0;  ///< Sigmoid sharpness \a M [1/rad]
  bool enable_stall_model = true; ///< Blend the linear model into a flat-plate model past stall.
};

/// \brief Lateral-directional aerodynamic coefficients (per radian).
struct LateralCoefficients {
  double CY0 = 0.0;   ///< Side force at zero sideslip [-]
  double CY_beta = 0.0; ///< Side force due to sideslip [1/rad] (negative)
  double CY_p = 0.0;  ///< Side force due to roll rate, non-dim by \f$pb/2V_a\f$ [1/rad]
  double CY_r = 0.0;  ///< Side force due to yaw rate, non-dim by \f$rb/2V_a\f$ [1/rad]
  double CY_da = 0.0; ///< Side force due to aileron [1/rad]
  double CY_dr = 0.0; ///< Side force due to rudder [1/rad] (positive with TE-left-positive)

  double Cl0 = 0.0;   ///< Rolling moment at zero sideslip [-]
  double Cl_beta = 0.0; ///< Dihedral effect [1/rad] (negative = stable)
  double Cl_p = 0.0;  ///< Roll damping [1/rad] (negative)
  double Cl_r = 0.0;  ///< Roll due to yaw rate [1/rad]
  double Cl_da = 0.0; ///< Roll control power [1/rad] (positive)
  double Cl_dr = 0.0; ///< Roll due to rudder [1/rad]

  double Cn0 = 0.0;   ///< Yawing moment at zero sideslip [-]
  double Cn_beta = 0.0; ///< Weathercock stability [1/rad] (positive = stable)
  double Cn_p = 0.0;  ///< Yaw due to roll rate [1/rad] (adverse yaw, negative)
  double Cn_r = 0.0;  ///< Yaw damping [1/rad] (negative)
  double Cn_da = 0.0; ///< Yaw due to aileron [1/rad] (adverse, negative)
  double Cn_dr = 0.0; ///< Yaw control power [1/rad] (negative with TE-left-positive)
};

/// \brief Propeller / motor parameters for the momentum-theory thrust model.
struct PropulsionParameters {
  double disk_area = 0.05;     ///< Propeller disk area \f$S_{prop}\f$ [m^2]
  double efficiency = 1.0;     ///< Propeller efficiency factor \f$C_{prop}\f$ [-]
  double k_motor = 45.0;       ///< Motor constant: exit velocity at full throttle [m/s]
  double k_torque = 5.0e-6;    ///< Reaction-torque constant \f$k_{T_p}\f$ [N*m*s^2]
  double k_omega = 300.0;      ///< Motor speed constant \f$k_\Omega\f$ [rad/s per unit throttle]
  Vec3 thrust_offset = Vec3::Zero();  ///< Thrust application point relative to the CG, body axes [m]
};

/// \brief Actuator magnitude limits, rate limits and first-order lag.
struct ActuatorLimits {
  double elevator_min = -0.436;  ///< Minimum elevator deflection [rad] (-25 deg)
  double elevator_max = 0.436;   ///< Maximum elevator deflection [rad] (+25 deg)
  double aileron_min = -0.349;   ///< Minimum aileron deflection [rad] (-20 deg)
  double aileron_max = 0.349;    ///< Maximum aileron deflection [rad] (+20 deg)
  double rudder_min = -0.436;    ///< Minimum rudder deflection [rad] (-25 deg)
  double rudder_max = 0.436;     ///< Maximum rudder deflection [rad] (+25 deg)
  double throttle_min = 0.0;     ///< Minimum throttle [-]
  double throttle_max = 1.0;     ///< Maximum throttle [-]

  double surface_rate_limit = 5.24;  ///< Control-surface rate limit [rad/s] (300 deg/s)
  double throttle_rate_limit = 2.0;  ///< Throttle rate limit [1/s]
  double surface_time_constant = 0.04;  ///< First-order actuator lag for surfaces [s]
  double throttle_time_constant = 0.20; ///< First-order lag for the propulsion system [s]
};

/// \brief Complete airframe description.
struct AircraftParameters {
  std::string name = "aether6-synthetic";  ///< Human-readable airframe name.
  bool synthetic = true;  ///< True when the coefficients are synthetic (not a real aircraft).

  double mass = 13.5;   ///< Vehicle mass [kg]
  double Jx = 0.824;    ///< Roll inertia about body x [kg*m^2]
  double Jy = 1.135;    ///< Pitch inertia about body y [kg*m^2]
  double Jz = 1.759;    ///< Yaw inertia about body z [kg*m^2]
  double Jxz = 0.120;   ///< Product of inertia (x-z plane) [kg*m^2]

  double wing_area = 0.52;  ///< Reference wing area \a S [m^2]
  double wing_span = 2.60;  ///< Wing span \a b [m]
  double mean_chord = 0.20; ///< Mean aerodynamic chord \a c [m]

  LongitudinalCoefficients lon;  ///< Longitudinal coefficient set.
  LateralCoefficients lat;       ///< Lateral-directional coefficient set.
  PropulsionParameters prop;     ///< Propulsion parameters.
  ActuatorLimits actuators;      ///< Actuator limits.

  /// Aspect ratio \f$AR = b^2/S\f$ [-].
  double aspectRatio() const { return wing_span * wing_span / wing_area; }

  /// Inertia tensor in body axes [kg*m^2]. The airframe is assumed symmetric about
  /// the x-z plane, so \f$J_{xy}=J_{yz}=0\f$.
  Mat3 inertia() const {
    Mat3 J;
    J << Jx, 0.0, -Jxz,
         0.0, Jy, 0.0,
        -Jxz, 0.0, Jz;
    return J;
  }

  /// Inverse inertia tensor [1/(kg*m^2)].
  Mat3 inertiaInverse() const { return inertia().inverse(); }

  /// Throw std::invalid_argument if the parameter set is not physically usable.
  void validate() const;
};

/// \brief The default synthetic Aether-6 parameter set, identical to
/// `configs/aircraft/aether6_uav.yaml`. Provided so unit tests and library users can
/// construct a valid airframe without touching the filesystem.
AircraftParameters defaultAircraft();

}  // namespace aether::dynamics
