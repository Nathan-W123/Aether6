/// \file Atmosphere.hpp
/// \brief 1976 U.S. Standard Atmosphere (ISA) model, troposphere + lower stratosphere.
#pragma once

#include "aether/core/Constants.hpp"

namespace aether::env {

/// \brief Thermodynamic state of the air at one altitude.
struct AtmosphereState {
  double density = constants::kRhoSeaLevel;          ///< \f$\rho\f$ [kg/m^3]
  double pressure = constants::kPressureSeaLevel;    ///< \f$p\f$ [Pa]
  double temperature = constants::kTemperatureSeaLevel;  ///< \f$T\f$ [K]
  double speed_of_sound = 340.294;                   ///< \f$a=\sqrt{\gamma R T}\f$ [m/s]
};

/// \brief ISA atmosphere valid from sea level to ~20 km geopotential altitude.
///
/// Troposphere (0..11 km): linear lapse \f$T = T_0 - L h\f$ with
/// \f$p = p_0 (T/T_0)^{g/(LR)}\f$.
/// Lower stratosphere (11..20 km): isothermal, \f$p = p_{11}\exp(-g(h-11000)/(RT_{11}))\f$.
/// Below sea level the troposphere relations are simply extrapolated.
class Atmosphere {
 public:
  /// Construct with an optional uniform density scale factor (used for Monte-Carlo dispersion).
  /// \param density_scale Multiplies the ISA density [-]; 1.0 is nominal.
  explicit Atmosphere(double density_scale = 1.0) : density_scale_(density_scale) {}

  /// Evaluate the atmosphere at geometric altitude \a altitude_m above mean sea level [m].
  AtmosphereState at(double altitude_m) const;

  /// Convenience accessor returning only density [kg/m^3].
  double density(double altitude_m) const { return at(altitude_m).density; }

  /// Density scale factor applied on top of ISA [-].
  double densityScale() const { return density_scale_; }

 private:
  double density_scale_;
};

}  // namespace aether::env
