#include "aether/env/Atmosphere.hpp"

#include <cmath>

namespace aether::env {

AtmosphereState Atmosphere::at(double altitude_m) const {
  using namespace constants;
  AtmosphereState s;

  if (altitude_m <= kTropopauseAltitude) {
    const double T = kTemperatureSeaLevel - kLapseRate * altitude_m;
    // Guard against the (non-physical) extrapolation crossing absolute zero.
    const double Tsafe = T > 1.0 ? T : 1.0;
    const double exponent = kGravity / (kLapseRate * kGasConstantAir);
    s.temperature = Tsafe;
    s.pressure = kPressureSeaLevel * std::pow(Tsafe / kTemperatureSeaLevel, exponent);
  } else {
    const double T11 = kTemperatureSeaLevel - kLapseRate * kTropopauseAltitude;
    const double exponent = kGravity / (kLapseRate * kGasConstantAir);
    const double p11 = kPressureSeaLevel * std::pow(T11 / kTemperatureSeaLevel, exponent);
    s.temperature = T11;
    s.pressure = p11 * std::exp(-kGravity * (altitude_m - kTropopauseAltitude) /
                                (kGasConstantAir * T11));
  }

  s.density = density_scale_ * s.pressure / (kGasConstantAir * s.temperature);
  s.speed_of_sound = std::sqrt(kGamma * kGasConstantAir * s.temperature);
  return s;
}

}  // namespace aether::env
