/// \file Actuators.hpp
/// \brief Control-surface and throttle actuator dynamics: first-order lag, rate limit,
///        magnitude saturation.
#pragma once

#include "aether/core/Types.hpp"
#include "aether/dynamics/AircraftParameters.hpp"

namespace aether::dynamics {

/// \brief Actuator bank for elevator, aileron, rudder and throttle.
///
/// Each channel is modelled as a first-order lag \f$\dot\delta = (\delta_{cmd}-\delta)/\tau\f$
/// whose output slew is clipped to the configured rate limit and whose position is clipped to
/// the magnitude limits. Commands are saturated **before** the lag so the internal state never
/// integrates an unreachable demand.
class ActuatorBank {
 public:
  explicit ActuatorBank(const AircraftParameters& params) : p_(params) {}

  /// Set the actuator positions directly (used to start a run from a trimmed condition).
  void reset(const ControlInput& state) { state_ = saturate(state); }

  /// Advance the actuators by \a dt seconds toward \a cmd and return the new positions.
  /// \param cmd Commanded deflections.
  /// \param dt Time step [s].
  const ControlInput& update(const ControlInput& cmd, double dt);

  /// Current actuator positions (what the aerodynamic model sees).
  const ControlInput& state() const { return state_; }

  /// Clip a command to the configured magnitude limits.
  ControlInput saturate(const ControlInput& c) const;

 private:
  AircraftParameters p_;
  ControlInput state_{};
};

}  // namespace aether::dynamics
