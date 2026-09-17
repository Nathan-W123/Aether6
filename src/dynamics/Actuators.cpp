#include "aether/dynamics/Actuators.hpp"

#include <algorithm>

namespace aether::dynamics {

ControlInput ActuatorBank::saturate(const ControlInput& c) const {
  const auto& a = p_.actuators;
  ControlInput out;
  out.elevator = std::clamp(c.elevator, a.elevator_min, a.elevator_max);
  out.aileron = std::clamp(c.aileron, a.aileron_min, a.aileron_max);
  out.rudder = std::clamp(c.rudder, a.rudder_min, a.rudder_max);
  out.throttle = std::clamp(c.throttle, a.throttle_min, a.throttle_max);
  return out;
}

const ControlInput& ActuatorBank::update(const ControlInput& cmd, double dt) {
  const auto& a = p_.actuators;
  const ControlInput target = saturate(cmd);

  auto channel = [dt](double current, double demand, double tau, double rate_limit,
                      double lo, double hi) {
    const double rate = std::clamp((demand - current) / tau, -rate_limit, rate_limit);
    return std::clamp(current + rate * dt, lo, hi);
  };

  state_.elevator = channel(state_.elevator, target.elevator, a.surface_time_constant,
                            a.surface_rate_limit, a.elevator_min, a.elevator_max);
  state_.aileron = channel(state_.aileron, target.aileron, a.surface_time_constant,
                           a.surface_rate_limit, a.aileron_min, a.aileron_max);
  state_.rudder = channel(state_.rudder, target.rudder, a.surface_time_constant,
                          a.surface_rate_limit, a.rudder_min, a.rudder_max);
  state_.throttle = channel(state_.throttle, target.throttle, a.throttle_time_constant,
                            a.throttle_rate_limit, a.throttle_min, a.throttle_max);
  return state_;
}

}  // namespace aether::dynamics
