#include "aether/control/PidAutopilot.hpp"

#include <algorithm>
#include <cmath>

#include "aether/core/Constants.hpp"
#include "aether/math/Rotation.hpp"

namespace aether::control {

PidAutopilotConfig PidAutopilot::defaultConfig(const dynamics::AircraftParameters& params) {
  const auto& a = params.actuators;
  PidAutopilotConfig c;

  // Inner loop: bank angle -> aileron. Roll subsidence is fast (tau ~ 0.06 s) so a
  // proportional-plus-rate loop with a light integrator is enough.
  c.roll_to_aileron.kp = 1.2;
  c.roll_to_aileron.ki = 0.15;
  c.roll_to_aileron.kd = 0.18;
  c.roll_to_aileron.output_min = a.aileron_min;
  c.roll_to_aileron.output_max = a.aileron_max;
  c.roll_to_aileron.anti_windup_gain = 2.0;

  // Inner loop: pitch attitude -> elevator (sign is negative because a positive elevator
  // deflection produces a nose-down moment).
  c.pitch_to_elevator.kp = -2.2;
  c.pitch_to_elevator.ki = -0.55;
  c.pitch_to_elevator.kd = -0.42;
  c.pitch_to_elevator.output_min = a.elevator_min;
  c.pitch_to_elevator.output_max = a.elevator_max;
  c.pitch_to_elevator.anti_windup_gain = 2.0;

  // Turn coordination: drive sideslip to zero with the rudder. A positive rudder deflection
  // (trailing edge left) makes a nose-LEFT (negative) yawing moment, and positive sideslip
  // must be removed with a nose-RIGHT moment, so this loop has negative gains.
  c.sideslip_to_rudder.kp = -1.4;
  c.sideslip_to_rudder.ki = -0.35;
  c.sideslip_to_rudder.kd = 0.0;
  c.sideslip_to_rudder.output_min = a.rudder_min;
  c.sideslip_to_rudder.output_max = a.rudder_max;
  c.sideslip_to_rudder.anti_windup_gain = 2.0;

  // Outer loop: course -> bank command.
  c.course_to_roll.kp = 1.1;
  c.course_to_roll.ki = 0.08;
  c.course_to_roll.kd = 0.0;
  c.course_to_roll.output_min = -c.max_bank;
  c.course_to_roll.output_max = c.max_bank;
  c.course_to_roll.anti_windup_gain = 1.0;

  // Outer loop: altitude -> pitch command.
  c.altitude_to_pitch.kp = 0.035;
  c.altitude_to_pitch.ki = 0.006;
  c.altitude_to_pitch.kd = 0.055;
  c.altitude_to_pitch.output_min = -c.max_pitch;
  c.altitude_to_pitch.output_max = c.max_pitch;
  c.altitude_to_pitch.anti_windup_gain = 1.0;
  c.altitude_to_pitch.derivative_filter_tau = 0.3;

  // Outer loop: airspeed -> throttle.
  c.airspeed_to_throttle.kp = 0.09;
  c.airspeed_to_throttle.ki = 0.045;
  c.airspeed_to_throttle.kd = 0.0;
  c.airspeed_to_throttle.output_min = a.throttle_min;
  c.airspeed_to_throttle.output_max = a.throttle_max;
  c.airspeed_to_throttle.anti_windup_gain = 2.0;

  return c;
}

PidAutopilot::PidAutopilot(const PidAutopilotConfig& cfg,
                           const dynamics::AircraftParameters& params)
    : cfg_(cfg),
      params_(params),
      roll_(cfg.roll_to_aileron),
      pitch_(cfg.pitch_to_elevator),
      sideslip_(cfg.sideslip_to_rudder),
      course_(cfg.course_to_roll),
      altitude_(cfg.altitude_to_pitch),
      airspeed_(cfg.airspeed_to_throttle) {
  // Output saturations are a property of the airframe and of the configured command
  // limits, not a tuning choice, so they are always derived here. This also means a YAML
  // gain block does not have to restate them.
  const auto& a = params_.actuators;
  roll_.config().output_min = a.aileron_min;
  roll_.config().output_max = a.aileron_max;
  pitch_.config().output_min = a.elevator_min;
  pitch_.config().output_max = a.elevator_max;
  sideslip_.config().output_min = a.rudder_min;
  sideslip_.config().output_max = a.rudder_max;
  airspeed_.config().output_min = a.throttle_min;
  airspeed_.config().output_max = a.throttle_max;
  course_.config().output_min = -cfg_.max_bank;
  course_.config().output_max = cfg_.max_bank;
  altitude_.config().output_min = -cfg_.max_pitch;
  altitude_.config().output_max = cfg_.max_pitch;
}

void PidAutopilot::reset(const TrimReference& trim) {
  trim_ = trim.controls;
  roll_.reset();
  pitch_.reset();
  sideslip_.reset();
  course_.reset();
  altitude_.reset();
  airspeed_.reset();
  // Preload the integrators so the autopilot starts out holding the trimmed condition:
  // the inner loops hold the trimmed surface positions and the outer loops hold the trimmed
  // attitude. Without this the cascade has to ramp up from zero, which shows up as a
  // spurious pitch-down transient at the start of every run.
  roll_.setIntegrator(trim_.aileron);
  pitch_.setIntegrator(trim_.elevator);
  sideslip_.setIntegrator(trim_.rudder);
  airspeed_.setIntegrator(trim_.throttle);
  altitude_.setIntegrator(trim.pitch);
  course_.setIntegrator(trim.roll);
  altitude_cmd_initialised_ = false;
  yaw_washout_ = 0.0;
  diag_ = AutopilotDiagnostics{};
}

ControlInput PidAutopilot::update(const AutopilotCommand& cmd, const VehicleFeedback& fb,
                                  double dt) {
  if (!altitude_cmd_initialised_) {
    altitude_cmd_filtered_ = fb.altitude;
    altitude_cmd_initialised_ = true;
  }
  // Slew-limit the altitude command so a large step cannot saturate the pitch loop.
  const double max_delta = cfg_.max_climb_rate * dt;
  altitude_cmd_filtered_ +=
      std::clamp(cmd.altitude - altitude_cmd_filtered_, -max_delta, max_delta);

  // --- Outer loops -------------------------------------------------------------
  // Above the alpha limit the bank command is faded out: in a turn the load factor is what
  // drives angle of attack, so widening the turn is the cheapest way to buy stall margin.
  const double protection_authority = envelopeProtectionAuthority(fb.alpha, cfg_.protection);
  const double course_error =
      (1.0 - protection_authority) * math::wrapPi(cmd.course - fb.course);
  const double roll_cmd = (1.0 - protection_authority) * course_.updateWithError(course_error, dt);

  const double altitude_error = altitude_cmd_filtered_ - fb.altitude;
  // Derivative of the altitude error is -climb_rate for a slowly varying command.
  const double pitch_cmd = altitude_.updateWithError(altitude_error, dt, -fb.climb_rate);

  const double airspeed_error = cmd.airspeed - fb.airspeed;
  const double throttle_cmd = airspeed_.updateWithError(airspeed_error, dt);

  // --- Inner loops -------------------------------------------------------------
  const double roll_error = math::wrapPi(roll_cmd - fb.roll);
  const double aileron = roll_.updateWithError(roll_error, dt, -fb.rate.x());

  const double pitch_error = math::wrapPi(pitch_cmd - fb.pitch);
  bool protection_active = false;
  const double elevator =
      applyEnvelopeProtection(pitch_.updateWithError(pitch_error, dt, -fb.rate.y()), fb.alpha,
                              fb.rate.y(), cfg_.protection, &protection_active);

  // Yaw damper: washed-out yaw-rate feedback so a steady turn is not opposed.
  if (cfg_.yaw_damper_tau > 0.0) {
    const double alpha = dt / (cfg_.yaw_damper_tau + dt);
    yaw_washout_ += alpha * (fb.rate.z() - yaw_washout_);
  }
  const double yaw_rate_ac = fb.rate.z() - yaw_washout_;
  // A positive yaw rate needs a nose-left (negative-yaw) moment, which a positive rudder
  // deflection provides.
  const double rudder_damper = cfg_.yaw_damper_gain * yaw_rate_ac;
  const double rudder_beta = sideslip_.updateWithError(fb.beta, dt);
  const double rudder = std::clamp(rudder_beta + rudder_damper, params_.actuators.rudder_min,
                                   params_.actuators.rudder_max);

  diag_.roll_command = roll_cmd;
  diag_.pitch_command = pitch_cmd;
  diag_.altitude_command_filtered = altitude_cmd_filtered_;
  diag_.course_error = course_error;
  diag_.altitude_error = cmd.altitude - fb.altitude;
  diag_.airspeed_error = airspeed_error;
  diag_.envelope_protection_active = protection_active;

  ControlInput u;
  u.elevator = elevator;
  u.aileron = aileron;
  u.rudder = rudder;
  u.throttle = throttle_cmd;
  return u;
}

}  // namespace aether::control
