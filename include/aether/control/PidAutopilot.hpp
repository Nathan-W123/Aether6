/// \file PidAutopilot.hpp
/// \brief Classical successive-loop-closure autopilot built from PID loops.
#pragma once

#include "aether/control/Autopilot.hpp"
#include "aether/control/Pid.hpp"

namespace aether::control {

/// \brief Gains and limits of the PID autopilot cascade.
struct PidAutopilotConfig {
  PidConfig roll_to_aileron;     ///< Inner loop: bank error -> aileron [rad/rad]
  PidConfig pitch_to_elevator;   ///< Inner loop: pitch error -> elevator [rad/rad]
  PidConfig sideslip_to_rudder;  ///< Turn coordination: sideslip -> rudder [rad/rad]
  PidConfig course_to_roll;      ///< Outer loop: course error -> bank command [rad/rad]
  PidConfig altitude_to_pitch;   ///< Outer loop: altitude error -> pitch command [rad/m]
  PidConfig airspeed_to_throttle;///< Outer loop: airspeed error -> throttle [1/(m/s)]

  double max_bank = 0.61;         ///< Bank-angle command limit [rad] (35 deg)
  double max_pitch = 0.44;        ///< Pitch-attitude command limit [rad] (25 deg)
  double max_climb_rate = 4.0;    ///< Slew limit applied to the altitude command [m/s]
  double yaw_damper_gain = 0.6;   ///< Washed-out yaw-rate feedback gain [rad/(rad/s)]
  double yaw_damper_tau = 2.0;    ///< Washout filter time constant [s]
  EnvelopeProtection protection;  ///< Angle-of-attack envelope protection.
};

/// \brief Successive loop closure: course -> bank -> aileron, altitude -> pitch -> elevator,
/// airspeed -> throttle, with a washed-out yaw damper and a sideslip regulator on the rudder.
///
/// All six loops use aether::control::Pid, so every integrator carries the same
/// back-calculation anti-windup. Commanded altitude is slew limited to `max_climb_rate` so
/// large step commands cannot saturate the pitch loop.
class PidAutopilot final : public Autopilot {
 public:
  /// \param cfg Gains and limits.
  /// \param params Airframe parameters (used only for the actuator limits).
  PidAutopilot(const PidAutopilotConfig& cfg, const dynamics::AircraftParameters& params);

  std::string name() const override { return "pid"; }
  ControlInput update(const AutopilotCommand& cmd, const VehicleFeedback& fb,
                      double dt) override;
  void reset(const TrimReference& trim) override;
  const AutopilotDiagnostics& diagnostics() const override { return diag_; }

  /// Default gain set tuned for the shipped synthetic airframe.
  static PidAutopilotConfig defaultConfig(const dynamics::AircraftParameters& params);

 private:
  PidAutopilotConfig cfg_;
  dynamics::AircraftParameters params_;

  Pid roll_, pitch_, sideslip_, course_, altitude_, airspeed_;
  AutopilotDiagnostics diag_;
  ControlInput trim_{};
  double altitude_cmd_filtered_ = 0.0;
  bool altitude_cmd_initialised_ = false;
  double yaw_washout_ = 0.0;
};

}  // namespace aether::control
