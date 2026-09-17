#include "aether/control/LqrAutopilot.hpp"

#include <algorithm>
#include <cmath>

#include "aether/core/Constants.hpp"
#include "aether/math/LinearAlgebra.hpp"
#include "aether/math/Rotation.hpp"

namespace aether::control {

LqrAutopilot::LqrAutopilot(const analysis::TrimResult& trim,
                           const analysis::LinearModel& linear, const LqrWeights& w,
                           const LqrAutopilotLimits& limits,
                           const dynamics::AircraftParameters& params)
    : params_(params), limits_(limits) {
  const dynamics::AircraftState s = dynamics::AircraftState::fromVec(trim.state);
  u0_ = s.velocity.x();
  v0_ = s.velocity.y();
  w0_ = s.velocity.z();
  const Vec3 e = s.euler();
  phi0_ = e.x();
  theta0_ = e.y();
  Va0_ = std::max(s.velocity.norm(), constants::kMinAirspeed);
  trim_ = trim.controls;

  const analysis::ReducedModels red = analysis::extractReducedModels(linear);

  // ---------------- Longitudinal design ----------------
  // States: [du, dw, dq, dtheta, h_err, int_h_err, int_Va_err]; inputs: [de, dt].
  MatX A_lon = MatX::Zero(7, 7);
  MatX B_lon = MatX::Zero(7, 2);
  A_lon.topLeftCorner(4, 4) = red.A_lon;
  B_lon.topRows(4) = red.B_lon;
  // d(h_err)/dt from  hdot = u sin(theta) - v sin(phi) cos(theta) - w cos(phi) cos(theta),
  // linearised about (phi0 = 0):  dhdot = sin(th0) du - cos(th0) dw + (u0 cos(th0) + w0 sin(th0)) dtheta.
  A_lon(4, 0) = std::sin(theta0_);
  A_lon(4, 1) = -std::cos(theta0_);
  A_lon(4, 3) = u0_ * std::cos(theta0_) + w0_ * std::sin(theta0_);
  A_lon(5, 4) = 1.0;                  // int_h_err' = h_err
  A_lon(6, 0) = u0_ / Va0_;           // int_Va_err' = dVa
  A_lon(6, 1) = w0_ / Va0_;

  MatX Q_lon = MatX::Zero(7, 7);
  Q_lon.diagonal() << w.q_u, w.q_w, w.q_pitch_rate, w.q_pitch, w.q_altitude,
      w.q_int_altitude, w.q_int_airspeed;
  MatX R_lon = MatX::Zero(2, 2);
  R_lon.diagonal() << w.r_elevator, w.r_throttle;

  const auto lon = math::lqr(A_lon, B_lon, Q_lon, R_lon);
  K_lon_ = lon.K;
  lon_cl_ = lon.closed_loop;
  care_residuals_(0) = lon.care_residual;

  // ---------------- Lateral design ----------------
  // States: [dv, dp, dr, dphi, chi_err, int_chi_err]; inputs: [da, dr].
  MatX A_lat = MatX::Zero(6, 6);
  MatX B_lat = MatX::Zero(6, 2);
  A_lat.topLeftCorner(4, 4) = red.A_lat;
  B_lat.topRows(4) = red.B_lat;
  // Course kinematics of a coordinated turn: chi_dot = g*tan(phi)/Va ~ (g/Va0) * dphi.
  // Using the yaw-rate relation chi_dot = r/cos(theta0) instead would let the optimiser
  // steer with the rudder, which is aerodynamically the wrong actuator for a fixed-wing
  // aircraft and produces a large steady sideslip in turns.
  A_lat(4, 3) = constants::kGravity / Va0_;
  A_lat(5, 4) = 1.0;                                     // int_chi_err' = chi_err

  MatX Q_lat = MatX::Zero(6, 6);
  Q_lat.diagonal() << w.q_v, w.q_roll_rate, w.q_yaw_rate, w.q_roll, w.q_course,
      w.q_int_course;
  MatX R_lat = MatX::Zero(2, 2);
  R_lat.diagonal() << w.r_aileron, w.r_rudder;

  const auto lat = math::lqr(A_lat, B_lat, Q_lat, R_lat);
  K_lat_ = lat.K;
  lat_cl_ = lat.closed_loop;
  care_residuals_(1) = lat.care_residual;

  // ---- Derive the error-state saturations from the gains -----------------------------
  // Without this, a large course change (e.g. a 90 deg corner in the waypoint list) presents
  // the feedback law with a course error whose proportional aileron demand far exceeds the
  // aileron authority; the surface then saturates and the aircraft rolls until the bank term
  // balances the course term, i.e. to phi = (K_chi/K_phi) * chi_err, which can be well past
  // the structural limit. Clamping chi_err at phi_max * K_phi / K_chi makes that balance
  // point exactly phi_max. The same argument gives the altitude clamp.
  if (limits_.derive_limits_from_gains) {
    auto ratioLimit = [](double budget, double k_attitude, double k_error, double fallback) {
      if (std::abs(k_error) < 1e-12) return fallback;
      return std::min(fallback, budget * std::abs(k_attitude) / std::abs(k_error));
    };
    const double a = std::clamp(limits_.integrator_authority, 0.0, 0.9);
    limits_.course_error_limit = ratioLimit((1.0 - a) * limits_.max_bank, K_lat_(0, 3),
                                            K_lat_(0, 4), limits_.course_error_limit);
    limits_.integrator_course_limit =
        ratioLimit(a * limits_.max_bank, K_lat_(0, 3), K_lat_(0, 5),
                   limits_.integrator_course_limit);
    limits_.altitude_error_limit = ratioLimit((1.0 - a) * limits_.max_pitch, K_lon_(0, 3),
                                              K_lon_(0, 4), limits_.altitude_error_limit);
    limits_.integrator_altitude_limit =
        ratioLimit(a * limits_.max_pitch, K_lon_(0, 3), K_lon_(0, 5),
                   limits_.integrator_altitude_limit);
    // The airspeed integrator is allowed to bias the throttle by at most 80% of its range.
    if (std::abs(K_lon_(1, 6)) > 1e-12)
      limits_.integrator_airspeed_limit =
          std::min(limits_.integrator_airspeed_limit, 0.8 / std::abs(K_lon_(1, 6)));
  }
}

void LqrAutopilot::reset(const TrimReference& trim) {
  trim_ = trim.controls;
  int_altitude_ = 0.0;
  int_airspeed_ = 0.0;
  int_course_ = 0.0;
  altitude_cmd_initialised_ = false;
  diag_ = AutopilotDiagnostics{};
}

ControlInput LqrAutopilot::update(const AutopilotCommand& cmd, const VehicleFeedback& fb,
                                  double dt) {
  if (!altitude_cmd_initialised_) {
    altitude_cmd_filtered_ = fb.altitude;
    altitude_cmd_initialised_ = true;
  }
  const double max_delta = limits_.max_climb_rate * dt;
  altitude_cmd_filtered_ +=
      std::clamp(cmd.altitude - altitude_cmd_filtered_, -max_delta, max_delta);

  const double h_err = std::clamp(fb.altitude - altitude_cmd_filtered_,
                                  -limits_.altitude_error_limit, limits_.altitude_error_limit);
  const double va_err = fb.airspeed - cmd.airspeed;
  // Envelope protection authority. Above the alpha limit the course error - and therefore
  // the commanded bank - is faded out, so the aircraft widens the turn instead of stalling.
  const double protection_authority =
      envelopeProtectionAuthority(fb.alpha, limits_.protection);
  const double chi_err = (1.0 - protection_authority) *
                         std::clamp(math::wrapPi(fb.course - cmd.course),
                                    -limits_.course_error_limit, limits_.course_error_limit);

  // ---------------- Longitudinal ----------------
  // The design model was linearised about a still-air trim, so the velocity states are
  // air-relative. Feeding air-relative velocities makes the loop invariant to a steady wind:
  // a crosswind crab is then not mistaken for a sideslip.
  VecX x_lon(7);
  x_lon << fb.velocity_air_body.x() - u0_, fb.velocity_air_body.z() - w0_, fb.rate.y() - 0.0,
      math::wrapPi(fb.pitch - theta0_), h_err, int_altitude_, int_airspeed_;
  const VecX u_lon = -K_lon_ * x_lon;

  bool protection_active = false;
  const double elevator_raw =
      applyEnvelopeProtection(trim_.elevator + u_lon(0), fb.alpha, fb.rate.y(),
                              limits_.protection, &protection_active);
  const double throttle_raw = trim_.throttle + u_lon(1);
  const double elevator = std::clamp(elevator_raw, params_.actuators.elevator_min,
                                     params_.actuators.elevator_max);
  const double throttle = std::clamp(throttle_raw, params_.actuators.throttle_min,
                                     params_.actuators.throttle_max);

  // ---------------- Lateral ----------------
  VecX x_lat(6);
  x_lat << fb.velocity_air_body.y() - v0_, fb.rate.x(), fb.rate.z(),
      math::wrapPi(fb.roll - phi0_), chi_err, int_course_;
  const VecX u_lat = -K_lat_ * x_lat;

  const double aileron_raw = trim_.aileron + u_lat(0);
  const double rudder_raw = trim_.rudder + u_lat(1);
  const double aileron = std::clamp(aileron_raw, params_.actuators.aileron_min,
                                    params_.actuators.aileron_max);
  const double rudder = std::clamp(rudder_raw, params_.actuators.rudder_min,
                                   params_.actuators.rudder_max);

  // ---------------- Integral states with anti-windup ----------------
  // Conditional integration: hold the integrator when the corresponding actuator is
  // saturated and the error would drive it further into the stop.
  auto integrate = [dt](double& state, double error, double limit, bool blocked) {
    if (!blocked) state += error * dt;
    state = std::clamp(state, -limit, limit);
  };
  // While the protection is commanding nose-down, hold the altitude integrator: it is being
  // deliberately overridden and must not wind up against the protection.
  const bool elev_sat = (elevator != elevator_raw) || protection_active;
  const bool thr_sat = (throttle != throttle_raw);
  const bool ail_sat = (aileron != aileron_raw);
  const bool rud_sat = (rudder != rudder_raw);

  integrate(int_altitude_, h_err, limits_.integrator_altitude_limit, elev_sat);
  integrate(int_airspeed_, va_err, limits_.integrator_airspeed_limit, thr_sat);
  integrate(int_course_, chi_err, limits_.integrator_course_limit,
            (ail_sat && rud_sat) || protection_authority > 0.0);

  diag_.roll_command = 0.0;  // the LQR closes the course loop directly on the bank state
  diag_.pitch_command = 0.0;
  diag_.altitude_command_filtered = altitude_cmd_filtered_;
  diag_.course_error = -chi_err;
  diag_.altitude_error = cmd.altitude - fb.altitude;
  diag_.airspeed_error = cmd.airspeed - fb.airspeed;
  diag_.envelope_protection_active = protection_active;

  ControlInput u;
  u.elevator = elevator;
  u.aileron = aileron;
  u.rudder = rudder;
  u.throttle = throttle;
  return u;
}

}  // namespace aether::control
