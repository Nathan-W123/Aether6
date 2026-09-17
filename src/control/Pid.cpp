#include "aether/control/Pid.hpp"

#include <algorithm>
#include <cmath>

namespace aether::control {

void Pid::reset() {
  integral_ = 0.0;
  prev_error_ = 0.0;
  prev_measurement_ = 0.0;
  derivative_state_ = 0.0;
  initialised_ = false;
  last_output_ = 0.0;
  saturated_ = false;
}

double Pid::update(double setpoint, double measurement, double dt, double measurement_rate) {
  const double error = setpoint - measurement;

  double derivative = 0.0;
  if (cfg_.kd != 0.0) {
    if (std::isfinite(measurement_rate)) {
      // Derivative supplied externally (e.g. a rate gyro). With a piecewise-constant
      // setpoint, d(error)/dt = -d(measurement)/dt, so both derivative conventions agree.
      derivative = -measurement_rate;
    } else if (initialised_ && dt > 0.0) {
      const double raw = cfg_.derivative_on_measurement ? -(measurement - prev_measurement_) / dt
                                                        : (error - prev_error_) / dt;
      if (cfg_.derivative_filter_tau > 0.0) {
        const double alpha = dt / (cfg_.derivative_filter_tau + dt);
        derivative_state_ += alpha * (raw - derivative_state_);
        derivative = derivative_state_;
      } else {
        derivative = raw;
      }
    }
  }

  const double proportional = cfg_.kp * error;
  const double raw_output = proportional + integral_ + cfg_.kd * derivative;
  const double output = std::clamp(raw_output, cfg_.output_min, cfg_.output_max);
  saturated_ = (output != raw_output);

  if (dt > 0.0) {
    // Conditional integration: freeze when the error drives further into saturation.
    const bool driving_deeper =
        (raw_output > cfg_.output_max && error > 0.0) || (raw_output < cfg_.output_min && error < 0.0);
    double increment = driving_deeper ? 0.0 : cfg_.ki * error * dt;
    // Back-calculation: bleed the integrator back towards the achievable output.
    increment += cfg_.anti_windup_gain * (output - raw_output) * dt;
    integral_ += increment;
  }

  prev_error_ = error;
  prev_measurement_ = measurement;
  initialised_ = true;
  last_output_ = output;
  return output;
}

double Pid::updateWithError(double error, double dt, double error_rate) {
  double derivative = 0.0;
  if (cfg_.kd != 0.0) {
    if (std::isfinite(error_rate)) {
      derivative = error_rate;
    } else if (initialised_ && dt > 0.0) {
      const double raw = (error - prev_error_) / dt;
      if (cfg_.derivative_filter_tau > 0.0) {
        const double alpha = dt / (cfg_.derivative_filter_tau + dt);
        derivative_state_ += alpha * (raw - derivative_state_);
        derivative = derivative_state_;
      } else {
        derivative = raw;
      }
    }
  }

  const double raw_output = cfg_.kp * error + integral_ + cfg_.kd * derivative;
  const double output = std::clamp(raw_output, cfg_.output_min, cfg_.output_max);
  saturated_ = (output != raw_output);

  if (dt > 0.0) {
    const bool driving_deeper =
        (raw_output > cfg_.output_max && error > 0.0) || (raw_output < cfg_.output_min && error < 0.0);
    double increment = driving_deeper ? 0.0 : cfg_.ki * error * dt;
    increment += cfg_.anti_windup_gain * (output - raw_output) * dt;
    integral_ += increment;
  }

  prev_error_ = error;
  initialised_ = true;
  last_output_ = output;
  return output;
}

}  // namespace aether::control
