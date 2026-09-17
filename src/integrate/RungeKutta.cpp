#include "aether/integrate/RungeKutta.hpp"

#include <algorithm>
#include <cmath>

namespace aether::integrate {

StepReport RungeKutta4::step(const OdeFunction& f, double t, const VecX& x, double dt) {
  const VecX k1 = f(t, x);
  const VecX k2 = f(t + 0.5 * dt, x + 0.5 * dt * k1);
  const VecX k3 = f(t + 0.5 * dt, x + 0.5 * dt * k2);
  const VecX k4 = f(t + dt, x + dt * k3);

  StepReport r;
  r.state = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  project(r.state);
  r.time = t + dt;
  r.dt_used = dt;
  r.dt_next = dt;
  r.accepted = true;
  r.function_evals = 4;
  return r;
}

void DormandPrince54::reset() {
  has_k1_ = false;
  error_prev_ = 1.0;
}

StepReport DormandPrince54::step(const OdeFunction& f, double t, const VecX& x, double dt) {
  // --- Dormand-Prince 5(4) Butcher tableau -----------------------------------------
  constexpr double c2 = 1.0 / 5.0, c3 = 3.0 / 10.0, c4 = 4.0 / 5.0, c5 = 8.0 / 9.0;

  constexpr double a21 = 1.0 / 5.0;
  constexpr double a31 = 3.0 / 40.0, a32 = 9.0 / 40.0;
  constexpr double a41 = 44.0 / 45.0, a42 = -56.0 / 15.0, a43 = 32.0 / 9.0;
  constexpr double a51 = 19372.0 / 6561.0, a52 = -25360.0 / 2187.0,
                   a53 = 64448.0 / 6561.0, a54 = -212.0 / 729.0;
  constexpr double a61 = 9017.0 / 3168.0, a62 = -355.0 / 33.0, a63 = 46732.0 / 5247.0,
                   a64 = 49.0 / 176.0, a65 = -5103.0 / 18656.0;
  // 5th-order weights (also row 7 of A, giving the FSAL property).
  constexpr double b1 = 35.0 / 384.0, b3 = 500.0 / 1113.0, b4 = 125.0 / 192.0,
                   b5 = -2187.0 / 6784.0, b6 = 11.0 / 84.0;
  // 4th-order embedded weights.
  constexpr double bs1 = 5179.0 / 57600.0, bs3 = 7571.0 / 16695.0, bs4 = 393.0 / 640.0,
                   bs5 = -92097.0 / 339200.0, bs6 = 187.0 / 2100.0, bs7 = 1.0 / 40.0;

  int evals = 0;
  VecX k1;
  if (has_k1_ && std::abs(k1_time_ - t) < 1e-14 && k1_.size() == x.size()) {
    k1 = k1_;
  } else {
    k1 = f(t, x);
    ++evals;
  }

  const VecX k2 = f(t + c2 * dt, x + dt * (a21 * k1));
  const VecX k3 = f(t + c3 * dt, x + dt * (a31 * k1 + a32 * k2));
  const VecX k4 = f(t + c4 * dt, x + dt * (a41 * k1 + a42 * k2 + a43 * k3));
  const VecX k5 = f(t + c5 * dt, x + dt * (a51 * k1 + a52 * k2 + a53 * k3 + a54 * k4));
  const VecX k6 = f(t + dt, x + dt * (a61 * k1 + a62 * k2 + a63 * k3 + a64 * k4 + a65 * k5));
  evals += 5;

  const VecX y5 = x + dt * (b1 * k1 + b3 * k3 + b4 * k4 + b5 * k5 + b6 * k6);
  const VecX k7 = f(t + dt, y5);
  ++evals;

  const VecX y4 = x + dt * (bs1 * k1 + bs3 * k3 + bs4 * k4 + bs5 * k5 + bs6 * k6 + bs7 * k7);

  // Scaled RMS error norm (Hairer & Wanner II.4).
  const VecX diff = y5 - y4;
  double acc = 0.0;
  for (Eigen::Index i = 0; i < x.size(); ++i) {
    const double sc = tol_.abs_tol + tol_.rel_tol * std::max(std::abs(x(i)), std::abs(y5(i)));
    const double e = diff(i) / sc;
    acc += e * e;
  }
  double err = std::sqrt(acc / static_cast<double>(std::max<Eigen::Index>(x.size(), 1)));
  if (!std::isfinite(err)) err = 1e10;

  StepReport r;
  r.function_evals = evals;
  r.error_norm = err;

  // PI step-size controller.
  constexpr double kAlpha = 0.7 / 5.0;
  constexpr double kBeta = 0.4 / 5.0;
  const double e_safe = std::max(err, 1e-16);
  double scale = tol_.safety * std::pow(e_safe, -kAlpha) * std::pow(error_prev_, kBeta);
  scale = std::clamp(scale, tol_.min_scale, tol_.max_scale);
  double dt_next = std::clamp(dt * scale, tol_.min_step, tol_.max_step);

  // A step that is already at the smallest permitted size cannot be shrunk any further, so
  // it is accepted and flagged rather than rejected forever.
  const bool at_min_step = dt <= tol_.min_step * (1.0 + 1e-12);
  if (err <= 1.0 || at_min_step) {
    r.accepted = true;
    r.tolerance_not_met = (err > 1.0);
    r.state = y5;
    project(r.state);
    r.time = t + dt;
    r.dt_used = dt;
    r.dt_next = dt_next;
    error_prev_ = std::max(err, 1e-4);
    // FSAL: k7 was evaluated at (t+dt, y5). It is only reusable when the projection did
    // not move the state, which is true to within the renormalisation tolerance; we
    // conservatively re-evaluate whenever a projection is installed.
    if (!projection_) {
      k1_ = k7;
      k1_time_ = r.time;
      has_k1_ = true;
    } else {
      has_k1_ = false;
    }
  } else {
    r.accepted = false;
    r.state = x;
    r.time = t;
    r.dt_used = 0.0;
    // On rejection do not allow the step to grow.
    r.dt_next = std::clamp(dt * std::min(scale, 1.0), tol_.min_step, dt);
    if (has_k1_ && std::abs(k1_time_ - t) > 1e-14) has_k1_ = false;
  }
  return r;
}

}  // namespace aether::integrate
