#include "aether/integrate/Integrator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "aether/integrate/RungeKutta.hpp"

namespace aether::integrate {

VecX integrateTo(const OdeFunction& f, Integrator& integrator, double t0, const VecX& x0,
                 double t_end, double dt_initial, IntegrationStats* stats) {
  if (!(dt_initial > 0.0)) throw std::invalid_argument("integrateTo: dt_initial must be > 0");
  if (t_end < t0) throw std::invalid_argument("integrateTo: t_end must be >= t0");

  IntegrationStats local;
  local.min_dt = dt_initial;
  local.max_dt = 0.0;

  VecX x = x0;
  double t = t0;

  if (!integrator.adaptive()) {
    const double span = t_end - t0;
    const int n = std::max(1, static_cast<int>(std::ceil(span / dt_initial - 1e-12)));
    const double h = span / static_cast<double>(n);
    for (int i = 0; i < n; ++i) {
      const StepReport r = integrator.step(f, t, x, h);
      x = r.state;
      t = r.time;
      ++local.accepted_steps;
      local.function_evals += r.function_evals;
    }
    local.min_dt = local.max_dt = (n > 0 ? h : 0.0);
  } else {
    double dt = std::min(dt_initial, t_end - t0 > 0 ? t_end - t0 : dt_initial);
    int guard = 0;
    const int kMaxSteps = 100000000;
    while (t < t_end - 1e-14) {
      dt = std::min(dt, t_end - t);
      const StepReport r = integrator.step(f, t, x, dt);
      local.function_evals += r.function_evals;
      if (r.accepted) {
        x = r.state;
        t = r.time;
        ++local.accepted_steps;
        if (r.tolerance_not_met) ++local.tolerance_not_met;
        local.min_dt = local.accepted_steps == 1 ? r.dt_used : std::min(local.min_dt, r.dt_used);
        local.max_dt = std::max(local.max_dt, r.dt_used);
        dt = r.dt_next;
      } else {
        ++local.rejected_steps;
        dt = r.dt_next;
      }
      if (++guard > kMaxSteps)
        throw std::runtime_error("integrateTo: exceeded the maximum number of steps");
    }
  }

  if (stats) *stats = local;
  return x;
}

std::unique_ptr<Integrator> makeIntegrator(const std::string& name,
                                           const ToleranceSettings& tol) {
  if (name == "rk4") return std::make_unique<RungeKutta4>();
  if (name == "dopri54" || name == "rk45" || name == "dormand_prince")
    return std::make_unique<DormandPrince54>(tol);
  throw std::invalid_argument("makeIntegrator: unknown integrator '" + name +
                              "' (expected 'rk4' or 'dopri54')");
}

}  // namespace aether::integrate
