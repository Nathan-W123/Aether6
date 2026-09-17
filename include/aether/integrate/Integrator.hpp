/// \file Integrator.hpp
/// \brief Generic ODE integrator interface shared by the fixed-step and adaptive schemes.
#pragma once

#include <functional>
#include <memory>
#include <string>

#include "aether/core/Types.hpp"

namespace aether::integrate {

/// Right-hand side of an ODE \f$\dot x = f(t,x)\f$.
using OdeFunction = std::function<VecX(double, const VecX&)>;

/// Optional state projection applied after every accepted step (e.g. quaternion
/// renormalisation). Must be idempotent.
using Projection = std::function<void(VecX&)>;

/// \brief Outcome of a single integrator step.
struct StepReport {
  VecX state;                ///< State after the step (unchanged if the step was rejected).
  double time = 0.0;         ///< Time after the step.
  double dt_used = 0.0;      ///< Step size actually taken [s] (0 if rejected).
  double dt_next = 0.0;      ///< Suggested step size for the next attempt [s].
  bool accepted = true;      ///< Whether the step met the error tolerance.
  int function_evals = 0;    ///< Number of right-hand-side evaluations consumed.
  double error_norm = 0.0;   ///< Scaled local-error norm (adaptive schemes only).
  /// True when the step was accepted only because the controller had already reached
  /// `min_step` and could not shrink further — i.e. the requested tolerance is not
  /// achievable at this point of the trajectory (usually because it is below the
  /// double-precision round-off floor of the right-hand side).
  bool tolerance_not_met = false;
};

/// \brief Error-control settings for adaptive schemes.
struct ToleranceSettings {
  double rel_tol = 1e-8;   ///< Relative tolerance [-]
  double abs_tol = 1e-10;  ///< Absolute tolerance (state units)
  /// Smallest step the controller may propose [s]. A step at this size is accepted even if
  /// it misses the tolerance, so an unachievable tolerance degrades gracefully instead of
  /// stalling the integration; `IntegrationStats::tolerance_not_met` reports how often.
  double min_step = 1e-7;
  double max_step = 0.1;   ///< Largest step the controller may propose [s]
  double safety = 0.9;     ///< Step-size safety factor [-]
  double min_scale = 0.2;  ///< Lower bound on the per-step size change [-]
  double max_scale = 5.0;  ///< Upper bound on the per-step size change [-]
};

/// \brief Base class for explicit Runge-Kutta integrators.
class Integrator {
 public:
  virtual ~Integrator() = default;

  /// Short machine-readable name, e.g. "rk4" or "dopri54".
  virtual std::string name() const = 0;

  /// Classical order of accuracy of the propagated solution.
  virtual int order() const = 0;

  /// True when the scheme performs local error control and may reject steps.
  virtual bool adaptive() const = 0;

  /// Attempt a single step of size \a dt from \f$(t,x)\f$.
  virtual StepReport step(const OdeFunction& f, double t, const VecX& x, double dt) = 0;

  /// Clear any internal state carried between steps (e.g. FSAL caching).
  virtual void reset() {}

  /// Install a projection applied to the state after every accepted step.
  void setProjection(Projection p) { projection_ = std::move(p); }

  /// Apply the installed projection, if any.
  void project(VecX& x) const {
    if (projection_) projection_(x);
  }

 protected:
  Projection projection_;
};

/// \brief Summary of a full integration run.
struct IntegrationStats {
  int accepted_steps = 0;    ///< Number of accepted steps.
  int rejected_steps = 0;    ///< Number of rejected steps.
  int tolerance_not_met = 0; ///< Steps forced through at `min_step` (see StepReport).
  long function_evals = 0;   ///< Total right-hand-side evaluations.
  double min_dt = 0.0;       ///< Smallest accepted step [s].
  double max_dt = 0.0;       ///< Largest accepted step [s].
};

/// \brief Integrate from \a t0 to \a t_end, returning the final state.
///
/// Fixed-step integrators take exactly `ceil((t_end-t0)/dt_initial)` uniform steps; adaptive
/// integrators use \a dt_initial as the first trial step and then follow their controller,
/// always landing exactly on \a t_end.
///
/// \param f          Right-hand side.
/// \param integrator Integrator to use (its projection hook is honoured).
/// \param t0         Start time [s].
/// \param x0         Initial state.
/// \param t_end      End time [s], must satisfy `t_end >= t0`.
/// \param dt_initial Initial/fixed step [s], must be > 0.
/// \param stats      Optional statistics output.
VecX integrateTo(const OdeFunction& f, Integrator& integrator, double t0, const VecX& x0,
                 double t_end, double dt_initial, IntegrationStats* stats = nullptr);

/// \brief Factory: build an integrator by name ("rk4" or "dopri54").
/// \throws std::invalid_argument for an unknown name.
std::unique_ptr<Integrator> makeIntegrator(const std::string& name,
                                           const ToleranceSettings& tol = {});

}  // namespace aether::integrate
