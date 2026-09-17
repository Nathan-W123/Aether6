/// \file RungeKutta.hpp
/// \brief Classical fixed-step RK4 and the adaptive Dormand-Prince 5(4) pair.
#pragma once

#include "aether/integrate/Integrator.hpp"

namespace aether::integrate {

/// \brief Classical fourth-order Runge-Kutta, fixed step, 4 evaluations per step.
///
/// Butcher tableau \f$c=[0,\tfrac12,\tfrac12,1]\f$, \f$b=[\tfrac16,\tfrac13,\tfrac13,\tfrac16]\f$.
/// Never rejects a step; `error_norm` is left at zero.
class RungeKutta4 final : public Integrator {
 public:
  std::string name() const override { return "rk4"; }
  int order() const override { return 4; }
  bool adaptive() const override { return false; }
  StepReport step(const OdeFunction& f, double t, const VecX& x, double dt) override;
};

/// \brief Dormand-Prince 5(4) embedded pair (DOPRI5) with PI step-size control.
///
/// Propagates the fifth-order solution (local extrapolation) and uses the embedded
/// fourth-order solution for the error estimate. The method is first-same-as-last, so an
/// accepted step costs six new evaluations instead of seven.
///
/// The step-size controller is the standard PI controller of Hairer & Wanner
/// (*Solving Ordinary Differential Equations I*, 2nd ed., II.4):
/// \f$ h_{new} = h\,\min(f_{max}, \max(f_{min}, S\,\varepsilon^{-\alpha}\varepsilon_{prev}^{\beta})) \f$
/// with \f$\alpha = 0.7/5\f$ and \f$\beta = 0.4/5\f$.
class DormandPrince54 final : public Integrator {
 public:
  explicit DormandPrince54(const ToleranceSettings& tol = {}) : tol_(tol) {}

  std::string name() const override { return "dopri54"; }
  int order() const override { return 5; }
  bool adaptive() const override { return true; }
  StepReport step(const OdeFunction& f, double t, const VecX& x, double dt) override;
  void reset() override;

  /// Current tolerance settings.
  const ToleranceSettings& tolerances() const { return tol_; }
  /// Replace the tolerance settings (resets the FSAL cache).
  void setTolerances(const ToleranceSettings& t) {
    tol_ = t;
    reset();
  }

 private:
  ToleranceSettings tol_;
  VecX k1_;
  bool has_k1_ = false;
  double k1_time_ = 0.0;
  double error_prev_ = 1.0;
};

}  // namespace aether::integrate
