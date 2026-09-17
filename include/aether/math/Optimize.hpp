/// \file Optimize.hpp
/// \brief Levenberg-Marquardt solver for small dense nonlinear least-squares problems.
#pragma once

#include <functional>
#include <string>

#include "aether/core/Types.hpp"

namespace aether::math {

/// Residual function \f$r(x)\in\mathbb{R}^m\f$ for \f$x\in\mathbb{R}^n\f$.
using ResidualFunction = std::function<VecX(const VecX&)>;

/// \brief Levenberg-Marquardt stopping criteria and damping schedule.
struct LmOptions {
  int max_iterations = 200;      ///< Maximum outer iterations.
  double cost_tol = 1e-18;       ///< Stop when \f$\tfrac12\|r\|^2\f$ falls below this.
  double step_tol = 1e-14;       ///< Stop when the parameter step norm falls below this.
  double gradient_tol = 1e-14;   ///< Stop when \f$\|J^\top r\|_\infty\f$ falls below this.
  double lambda_init = 1e-3;     ///< Initial damping parameter.
  double lambda_up = 10.0;       ///< Damping growth factor after a rejected step.
  double lambda_down = 0.3;      ///< Damping shrink factor after an accepted step.
  double jacobian_epsilon = 1e-7;///< Relative step for the numerical Jacobian.
  double jacobian_scale = 1e-3;  ///< Floor on the numerical-Jacobian step.
};

/// \brief Result of a Levenberg-Marquardt solve.
struct LmResult {
  VecX x;                    ///< Best parameter vector found.
  VecX residual;             ///< Residual at \a x.
  double cost = 0.0;         ///< \f$\tfrac12\|r(x)\|^2\f$.
  double residual_norm = 0.0;///< \f$\|r(x)\|_2\f$.
  double residual_inf = 0.0; ///< \f$\|r(x)\|_\infty\f$.
  int iterations = 0;        ///< Outer iterations performed.
  bool converged = false;    ///< True when a tolerance (not the iteration cap) stopped it.
  std::string message;       ///< Human-readable stopping reason.
};

/// \brief Minimise \f$\tfrac12\|r(x)\|^2\f$ starting from \a x0.
///
/// The Jacobian is formed by central differences and the normal equations
/// \f$(J^\top J + \lambda\,\mathrm{diag}(J^\top J))\Delta = -J^\top r\f$ are solved with a
/// rank-revealing QR, so rank-deficient problems degrade gracefully instead of failing.
LmResult levenbergMarquardt(const ResidualFunction& r, const VecX& x0,
                            const LmOptions& opts = {});

}  // namespace aether::math
