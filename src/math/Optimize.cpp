#include "aether/math/Optimize.hpp"

#include <algorithm>
#include <cmath>

#include "aether/math/NumericalDiff.hpp"

namespace aether::math {

LmResult levenbergMarquardt(const ResidualFunction& r, const VecX& x0, const LmOptions& o) {
  LmResult out;
  VecX x = x0;
  VecX res = r(x);
  double cost = 0.5 * res.squaredNorm();
  double lambda = o.lambda_init;

  out.message = "iteration limit reached";
  int it = 0;
  for (; it < o.max_iterations; ++it) {
    if (cost < o.cost_tol) {
      out.converged = true;
      out.message = "cost tolerance met";
      break;
    }
    const MatX J = numericalJacobian(r, x, o.jacobian_epsilon, o.jacobian_scale);
    const VecX g = J.transpose() * res;
    if (g.lpNorm<Eigen::Infinity>() < o.gradient_tol) {
      out.converged = true;
      out.message = "gradient tolerance met";
      break;
    }
    const MatX H = J.transpose() * J;
    VecX diag = H.diagonal();
    for (Eigen::Index i = 0; i < diag.size(); ++i)
      if (!(diag(i) > 0.0)) diag(i) = 1.0;

    bool step_taken = false;
    for (int inner = 0; inner < 40; ++inner) {
      const MatX A = H + lambda * MatX(diag.asDiagonal());
      const VecX dx = A.colPivHouseholderQr().solve(-g);
      if (!dx.allFinite()) {
        lambda *= o.lambda_up;
        continue;
      }
      const VecX xn = x + dx;
      const VecX rn = r(xn);
      const double cn = 0.5 * rn.squaredNorm();
      if (std::isfinite(cn) && cn < cost) {
        x = xn;
        res = rn;
        cost = cn;
        lambda = std::max(lambda * o.lambda_down, 1e-14);
        step_taken = true;
        if (dx.norm() < o.step_tol) {
          out.converged = true;
          out.message = "step tolerance met";
          it = o.max_iterations;  // force the outer loop to exit
        }
        break;
      }
      lambda *= o.lambda_up;
      if (lambda > 1e14) break;
    }
    if (!step_taken) {
      out.message = "no further decrease possible";
      out.converged = cost < 1e-12;
      break;
    }
    if (it >= o.max_iterations) break;
  }

  out.x = x;
  out.residual = res;
  out.cost = cost;
  out.residual_norm = res.norm();
  out.residual_inf = res.lpNorm<Eigen::Infinity>();
  out.iterations = std::min(it, o.max_iterations);
  return out;
}

}  // namespace aether::math
