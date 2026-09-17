#include "aether/math/LinearAlgebra.hpp"

#include <algorithm>
#include <cmath>

namespace aether::math {

MatX expm(const MatX& A) {
  const Eigen::Index n = A.rows();
  if (A.cols() != n) throw std::invalid_argument("expm: matrix must be square");
  if (n == 0) return MatX(0, 0);

  const double norm = A.cwiseAbs().rowwise().sum().maxCoeff();  // infinity norm
  int s = 0;
  if (norm > 0.5) s = static_cast<int>(std::ceil(std::log2(norm / 0.5)));
  if (s < 0) s = 0;
  if (s > 60) s = 60;  // guard against pathological inputs
  const MatX As = A / std::pow(2.0, s);

  const int q = 6;
  const MatX I = MatX::Identity(n, n);
  MatX X = As;
  double c = 0.5;
  MatX E = I + c * As;
  MatX D = I - c * As;
  // D(x) = N(-x), so the k-th coefficient enters the denominator with sign (-1)^k:
  // k = 2 adds, k = 3 subtracts, and so on.
  bool plus = true;
  for (int k = 2; k <= q; ++k) {
    c = c * static_cast<double>(q - k + 1) / static_cast<double>(k * (2 * q - k + 1));
    X = As * X;
    const MatX cX = c * X;
    E += cX;
    D = plus ? (D + cX).eval() : (D - cX).eval();
    plus = !plus;
  }
  MatX F = D.partialPivLu().solve(E);
  for (int k = 0; k < s; ++k) F = (F * F).eval();
  return F;
}

DiscretizedSystem vanLoanDiscretize(const MatX& A, const MatX& Qc, double dt) {
  const Eigen::Index n = A.rows();
  MatX M = MatX::Zero(2 * n, 2 * n);
  M.topLeftCorner(n, n) = -A * dt;
  M.topRightCorner(n, n) = Qc * dt;
  M.bottomRightCorner(n, n) = A.transpose() * dt;
  const MatX E = expm(M);

  DiscretizedSystem out;
  out.Ad = E.bottomRightCorner(n, n).transpose();
  out.Qd = symmetrize(out.Ad * E.topRightCorner(n, n));
  return out;
}

namespace {

/// One Newton step of the matrix sign iteration with determinantal scaling.
MatX signFunction(const MatX& H, int max_iter, double tol, int* iters, bool* converged) {
  const Eigen::Index m = H.rows();
  MatX Z = H;
  *converged = false;
  int k = 0;
  for (; k < max_iter; ++k) {
    Eigen::FullPivLU<MatX> lu(Z);
    if (!lu.isInvertible()) {
      throw std::runtime_error(
          "solveCare: Hamiltonian became singular during the sign iteration; the system "
          "likely has uncontrollable or undetectable modes on the imaginary axis.");
    }
    const MatX Zinv = lu.inverse();
    // Determinantal scaling keeps the iteration well conditioned (Byers, 1987).
    double detZ = std::abs(lu.determinant());
    double c = 1.0;
    if (detZ > 0.0 && std::isfinite(detZ)) {
      c = std::pow(detZ, -1.0 / static_cast<double>(m));
      if (!std::isfinite(c) || c <= 0.0) c = 1.0;
    }
    const MatX Znext = 0.5 * (c * Z + Zinv / c);
    const double delta = (Znext - Z).norm();
    Z = Znext;
    if (delta <= tol * std::max(1.0, Z.norm())) {
      *converged = true;
      ++k;
      break;
    }
  }
  *iters = k;
  return Z;
}

}  // namespace

CareSolution solveCare(const MatX& A, const MatX& B, const MatX& Q, const MatX& R) {
  const Eigen::Index n = A.rows();
  if (A.cols() != n) throw std::invalid_argument("solveCare: A must be square");
  if (B.rows() != n) throw std::invalid_argument("solveCare: B row count must match A");
  if (Q.rows() != n || Q.cols() != n) throw std::invalid_argument("solveCare: Q size mismatch");
  if (R.rows() != B.cols() || R.cols() != B.cols())
    throw std::invalid_argument("solveCare: R size must match the input dimension");

  const MatX Rinv = R.inverse();
  const MatX G = B * Rinv * B.transpose();

  MatX H(2 * n, 2 * n);
  H.topLeftCorner(n, n) = A;
  H.topRightCorner(n, n) = -G;
  H.bottomLeftCorner(n, n) = -Q;
  H.bottomRightCorner(n, n) = -A.transpose();

  CareSolution sol;
  MatX S = signFunction(H, 200, 1e-14, &sol.iterations, &sol.converged);

  // W = S + I. The stabilising solution satisfies  [W12; W22] P = -[W11; W21]
  // in the least-squares sense (Roberts 1971 / Byers 1987).
  const MatX W = S + MatX::Identity(2 * n, 2 * n);
  MatX lhs(2 * n, n);
  lhs.topRows(n) = W.topRightCorner(n, n);
  lhs.bottomRows(n) = W.bottomRightCorner(n, n);
  MatX rhs(2 * n, n);
  rhs.topRows(n) = -W.topLeftCorner(n, n);
  rhs.bottomRows(n) = -W.bottomLeftCorner(n, n);

  MatX P = lhs.colPivHouseholderQr().solve(rhs);
  P = symmetrize(P);

  // Kleinman-Newton polishing: solve the Lyapunov equation for the closed loop until the
  // Riccati residual stops improving. For n <= 16 the Kronecker form is cheap and exact.
  auto residualOf = [&](const MatX& Pm) {
    const MatX res = A.transpose() * Pm + Pm * A - Pm * G * Pm + Q;
    return res.norm();
  };
  double best = residualOf(P);
  for (int it = 0; it < 12; ++it) {
    const MatX K = Rinv * B.transpose() * P;
    const MatX Acl = A - B * K;
    // Solve Acl^T X + X Acl + (Q + K^T R K) = 0 via vectorisation.
    const MatX rhsL = -(Q + K.transpose() * R * K);
    const MatX I = MatX::Identity(n, n);
    MatX kron = MatX::Zero(n * n, n * n);
    // vec(Acl^T X) = (I kron Acl^T) vec(X); vec(X Acl) = (Acl^T kron I) vec(X)
    for (Eigen::Index i = 0; i < n; ++i) {
      for (Eigen::Index j = 0; j < n; ++j) {
        kron.block(i * n, j * n, n, n) += I(i, j) * Acl.transpose();
        kron.block(i * n, j * n, n, n) += Acl.transpose()(i, j) * I;
      }
    }
    VecX vecRhs(n * n);
    for (Eigen::Index j = 0; j < n; ++j) vecRhs.segment(j * n, n) = rhsL.col(j);
    Eigen::FullPivLU<MatX> lu(kron);
    if (!lu.isInvertible()) break;
    const VecX vecX = lu.solve(vecRhs);
    MatX Xn(n, n);
    for (Eigen::Index j = 0; j < n; ++j) Xn.col(j) = vecX.segment(j * n, n);
    Xn = symmetrize(Xn);
    const double r = residualOf(Xn);
    if (!std::isfinite(r) || r >= best) break;
    P = Xn;
    best = r;
    if (best < 1e-12 * std::max(1.0, Q.norm())) break;
  }

  sol.P = P;
  sol.residual = best;
  return sol;
}

LqrResult lqr(const MatX& A, const MatX& B, const MatX& Q, const MatX& R) {
  const CareSolution care = solveCare(A, B, Q, R);
  LqrResult out;
  out.P = care.P;
  out.K = R.inverse() * B.transpose() * care.P;
  out.care_residual = care.residual;
  Eigen::EigenSolver<MatX> es(A - B * out.K, false);
  out.closed_loop = es.eigenvalues();
  return out;
}

MatX controllabilityMatrix(const MatX& A, const MatX& B) {
  const Eigen::Index n = A.rows();
  const Eigen::Index m = B.cols();
  MatX C(n, n * m);
  MatX block = B;
  for (Eigen::Index i = 0; i < n; ++i) {
    C.block(0, i * m, n, m) = block;
    block = (A * block).eval();
  }
  return C;
}

int numericalRank(const MatX& M, double rel_tol) {
  if (M.size() == 0) return 0;
  Eigen::JacobiSVD<MatX> svd(M);
  const VecX sv = svd.singularValues();
  const double thresh = sv(0) * rel_tol;
  int r = 0;
  for (Eigen::Index i = 0; i < sv.size(); ++i)
    if (sv(i) > thresh) ++r;
  return r;
}

}  // namespace aether::math
