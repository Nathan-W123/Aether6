/// \file LinearAlgebra.hpp
/// \brief Dense linear-algebra helpers: matrix exponential, Van Loan discretisation,
///        continuous-time algebraic Riccati solution and LQR synthesis.
///
/// These routines are written for the small (n <= ~16) dense systems produced by
/// linearising the aircraft model, so they favour clarity and robustness over speed.
#pragma once

#include <Eigen/Dense>
#include <stdexcept>
#include <string>

#include "aether/core/Types.hpp"

namespace aether::math {

/// \brief Matrix exponential via scaling-and-squaring with a Pade(6,6) approximant.
///
/// Implements Algorithm 11.3.1 of Golub & Van Loan, *Matrix Computations*. The input is
/// scaled by \f$2^{-s}\f$ until its infinity norm is below 1/2, the Pade approximant is
/// evaluated, and the result is squared \a s times.
///
/// \param A Square matrix [-].
/// \return \f$e^{A}\f$.
MatX expm(const MatX& A);

/// \brief Result of Van Loan's discretisation of a linear stochastic system.
struct DiscretizedSystem {
  MatX Ad;  ///< Discrete state-transition matrix \f$e^{A\Delta t}\f$.
  MatX Qd;  ///< Discrete process-noise covariance \f$\int_0^{\Delta t} e^{A\tau}Q_c e^{A^\top\tau}d\tau\f$.
};

/// \brief Exactly discretise \f$\dot x = Ax + w,\ \mathrm{cov}(w)=Q_c\delta(t)\f$ over \a dt.
///
/// Uses Van Loan's method: form \f$M = \begin{bmatrix}-A & Q_c\\ 0 & A^\top\end{bmatrix}\Delta t\f$,
/// exponentiate, then \f$A_d = (M_{22})^\top\f$ and \f$Q_d = A_d M_{12}\f$.
/// \param A  Continuous system matrix.
/// \param Qc Continuous process-noise power spectral density (symmetric PSD).
/// \param dt Time step [s].
DiscretizedSystem vanLoanDiscretize(const MatX& A, const MatX& Qc, double dt);

/// \brief Force a matrix to be exactly symmetric: \f$(M + M^\top)/2\f$.
inline MatX symmetrize(const MatX& M) { return 0.5 * (M + M.transpose()); }

/// \brief Outcome of a continuous algebraic Riccati equation (CARE) solve.
struct CareSolution {
  MatX P;                  ///< Symmetric positive-(semi)definite stabilising solution.
  double residual = 0.0;   ///< \f$\|A^\top P + PA - PBR^{-1}B^\top P + Q\|_F\f$.
  int iterations = 0;      ///< Newton iterations used by the matrix-sign iteration.
  bool converged = false;  ///< True when the sign iteration met its tolerance.
};

/// \brief Solve \f$A^\top P + P A - P B R^{-1} B^\top P + Q = 0\f$ for the stabilising \f$P\f$.
///
/// Uses Roberts' matrix-sign-function method applied to the Hamiltonian
/// \f$H = \begin{bmatrix}A & -BR^{-1}B^\top\\ -Q & -A^\top\end{bmatrix}\f$ with
/// determinantal scaling, followed by a least-squares extraction of \f$P\f$ from the
/// stable invariant subspace. A Kleinman-Newton polishing step refines the answer.
///
/// \throws std::runtime_error if the Hamiltonian is singular (uncontrollable/undetectable mode
///         on the imaginary axis).
CareSolution solveCare(const MatX& A, const MatX& B, const MatX& Q, const MatX& R);

/// \brief Infinite-horizon continuous LQR gain.
struct LqrResult {
  MatX K;                       ///< Optimal gain, \f$u = -Kx\f$.
  MatX P;                       ///< Riccati solution.
  Eigen::VectorXcd closed_loop; ///< Eigenvalues of \f$A - BK\f$.
  double care_residual = 0.0;   ///< Frobenius residual of the Riccati equation.
};

/// \brief Design \f$u=-Kx\f$ minimising \f$\int (x^\top Q x + u^\top R u)\,dt\f$.
LqrResult lqr(const MatX& A, const MatX& B, const MatX& Q, const MatX& R);

/// \brief Controllability matrix \f$[B\ AB\ \dots\ A^{n-1}B]\f$.
MatX controllabilityMatrix(const MatX& A, const MatX& B);

/// \brief Numerical rank from a rank-revealing SVD with a relative tolerance.
int numericalRank(const MatX& M, double rel_tol = 1e-9);

}  // namespace aether::math
