/// \file Linearize.hpp
/// \brief Numerical linearisation of the 6-DOF model and eigenvalue-based stability analysis.
#pragma once

#include <complex>
#include <string>
#include <vector>

#include "aether/core/Types.hpp"
#include "aether/dynamics/RigidBody.hpp"

namespace aether::analysis {

/// \brief Continuous-time linear model \f$\dot{\delta x} = A\,\delta x + B\,\delta u\f$.
///
/// The 12-element error state is \f$[\delta p^n,\ \delta v^b,\ \delta\theta,\ \delta\omega^b]\f$
/// where \f$\delta\theta\f$ is a **body-frame rotation vector**: the perturbed attitude is
/// \f$q = q_{ref}\otimes\exp(\delta\theta/2)\f$. This keeps the quaternion norm constraint out
/// of the linear model, so \a A has no spurious constrained mode. To first order and about a
/// level reference, \f$\delta\theta\f$ coincides with perturbations of the Euler angles
/// \f$(\phi,\theta,\psi)\f$.
struct LinearModel {
  MatX A;  ///< 12x12 state matrix [1/s]
  MatX B;  ///< 12x4 input matrix

  StateVec x_ref = StateVec::Zero();  ///< Reference (trim) state.
  ControlInput u_ref;                 ///< Reference (trim) controls.
  double density = 1.225;             ///< Air density at the reference condition [kg/m^3]

  std::vector<std::string> state_names;  ///< Names of the 12 error states.
  std::vector<std::string> state_units;  ///< Units of the 12 error states.
  std::vector<std::string> input_names;  ///< Names of the 4 inputs.
  std::vector<std::string> input_units;  ///< Units of the 4 inputs.
};

/// \brief Reduced decoupled models extracted from the full 12-state linearisation.
struct ReducedModels {
  MatX A_lon;  ///< 4x4 longitudinal matrix, states \f$[\delta u,\delta w,\delta q,\delta\theta]\f$
  MatX B_lon;  ///< 4x2 longitudinal input matrix, inputs \f$[\delta_e,\delta_t]\f$
  MatX A_lat;  ///< 4x4 lateral matrix, states \f$[\delta v,\delta p,\delta r,\delta\phi]\f$
  MatX B_lat;  ///< 4x2 lateral input matrix, inputs \f$[\delta_a,\delta_r]\f$
};

/// \brief Characterisation of one eigenvalue of a linear model.
struct ModeInfo {
  std::string name;                    ///< Classical mode name, or "eigenvalue N".
  std::complex<double> eigenvalue{0.0, 0.0};  ///< \f$\lambda\f$ [1/s]
  double natural_frequency = 0.0;      ///< \f$\omega_n=|\lambda|\f$ [rad/s]
  double damping_ratio = 0.0;          ///< \f$\zeta=-\mathrm{Re}(\lambda)/|\lambda|\f$ [-]
  double period = 0.0;                 ///< \f$2\pi/|\mathrm{Im}(\lambda)|\f$ [s], 0 if aperiodic
  double time_to_half = 0.0;           ///< Time to halve the amplitude [s]; negative = time to double
  bool stable = false;                 ///< True when \f$\mathrm{Re}(\lambda) < 0\f$.
};

/// \brief Numerically linearise the nonlinear model about \a x_ref, \a u_ref.
///
/// Central differences are taken in the 12-dimensional tangent space (see LinearModel) and in
/// the 4 control inputs.
/// \param model Nonlinear model.
/// \param x_ref Reference state (typically a trim point).
/// \param u_ref Reference control input.
/// \param env   Environment sample held fixed during differentiation.
/// \param eps   Relative perturbation size for the central differences.
LinearModel linearize(const dynamics::RigidBody6DOF& model, const StateVec& x_ref,
                      const ControlInput& u_ref, const dynamics::EnvironmentSample& env,
                      double eps = 1e-6);

/// \brief Extract the classical decoupled longitudinal and lateral models.
ReducedModels extractReducedModels(const LinearModel& full);

/// \brief Eigenvalues of \a A with damping/frequency metrics (unnamed).
std::vector<ModeInfo> analyseEigenvalues(const MatX& A);

/// \brief Eigen-analysis of the longitudinal reduced model, naming the short-period and
/// phugoid modes (the higher-frequency oscillatory pair is the short period).
std::vector<ModeInfo> classifyLongitudinal(const MatX& A_lon);

/// \brief Eigen-analysis of the lateral reduced model, naming the dutch-roll, roll-subsidence
/// and spiral modes (the oscillatory pair is dutch roll; of the real roots the fastest is
/// roll subsidence and the slowest is spiral).
std::vector<ModeInfo> classifyLateral(const MatX& A_lat);

/// \brief Write the linear model (A, B, reference point, eigenvalues, modes) to a directory
/// as CSV files consumable by the Python tooling.
/// \param model Full linear model.
/// \param out_dir Destination directory (created if needed).
void exportLinearModel(const LinearModel& model, const std::string& out_dir);

}  // namespace aether::analysis
