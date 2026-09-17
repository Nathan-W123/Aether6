/// \file Trim.hpp
/// \brief Numerical trim of the nonlinear 6-DOF model.
#pragma once

#include <string>

#include "aether/core/Types.hpp"
#include "aether/dynamics/RigidBody.hpp"

namespace aether::analysis {

/// \brief Flight condition to trim for.
struct TrimSpec {
  double airspeed = 25.0;      ///< Target true airspeed \f$V_a\f$ [m/s]
  double altitude = 120.0;     ///< Altitude above the NED origin [m]
  double flight_path_angle = 0.0;  ///< Target flight-path angle \f$\gamma\f$ [rad] (0 = level)
  double turn_rate = 0.0;      ///< Target turn rate \f$\dot\psi\f$ [rad/s] (0 = straight)
  double sideslip = 0.0;       ///< Target sideslip \f$\beta\f$ [rad]
  double density_override = -1.0;  ///< If > 0, use this density instead of the ISA value [kg/m^3]
};

/// \brief Converged trim point: state, controls and residual diagnostics.
struct TrimResult {
  StateVec state = StateVec::Zero();  ///< Trimmed 13-element state.
  ControlInput controls;              ///< Trimmed control deflections.

  double alpha = 0.0;   ///< Trim angle of attack [rad]
  double beta = 0.0;    ///< Trim sideslip [rad]
  double phi = 0.0;     ///< Trim bank angle [rad]
  double theta = 0.0;   ///< Trim pitch attitude [rad]
  double density = 0.0; ///< Air density used [kg/m^3]

  VecX residual;                ///< Final residual vector (8 elements, see TrimSolver).
  double residual_norm = 0.0;   ///< 2-norm of the residual.
  double residual_inf = 0.0;    ///< Infinity norm of the residual.
  int iterations = 0;           ///< Levenberg-Marquardt iterations used.
  bool converged = false;       ///< True when the residual met the convergence threshold.
  std::string message;          ///< Human-readable outcome.

  /// Names of the residual entries, aligned with `residual`.
  static const char* const* residualNames();
  /// Physical units of the residual entries, aligned with `residual`.
  static const char* const* residualUnits();
};

/// \brief Solves for steady flight conditions of the nonlinear model.
///
/// The unknown vector is \f$[\alpha,\beta,\phi,\theta,\delta_e,\delta_a,\delta_r,\delta_t]\f$
/// and the residual vector is
/// \f$[\dot u,\dot v,\dot w,\dot p,\dot q,\dot r,\ \dot h - V_a\sin\gamma,\ \beta-\beta^*]\f$.
/// Steady flight means the body-frame velocity and angular rate are constant, so all six
/// dynamic residuals vanish at a trim point; for a coordinated turn the body rate is set from
/// the requested turn rate:
/// \f$\omega^b = [-\dot\psi\sin\theta,\ \dot\psi\cos\theta\sin\phi,\ \dot\psi\cos\theta\cos\phi]^\top\f$.
/// Trim is computed in still air.
class TrimSolver {
 public:
  explicit TrimSolver(const dynamics::RigidBody6DOF& model) : model_(model) {}

  /// Solve for the requested condition.
  /// \param spec Flight condition.
  /// \param tolerance Convergence threshold on the infinity norm of the residual.
  TrimResult solve(const TrimSpec& spec, double tolerance = 1e-9) const;

  /// Physical residual vector for an arbitrary unknown vector (8 elements, exposed for
  /// testing and plotting).
  /// \param spec Flight condition.
  /// \param z Unknown vector \f$[\alpha,\beta,\phi,\theta,\delta_e,\delta_a,\delta_r,\delta_t]\f$.
  VecX residual(const TrimSpec& spec, const VecX& z) const;

  /// Residual vector actually minimised: the 8 physical residuals followed by 6 one-sided
  /// penalties that keep the search inside the actuator box and inside a sensible
  /// alpha/beta envelope.
  ///
  /// The penalties are **exactly zero** on a feasible point, so a reachable flight condition
  /// converges to the same answer with or without them. They matter when the requested
  /// condition is *not* reachable: without them the optimiser happily reports a 44 deg rudder
  /// and 26 deg of sideslip while chasing an impossible target, which is a physically
  /// meaningless answer even though it is correctly flagged as not converged.
  VecX penalisedResidual(const TrimSpec& spec, const VecX& z) const;

  /// Build the full 13-element state and control input from an unknown vector.
  void unpack(const TrimSpec& spec, const VecX& z, StateVec* x, ControlInput* u) const;

 private:
  const dynamics::RigidBody6DOF& model_;
};

}  // namespace aether::analysis
