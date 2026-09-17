/// \file Types.hpp
/// \brief Fundamental vector/matrix aliases and the 6-DOF rigid-body state vector.
///
/// \par Coordinate frames
/// - **NED (n)**: local-level North-East-Down inertial frame, flat-Earth approximation.
///   Origin is the scenario reference point on the ground; \f$p_D\f$ is positive **downwards**,
///   so altitude above the reference is \f$h = -p_D\f$.
/// - **Body (b)**: forward-right-down (FRD) frame fixed to the airframe. \f$x_b\f$ out the
///   nose, \f$y_b\f$ out the right wing, \f$z_b\f$ down through the belly. Origin at the
///   centre of mass.
/// - **Wind/stability**: used only to express aerodynamic force directions; see Aerodynamics.
///
/// \par Attitude convention
/// The quaternion \f$q_{nb} = [q_w, q_x, q_y, q_z]^\top\f$ is **Hamilton**, unit norm, and
/// rotates a *body* vector into the *NED* frame: \f$ v^n = R(q_{nb})\, v^b \f$.
#pragma once

#include <Eigen/Dense>
#include <array>
#include <cstddef>

namespace aether {

using Vec3 = Eigen::Vector3d;   ///< 3-vector, double precision.
using Vec4 = Eigen::Vector4d;   ///< 4-vector, double precision.
using Mat3 = Eigen::Matrix3d;   ///< 3x3 matrix, double precision.
using VecX = Eigen::VectorXd;   ///< Dynamically sized column vector.
using MatX = Eigen::MatrixXd;   ///< Dynamically sized matrix.

/// Number of scalar states in the nonlinear 6-DOF model.
inline constexpr int kStateDim = 13;

/// Number of scalar states in the error/tangent representation used for linearisation
/// (attitude is represented by a 3-parameter rotation vector instead of a quaternion).
inline constexpr int kErrorStateDim = 12;

/// Number of control inputs: elevator, aileron, rudder, throttle.
inline constexpr int kInputDim = 4;

using StateVec = Eigen::Matrix<double, kStateDim, 1>;  ///< Packed 13x1 state.

/// \brief Indices of each quantity inside the packed 13x1 state vector.
///
/// | Index | Symbol      | Meaning                                   | Unit  |
/// |-------|-------------|-------------------------------------------|-------|
/// | 0..2  | \f$p^n\f$   | Position in NED (north, east, down)       | m     |
/// | 3..5  | \f$v^b\f$   | Velocity w.r.t. ground, in body axes (u,v,w) | m/s |
/// | 6..9  | \f$q_{nb}\f$| Attitude quaternion (w, x, y, z)          | -     |
/// | 10..12| \f$\omega^b\f$ | Body angular rate (p, q, r)            | rad/s |
struct StateIndex {
  static constexpr int kPosN = 0;   ///< North position [m].
  static constexpr int kPosE = 1;   ///< East position [m].
  static constexpr int kPosD = 2;   ///< Down position [m] (altitude = -kPosD).
  static constexpr int kVelU = 3;   ///< Body x velocity [m/s].
  static constexpr int kVelV = 4;   ///< Body y velocity [m/s].
  static constexpr int kVelW = 5;   ///< Body z velocity [m/s].
  static constexpr int kQuatW = 6;  ///< Quaternion scalar part [-].
  static constexpr int kQuatX = 7;  ///< Quaternion x [-].
  static constexpr int kQuatY = 8;  ///< Quaternion y [-].
  static constexpr int kQuatZ = 9;  ///< Quaternion z [-].
  static constexpr int kRateP = 10; ///< Roll rate [rad/s].
  static constexpr int kRateQ = 11; ///< Pitch rate [rad/s].
  static constexpr int kRateR = 12; ///< Yaw rate [rad/s].
};

/// \brief Indices in the 12-element error-state / tangent-space vector.
///
/// Ordering: \f$[\delta p^n,\ \delta v^b,\ \delta\theta,\ \delta\omega^b]\f$ where
/// \f$\delta\theta\f$ is a body-frame rotation vector such that
/// \f$q = q_0 \otimes \exp(\delta\theta/2)\f$.
struct ErrorIndex {
  static constexpr int kPos = 0;   ///< Position error block start [m].
  static constexpr int kVel = 3;   ///< Body velocity error block start [m/s].
  static constexpr int kAtt = 6;   ///< Attitude error (rotation vector) block start [rad].
  static constexpr int kRate = 9;  ///< Body rate error block start [rad/s].
};

/// \brief Aircraft control inputs. All deflections follow the standard sign convention
/// documented in docs/model.md.
struct ControlInput {
  double elevator = 0.0;  ///< \f$\delta_e\f$ [rad]. Positive = trailing edge **down** (nose-down moment).
  double aileron = 0.0;   ///< \f$\delta_a\f$ [rad]. Positive = **right roll** command (positive rolling moment).
  double rudder = 0.0;    ///< \f$\delta_r\f$ [rad]. Positive = trailing edge **left** (nose-left / negative yaw moment).
  double throttle = 0.0;  ///< \f$\delta_t\f$ [-], saturated to [0, 1].

  /// Pack into a 4-vector ordered [elevator, aileron, rudder, throttle].
  Eigen::Matrix<double, kInputDim, 1> vec() const {
    Eigen::Matrix<double, kInputDim, 1> u;
    u << elevator, aileron, rudder, throttle;
    return u;
  }

  /// Unpack from a 4-vector ordered [elevator, aileron, rudder, throttle].
  static ControlInput fromVec(const Eigen::Matrix<double, kInputDim, 1>& u) {
    ControlInput c;
    c.elevator = u(0);
    c.aileron = u(1);
    c.rudder = u(2);
    c.throttle = u(3);
    return c;
  }
};

/// \brief Forces and moments acting on the airframe, expressed in body axes.
struct Wrench {
  Vec3 force = Vec3::Zero();   ///< Resultant force in body axes [N].
  Vec3 moment = Vec3::Zero();  ///< Resultant moment about the CG in body axes [N*m].

  Wrench& operator+=(const Wrench& o) {
    force += o.force;
    moment += o.moment;
    return *this;
  }
  friend Wrench operator+(Wrench a, const Wrench& b) { return a += b; }
};

}  // namespace aether
