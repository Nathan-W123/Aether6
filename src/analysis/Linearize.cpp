#include "aether/analysis/Linearize.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>

#include "aether/core/Constants.hpp"
#include "aether/math/Rotation.hpp"

namespace aether::analysis {

namespace {

/// Apply a tangent-space perturbation to the packed state.
StateVec boxPlusState(const StateVec& x, const VecX& d) {
  StateVec out = x;
  out.segment<3>(StateIndex::kPosN) += d.segment<3>(ErrorIndex::kPos);
  out.segment<3>(StateIndex::kVelU) += d.segment<3>(ErrorIndex::kVel);
  out.segment<4>(StateIndex::kQuatW) =
      math::boxPlus(x.segment<4>(StateIndex::kQuatW), d.segment<3>(ErrorIndex::kAtt));
  out.segment<3>(StateIndex::kRateP) += d.segment<3>(ErrorIndex::kRate);
  return out;
}

}  // namespace

LinearModel linearize(const dynamics::RigidBody6DOF& model, const StateVec& x_ref,
                      const ControlInput& u_ref, const dynamics::EnvironmentSample& env,
                      double eps) {
  const Vec3 omega_ref = x_ref.segment<3>(StateIndex::kRateP);

  // Error-state derivative: the attitude row is the exact tangent velocity
  // d(dtheta)/dt = omega - R(exp(dtheta))^T omega_ref  (see docs/model.md).
  auto errorDerivative = [&](const VecX& d, const ControlInput& u) {
    const StateVec x = boxPlusState(x_ref, d);
    const StateVec dx = model.derivative(x, u, env);
    VecX out(kErrorStateDim);
    out.segment<3>(ErrorIndex::kPos) = dx.segment<3>(StateIndex::kPosN);
    out.segment<3>(ErrorIndex::kVel) = dx.segment<3>(StateIndex::kVelU);
    const Mat3 Rd = math::quatToRotation(math::expMapQuat(d.segment<3>(ErrorIndex::kAtt)));
    out.segment<3>(ErrorIndex::kAtt) = x.segment<3>(StateIndex::kRateP) - Rd.transpose() * omega_ref;
    out.segment<3>(ErrorIndex::kRate) = dx.segment<3>(StateIndex::kRateP);
    return out;
  };

  LinearModel lm;
  lm.x_ref = x_ref;
  lm.u_ref = u_ref;
  lm.density = env.density;
  lm.A = MatX::Zero(kErrorStateDim, kErrorStateDim);
  lm.B = MatX::Zero(kErrorStateDim, kInputDim);

  // --- A: central differences in the tangent space --------------------------------
  // Perturbation scales are chosen per block so a single relative epsilon maps to a
  // sensible absolute step for positions (m), velocities (m/s), angles (rad) and rates (rad/s).
  VecX scale(kErrorStateDim);
  scale.segment<3>(ErrorIndex::kPos).setConstant(1.0);
  scale.segment<3>(ErrorIndex::kVel).setConstant(1.0);
  scale.segment<3>(ErrorIndex::kAtt).setConstant(1.0);
  scale.segment<3>(ErrorIndex::kRate).setConstant(1.0);

  for (int j = 0; j < kErrorStateDim; ++j) {
    const double h = eps * scale(j);
    VecX dp = VecX::Zero(kErrorStateDim);
    VecX dm = VecX::Zero(kErrorStateDim);
    dp(j) = h;
    dm(j) = -h;
    lm.A.col(j) = (errorDerivative(dp, u_ref) - errorDerivative(dm, u_ref)) / (2.0 * h);
  }

  // --- B: central differences in the controls -------------------------------------
  const VecX zero = VecX::Zero(kErrorStateDim);
  Eigen::Matrix<double, kInputDim, 1> uvec = u_ref.vec();
  for (int j = 0; j < kInputDim; ++j) {
    const double h = eps * (j == 3 ? 1.0 : 1.0);  // rad for surfaces, [-] for throttle
    Eigen::Matrix<double, kInputDim, 1> up = uvec, um = uvec;
    up(j) += h;
    um(j) -= h;
    lm.B.col(j) = (errorDerivative(zero, ControlInput::fromVec(up)) -
                   errorDerivative(zero, ControlInput::fromVec(um))) /
                  (2.0 * h);
  }

  lm.state_names = {"dp_n", "dp_e", "dp_d", "du", "dv", "dw",
                    "dtheta_x", "dtheta_y", "dtheta_z", "dp_rate", "dq_rate", "dr_rate"};
  lm.state_units = {"m", "m", "m", "m/s", "m/s", "m/s",
                    "rad", "rad", "rad", "rad/s", "rad/s", "rad/s"};
  lm.input_names = {"elevator", "aileron", "rudder", "throttle"};
  lm.input_units = {"rad", "rad", "rad", "-"};
  return lm;
}

ReducedModels extractReducedModels(const LinearModel& full) {
  // Longitudinal: [du, dw, dq, dtheta_y] with inputs [elevator, throttle].
  const std::array<int, 4> lon_idx = {ErrorIndex::kVel + 0, ErrorIndex::kVel + 2,
                                      ErrorIndex::kRate + 1, ErrorIndex::kAtt + 1};
  const std::array<int, 2> lon_in = {0, 3};
  // Lateral: [dv, dp, dr, dtheta_x] with inputs [aileron, rudder].
  const std::array<int, 4> lat_idx = {ErrorIndex::kVel + 1, ErrorIndex::kRate + 0,
                                      ErrorIndex::kRate + 2, ErrorIndex::kAtt + 0};
  const std::array<int, 2> lat_in = {1, 2};

  ReducedModels r;
  r.A_lon = MatX::Zero(4, 4);
  r.B_lon = MatX::Zero(4, 2);
  r.A_lat = MatX::Zero(4, 4);
  r.B_lat = MatX::Zero(4, 2);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      r.A_lon(i, j) = full.A(lon_idx[i], lon_idx[j]);
      r.A_lat(i, j) = full.A(lat_idx[i], lat_idx[j]);
    }
    for (int j = 0; j < 2; ++j) {
      r.B_lon(i, j) = full.B(lon_idx[i], lon_in[j]);
      r.B_lat(i, j) = full.B(lat_idx[i], lat_in[j]);
    }
  }
  return r;
}

namespace {

ModeInfo describe(const std::complex<double>& lam) {
  ModeInfo m;
  m.eigenvalue = lam;
  m.natural_frequency = std::abs(lam);
  m.damping_ratio = m.natural_frequency > 1e-12 ? -lam.real() / m.natural_frequency : 0.0;
  m.period = std::abs(lam.imag()) > 1e-9 ? 2.0 * constants::kPi / std::abs(lam.imag()) : 0.0;
  m.time_to_half = std::abs(lam.real()) > 1e-12 ? std::log(2.0) / (-lam.real()) : 0.0;
  m.stable = lam.real() < 0.0;
  return m;
}

}  // namespace

std::vector<ModeInfo> analyseEigenvalues(const MatX& A) {
  Eigen::EigenSolver<MatX> es(A, false);
  const Eigen::VectorXcd ev = es.eigenvalues();
  std::vector<ModeInfo> out;
  out.reserve(static_cast<std::size_t>(ev.size()));
  for (Eigen::Index i = 0; i < ev.size(); ++i) {
    ModeInfo m = describe(ev(i));
    m.name = "eigenvalue_" + std::to_string(i);
    out.push_back(m);
  }
  return out;
}

std::vector<ModeInfo> classifyLongitudinal(const MatX& A_lon) {
  std::vector<ModeInfo> modes = analyseEigenvalues(A_lon);
  // Collect distinct oscillatory pairs (positive imaginary part representative).
  std::vector<std::size_t> osc;
  for (std::size_t i = 0; i < modes.size(); ++i)
    if (modes[i].eigenvalue.imag() > 1e-9) osc.push_back(i);
  std::sort(osc.begin(), osc.end(), [&](std::size_t a, std::size_t b) {
    return modes[a].natural_frequency > modes[b].natural_frequency;
  });
  if (osc.size() >= 1) {
    modes[osc[0]].name = "short_period";
    for (auto& m : modes)
      if (std::abs(m.eigenvalue - std::conj(modes[osc[0]].eigenvalue)) < 1e-9)
        m.name = "short_period";
  }
  if (osc.size() >= 2) {
    const auto lam = modes[osc[1]].eigenvalue;
    for (auto& m : modes)
      if (std::abs(m.eigenvalue - lam) < 1e-9 || std::abs(m.eigenvalue - std::conj(lam)) < 1e-9)
        m.name = "phugoid";
  }
  int real_index = 0;
  for (auto& m : modes)
    if (m.name.rfind("eigenvalue_", 0) == 0)
      m.name = "longitudinal_real_" + std::to_string(real_index++);
  return modes;
}

std::vector<ModeInfo> classifyLateral(const MatX& A_lat) {
  std::vector<ModeInfo> modes = analyseEigenvalues(A_lat);
  std::vector<std::size_t> real_idx;
  std::complex<double> dutch{0.0, 0.0};
  bool has_dutch = false;
  for (std::size_t i = 0; i < modes.size(); ++i) {
    if (std::abs(modes[i].eigenvalue.imag()) > 1e-9) {
      if (modes[i].eigenvalue.imag() > 0.0) {
        dutch = modes[i].eigenvalue;
        has_dutch = true;
      }
    } else {
      real_idx.push_back(i);
    }
  }
  if (has_dutch) {
    for (auto& m : modes)
      if (std::abs(m.eigenvalue - dutch) < 1e-9 || std::abs(m.eigenvalue - std::conj(dutch)) < 1e-9)
        m.name = "dutch_roll";
  }
  // Fastest real root = roll subsidence, slowest = spiral.
  std::sort(real_idx.begin(), real_idx.end(), [&](std::size_t a, std::size_t b) {
    return modes[a].eigenvalue.real() < modes[b].eigenvalue.real();
  });
  if (real_idx.size() >= 1) modes[real_idx.front()].name = "roll_subsidence";
  if (real_idx.size() >= 2) modes[real_idx.back()].name = "spiral";
  int extra = 0;
  for (auto& m : modes)
    if (m.name.rfind("eigenvalue_", 0) == 0) m.name = "lateral_real_" + std::to_string(extra++);
  return modes;
}

void exportLinearModel(const LinearModel& model, const std::string& out_dir) {
  namespace fs = std::filesystem;
  fs::create_directories(out_dir);

  auto writeMatrix = [](const std::string& path, const MatX& M,
                        const std::vector<std::string>& cols,
                        const std::vector<std::string>& rows) {
    std::ofstream f(path);
    f << std::setprecision(17);
    f << "row";
    for (Eigen::Index j = 0; j < M.cols(); ++j)
      f << "," << (j < static_cast<Eigen::Index>(cols.size()) ? cols[static_cast<std::size_t>(j)]
                                                              : "c" + std::to_string(j));
    f << "\n";
    for (Eigen::Index i = 0; i < M.rows(); ++i) {
      f << (i < static_cast<Eigen::Index>(rows.size()) ? rows[static_cast<std::size_t>(i)]
                                                       : "r" + std::to_string(i));
      for (Eigen::Index j = 0; j < M.cols(); ++j) f << "," << M(i, j);
      f << "\n";
    }
  };

  writeMatrix(out_dir + "/A_full.csv", model.A, model.state_names, model.state_names);
  writeMatrix(out_dir + "/B_full.csv", model.B, model.input_names, model.state_names);

  const ReducedModels red = extractReducedModels(model);
  const std::vector<std::string> lon_states = {"du", "dw", "dq", "dtheta"};
  const std::vector<std::string> lon_inputs = {"elevator", "throttle"};
  const std::vector<std::string> lat_states = {"dv", "dp", "dr", "dphi"};
  const std::vector<std::string> lat_inputs = {"aileron", "rudder"};
  writeMatrix(out_dir + "/A_lon.csv", red.A_lon, lon_states, lon_states);
  writeMatrix(out_dir + "/B_lon.csv", red.B_lon, lon_inputs, lon_states);
  writeMatrix(out_dir + "/A_lat.csv", red.A_lat, lat_states, lat_states);
  writeMatrix(out_dir + "/B_lat.csv", red.B_lat, lat_inputs, lat_states);

  auto writeModes = [](const std::string& path, const std::vector<ModeInfo>& modes,
                       const std::string& group) {
    std::ofstream f(path);
    f << std::setprecision(17);
    f << "group,mode,real,imag,natural_frequency_rad_s,damping_ratio,period_s,"
         "time_to_half_s,stable\n";
    for (const auto& m : modes) {
      f << group << "," << m.name << "," << m.eigenvalue.real() << "," << m.eigenvalue.imag()
        << "," << m.natural_frequency << "," << m.damping_ratio << "," << m.period << ","
        << m.time_to_half << "," << (m.stable ? 1 : 0) << "\n";
    }
  };

  // One combined mode file plus the individual groups.
  {
    std::ofstream f(out_dir + "/modes.csv");
    f << std::setprecision(17);
    f << "group,mode,real,imag,natural_frequency_rad_s,damping_ratio,period_s,"
         "time_to_half_s,stable\n";
    auto dump = [&](const std::string& group, const std::vector<ModeInfo>& modes) {
      for (const auto& m : modes)
        f << group << "," << m.name << "," << m.eigenvalue.real() << "," << m.eigenvalue.imag()
          << "," << m.natural_frequency << "," << m.damping_ratio << "," << m.period << ","
          << m.time_to_half << "," << (m.stable ? 1 : 0) << "\n";
    };
    dump("full", analyseEigenvalues(model.A));
    dump("longitudinal", classifyLongitudinal(red.A_lon));
    dump("lateral", classifyLateral(red.A_lat));
  }
  writeModes(out_dir + "/modes_longitudinal.csv", classifyLongitudinal(red.A_lon), "longitudinal");
  writeModes(out_dir + "/modes_lateral.csv", classifyLateral(red.A_lat), "lateral");

  {
    std::ofstream f(out_dir + "/reference_point.csv");
    f << std::setprecision(17);
    f << "quantity,value,unit\n";
    const dynamics::AircraftState s = dynamics::AircraftState::fromVec(model.x_ref);
    const Vec3 e = s.euler();
    f << "pos_n," << s.position.x() << ",m\n";
    f << "pos_e," << s.position.y() << ",m\n";
    f << "pos_d," << s.position.z() << ",m\n";
    f << "u," << s.velocity.x() << ",m/s\n";
    f << "v," << s.velocity.y() << ",m/s\n";
    f << "w," << s.velocity.z() << ",m/s\n";
    f << "roll," << e.x() << ",rad\n";
    f << "pitch," << e.y() << ",rad\n";
    f << "yaw," << e.z() << ",rad\n";
    f << "p," << s.rate.x() << ",rad/s\n";
    f << "q," << s.rate.y() << ",rad/s\n";
    f << "r," << s.rate.z() << ",rad/s\n";
    f << "elevator," << model.u_ref.elevator << ",rad\n";
    f << "aileron," << model.u_ref.aileron << ",rad\n";
    f << "rudder," << model.u_ref.rudder << ",rad\n";
    f << "throttle," << model.u_ref.throttle << ",-\n";
    f << "density," << model.density << ",kg/m^3\n";
  }
}

}  // namespace aether::analysis
