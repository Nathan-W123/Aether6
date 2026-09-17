#include "aether/dynamics/AircraftParameters.hpp"

#include <cmath>
#include <stdexcept>

namespace aether::dynamics {

void AircraftParameters::validate() const {
  auto require = [](bool ok, const char* what) {
    if (!ok) throw std::invalid_argument(std::string("AircraftParameters: ") + what);
  };
  require(mass > 0.0, "mass must be positive");
  require(wing_area > 0.0, "wing_area must be positive");
  require(wing_span > 0.0, "wing_span must be positive");
  require(mean_chord > 0.0, "mean_chord must be positive");
  require(Jx > 0.0 && Jy > 0.0 && Jz > 0.0, "principal inertias must be positive");
  const Mat3 J = inertia();
  Eigen::SelfAdjointEigenSolver<Mat3> es(J);
  require(es.eigenvalues().minCoeff() > 0.0, "inertia tensor must be positive definite");
  // Triangle inequality on the principal moments (a necessary condition for a real body).
  const Vec3 pm = es.eigenvalues();
  require(pm(0) + pm(1) >= pm(2) - 1e-9, "principal moments violate the triangle inequality");
  require(lon.oswald > 0.0 && lon.oswald <= 1.0, "oswald efficiency must be in (0, 1]");
  require(lon.CL_alpha > 0.0, "CL_alpha must be positive");
  require(lon.CD0 > 0.0, "CD0 must be positive");
  require(prop.disk_area > 0.0, "propeller disk area must be positive");
  require(prop.k_motor > 0.0, "k_motor must be positive");
  require(actuators.elevator_max > actuators.elevator_min, "elevator limits inverted");
  require(actuators.aileron_max > actuators.aileron_min, "aileron limits inverted");
  require(actuators.rudder_max > actuators.rudder_min, "rudder limits inverted");
  require(actuators.throttle_max > actuators.throttle_min, "throttle limits inverted");
  require(actuators.surface_time_constant > 0.0, "surface time constant must be positive");
  require(actuators.throttle_time_constant > 0.0, "throttle time constant must be positive");
}

AircraftParameters defaultAircraft() {
  AircraftParameters p;
  p.name = "Aether-6 (synthetic small UAV)";
  p.synthetic = true;

  p.mass = 13.5;
  p.Jx = 0.824;
  p.Jy = 1.135;
  p.Jz = 1.759;
  p.Jxz = 0.120;

  p.wing_area = 0.52;
  p.wing_span = 2.60;
  p.mean_chord = 0.20;

  p.lon.CL0 = 0.28;
  p.lon.CL_alpha = 5.00;
  p.lon.CL_q = 5.50;
  p.lon.CL_de = 0.43;
  p.lon.CD0 = 0.030;
  p.lon.CD_q = 0.0;
  p.lon.CD_de = 0.020;
  p.lon.oswald = 0.85;
  p.lon.Cm0 = 0.060;
  p.lon.Cm_alpha = -1.20;
  p.lon.Cm_q = -19.0;
  p.lon.Cm_de = -1.50;
  p.lon.alpha_stall = 0.30;
  p.lon.stall_sharpness = 50.0;
  p.lon.enable_stall_model = true;

  p.lat.CY0 = 0.0;
  p.lat.CY_beta = -0.400;
  p.lat.CY_p = -0.030;
  p.lat.CY_r = 0.150;
  p.lat.CY_da = 0.0;
  p.lat.CY_dr = 0.155;

  p.lat.Cl0 = 0.0;
  p.lat.Cl_beta = -0.080;
  p.lat.Cl_p = -0.520;
  p.lat.Cl_r = 0.120;
  p.lat.Cl_da = 0.160;
  p.lat.Cl_dr = 0.007;

  p.lat.Cn0 = 0.0;
  p.lat.Cn_beta = 0.076;
  p.lat.Cn_p = -0.060;
  p.lat.Cn_r = -0.060;
  p.lat.Cn_da = -0.012;
  p.lat.Cn_dr = -0.042;

  p.prop.disk_area = 0.050;
  p.prop.efficiency = 1.0;
  p.prop.k_motor = 45.0;
  p.prop.k_torque = 5.0e-6;
  p.prop.k_omega = 300.0;
  p.prop.thrust_offset = Vec3::Zero();

  return p;
}

}  // namespace aether::dynamics
