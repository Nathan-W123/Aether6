/// \file test_dynamics.cpp
/// \brief Rigid-body dynamics: sign conventions, conservation properties, free fall,
///        aerodynamic and propulsion behaviour, actuator limits and the atmosphere model.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "aether/core/Constants.hpp"
#include "aether/dynamics/Actuators.hpp"
#include "aether/dynamics/RigidBody.hpp"
#include "aether/env/Atmosphere.hpp"
#include "aether/integrate/RungeKutta.hpp"
#include "aether/math/Rotation.hpp"

using namespace aether;
using Catch::Approx;

namespace {

/// Level-flight state at the given airspeed with all angles zero.
StateVec levelState(double airspeed, double altitude = 120.0) {
  dynamics::AircraftState s;
  s.position = Vec3(0.0, 0.0, -altitude);
  s.velocity = Vec3(airspeed, 0.0, 0.0);
  s.quaternion = Vec4(1, 0, 0, 0);
  s.rate = Vec3::Zero();
  return s.vec();
}

dynamics::EnvironmentSample stillAir(double density = 1.225) {
  dynamics::EnvironmentSample e;
  e.density = density;
  e.wind_ned = Vec3::Zero();
  e.gravity = constants::kGravity;
  return e;
}

}  // namespace

TEST_CASE("air data is computed with the documented conventions", "[dynamics][aero]") {
  // Positive w (downward body velocity) means the relative wind comes from below: alpha > 0.
  const dynamics::AirData a = dynamics::computeAirData(Vec3(20.0, 0.0, 2.0), 1.225);
  REQUIRE(a.alpha > 0.0);
  REQUIRE(a.alpha == Approx(std::atan2(2.0, 20.0)));
  REQUIRE(a.beta == Approx(0.0).margin(1e-12));
  REQUIRE(a.airspeed == Approx(std::hypot(20.0, 2.0)));
  REQUIRE(a.dynamic_pressure == Approx(0.5 * 1.225 * a.airspeed * a.airspeed));

  // Positive v (rightward body velocity) is positive sideslip.
  const dynamics::AirData b = dynamics::computeAirData(Vec3(20.0, 3.0, 0.0), 1.225);
  REQUIRE(b.beta > 0.0);

  // At rest the model must not produce NaNs.
  const dynamics::AirData z = dynamics::computeAirData(Vec3::Zero(), 1.225);
  REQUIRE(std::isfinite(z.alpha));
  REQUIRE(std::isfinite(z.beta));
  REQUIRE(z.dynamic_pressure == Approx(0.0));
}

TEST_CASE("aerodynamic force and moment signs follow the stated conventions",
          "[dynamics][aero][signs]") {
  const auto p = dynamics::defaultAircraft();
  const dynamics::Aerodynamics aero(p);
  const dynamics::AirData air = dynamics::computeAirData(Vec3(25.0, 0.0, 0.0), 1.225);
  const Vec3 zero_rate = Vec3::Zero();
  ControlInput u;

  SECTION("lift acts upward in body axes and drag opposes the relative wind") {
    const dynamics::AirData a = dynamics::computeAirData(Vec3(25.0, 0.0, 2.0), 1.225);
    const Wrench w = aero.wrench(a, zero_rate, u);
    REQUIRE(w.force.z() < 0.0);  // lift is along -z_body
    // Drag is defined along the relative wind, not along body x: at positive alpha the
    // body-x component can be positive because the lift vector tilts forward. What must
    // hold is that the force projected onto the relative wind direction is negative.
    const Vec3 wind_dir = a.velocity_rel.normalized();
    REQUIRE(w.force.dot(wind_dir) < 0.0);
  }

  SECTION("increasing alpha increases lift below the stall") {
    double previous = -1e9;
    for (double alpha = 0.0; alpha < 0.20; alpha += 0.02) {
      const double cl = aero.liftCoefficient(alpha);
      REQUIRE(cl > previous);
      previous = cl;
    }
  }

  SECTION("post-stall lift falls back below the linear extrapolation") {
    const double alpha = 0.6;  // ~34 deg, well past the 17 deg stall blend
    const double linear = p.lon.CL0 + p.lon.CL_alpha * alpha;
    REQUIRE(aero.liftCoefficient(alpha) < linear);
    // Drag keeps increasing past the stall.
    REQUIRE(aero.dragCoefficient(alpha, 0.0) > aero.dragCoefficient(0.1, 0.0));
  }

  SECTION("positive elevator produces a nose-down pitching moment") {
    ControlInput up = u, down = u;
    down.elevator = 0.1;   // trailing edge down
    up.elevator = -0.1;    // trailing edge up
    REQUIRE(aero.wrench(air, zero_rate, down).moment.y() <
            aero.wrench(air, zero_rate, up).moment.y());
    REQUIRE(aero.wrench(air, zero_rate, down).moment.y() < 0.0);
    // ... and increases lift.
    REQUIRE(aero.wrench(air, zero_rate, down).force.z() <
            aero.wrench(air, zero_rate, up).force.z());
  }

  SECTION("positive aileron produces a positive (right) rolling moment") {
    ControlInput right = u;
    right.aileron = 0.1;
    REQUIRE(aero.wrench(air, zero_rate, right).moment.x() > 0.0);
  }

  SECTION("positive rudder gives right side force and a nose-left yawing moment") {
    ControlInput r = u;
    r.rudder = 0.1;
    const Wrench w = aero.wrench(air, zero_rate, r);
    REQUIRE(w.force.y() > 0.0);
    REQUIRE(w.moment.z() < 0.0);
  }

  SECTION("static stability: positive alpha gives a nose-down moment increment") {
    const dynamics::AirData a1 = dynamics::computeAirData(Vec3(25.0, 0.0, 0.0), 1.225);
    const dynamics::AirData a2 = dynamics::computeAirData(Vec3(25.0, 0.0, 3.0), 1.225);
    REQUIRE(aero.wrench(a2, zero_rate, u).moment.y() < aero.wrench(a1, zero_rate, u).moment.y());
  }

  SECTION("weathercock stability: positive sideslip gives a nose-right yawing moment") {
    const dynamics::AirData b = dynamics::computeAirData(Vec3(25.0, 3.0, 0.0), 1.225);
    REQUIRE(aero.wrench(b, zero_rate, u).moment.z() > 0.0);
    // and dihedral effect rolls away from the sideslip
    REQUIRE(aero.wrench(b, zero_rate, u).moment.x() < 0.0);
  }

  SECTION("damping derivatives oppose the body rates") {
    // Compare against the zero-rate wrench so the static terms (Cm0 etc.) cancel and the
    // test really checks the rate derivative.
    const Wrench base = aero.wrench(air, zero_rate, u);
    REQUIRE(aero.wrench(air, Vec3(0.5, 0, 0), u).moment.x() < base.moment.x());  // roll
    REQUIRE(aero.wrench(air, Vec3(0, 0.5, 0), u).moment.y() < base.moment.y());  // pitch
    REQUIRE(aero.wrench(air, Vec3(0, 0, 0.5), u).moment.z() < base.moment.z());  // yaw
    // ... and the increments are antisymmetric in the sign of the rate.
    REQUIRE(aero.wrench(air, Vec3(-0.5, 0, 0), u).moment.x() > base.moment.x());
    REQUIRE(aero.wrench(air, Vec3(0, -0.5, 0), u).moment.y() > base.moment.y());
    REQUIRE(aero.wrench(air, Vec3(0, 0, -0.5), u).moment.z() > base.moment.z());
  }
}

TEST_CASE("propeller thrust behaves like momentum theory", "[dynamics][propulsion]") {
  const auto p = dynamics::defaultAircraft();
  const dynamics::Propulsion prop(p);

  REQUIRE(prop.thrust(0.0, 0.0, 1.225) == Approx(0.0).margin(1e-12));
  REQUIRE(prop.thrust(1.0, 0.0, 1.225) > prop.thrust(0.5, 0.0, 1.225));
  // Thrust falls with airspeed and goes negative past the slipstream exit velocity.
  REQUIRE(prop.thrust(0.5, 30.0, 1.225) < prop.thrust(0.5, 10.0, 1.225));
  REQUIRE(prop.thrust(0.2, 40.0, 1.225) < 0.0);
  // Reaction torque opposes a right-handed propeller.
  REQUIRE(prop.reactionTorque(0.7) < 0.0);
  REQUIRE(prop.reactionTorque(0.0) == Approx(0.0));
  // Throttle outside [0,1] is clamped.
  REQUIRE(prop.thrust(5.0, 0.0, 1.225) == Approx(prop.thrust(1.0, 0.0, 1.225)));
}

TEST_CASE("thrust offset produces the expected moment", "[dynamics][propulsion]") {
  auto p = dynamics::defaultAircraft();
  p.prop.thrust_offset = Vec3(0.0, 0.0, 0.10);  // thrust line 0.10 m below the CG
  const dynamics::Propulsion prop(p);
  const Wrench w = prop.wrench(0.7, 20.0, 1.225);
  // r x F with r = (0,0,0.1) (below the CG in FRD) and F = (T,0,0) gives +0.1*T about body
  // y, i.e. a nose-up moment, which is what a thrust line below the CG must produce.
  REQUIRE(w.force.x() > 0.0);
  REQUIRE(w.moment.y() == Approx(0.10 * w.force.x()).epsilon(1e-12));
}

TEST_CASE("gravity is transformed correctly between frames", "[dynamics][gravity]") {
  const auto p = dynamics::defaultAircraft();
  const dynamics::RigidBody6DOF model(p);

  // Wings level: gravity acts along +z in body axes.
  StateVec x = levelState(0.0);
  dynamics::DynamicsDiagnostics d;
  model.totalWrench(x, ControlInput{}, stillAir(0.0), &d);
  REQUIRE(d.gravity_body.x() == Approx(0.0).margin(1e-12));
  REQUIRE(d.gravity_body.y() == Approx(0.0).margin(1e-12));
  REQUIRE(d.gravity_body.z() == Approx(p.mass * constants::kGravity).epsilon(1e-12));

  // Pitched 90 deg nose-up: gravity acts along -x in body axes.
  dynamics::AircraftState s = dynamics::AircraftState::fromVec(x);
  s.quaternion = math::eulerToQuat(0.0, constants::kPi / 2, 0.0);
  model.totalWrench(s.vec(), ControlInput{}, stillAir(0.0), &d);
  REQUIRE(d.gravity_body.x() == Approx(-p.mass * constants::kGravity).epsilon(1e-9));
  REQUIRE(d.gravity_body.z() == Approx(0.0).margin(1e-9));

  // Rolled 90 deg right: gravity acts along +y in body axes.
  s.quaternion = math::eulerToQuat(constants::kPi / 2, 0.0, 0.0);
  model.totalWrench(s.vec(), ControlInput{}, stillAir(0.0), &d);
  REQUIRE(d.gravity_body.y() == Approx(p.mass * constants::kGravity).epsilon(1e-9));
}

TEST_CASE("with no forces the vehicle travels in a straight line at constant speed",
          "[dynamics][conservation]") {
  const auto p = dynamics::defaultAircraft();
  dynamics::RigidBody6DOF model(p);
  // Zero density removes aerodynamics and thrust; zero gravity removes weight.
  dynamics::EnvironmentSample env = stillAir(0.0);
  env.gravity = 0.0;

  StateVec x = levelState(25.0);
  dynamics::AircraftState s0 = dynamics::AircraftState::fromVec(x);
  s0.velocity = Vec3(25.0, 3.0, -2.0);
  s0.quaternion = math::eulerToQuat(0.2, -0.1, 0.5);
  x = s0.vec();
  const Vec3 v_ned0 = math::rotateBodyToNed(s0.quaternion, s0.velocity);

  integrate::RungeKutta4 rk4;
  rk4.setProjection([](VecX& st) {
    st.segment<4>(StateIndex::kQuatW) = math::quatNormalize(st.segment<4>(StateIndex::kQuatW));
  });
  const double T = 10.0;
  const VecX xf = integrate::integrateTo(
      [&](double, const VecX& st) { return model.derivative(st, ControlInput{}, env); }, rk4,
      0.0, x, T, 0.01);

  const dynamics::AircraftState sf = dynamics::AircraftState::fromVec(xf);
  const Vec3 v_ned_f = math::rotateBodyToNed(sf.quaternion, sf.velocity);
  REQUIRE((v_ned_f - v_ned0).norm() == Approx(0.0).margin(1e-9));
  REQUIRE((sf.position - (s0.position + v_ned0 * T)).norm() == Approx(0.0).margin(1e-7));
  REQUIRE(sf.rate.norm() == Approx(0.0).margin(1e-12));
  REQUIRE(sf.quaternion.norm() == Approx(1.0).margin(1e-12));
}

TEST_CASE("free fall reproduces the analytic constant-acceleration solution",
          "[dynamics][conservation]") {
  const auto p = dynamics::defaultAircraft();
  dynamics::RigidBody6DOF model(p);
  dynamics::EnvironmentSample env = stillAir(0.0);  // vacuum, gravity on

  dynamics::AircraftState s0;
  s0.position = Vec3(0.0, 0.0, -1000.0);
  s0.velocity = Vec3::Zero();
  s0.quaternion = Vec4(1, 0, 0, 0);

  integrate::RungeKutta4 rk4;
  const double T = 5.0;
  const VecX xf = integrate::integrateTo(
      [&](double, const VecX& st) { return model.derivative(st, ControlInput{}, env); }, rk4,
      0.0, s0.vec(), T, 0.001);
  const dynamics::AircraftState sf = dynamics::AircraftState::fromVec(xf);

  const double expected_drop = 0.5 * constants::kGravity * T * T;
  const double expected_speed = constants::kGravity * T;
  REQUIRE(-sf.position.z() == Approx(1000.0 - expected_drop).epsilon(1e-10));
  REQUIRE(sf.velocity.z() == Approx(expected_speed).epsilon(1e-10));
  REQUIRE(sf.velocity.x() == Approx(0.0).margin(1e-10));
}

TEST_CASE("torque-free rotation conserves angular momentum and kinetic energy",
          "[dynamics][conservation]") {
  const auto p = dynamics::defaultAircraft();
  dynamics::RigidBody6DOF model(p);
  dynamics::EnvironmentSample env = stillAir(0.0);
  env.gravity = 0.0;

  dynamics::AircraftState s0;
  s0.velocity = Vec3(20.0, 0.0, 0.0);
  s0.rate = Vec3(0.8, 0.5, -0.3);
  const Mat3 J = p.inertia();
  const Vec3 H0_body = J * s0.rate;
  const double E0 = 0.5 * s0.rate.dot(J * s0.rate);

  integrate::RungeKutta4 rk4;
  rk4.setProjection([](VecX& st) {
    st.segment<4>(StateIndex::kQuatW) = math::quatNormalize(st.segment<4>(StateIndex::kQuatW));
  });
  const VecX xf = integrate::integrateTo(
      [&](double, const VecX& st) { return model.derivative(st, ControlInput{}, env); }, rk4,
      0.0, s0.vec(), 20.0, 0.001);
  const dynamics::AircraftState sf = dynamics::AircraftState::fromVec(xf);

  // Angular momentum is conserved in the inertial frame.
  const Vec3 H0_ned = math::rotateBodyToNed(s0.quaternion, H0_body);
  const Vec3 Hf_ned = math::rotateBodyToNed(sf.quaternion, J * sf.rate);
  REQUIRE((Hf_ned - H0_ned).norm() / H0_ned.norm() == Approx(0.0).margin(1e-9));
  const double Ef = 0.5 * sf.rate.dot(J * sf.rate);
  REQUIRE(Ef == Approx(E0).epsilon(1e-9));
}

TEST_CASE("quaternion stays normalised without explicit projection", "[dynamics][quaternion]") {
  const auto p = dynamics::defaultAircraft();
  dynamics::RigidBody6DOF model(p);
  dynamics::EnvironmentSample env = stillAir();

  dynamics::AircraftState s0;
  s0.position = Vec3(0, 0, -150);
  s0.velocity = Vec3(25.0, 1.0, 2.0);
  s0.rate = Vec3(0.4, 0.3, 0.2);
  // Deliberately start off the unit sphere: the Baumgarte term must pull it back.
  s0.quaternion = Vec4(1.02, 0.03, -0.01, 0.02);

  integrate::RungeKutta4 rk4;  // no projection installed
  const VecX xf = integrate::integrateTo(
      [&](double, const VecX& st) { return model.derivative(st, ControlInput{}, env); }, rk4,
      0.0, s0.vec(), 10.0, 0.002);
  REQUIRE(xf.segment<4>(StateIndex::kQuatW).norm() == Approx(1.0).margin(1e-6));
}

TEST_CASE("low-airspeed guard keeps the derivative finite", "[dynamics][robustness]") {
  const auto p = dynamics::defaultAircraft();
  dynamics::RigidBody6DOF model(p);
  dynamics::EnvironmentSample env = stillAir();

  dynamics::AircraftState s;
  s.position = Vec3(0, 0, -100);
  s.velocity = Vec3(1e-9, 0.0, 0.0);
  s.rate = Vec3(2.0, -1.5, 1.0);  // large rates at (almost) zero airspeed
  ControlInput u;
  u.elevator = 0.3;
  u.throttle = 0.5;

  const StateVec dx = model.derivative(s.vec(), u, env);
  REQUIRE(dx.allFinite());
  // Aerodynamic forces must vanish as Va -> 0 even though the rate terms are normalised
  // with a clamped airspeed: the whole wrench is scaled by the true dynamic pressure.
  dynamics::DynamicsDiagnostics d;
  model.totalWrench(s.vec(), u, env, &d);
  REQUIRE(d.aero.force.norm() < 1e-12);
  REQUIRE(d.aero.moment.norm() < 1e-12);
}

TEST_CASE("wind enters only through the air-relative velocity", "[dynamics][wind]") {
  const auto p = dynamics::defaultAircraft();
  dynamics::RigidBody6DOF model(p);

  dynamics::AircraftState s;
  s.position = Vec3(0, 0, -150);
  s.velocity = Vec3(25.0, 0.0, 1.5);
  ControlInput u;
  u.throttle = 0.6;

  dynamics::EnvironmentSample still = stillAir();
  dynamics::EnvironmentSample windy = stillAir();
  windy.wind_ned = Vec3(5.0, 0.0, 0.0);

  dynamics::DynamicsDiagnostics d_still, d_windy;
  model.totalWrench(s.vec(), u, still, &d_still);
  model.totalWrench(s.vec(), u, windy, &d_windy);
  REQUIRE(d_windy.air.airspeed == Approx(d_still.air.airspeed - 5.0).margin(0.02));
  // Same air-relative state in still air must give the same forces.
  dynamics::AircraftState s2 = s;
  s2.velocity = Vec3(20.0, 0.0, 1.5);
  dynamics::DynamicsDiagnostics d2;
  model.totalWrench(s2.vec(), u, still, &d2);
  REQUIRE((d2.aero.force - d_windy.aero.force).norm() == Approx(0.0).margin(1e-9));
}

TEST_CASE("specific force excludes gravity", "[dynamics][sensors]") {
  const auto p = dynamics::defaultAircraft();
  dynamics::RigidBody6DOF model(p);
  dynamics::EnvironmentSample env = stillAir(0.0);  // vacuum: only gravity acts

  dynamics::AircraftState s;
  s.position = Vec3(0, 0, -500);
  dynamics::DynamicsDiagnostics d;
  model.totalWrench(s.vec(), ControlInput{}, env, &d);
  // In free fall an accelerometer reads zero.
  REQUIRE(d.specific_force.norm() == Approx(0.0).margin(1e-12));
}

TEST_CASE("actuators respect magnitude limits and rate limits and lag", "[dynamics][actuators]") {
  auto p = dynamics::defaultAircraft();
  p.actuators.surface_rate_limit = 1.0;      // rad/s
  p.actuators.surface_time_constant = 0.001; // effectively instantaneous, so the rate binds
  dynamics::ActuatorBank bank(p);
  bank.reset(ControlInput{});

  ControlInput cmd;
  cmd.elevator = 10.0;  // far beyond the limit
  const double dt = 0.01;
  bank.update(cmd, dt);
  REQUIRE(bank.state().elevator == Approx(1.0 * dt).epsilon(1e-9));

  for (int i = 0; i < 1000; ++i) bank.update(cmd, dt);
  REQUIRE(bank.state().elevator == Approx(p.actuators.elevator_max));

  // First-order lag behaviour with a slow time constant.
  auto q = dynamics::defaultAircraft();
  q.actuators.surface_time_constant = 0.5;
  q.actuators.surface_rate_limit = 1e6;
  dynamics::ActuatorBank lag(q);
  lag.reset(ControlInput{});
  ControlInput small;
  small.elevator = 0.1;
  const double step = 0.001;
  for (int i = 0; i < 500; ++i) lag.update(small, step);  // t = 0.5 s = one time constant
  REQUIRE(lag.state().elevator == Approx(0.1 * (1.0 - std::exp(-1.0))).epsilon(0.02));

  // Throttle is clamped to [0, 1].
  ControlInput hot;
  hot.throttle = 5.0;
  for (int i = 0; i < 5000; ++i) lag.update(hot, step);
  REQUIRE(lag.state().throttle <= 1.0);
  REQUIRE(lag.state().throttle == Approx(1.0).margin(1e-6));
}

TEST_CASE("ISA atmosphere matches standard values", "[env][atmosphere]") {
  const env::Atmosphere atm;
  const auto sl = atm.at(0.0);
  REQUIRE(sl.temperature == Approx(288.15).epsilon(1e-9));
  REQUIRE(sl.pressure == Approx(101325.0).epsilon(1e-9));
  REQUIRE(sl.density == Approx(1.225).epsilon(1e-3));
  REQUIRE(sl.speed_of_sound == Approx(340.29).epsilon(1e-3));

  // Published ISA values at 5 km and 11 km.
  REQUIRE(atm.at(5000.0).temperature == Approx(255.65).epsilon(1e-4));
  REQUIRE(atm.at(5000.0).pressure == Approx(54019.9).epsilon(2e-4));
  REQUIRE(atm.at(5000.0).density == Approx(0.73643).epsilon(2e-3));
  REQUIRE(atm.at(11000.0).temperature == Approx(216.65).epsilon(1e-4));
  REQUIRE(atm.at(11000.0).pressure == Approx(22632.0).epsilon(1e-3));

  // Isothermal stratosphere above the tropopause.
  REQUIRE(atm.at(15000.0).temperature == Approx(216.65).epsilon(1e-6));
  REQUIRE(atm.at(15000.0).density < atm.at(11000.0).density);

  // Density decreases monotonically with altitude.
  double previous = 1e9;
  for (double h = 0.0; h <= 20000.0; h += 250.0) {
    const double rho = atm.density(h);
    REQUIRE(rho < previous);
    previous = rho;
  }

  // Density scaling for Monte-Carlo dispersion.
  REQUIRE(env::Atmosphere(1.1).density(1000.0) == Approx(1.1 * atm.density(1000.0)));
}

TEST_CASE("aircraft parameter validation rejects unphysical inputs", "[dynamics][config]") {
  auto p = dynamics::defaultAircraft();
  REQUIRE_NOTHROW(p.validate());
  REQUIRE(p.aspectRatio() == Approx(p.wing_span * p.wing_span / p.wing_area));

  auto bad = p;
  bad.mass = -1.0;
  REQUIRE_THROWS_AS(bad.validate(), std::invalid_argument);

  bad = p;
  bad.Jxz = 10.0;  // destroys positive definiteness
  REQUIRE_THROWS_AS(bad.validate(), std::invalid_argument);

  bad = p;
  bad.lon.oswald = 1.5;
  REQUIRE_THROWS_AS(bad.validate(), std::invalid_argument);

  bad = p;
  bad.actuators.elevator_max = bad.actuators.elevator_min - 0.1;
  REQUIRE_THROWS_AS(bad.validate(), std::invalid_argument);
}
