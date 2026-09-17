#include "aether/guidance/WaypointFollower.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "aether/core/Constants.hpp"
#include "aether/math/Rotation.hpp"

namespace aether::guidance {

TerminalBehaviour parseTerminalBehaviour(const std::string& name) {
  if (name == "loop") return TerminalBehaviour::kLoop;
  if (name == "orbit") return TerminalBehaviour::kOrbit;
  if (name == "hold") return TerminalBehaviour::kHold;
  throw std::invalid_argument("parseTerminalBehaviour: unknown behaviour '" + name +
                              "' (expected loop, orbit or hold)");
}

WaypointFollower::WaypointFollower(const GuidanceConfig& config) : cfg_(config) {
  if (cfg_.waypoints.size() < 2)
    throw std::invalid_argument("WaypointFollower: at least two waypoints are required");
  reset();
}

void WaypointFollower::reset() {
  active_ = 1;
  laps_ = 0;
  orbiting_ = false;
  diag_ = GuidanceDiagnostics{};
  diag_.active_index = 1;
}

control::AutopilotCommand WaypointFollower::followOrbit(const Vec3& position_ned) {
  const Waypoint& centre = cfg_.waypoints.back();
  const double dn = position_ned.x() - centre.north;
  const double de = position_ned.y() - centre.east;
  const double d = std::hypot(dn, de);
  const double angular_position = std::atan2(de, dn);
  const double lambda = cfg_.orbit_direction >= 0 ? 1.0 : -1.0;
  const double rho = std::max(cfg_.orbit_radius, 1.0);

  control::AutopilotCommand cmd;
  cmd.course = math::wrapPi(angular_position +
                            lambda * (0.5 * constants::kPi +
                                      std::atan(cfg_.orbit_gain * (d - rho) / rho)));
  cmd.altitude = centre.altitude;
  cmd.airspeed = centre.airspeed;

  diag_.orbiting = true;
  diag_.cross_track_error = d - rho;
  diag_.distance_to_waypoint = d;
  diag_.path_course = cmd.course;
  diag_.along_track = 0.0;
  diag_.leg_length = 2.0 * constants::kPi * rho;
  return cmd;
}

control::AutopilotCommand WaypointFollower::update(const Vec3& position_ned, double course) {
  const std::size_t n = cfg_.waypoints.size();

  if (orbiting_) return followOrbit(position_ned);

  // Geometry of the leg that ends at waypoint `index`. In loop mode `index` wraps to 0, so
  // the closing leg (last waypoint -> first waypoint) is flown too.
  struct LegGeometry {
    Vec3 start = Vec3::Zero();
    Vec3 end = Vec3::Zero();
    Vec3 direction = Vec3(1.0, 0.0, 0.0);
    double length = 0.0;
    double course = 0.0;
    double start_altitude = 0.0;
    double end_altitude = 0.0;
    double airspeed = 25.0;
  };
  auto legFor = [&](std::size_t index) {
    const std::size_t from_index = index == 0 ? n - 1 : index - 1;
    const Waypoint& from = cfg_.waypoints[from_index];
    const Waypoint& to = cfg_.waypoints[index];
    LegGeometry g;
    g.start = from.ned();
    g.end = to.ned();
    Vec3 q = g.end - g.start;
    q.z() = 0.0;  // the path direction is horizontal; altitude is handled separately
    g.length = q.norm();
    g.direction = g.length > 1e-6 ? (q / g.length).eval() : Vec3(1.0, 0.0, 0.0);
    g.course = std::atan2(g.direction.y(), g.direction.x());
    g.start_altitude = from.altitude;
    g.end_altitude = to.altitude;
    g.airspeed = to.airspeed;
    return g;
  };

  // ---- Switching decision, taken before the command is computed so there is no one-step
  // lag between passing a waypoint and steering towards the next one. ---------------------
  {
    const LegGeometry leg = legFor(active_);
    const Vec3 to_wp = leg.end - position_ned;
    bool advance = std::hypot(to_wp.x(), to_wp.y()) < cfg_.capture_radius;
    if (cfg_.use_half_plane_switching) {
      // Half-plane normal: the bisector of the incoming and outgoing leg directions. For a
      // final waypoint with no outgoing leg the normal is the incoming direction.
      Vec3 qnext = leg.direction;
      const bool has_next = (active_ + 1 < n) || cfg_.terminal == TerminalBehaviour::kLoop;
      if (has_next) {
        Vec3 qn = cfg_.waypoints[(active_ + 1) % n].ned() - leg.end;
        qn.z() = 0.0;
        if (qn.norm() > 1e-6) qnext = qn.normalized();
      }
      Vec3 normal = leg.direction + qnext;
      normal.z() = 0.0;
      if (normal.norm() > 1e-6) {
        normal.normalize();
        const Vec3 rel = position_ned - leg.end;
        if (rel.x() * normal.x() + rel.y() * normal.y() >= 0.0) advance = true;
      }
    }
    if (advance) {
      if (active_ + 1 < n) {
        ++active_;
      } else {
        switch (cfg_.terminal) {
          case TerminalBehaviour::kLoop:
            // Wrap to waypoint 0: fly the closing leg back to the start of the circuit.
            active_ = 0;
            ++laps_;
            break;
          case TerminalBehaviour::kOrbit:
            orbiting_ = true;
            break;
          case TerminalBehaviour::kHold:
            // Keep the final leg active; the vector field simply extends past the waypoint.
            break;
        }
      }
    }
  }
  if (orbiting_) return followOrbit(position_ned);

  // ---- Straight-line path following on the (possibly new) active leg --------------------
  const LegGeometry leg = legFor(active_);

  // Signed cross-track error: positive when the vehicle is to the right of the path.
  const Vec3 ep = position_ned - leg.start;
  const double e_py = -std::sin(leg.course) * ep.x() + std::cos(leg.course) * ep.y();
  const double s_along = std::cos(leg.course) * ep.x() + std::sin(leg.course) * ep.y();

  control::AutopilotCommand cmd;
  cmd.course = math::wrapPi(leg.course - cfg_.chi_infinity * (2.0 / constants::kPi) *
                                             std::atan(cfg_.path_gain * e_py));

  // Altitude: linear interpolation along the leg, clamped to the leg endpoints.
  const double frac = leg.length > 1e-6 ? std::clamp(s_along / leg.length, 0.0, 1.0) : 1.0;
  cmd.altitude = leg.start_altitude + frac * (leg.end_altitude - leg.start_altitude);
  cmd.airspeed = leg.airspeed;

  const Vec3 to_wp = leg.end - position_ned;
  diag_.active_index = static_cast<int>(active_);
  diag_.laps_completed = laps_;
  diag_.cross_track_error = e_py;
  diag_.along_track = s_along;
  diag_.leg_length = leg.length;
  diag_.distance_to_waypoint = std::hypot(to_wp.x(), to_wp.y());
  diag_.path_course = leg.course;
  diag_.orbiting = false;

  (void)course;  // the command is a pure function of position; kept for interface symmetry
  return cmd;
}

}  // namespace aether::guidance
