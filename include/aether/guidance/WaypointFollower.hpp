/// \file WaypointFollower.hpp
/// \brief Straight-line path following with half-plane waypoint switching and terminal orbit.
#pragma once

#include <string>
#include <vector>

#include "aether/control/Autopilot.hpp"
#include "aether/core/Types.hpp"

namespace aether::guidance {

/// \brief A single waypoint with an associated airspeed command.
struct Waypoint {
  double north = 0.0;      ///< North coordinate [m]
  double east = 0.0;       ///< East coordinate [m]
  double altitude = 100.0; ///< Altitude above the NED origin [m]
  double airspeed = 25.0;  ///< Commanded airspeed while flying to this waypoint [m/s]

  /// NED position of the waypoint [m] (down = -altitude).
  Vec3 ned() const { return Vec3(north, east, -altitude); }
};

/// \brief What the vehicle does after the last waypoint.
enum class TerminalBehaviour {
  kLoop,   ///< Cycle back to the first waypoint and fly the circuit again.
  kOrbit,  ///< Loiter on a circle centred on the last waypoint.
  kHold    ///< Keep flying the final leg's course and altitude indefinitely.
};

/// \brief Configuration of the waypoint follower.
struct GuidanceConfig {
  std::vector<Waypoint> waypoints;  ///< Mission waypoints, at least two.
  TerminalBehaviour terminal = TerminalBehaviour::kLoop;  ///< End-of-mission behaviour.

  double path_gain = 0.04;      ///< Cross-track gain \f$k_{path}\f$ [1/m]
  double chi_infinity = 1.05;   ///< Approach course limit \f$\chi_\infty\f$ [rad] (~60 deg)
  double capture_radius = 30.0; ///< Waypoint capture radius used as a switching fallback [m]
  double orbit_radius = 90.0;   ///< Radius of the terminal loiter circle [m]
  double orbit_gain = 2.5;      ///< Orbit-following gain \f$k_{orbit}\f$ [-]
  int orbit_direction = 1;      ///< +1 = clockwise (viewed from above), -1 = counter-clockwise
  bool use_half_plane_switching = true;  ///< Switch on crossing the bisector half-plane.
};

/// \brief Diagnostic outputs of the guidance law.
struct GuidanceDiagnostics {
  int active_index = 1;           ///< Index of the waypoint currently being flown to.
  int laps_completed = 0;         ///< Number of completed circuits (loop mode).
  double cross_track_error = 0.0; ///< Signed lateral deviation from the path [m]
  double along_track = 0.0;       ///< Distance travelled along the active leg [m]
  double leg_length = 0.0;        ///< Length of the active leg [m]
  double distance_to_waypoint = 0.0;  ///< Straight-line distance to the active waypoint [m]
  double path_course = 0.0;       ///< Course of the active leg [rad]
  bool orbiting = false;          ///< True when the terminal orbit is active.
};

/// \brief Waypoint-following guidance producing course, altitude and airspeed commands.
///
/// Straight-line following uses the standard vector-field law
/// \f[ \chi_c = \chi_q - \chi_\infty \frac{2}{\pi}\arctan(k_{path}\, e_{py}) \f]
/// where \f$e_{py}\f$ is the signed cross-track error and \f$\chi_q\f$ the course of the leg
/// (Beard & McLain 2012, Ch. 10). The active waypoint advances when the vehicle crosses the
/// half-plane whose normal bisects the incoming and outgoing legs, or when it comes within
/// `capture_radius` of the waypoint — whichever happens first. Altitude is commanded by
/// linear interpolation along the active leg, so altitude changes are spread over the leg
/// instead of stepping at the waypoint.
class WaypointFollower {
 public:
  /// \throws std::invalid_argument if fewer than two waypoints are supplied.
  explicit WaypointFollower(const GuidanceConfig& config);

  /// Update the guidance law and return the outer-loop commands.
  /// \param position_ned Current NED position [m].
  /// \param course Current ground-track course [rad] (used only for the terminal hold mode).
  control::AutopilotCommand update(const Vec3& position_ned, double course);

  /// Restart the mission from the first leg.
  void reset();

  /// Diagnostics from the most recent update.
  const GuidanceDiagnostics& diagnostics() const { return diag_; }

  /// Mission waypoints.
  const std::vector<Waypoint>& waypoints() const { return cfg_.waypoints; }

  /// Configuration in use.
  const GuidanceConfig& config() const { return cfg_; }

 private:
  control::AutopilotCommand followOrbit(const Vec3& position_ned);

  GuidanceConfig cfg_;
  std::size_t active_ = 1;
  int laps_ = 0;
  bool orbiting_ = false;
  GuidanceDiagnostics diag_;
};

/// \brief Parse a terminal-behaviour name ("loop", "orbit", "hold").
/// \throws std::invalid_argument for an unknown name.
TerminalBehaviour parseTerminalBehaviour(const std::string& name);

}  // namespace aether::guidance
