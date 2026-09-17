"""Figure generators.

Each function takes already-loaded data and an output path, writes one PNG, and returns
that path. Conventions followed throughout (see :mod:`aether_viz.style`):

* categorical hues are assigned in fixed slot order and never cycled;
* every panel has a single y-axis - two quantities of different scale become two panels;
* any panel with more than one series carries a legend, so identity is never colour-alone;
* signed quantities use the diverging pair, magnitudes use the single-hue sequential ramp.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.lines import Line2D

from .io import RunData, quaternion_to_rotation
from .style import DIVERGING, INK, PALETTE, SEQUENTIAL, annotate_source, finish_axes, series_color

DEG = 180.0 / np.pi


def _save(fig, path: str | Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)
    return path


def _provenance(run: RunData) -> str:
    s = run.summary
    if not s:
        return f"aether6 - {run.directory}"
    return (
        f"aether6 - scenario '{s.get('scenario', run.name)}', controller {s.get('controller', '?')}, "
        f"feedback {s.get('feedback', '?')}, integrator {s.get('integrator', '?')} "
        f"dt={s.get('dt', '?')} s, seed {s.get('seed', '?')}"
    )


# ---------------------------------------------------------------------------------------
# Trajectory
# ---------------------------------------------------------------------------------------
def plot_trajectory_3d(run: RunData, path: str | Path) -> Path:
    """3-D flight path coloured by altitude, with the waypoint course and a ground shadow."""
    d = run.states
    fig = plt.figure(figsize=(11, 7.5))
    ax = fig.add_subplot(1, 1, 1, projection="3d")

    north, east, alt = d["pn_m"].to_numpy(), d["pe_m"].to_numpy(), d["altitude_m"].to_numpy()

    # Altitude is a magnitude, so it is encoded with the single-hue sequential ramp.
    cmap = plt.matplotlib.colors.LinearSegmentedColormap.from_list("aether_blue", SEQUENTIAL)
    points = np.stack([east, north, alt], axis=1)
    segments = np.stack([points[:-1], points[1:]], axis=1)
    from mpl_toolkits.mplot3d.art3d import Line3DCollection

    norm = plt.Normalize(alt.min(), alt.max())
    lc = Line3DCollection(segments, cmap=cmap, norm=norm, linewidths=1.7)
    lc.set_array(alt[:-1])
    ax.add_collection3d(lc)

    floor = max(0.0, alt.min() - 25.0)
    ax.plot(east, north, np.full_like(east, floor), color=INK["muted"], linewidth=0.8,
            alpha=0.55, label="ground track")

    wp = run.waypoints
    if len(wp):
        ax.scatter(wp["east_m"], wp["north_m"], wp["altitude_m"], color=PALETTE[1], s=45,
                   depthshade=False, label="waypoints", zorder=5)
        closed = pd.concat([wp, wp.iloc[[0]]], ignore_index=True)
        ax.plot(closed["east_m"], closed["north_m"], closed["altitude_m"], color=PALETTE[1],
                linewidth=1.0, linestyle="--", alpha=0.85, label="commanded course")
        for _, row in wp.iterrows():
            ax.text(row["east_m"], row["north_m"], row["altitude_m"] + 8,
                    f"W{int(row['index'])}", color=INK["secondary"], fontsize=8)

    ax.scatter([east[0]], [north[0]], [alt[0]], color=PALETTE[5], s=60, marker="^",
               depthshade=False, label="start")

    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_zlabel("Altitude [m]")
    ax.set_title("Closed-loop 3-D flight path", loc="left")
    ax.set_zlim(floor, max(alt.max() + 20.0, floor + 60.0))
    ax.view_init(elev=26, azim=-135)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper left")

    bar = fig.colorbar(lc, ax=ax, shrink=0.55, pad=0.1)
    bar.set_label("Altitude [m]", color=INK["secondary"])
    annotate_source(fig, _provenance(run))
    return _save(fig, path)


def plot_ground_track(run: RunData, path: str | Path) -> Path:
    """Plan view of the ground track with the commanded course and cross-track error."""
    d = run.states
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 6.0), width_ratios=[1.25, 1.0])

    ax = axes[0]
    wp = run.waypoints
    if len(wp):
        closed = pd.concat([wp, wp.iloc[[0]]], ignore_index=True)
        ax.plot(closed["east_m"], closed["north_m"], color=PALETTE[1], linewidth=1.3,
                linestyle="--", label="commanded course")
        ax.scatter(wp["east_m"], wp["north_m"], color=PALETTE[1], s=42, zorder=5,
                   label="waypoints")
        for _, row in wp.iterrows():
            ax.annotate(f"W{int(row['index'])}", (row["east_m"], row["north_m"]),
                        textcoords="offset points", xytext=(7, 7),
                        color=INK["secondary"], fontsize=8.5)
    ax.plot(d["pe_m"], d["pn_m"], color=PALETTE[0], linewidth=1.5, label="flown track")
    ax.scatter([d["pe_m"].iloc[0]], [d["pn_m"].iloc[0]], color=PALETTE[5], s=55, marker="^",
               zorder=6, label="start")
    ax.set_aspect("equal", adjustable="datalim")
    finish_axes(ax, "Ground track", "East [m]", "North [m]", legend=True, legend_loc="lower left")

    ax = axes[1]
    e = d["cross_track_m"].to_numpy()
    t = run.time
    # Signed quantity: the diverging pair, with a neutral zero line.
    ax.axhline(0.0, color=INK["muted"], linewidth=1.0)
    ax.fill_between(t, 0, np.clip(e, 0, None), color=DIVERGING[2], alpha=0.35,
                    label="right of path")
    ax.fill_between(t, 0, np.clip(e, None, 0), color=DIVERGING[0], alpha=0.35,
                    label="left of path")
    ax.plot(t, e, color=INK["truth"], linewidth=1.0)
    rms = float(np.sqrt(np.mean(e**2)))
    ax.axhline(rms, color=INK["muted"], linewidth=0.9, linestyle=":")
    ax.annotate(f"RMS {rms:.1f} m", (t[-1], rms), textcoords="offset points",
                xytext=(-8, 6), ha="right", color=INK["secondary"], fontsize=8.5)
    finish_axes(ax, "Cross-track error", "Time [s]", "Cross-track [m]", legend=True)

    annotate_source(fig, _provenance(run))
    return _save(fig, path)


# ---------------------------------------------------------------------------------------
# State histories
# ---------------------------------------------------------------------------------------
def plot_states(run: RunData, path: str | Path) -> Path:
    """Position, velocity, Euler angles and body rates."""
    d = run.states
    t = run.time
    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)

    ax = axes[0]
    for i, (col, label) in enumerate([("pn_m", "north"), ("pe_m", "east"),
                                      ("altitude_m", "altitude")]):
        ax.plot(t, d[col], color=series_color(i), label=label)
    finish_axes(ax, "Position (NED origin at the start point)", None, "Position [m]",
                legend=True)

    ax = axes[1]
    for i, (col, label) in enumerate([("vn_mps", "north"), ("ve_mps", "east"),
                                      ("vd_mps", "down")]):
        ax.plot(t, d[col], color=series_color(i), label=label)
    finish_axes(ax, "Inertial velocity", None, "Velocity [m/s]", legend=True)

    ax = axes[2]
    for i, (col, label) in enumerate([("roll_rad", "roll $\\phi$"),
                                      ("pitch_rad", "pitch $\\theta$"),
                                      ("yaw_rad", "yaw $\\psi$")]):
        ax.plot(t, d[col] * DEG, color=series_color(i), label=label)
    finish_axes(ax, "Euler angles (3-2-1)", None, "Angle [deg]", legend=True)

    ax = axes[3]
    for i, (col, label) in enumerate([("p_radps", "roll rate $p$"),
                                      ("q_radps", "pitch rate $q$"),
                                      ("r_radps", "yaw rate $r$")]):
        ax.plot(t, d[col] * DEG, color=series_color(i), label=label)
    finish_axes(ax, "Body angular rates", "Time [s]", "Rate [deg/s]", legend=True)

    annotate_source(fig, _provenance(run))
    return _save(fig, path)


def plot_controls(run: RunData, path: str | Path) -> Path:
    """Commanded versus achieved control deflections, with the actuator limits shown."""
    d = run.states
    t = run.time
    fig, axes = plt.subplots(4, 1, figsize=(11, 10.5), sharex=True)

    channels = [
        ("elevator", "cmd_elevator_rad", "act_elevator_rad", DEG, "Elevator $\\delta_e$ [deg]", 25.0),
        ("aileron", "cmd_aileron_rad", "act_aileron_rad", DEG, "Aileron $\\delta_a$ [deg]", 20.0),
        ("rudder", "cmd_rudder_rad", "act_rudder_rad", DEG, "Rudder $\\delta_r$ [deg]", 25.0),
        ("throttle", "cmd_throttle", "act_throttle", 1.0, "Throttle $\\delta_t$ [-]", None),
    ]
    for ax, (name, cmd, act, scale, ylabel, limit) in zip(axes, channels):
        ax.plot(t, d[cmd] * scale, color=series_color(1), linewidth=1.0, alpha=0.75,
                label="commanded")
        ax.plot(t, d[act] * scale, color=series_color(0), linewidth=1.5, label="achieved")
        if limit is not None:
            for sign in (1, -1):
                ax.axhline(sign * limit, color=INK["muted"], linewidth=0.9, linestyle=":")
            ax.annotate("actuator limit", (t[-1], limit), textcoords="offset points",
                        xytext=(-6, 4), ha="right", color=INK["muted"], fontsize=8)
        else:
            ax.set_ylim(-0.05, 1.05)
            for level in (0.0, 1.0):
                ax.axhline(level, color=INK["muted"], linewidth=0.9, linestyle=":")
        finish_axes(ax, f"{name.capitalize()} command and actuator response", None, ylabel,
                    legend=True)
    axes[-1].set_xlabel("Time [s]")
    annotate_source(fig, _provenance(run))
    return _save(fig, path)


def plot_air_data(run: RunData, path: str | Path) -> Path:
    """Airspeed, angle of attack and sideslip, against their commands and limits."""
    d = run.states
    t = run.time
    fig, axes = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True)

    ax = axes[0]
    ax.plot(t, d["cmd_airspeed_mps"], color=series_color(1), linewidth=1.1, linestyle="--",
            label="commanded")
    ax.plot(t, d["airspeed_mps"], color=series_color(0), label="true airspeed $V_a$")
    if run.has("est_airspeed_mps"):
        ax.plot(t, d["est_airspeed_mps"], color=series_color(2), linewidth=1.0, alpha=0.8,
                label="EKF estimate")
    finish_axes(ax, "Airspeed", None, "$V_a$ [m/s]", legend=True)

    ax = axes[1]
    ax.plot(t, d["alpha_rad"] * DEG, color=series_color(0), label="truth")
    if run.has("est_alpha_rad"):
        ax.plot(t, d["est_alpha_rad"] * DEG, color=series_color(2), linewidth=1.0, alpha=0.8,
                label="EKF estimate")
    ax.axhline(17.2, color=PALETTE[7], linewidth=1.0, linestyle=":")
    ax.annotate("stall blend 17.2 deg", (t[-1], 17.2), textcoords="offset points",
                xytext=(-6, 4), ha="right", color=PALETTE[7], fontsize=8)
    finish_axes(ax, "Angle of attack", None, r"$\alpha$ [deg]", legend=True)

    ax = axes[2]
    ax.plot(t, d["beta_rad"] * DEG, color=series_color(0), label="truth")
    if run.has("est_beta_rad"):
        ax.plot(t, d["est_beta_rad"] * DEG, color=series_color(2), linewidth=1.0, alpha=0.8,
                label="EKF estimate")
    ax.axhline(0.0, color=INK["muted"], linewidth=0.9)
    finish_axes(ax, "Sideslip", "Time [s]", r"$\beta$ [deg]", legend=True)

    annotate_source(fig, _provenance(run))
    return _save(fig, path)


def plot_tracking(run: RunData, path: str | Path) -> Path:
    """Altitude, airspeed and course tracking errors, plus the active waypoint."""
    d = run.states
    t = run.time
    fig, axes = plt.subplots(4, 1, figsize=(11, 10.5), sharex=True)

    ax = axes[0]
    ax.plot(t, d["cmd_altitude_m"], color=series_color(1), linewidth=1.1, linestyle="--",
            label="commanded")
    ax.plot(t, d["altitude_m"], color=series_color(0), label="achieved")
    finish_axes(ax, "Altitude tracking", None, "Altitude [m]", legend=True)

    ax = axes[1]
    err = (d["altitude_m"] - d["cmd_altitude_m"]).to_numpy()
    ax.axhline(0.0, color=INK["muted"], linewidth=0.9)
    ax.fill_between(t, 0, np.clip(err, 0, None), color=DIVERGING[2], alpha=0.35, label="high")
    ax.fill_between(t, 0, np.clip(err, None, 0), color=DIVERGING[0], alpha=0.35, label="low")
    ax.plot(t, err, color=INK["truth"], linewidth=0.9)
    ax.annotate(f"RMS {np.sqrt(np.mean(err**2)):.2f} m", (0.995, 0.06),
                xycoords="axes fraction", ha="right", color=INK["secondary"], fontsize=8.5)
    finish_axes(ax, "Altitude error", None, "Error [m]", legend=True)

    ax = axes[2]
    verr = (d["airspeed_mps"] - d["cmd_airspeed_mps"]).to_numpy()
    ax.axhline(0.0, color=INK["muted"], linewidth=0.9)
    ax.plot(t, verr, color=series_color(0), linewidth=1.1)
    ax.annotate(f"RMS {np.sqrt(np.mean(verr**2)):.2f} m/s", (0.995, 0.06),
                xycoords="axes fraction", ha="right", color=INK["secondary"], fontsize=8.5)
    finish_axes(ax, "Airspeed error", None, "Error [m/s]")

    ax = axes[3]
    ax.step(t, d["wp_index"], color=series_color(3), where="post", linewidth=1.4,
            label="active waypoint")
    ax.step(t, d["wp_laps"], color=series_color(6), where="post", linewidth=1.2,
            label="laps completed")
    finish_axes(ax, "Guidance state", "Time [s]", "Index / count", legend=True)

    annotate_source(fig, _provenance(run))
    return _save(fig, path)


# ---------------------------------------------------------------------------------------
# Estimator
# ---------------------------------------------------------------------------------------
def plot_estimator(run: RunData, path: str | Path) -> Path:
    """Truth, raw measurements and EKF estimate with 3-sigma bounds."""
    d = run.states
    t = run.time
    fig, axes = plt.subplots(4, 2, figsize=(14, 12))

    def bounded(ax, truth, estimate, sigma, title, ylabel, meas=None, meas_label=None):
        err = estimate - truth
        if meas is not None:
            ax.plot(meas[0], meas[1] - np.interp(meas[0], t, truth), linestyle="none",
                    marker=".", markersize=2.5, color=INK["muted"], alpha=0.5,
                    label=meas_label)
        ax.fill_between(t, -3 * sigma, 3 * sigma, color=series_color(0), alpha=0.16,
                        label=r"EKF $\pm3\sigma$")
        ax.plot(t, err, color=series_color(0), linewidth=1.1, label="estimate error")
        ax.axhline(0.0, color=INK["muted"], linewidth=0.9)
        inside = float(np.mean(np.abs(err) <= 3 * sigma) * 100.0)
        ax.annotate(f"{inside:.1f}% inside $3\\sigma$", (0.995, 0.06),
                    xycoords="axes fraction", ha="right", color=INK["secondary"], fontsize=8.5)
        finish_axes(ax, title, "Time [s]", ylabel, legend=True)

    gps = run.sensors.get("gps")
    bounded(axes[0, 0], d["pn_m"].to_numpy(), d["est_pn_m"].to_numpy(),
            d["sig_pn"].to_numpy(), "North position error", "Error [m]",
            meas=(gps["time_s"].to_numpy(), gps["pn_m"].to_numpy()) if gps is not None else None,
            meas_label="GNSS fix")
    bounded(axes[0, 1], d["pe_m"].to_numpy(), d["est_pe_m"].to_numpy(),
            d["sig_pe"].to_numpy(), "East position error", "Error [m]",
            meas=(gps["time_s"].to_numpy(), gps["pe_m"].to_numpy()) if gps is not None else None,
            meas_label="GNSS fix")

    baro = run.sensors.get("baro")
    bounded(axes[1, 0], d["pd_m"].to_numpy(), d["est_pd_m"].to_numpy(),
            d["sig_pd"].to_numpy(), "Down position error", "Error [m]",
            meas=(baro["time_s"].to_numpy(), -baro["altitude_m"].to_numpy())
            if baro is not None else None, meas_label="barometer")
    bounded(axes[1, 1], d["vn_mps"].to_numpy(), d["est_vn_mps"].to_numpy(),
            d["sig_vn"].to_numpy(), "North velocity error", "Error [m/s]",
            meas=(gps["time_s"].to_numpy(), gps["vn_mps"].to_numpy()) if gps is not None else None,
            meas_label="GNSS velocity")

    def angle_error(a, b):
        return np.arctan2(np.sin(a - b), np.cos(a - b))

    bounded(axes[2, 0], np.zeros_like(t),
            angle_error(d["est_roll_rad"].to_numpy(), d["roll_rad"].to_numpy()) * DEG,
            d["sig_att_x"].to_numpy() * DEG, "Roll estimate error", "Error [deg]")
    bounded(axes[2, 1], np.zeros_like(t),
            angle_error(d["est_yaw_rad"].to_numpy(), d["yaw_rad"].to_numpy()) * DEG,
            d["sig_att_z"].to_numpy() * DEG, "Yaw estimate error", "Error [deg]")

    ax = axes[3, 0]
    for i, (est, truth, label) in enumerate([
            ("est_bgx", "true_bgx", "$b_{g,x}$"),
            ("est_bgy", "true_bgy", "$b_{g,y}$"),
            ("est_bgz", "true_bgz", "$b_{g,z}$")]):
        ax.plot(t, d[truth] * 1e3, color=series_color(i), linewidth=1.0, linestyle="--")
        ax.plot(t, d[est] * 1e3, color=series_color(i), linewidth=1.4, label=label)
    ax.plot([], [], color=INK["muted"], linestyle="--", label="truth (dashed)")
    finish_axes(ax, "Gyro bias estimate vs truth", "Time [s]", "Bias [mrad/s]", legend=True)

    ax = axes[3, 1]
    if run.has("est_baro_bias_m"):
        ax.plot(t, d["true_baro_bias_m"], color=INK["truth"], linewidth=1.1, linestyle="--",
                label="truth")
        ax.fill_between(t, d["est_baro_bias_m"] - 3 * d["sig_baro_bias"],
                        d["est_baro_bias_m"] + 3 * d["sig_baro_bias"],
                        color=series_color(3), alpha=0.18, label=r"$\pm3\sigma$")
        ax.plot(t, d["est_baro_bias_m"], color=series_color(3), linewidth=1.4,
                label="estimate")
        finish_axes(ax, "Barometer bias estimate", "Time [s]", "Bias [m]", legend=True)
    elif run.has("est_wind_n_mps"):
        for i, (est, truth, label) in enumerate([
                ("est_wind_n_mps", "wind_n_mps", "north"),
                ("est_wind_e_mps", "wind_e_mps", "east")]):
            ax.plot(t, d[truth], color=series_color(i), linewidth=1.0, linestyle="--")
            ax.plot(t, d[est], color=series_color(i), linewidth=1.4, label=label)
        ax.plot([], [], color=INK["muted"], linestyle="--", label="truth (dashed)")
        finish_axes(ax, "Wind estimate vs truth", "Time [s]", "Wind [m/s]", legend=True)
    else:
        ax.axis("off")

    annotate_source(fig, _provenance(run))
    return _save(fig, path)


def plot_wind_estimate(run: RunData, path: str | Path) -> Path:
    """Wind and turbulence truth against the EKF wind estimate."""
    d = run.states
    t = run.time
    fig, axes = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True)

    ax = axes[0]
    for i, (truth, est, label) in enumerate([("wind_n_mps", "est_wind_n_mps", "north"),
                                             ("wind_e_mps", "est_wind_e_mps", "east")]):
        ax.plot(t, d[truth], color=series_color(i), linewidth=1.0, linestyle="--", alpha=0.85)
        if run.has(est):
            ax.plot(t, d[est], color=series_color(i), linewidth=1.5, label=f"{label} estimate")
        else:
            ax.plot([], [], color=series_color(i), label=label)
    ax.plot([], [], color=INK["muted"], linestyle="--", label="truth (dashed)")
    finish_axes(ax, "Wind field: truth and EKF estimate", None, "Wind [m/s]", legend=True)

    ax = axes[1]
    for i, (col, label) in enumerate([("gust_u_mps", "$u_g$"), ("gust_v_mps", "$v_g$"),
                                      ("gust_w_mps", "$w_g$")]):
        ax.plot(t, d[col], color=series_color(i), linewidth=1.0, label=label)
    finish_axes(ax, "Dryden gust components (body axes)", None, "Gust [m/s]", legend=True)

    ax = axes[2]
    ax.plot(t, d["beta_rad"] * DEG, color=series_color(0), linewidth=1.2,
            label=r"sideslip $\beta$")
    ax.plot(t, d["cross_track_m"] / 10.0, color=series_color(1), linewidth=1.2,
            label="cross-track / 10 [m]")
    ax.axhline(0.0, color=INK["muted"], linewidth=0.9)
    finish_axes(ax, "Disturbance response", "Time [s]", "Response", legend=True)

    annotate_source(fig, _provenance(run))
    return _save(fig, path)


def plot_estimator_summary(run: RunData, path: str | Path) -> Path:
    """Overall estimator error norms with their 3-sigma envelopes."""
    d = run.states
    t = run.time
    fig, axes = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True)

    ax = axes[0]
    sigma = np.sqrt(d["sig_pn"] ** 2 + d["sig_pe"] ** 2 + d["sig_pd"] ** 2).to_numpy()
    ax.fill_between(t, 0, 3 * sigma, color=series_color(0), alpha=0.16,
                    label=r"$3\sigma$ envelope")
    ax.plot(t, d["est_pos_err_m"], color=series_color(0), linewidth=1.2, label="error norm")
    finish_axes(ax, "Position estimate error", None, "Error [m]", legend=True)

    ax = axes[1]
    sigma = np.sqrt(d["sig_vn"] ** 2 + d["sig_ve"] ** 2 + d["sig_vd"] ** 2).to_numpy()
    ax.fill_between(t, 0, 3 * sigma, color=series_color(1), alpha=0.16,
                    label=r"$3\sigma$ envelope")
    ax.plot(t, d["est_vel_err_mps"], color=series_color(1), linewidth=1.2, label="error norm")
    finish_axes(ax, "Velocity estimate error", None, "Error [m/s]", legend=True)

    ax = axes[2]
    sigma = np.sqrt(d["sig_att_x"] ** 2 + d["sig_att_y"] ** 2 + d["sig_att_z"] ** 2).to_numpy()
    ax.fill_between(t, 0, 3 * sigma * DEG, color=series_color(2), alpha=0.16,
                    label=r"$3\sigma$ envelope")
    ax.plot(t, d["est_att_err_rad"] * DEG, color=series_color(2), linewidth=1.2,
            label="error angle")
    finish_axes(ax, "Attitude estimate error", "Time [s]", "Error [deg]", legend=True)

    annotate_source(fig, _provenance(run))
    return _save(fig, path)


# ---------------------------------------------------------------------------------------
# Linear analysis
# ---------------------------------------------------------------------------------------
def plot_eigenvalues(linear: dict, path: str | Path) -> Path:
    """Open-loop pole map of the linearised model, with the classical modes named."""
    modes = linear["modes"]
    fig, axes = plt.subplots(1, 3, figsize=(14.5, 5.4))

    def pole_panel(ax, frame, title, xlim=None):
        ax.axvline(0.0, color=PALETTE[7], linewidth=1.1, linestyle="--", alpha=0.8)
        ax.axhline(0.0, color=INK["muted"], linewidth=0.9)
        names = [n for n in frame["mode"].unique()]
        for i, name in enumerate(names):
            sub = frame[frame["mode"] == name]
            ax.scatter(sub["real"], sub["imag"], s=88, marker="x", linewidths=2.1,
                       color=series_color(i), label=name.replace("_", " "), zorder=4)
        if xlim:
            ax.set_xlim(*xlim)
        finish_axes(ax, title, r"Re$(\lambda)$ [1/s]", r"Im$(\lambda)$ [rad/s]", legend=True)

    pole_panel(axes[0], modes[modes["group"] == "longitudinal"],
               "Longitudinal modes")
    pole_panel(axes[1], modes[modes["group"] == "lateral"], "Lateral-directional modes")

    ax = axes[2]
    full = modes[modes["group"] == "full"]
    ax.axvline(0.0, color=PALETTE[7], linewidth=1.1, linestyle="--", alpha=0.8)
    ax.axhline(0.0, color=INK["muted"], linewidth=0.9)
    ax.scatter(full["real"], full["imag"], s=70, marker="x", linewidths=2.0,
               color=series_color(0), label="12-state model")
    finish_axes(ax, "Full 12-state error model", r"Re$(\lambda)$ [1/s]",
                r"Im$(\lambda)$ [rad/s]", legend=True)

    # Name the classical modes directly on the first two panels.
    for ax, group in ((axes[0], "longitudinal"), (axes[1], "lateral")):
        sub = modes[(modes["group"] == group) & (modes["imag"] >= 0)]
        for _, row in sub.iterrows():
            label = row["mode"].replace("_", " ")
            detail = (f"{label}\n$\\omega_n$={row['natural_frequency_rad_s']:.2f} rad/s, "
                      f"$\\zeta$={row['damping_ratio']:.2f}")
            ax.annotate(detail, (row["real"], row["imag"]), textcoords="offset points",
                        xytext=(8, 8), fontsize=7.5, color=INK["secondary"])

    ref = linear["reference"].set_index("quantity")["value"]
    fig.suptitle(
        f"Open-loop stability at trim: $V_a$ = "
        f"{np.hypot(ref.get('u', np.nan), ref.get('w', np.nan)):.1f} m/s, "
        f"h = {-ref.get('pos_d', np.nan):.0f} m",
        x=0.008, ha="left", fontsize=12, fontweight="bold")
    annotate_source(fig, "aether6 - aether_trim, numerically linearised 6-DOF model")
    return _save(fig, path)


def plot_trim_sweep(linear: dict, path: str | Path) -> Path:
    """Trim controls and modal characteristics across an airspeed sweep."""
    sweep = linear["trim_sweep"]
    ok = sweep[sweep["converged"] > 0.5]
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))

    ax = axes[0, 0]
    ax.plot(ok["airspeed_mps"], ok["alpha_deg"], color=series_color(0), marker="o",
            label=r"$\alpha$")
    ax.plot(ok["airspeed_mps"], ok["elevator_deg"], color=series_color(1), marker="s",
            label=r"$\delta_e$")
    finish_axes(ax, "Trim angles", "Airspeed [m/s]", "Angle [deg]", legend=True)

    ax = axes[0, 1]
    ax.plot(ok["airspeed_mps"], ok["throttle"], color=series_color(2), marker="o")
    finish_axes(ax, "Trim throttle", "Airspeed [m/s]", "Throttle [-]")

    ax = axes[1, 0]
    ax.plot(ok["airspeed_mps"], ok["short_period_wn"], color=series_color(0), marker="o",
            label="short period")
    ax.plot(ok["airspeed_mps"], ok["dutch_roll_wn"], color=series_color(1), marker="s",
            label="dutch roll")
    ax.plot(ok["airspeed_mps"], ok["phugoid_wn"], color=series_color(2), marker="^",
            label="phugoid")
    finish_axes(ax, "Modal natural frequency", "Airspeed [m/s]", r"$\omega_n$ [rad/s]",
                legend=True)

    ax = axes[1, 1]
    ax.plot(ok["airspeed_mps"], ok["short_period_zeta"], color=series_color(0), marker="o",
            label="short period")
    ax.plot(ok["airspeed_mps"], ok["dutch_roll_zeta"], color=series_color(1), marker="s",
            label="dutch roll")
    ax.plot(ok["airspeed_mps"], ok["phugoid_zeta"], color=series_color(2), marker="^",
            label="phugoid")
    ax.axhline(0.0, color=INK["muted"], linewidth=0.9)
    finish_axes(ax, "Modal damping ratio", "Airspeed [m/s]", r"$\zeta$ [-]", legend=True)

    annotate_source(fig, "aether6 - aether_trim --sweep")
    return _save(fig, path)


# ---------------------------------------------------------------------------------------
# Monte Carlo
# ---------------------------------------------------------------------------------------
def plot_monte_carlo(mc: dict, path: str | Path) -> Path:
    """Trajectory envelope, dispersion scatter and outcome summary for a campaign."""
    trials = mc["trials"]
    env = mc["envelope"]
    traj = mc["trajectories"]
    summary = mc["summary"]

    fig = plt.figure(figsize=(14.5, 10))
    gs = fig.add_gridspec(2, 3)

    # --- ground-track spaghetti ---------------------------------------------------------
    ax = fig.add_subplot(gs[0, 0])
    if traj is not None and len(traj):
        ok_ids = set(trials.loc[trials["ok"] > 0.5, "index"])
        for trial_id, group in traj.groupby("trial"):
            failed = trial_id not in ok_ids
            ax.plot(group["east_m"], group["north_m"],
                    color=PALETTE[7] if failed else series_color(0),
                    linewidth=1.5 if failed else 0.6,
                    alpha=0.9 if failed else 0.25, zorder=3 if failed else 2)
        ax.plot([], [], color=series_color(0), linewidth=1.2, label="successful trials")
        ax.plot([], [], color=PALETTE[7], linewidth=1.5, label="failed trials")
    ax.set_aspect("equal", adjustable="datalim")
    finish_axes(ax, "Dispersed ground tracks", "East [m]", "North [m]", legend=True)

    # --- cross-track envelope -----------------------------------------------------------
    ax = fig.add_subplot(gs[0, 1])
    if env is not None and len(env):
        ax.fill_between(env["time_s"], env["cross_track_min"], env["cross_track_max"],
                        color=series_color(0), alpha=0.12, label="min-max")
        ax.fill_between(env["time_s"], env["cross_track_p05"], env["cross_track_p95"],
                        color=series_color(0), alpha=0.28, label="5-95%")
        ax.plot(env["time_s"], env["cross_track_p50"], color=series_color(0), linewidth=1.4,
                label="median")
    ax.axhline(0.0, color=INK["muted"], linewidth=0.9)
    finish_axes(ax, "Cross-track error envelope", "Time [s]", "Cross-track [m]", legend=True)

    # --- altitude envelope --------------------------------------------------------------
    ax = fig.add_subplot(gs[0, 2])
    if env is not None and len(env):
        ax.fill_between(env["time_s"], env["altitude_p05"], env["altitude_p95"],
                        color=series_color(1), alpha=0.28, label="5-95%")
        ax.plot(env["time_s"], env["altitude_p50"], color=series_color(1), linewidth=1.4,
                label="median")
    finish_axes(ax, "Altitude envelope", "Time [s]", "Altitude [m]", legend=True)

    # --- metric histograms --------------------------------------------------------------
    ok = trials[trials["ok"] > 0.5]
    ax = fig.add_subplot(gs[1, 0])
    ax.hist(ok["rms_cross_track_m"], bins=26, color=series_color(0), alpha=0.85,
            edgecolor=INK["surface"], linewidth=0.8)
    ax.axvline(ok["rms_cross_track_m"].median(), color=INK["truth"], linewidth=1.2,
               linestyle="--")
    ax.annotate(f"median {ok['rms_cross_track_m'].median():.1f} m", (0.97, 0.9),
                xycoords="axes fraction", ha="right", color=INK["secondary"], fontsize=8.5)
    finish_axes(ax, "RMS cross-track error", "Error [m]", "Trials")

    ax = fig.add_subplot(gs[1, 1])
    ax.hist(ok["estimator_position_rmse_m"], bins=26, color=series_color(2), alpha=0.85,
            edgecolor=INK["surface"], linewidth=0.8)
    ax.axvline(ok["estimator_position_rmse_m"].median(), color=INK["truth"], linewidth=1.2,
               linestyle="--")
    ax.annotate(f"median {ok['estimator_position_rmse_m'].median():.2f} m", (0.97, 0.9),
                xycoords="axes fraction", ha="right", color=INK["secondary"], fontsize=8.5)
    finish_axes(ax, "Estimator position RMSE", "RMSE [m]", "Trials")

    # --- dispersion vs outcome ----------------------------------------------------------
    ax = fig.add_subplot(gs[1, 2])
    bad = trials[trials["ok"] < 0.5]
    ax.scatter(ok["mass_kg"], ok["cl_alpha"], s=16, color=series_color(0), alpha=0.6,
               label="completed")
    ax.scatter(bad["mass_kg"], bad["cl_alpha"], s=44, color=PALETTE[7], marker="X",
               label="failed", zorder=4)
    finish_axes(ax, "Outcome against the dominant dispersions", "Mass [kg]",
                r"$C_{L\alpha}$ [1/rad]", legend=True)

    title = (f"Monte-Carlo campaign: {summary.get('trials', len(trials))} trials, "
             f"{summary.get('successes', int((trials['ok'] > 0.5).sum()))} completed, "
             f"failure rate {summary.get('failure_rate', float('nan')):.1%}")
    fig.suptitle(title, x=0.008, ha="left", fontsize=12.5, fontweight="bold")
    annotate_source(
        fig,
        f"aether6 - aether_mc, master seed {summary.get('master_seed', '?')}, "
        f"{summary.get('duration_per_trial_s', '?')} s per trial")
    return _save(fig, path)


def plot_monte_carlo_statistics(mc: dict, path: str | Path) -> Path:
    """Box-style summary of the campaign statistics, one panel per metric family."""
    stats = pd.DataFrame(mc["summary"].get("statistics", []))
    trials = mc["trials"]
    if stats.empty:
        raise ValueError("Monte-Carlo summary contains no statistics")

    families = [
        ("Guidance tracking", ["rms_cross_track", "max_cross_track", "rms_altitude_error",
                               "max_altitude_error", "rms_airspeed_error"]),
        ("Estimator accuracy", ["estimator_position_rmse", "estimator_velocity_rmse",
                                "estimator_attitude_rmse", "estimator_yaw_rmse"]),
        ("Vehicle state", ["max_bank", "laps_completed"]),
    ]
    fig, axes = plt.subplots(1, len(families) + 1, figsize=(15, 5.2),
                             width_ratios=[1.35, 1.15, 0.8, 0.9])

    for ax, (title, names) in zip(axes, families):
        rows = stats[stats["name"].isin(names)]
        ypos = np.arange(len(rows))[::-1]
        for y, (_, row) in zip(ypos, rows.iterrows()):
            ax.plot([row["min"], row["max"]], [y, y], color=INK["grid"], linewidth=5,
                    solid_capstyle="round")
            ax.plot([row["p05"], row["p95"]], [y, y], color=series_color(0), linewidth=5,
                    solid_capstyle="round")
            ax.plot([row["median"]], [y], marker="|", markersize=14, markeredgewidth=2.2,
                    color=INK["truth"])
            ax.annotate(f"{row['median']:.2f} {row['unit']}", (row["max"], y),
                        textcoords="offset points", xytext=(6, 0), va="center",
                        fontsize=8, color=INK["secondary"])
        ax.set_yticks(ypos)
        ax.set_yticklabels([n.replace("_", " ") for n in rows["name"]], fontsize=8.5)
        ax.margins(x=0.28)
        finish_axes(ax, title, "value", None)

    ax = axes[-1]
    reasons = mc["summary"].get("failure_reasons", {})
    if reasons:
        names = list(reasons)
        counts = [reasons[n] for n in names]
        ypos = np.arange(len(names))[::-1]
        ax.barh(ypos, counts, color=PALETTE[7], height=0.55)
        for y, c in zip(ypos, counts):
            ax.annotate(str(c), (c, y), textcoords="offset points", xytext=(5, 0),
                        va="center", fontsize=8.5, color=INK["secondary"])
        ax.set_yticks(ypos)
        ax.set_yticklabels([n.replace("_", " ") for n in names], fontsize=8.5)
        ax.margins(x=0.25)
    else:
        ax.annotate("no failures", (0.5, 0.5), xycoords="axes fraction", ha="center",
                    color=INK["secondary"])
        ax.set_xticks([])
        ax.set_yticks([])
    finish_axes(ax, f"Failures ({len(trials[trials['ok'] < 0.5])} of {len(trials)})",
                "Trials", None)

    fig.suptitle("Monte-Carlo statistics (bars: 5-95%, grey: min-max, tick: median)",
                 x=0.006, ha="left", fontsize=12, fontweight="bold")
    annotate_source(fig, "aether6 - aether_mc summary.json")
    return _save(fig, path)


# ---------------------------------------------------------------------------------------
# Integrators
# ---------------------------------------------------------------------------------------
def plot_integrators(study: dict, path: str | Path) -> Path:
    """Convergence order, cost/accuracy trade-off and the aircraft-model comparison."""
    conv = study["convergence"]
    tol = study["tolerance"]
    air = study["aircraft"]

    fig, axes = plt.subplots(1, 3, figsize=(15, 5.2))

    ax = axes[0]
    ax.loglog(conv["dt_s"], conv["global_error"], marker="o", color=series_color(0),
              label="RK4 (measured)")
    ref = conv["global_error"].iloc[0] * (conv["dt_s"] / conv["dt_s"].iloc[0]) ** 4
    ax.loglog(conv["dt_s"], ref, color=INK["muted"], linestyle="--", linewidth=1.1,
              label=r"$\mathcal{O}(h^4)$ reference")
    orders = conv["observed_order"][conv["observed_order"] > 0]
    ax.annotate(f"observed order {orders.mean():.2f}", (0.04, 0.9), xycoords="axes fraction",
                color=INK["secondary"], fontsize=9)
    finish_axes(ax, "RK4 convergence (damped oscillator, analytic solution)",
                "Step size $h$ [s]", "Global error", legend=True)

    ax = axes[1]
    if tol is not None:
        ax.loglog(tol["rel_tol"], tol["global_error"], marker="o", color=series_color(1),
                  label="achieved error")
        ax.loglog(tol["rel_tol"], tol["rel_tol"], color=INK["muted"], linestyle="--",
                  linewidth=1.1, label="requested tolerance")
    finish_axes(ax, "Dormand-Prince 5(4) error control", "Requested rel. tolerance",
                "Achieved global error", legend=True)

    ax = axes[2]
    rk4 = air.iloc[:6]
    dp = air.iloc[6:]
    ax.loglog(rk4["function_evals"], rk4["state_error"], marker="o", color=series_color(0),
              label="RK4 (fixed step)")
    ax.loglog(dp["function_evals"], dp["state_error"], marker="s", color=series_color(1),
              label="Dormand-Prince 5(4)")
    for _, row in rk4.iterrows():
        ax.annotate(f"h={row['dt_or_rtol']:g}", (row["function_evals"], row["state_error"]),
                    textcoords="offset points", xytext=(5, 5), fontsize=7,
                    color=INK["secondary"])
    for _, row in dp.iterrows():
        ax.annotate(f"rtol={row['dt_or_rtol']:.0e}",
                    (row["function_evals"], row["state_error"]),
                    textcoords="offset points", xytext=(5, -10), fontsize=7,
                    color=INK["secondary"])
    finish_axes(ax, "Cost vs accuracy on the 6-DOF aircraft model",
                "Right-hand-side evaluations", "State error vs reference", legend=True)

    fig.suptitle("Integrator accuracy and cost", x=0.006, ha="left", fontsize=12.5,
                 fontweight="bold")
    annotate_source(fig, "aether6 - aether_integrators")
    return _save(fig, path)


# ---------------------------------------------------------------------------------------
# Controller comparison
# ---------------------------------------------------------------------------------------
def plot_controller_comparison(runs: dict, path: str | Path) -> Path:
    """Compare several runs of the same mission (e.g. LQR vs PID, calm vs windy)."""
    fig, axes = plt.subplots(2, 2, figsize=(13, 8.5))

    for i, (label, run) in enumerate(runs.items()):
        d, t = run.states, run.time
        colour = series_color(i)
        axes[0, 0].plot(d["pe_m"], d["pn_m"], color=colour, linewidth=1.2, label=label)
        axes[0, 1].plot(t, d["cross_track_m"], color=colour, linewidth=1.0, label=label)
        axes[1, 0].plot(t, d["altitude_m"] - d["cmd_altitude_m"], color=colour,
                        linewidth=1.0, label=label)
        axes[1, 1].plot(t, d["roll_rad"] * DEG, color=colour, linewidth=1.0, label=label)

    first = next(iter(runs.values()))
    wp = first.waypoints
    if len(wp):
        closed = pd.concat([wp, wp.iloc[[0]]], ignore_index=True)
        axes[0, 0].plot(closed["east_m"], closed["north_m"], color=INK["muted"],
                        linestyle="--", linewidth=1.0, label="commanded course")
    axes[0, 0].set_aspect("equal", adjustable="datalim")
    finish_axes(axes[0, 0], "Ground track", "East [m]", "North [m]", legend=True)
    axes[0, 1].axhline(0.0, color=INK["muted"], linewidth=0.9)
    finish_axes(axes[0, 1], "Cross-track error", "Time [s]", "Cross-track [m]", legend=True)
    axes[1, 0].axhline(0.0, color=INK["muted"], linewidth=0.9)
    finish_axes(axes[1, 0], "Altitude error", "Time [s]", "Error [m]", legend=True)
    axes[1, 1].axhline(0.0, color=INK["muted"], linewidth=0.9)
    finish_axes(axes[1, 1], "Bank angle", "Time [s]", "Roll [deg]", legend=True)

    rows = []
    for label, run in runs.items():
        m = run.summary.get("tracking", {})
        rows.append(f"{label}: cross-track RMS {m.get('rms_cross_track_m', float('nan')):.1f} m, "
                    f"altitude RMS {m.get('rms_altitude_error_m', float('nan')):.2f} m")
    fig.suptitle("Controller / condition comparison\n" + "\n".join(rows),
                 x=0.006, ha="left", fontsize=10.5, fontweight="bold")
    annotate_source(fig, "aether6 - aether_sim, identical mission and seed")
    return _save(fig, path)
