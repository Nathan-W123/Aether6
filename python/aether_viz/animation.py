"""Animated 3-D trajectory with aircraft attitude.

The aircraft is drawn as a small wireframe glyph whose body-axis vertices are rotated into
NED by the logged attitude quaternion, so the animation shows the *actual* attitude history
rather than a heading-only approximation.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.animation import FuncAnimation, PillowWriter

from .io import RunData, quaternion_to_rotation
from .style import INK, PALETTE, series_color

DEG = 180.0 / np.pi

#: Aircraft wireframe in body axes (forward-right-down), in metres. Each entry is one
#: polyline with the colour it is drawn in. The wing and tail carry the accent colour so the
#: bank angle stays legible against the fuselage even at small on-screen sizes.
_GLYPH = [
    (np.array([[1.30, 0.0, 0.0], [-1.30, 0.0, 0.0]]), "fuselage"),        # fuselage
    (np.array([[0.05, -1.30, 0.0], [0.35, 0.0, 0.0], [0.05, 1.30, 0.0]]), "wing"),
    (np.array([[-1.05, -0.50, 0.0], [-1.05, 0.50, 0.0]]), "wing"),        # tailplane
    (np.array([[-1.28, 0.0, 0.0], [-1.00, 0.0, -0.50]]), "fuselage"),     # fin
]


def _glyph_segments(rotation: np.ndarray, position: np.ndarray, scale: float):
    """Return the glyph polylines placed at ``position`` with attitude ``rotation``.

    ``position`` and the returned points are in plotting coordinates (east, north, altitude).
    """
    out = []
    for part, _ in _GLYPH:
        ned = (rotation @ (part * scale).T).T          # body -> NED
        enu = np.stack([ned[:, 1], ned[:, 0], -ned[:, 2]], axis=1)  # NED -> (E, N, up)
        out.append(enu + position)
    return out


def _shrink_gif(path: Path, colors: int = 64) -> None:
    """Re-quantise a written GIF to an adaptive palette and re-save it optimised.

    Matplotlib writes full-colour frames; an engineering animation only needs a few dozen
    distinct colours, and quantising typically cuts the file by an order of magnitude.
    """
    try:
        from PIL import Image, ImageSequence
    except ImportError:  # pragma: no cover - Pillow ships with Matplotlib
        return
    with Image.open(path) as source:
        frames = [f.copy().convert("RGB").convert(
            "P", palette=Image.ADAPTIVE, colors=colors)
            for f in ImageSequence.Iterator(source)]
        duration = source.info.get("duration", 40)
    if not frames:
        return
    frames[0].save(path, save_all=True, append_images=frames[1:], loop=0,
                   duration=duration, optimize=True, disposal=2)


def animate_run(run: RunData, path: str | Path, duration_s: float | None = None,
                fps: int = 16, speedup: float = 30.0, glyph_scale: float = 42.0,
                trail_s: float = 60.0, figsize=(5.9, 4.6), dpi: int = 70,
                colors: int = 32) -> Path:
    """Write an animated GIF of the flight with the aircraft attitude.

    Parameters
    ----------
    run:
        Loaded simulation output.
    path:
        Destination ``.gif``.
    duration_s:
        Portion of the flight to animate, in simulated seconds. ``None`` animates all of it.
    fps:
        Frames per second of the output.
    speedup:
        Playback speed relative to real time, so one frame covers ``speedup / fps`` seconds.
    glyph_scale:
        Multiplier on the aircraft wireframe, in metres per metre of the real airframe. The
        real 2.6 m span would be invisible at kilometre scale, so the glyph is exaggerated.
    trail_s:
        Length of the fading trail behind the aircraft, in simulated seconds.
    colors:
        Adaptive-palette size used to re-quantise the finished GIF; 0 skips the step.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    d = run.states
    t = run.time
    end = t[-1] if duration_s is None else min(t[-1], float(duration_s))
    frame_dt = speedup / fps
    frame_times = np.arange(t[0], end, frame_dt)

    def at(col):
        return np.interp(frame_times, t, d[col].to_numpy())

    east, north, alt = at("pe_m"), at("pn_m"), at("altitude_m")
    quat = np.stack([at("qw"), at("qx"), at("qy"), at("qz")], axis=1)
    rotations = quaternion_to_rotation(quat)
    airspeed, roll = at("airspeed_mps"), at("roll_rad")
    altitude_cmd = at("cmd_altitude_m")

    # The animation manages its own margins, so the figure is created without the shared
    # constrained-layout engine (which would refuse the explicit subplots_adjust below).
    with mpl.rc_context({"figure.constrained_layout.use": False}):
        fig = plt.figure(figsize=figsize, dpi=dpi)
    ax = fig.add_subplot(1, 1, 1, projection="3d")

    pad = 80.0
    ax.set_xlim(d["pe_m"].min() - pad, d["pe_m"].max() + pad)
    ax.set_ylim(d["pn_m"].min() - pad, d["pn_m"].max() + pad)
    floor = max(0.0, d["altitude_m"].min() - 30.0)
    ax.set_zlim(floor, d["altitude_m"].max() + 25.0)
    ax.set_xlabel("East [m]", labelpad=-2)
    ax.set_ylabel("North [m]", labelpad=-2)
    ax.set_zlabel("Alt [m]", labelpad=-4)
    ax.tick_params(labelsize=7, pad=-1)
    ax.view_init(elev=28, azim=-128)
    ax.grid(True, alpha=0.25)

    wp = run.waypoints
    if len(wp):
        closed = pd.concat([wp, wp.iloc[[0]]], ignore_index=True)
        ax.plot(closed["east_m"], closed["north_m"], closed["altitude_m"],
                color=PALETTE[1], linewidth=1.0, linestyle="--", alpha=0.8)
        ax.scatter(wp["east_m"], wp["north_m"], wp["altitude_m"], color=PALETTE[1], s=22,
                   depthshade=False)

    full_path, = ax.plot(d["pe_m"], d["pn_m"], d["altitude_m"], color=INK["grid"],
                         linewidth=0.7, alpha=0.9)
    trail, = ax.plot([], [], [], color=series_color(0), linewidth=1.8)
    shadow, = ax.plot([], [], [], color=INK["muted"], linewidth=0.8, alpha=0.5)
    glyph = [ax.plot([], [], [], linewidth=2.4 if role == "wing" else 2.0,
                     color=PALETTE[1] if role == "wing" else INK["truth"],
                     solid_capstyle="round")[0] for _, role in _GLYPH]
    nose, = ax.plot([], [], [], marker="o", markersize=3.6, color=PALETTE[7],
                    linestyle="none")
    readout = ax.text2D(0.015, 0.95, "", transform=ax.transAxes, fontsize=8.5,
                        color=INK["secondary"], va="top", family="monospace")
    ax.text2D(0.015, 1.005, f"Aether-6 - {run.name}", transform=ax.transAxes,
              fontsize=10.5, color=INK["primary"], fontweight="bold")
    fig.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=0.94)

    trail_frames = max(2, int(trail_s / frame_dt))

    def update(k: int):
        lo = max(0, k - trail_frames)
        trail.set_data(east[lo:k + 1], north[lo:k + 1])
        trail.set_3d_properties(alt[lo:k + 1])
        shadow.set_data(east[lo:k + 1], north[lo:k + 1])
        shadow.set_3d_properties(np.full(k + 1 - lo, floor))
        position = np.array([east[k], north[k], alt[k]])
        segments = _glyph_segments(rotations[k], position, glyph_scale)
        for line, seg in zip(glyph, segments):
            line.set_data(seg[:, 0], seg[:, 1])
            line.set_3d_properties(seg[:, 2])
        nose.set_data([segments[0][0, 0]], [segments[0][0, 1]])
        nose.set_3d_properties([segments[0][0, 2]])
        readout.set_text(
            f"t   {frame_times[k]:6.1f} s\n"
            f"alt {alt[k]:6.1f} m  (cmd {altitude_cmd[k]:5.1f})\n"
            f"Va  {airspeed[k]:6.1f} m/s\n"
            f"roll{roll[k] * DEG:6.1f} deg")
        return [trail, shadow, readout, nose, *glyph]

    anim = FuncAnimation(fig, update, frames=len(frame_times), interval=1000 / fps,
                         blit=False)
    anim.save(path, writer=PillowWriter(fps=fps))
    plt.close(fig)
    if colors:
        _shrink_gif(path, colors)
    return path
