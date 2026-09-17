"""The four scenarios the dashboard offers as one-click starting points.

A visitor should be able to see what the project does without reading anything, so the
presets are ordered as an argument: perfect sensing works, real sensing still works, the
classical controller works less well, and strong turbulence is where the differences show.
"""

from __future__ import annotations

from .models import Controller, Feedback, SimulationRequest

PRESETS: list[dict] = [
    {
        "id": "ideal",
        "label": "Ideal",
        "tagline": "No wind, perfect sensing",
        "detail": "LQR flying on true states in still air — the reference the other cases "
                  "are measured against.",
        "request": SimulationRequest(controller=Controller.LQR, feedback=Feedback.TRUTH,
                                     wind_speed=0.0, target_altitude=120.0, airspeed=25.0,
                                     sensor_noise=1.0),
    },
    {
        "id": "nominal",
        "label": "Nominal",
        "tagline": "5 m/s wind, navigation filter in the loop",
        "detail": "The honest configuration: the controller sees only what the error-state "
                  "EKF estimates from noisy, multi-rate sensors.",
        "request": SimulationRequest(controller=Controller.LQR, feedback=Feedback.ESTIMATE,
                                     wind_speed=5.0, target_altitude=120.0, airspeed=25.0,
                                     sensor_noise=1.0),
    },
    {
        "id": "pid",
        "label": "Classical PID",
        "tagline": "Same mission, successive-loop-closure autopilot",
        "detail": "Cascaded PID loops in place of the LQR, on identical sensors and wind, "
                  "so the two designs can be compared directly.",
        "request": SimulationRequest(controller=Controller.PID, feedback=Feedback.ESTIMATE,
                                     wind_speed=5.0, target_altitude=120.0, airspeed=25.0,
                                     sensor_noise=1.0),
    },
    {
        "id": "gusty",
        "label": "Strong wind",
        "tagline": "12 m/s wind with matching Dryden turbulence",
        "detail": "Wind at roughly half the airspeed. The wind state in the filter and the "
                  "angle-of-attack protection in the controller both start to matter.",
        "request": SimulationRequest(controller=Controller.LQR, feedback=Feedback.ESTIMATE,
                                     wind_speed=12.0, target_altitude=140.0, airspeed=27.0,
                                     sensor_noise=1.0),
    },
]


def as_meta() -> list[dict]:
    """Preset list for /api/meta, with each request serialised to plain JSON."""
    return [{"id": p["id"], "label": p["label"], "tagline": p["tagline"], "detail": p["detail"],
             "request": p["request"].model_dump(mode="json")} for p in PRESETS]
