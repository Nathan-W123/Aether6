"""Runtime settings for the Aether-6 dashboard service.

Everything here is fixed at process start from the environment. **No value in this module
may come from a request** — the simulator binary path, the repository paths, the timeouts and
the concurrency limits are operator configuration, not user input.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path


def _env_int(name: str, default: int, lo: int, hi: int) -> int:
    try:
        value = int(os.environ.get(name, default))
    except ValueError:
        return default
    return max(lo, min(hi, value))


def _env_float(name: str, default: float, lo: float, hi: float) -> float:
    try:
        value = float(os.environ.get(name, default))
    except ValueError:
        return default
    return max(lo, min(hi, value))


def _default_repo_root() -> Path:
    """Repository root, overridable with AETHER_ROOT (set to /app inside the container)."""
    env = os.environ.get("AETHER_ROOT")
    if env:
        return Path(env).resolve()
    # web/backend/app/settings.py -> repository root is three levels up.
    return Path(__file__).resolve().parents[3]


@dataclass(frozen=True)
class Settings:
    """Immutable service configuration."""

    repo_root: Path = field(default_factory=_default_repo_root)

    #: Directory holding the compiled simulator binaries.
    bin_dir: Path = field(default_factory=lambda: Path(
        os.environ.get("AETHER_BIN_DIR", str(_default_repo_root() / "build" / "bin"))).resolve())

    #: Directory holding the committed reference results used for the precomputed panels.
    results_dir: Path = field(default_factory=lambda: Path(
        os.environ.get("AETHER_RESULTS_DIR", str(_default_repo_root() / "results"))).resolve())

    #: Directory holding the airframe YAML the generated scenarios point at.
    aircraft_file: Path = field(default_factory=lambda: Path(
        os.environ.get("AETHER_AIRCRAFT",
                       str(_default_repo_root() / "configs" / "aircraft" / "aether6_uav.yaml"))
    ).resolve())

    #: Built frontend, served as the site root when present.
    static_dir: Path = field(default_factory=lambda: Path(
        os.environ.get("AETHER_STATIC_DIR", str(_default_repo_root() / "web" / "frontend" / "dist"))
    ).resolve())

    #: Simulated duration of an interactive run [s]. Not user-controllable: it bounds the cost.
    #: 200 s is one full circuit plus margin even at the slowest allowed airspeed.
    duration_s: float = field(default_factory=lambda: _env_float("AETHER_DURATION_S", 200.0, 30.0, 400.0))

    #: Integration step for interactive runs [s]. 250 Hz keeps a run near 1.7 s wall clock
    #: while staying ~1e-7 m accurate over the run (see docs/benchmarks.md).
    dt_s: float = field(default_factory=lambda: _env_float("AETHER_DT_S", 0.004, 0.001, 0.02))

    #: Log decimation, so the CSV the service parses stays small (10 Hz at dt = 0.004 s).
    log_decimation: int = field(default_factory=lambda: _env_int("AETHER_LOG_DECIMATION", 25, 1, 500))

    #: Samples returned to the browser per run. The series are resampled to this length.
    series_points: int = field(default_factory=lambda: _env_int("AETHER_SERIES_POINTS", 600, 100, 2000))

    #: Wall-clock ceiling for one simulator invocation [s].
    simulate_timeout_s: float = field(default_factory=lambda: _env_float("AETHER_SIM_TIMEOUT_S", 45.0, 5.0, 300.0))

    #: Wall-clock ceiling for one interactive Monte-Carlo invocation [s].
    montecarlo_timeout_s: float = field(default_factory=lambda: _env_float("AETHER_MC_TIMEOUT_S", 120.0, 10.0, 600.0))

    #: Simulator processes allowed to run at once.
    max_concurrency: int = field(default_factory=lambda: _env_int("AETHER_MAX_CONCURRENCY", 2, 1, 16))

    #: How long a request waits for a concurrency slot before being refused with 503 [s].
    queue_timeout_s: float = field(default_factory=lambda: _env_float("AETHER_QUEUE_TIMEOUT_S", 20.0, 1.0, 120.0))

    #: Entries kept in the in-memory result cache.
    cache_size: int = field(default_factory=lambda: _env_int("AETHER_CACHE_SIZE", 64, 0, 512))

    #: Bounds on an interactive Monte-Carlo campaign. The full 256-trial campaign is served
    #: from precomputed results; a visitor may only launch a small live demo.
    mc_min_trials: int = 8
    mc_max_trials: int = field(default_factory=lambda: _env_int("AETHER_MC_MAX_TRIALS", 16, 1, 64))
    mc_duration_s: float = field(default_factory=lambda: _env_float("AETHER_MC_DURATION_S", 120.0, 30.0, 300.0))
    mc_threads: int = field(default_factory=lambda: _env_int("AETHER_MC_THREADS", 2, 1, 16))

    @property
    def sim_binary(self) -> Path:
        return self.bin_dir / "aether_sim"

    @property
    def mc_binary(self) -> Path:
        return self.bin_dir / "aether_mc"

    def describe(self) -> dict:
        """Operator-facing snapshot, surfaced by /health."""
        return {
            "duration_s": self.duration_s,
            "dt_s": self.dt_s,
            "series_points": self.series_points,
            "max_concurrency": self.max_concurrency,
            "simulate_timeout_s": self.simulate_timeout_s,
            "montecarlo_timeout_s": self.montecarlo_timeout_s,
            "mc_trial_range": [self.mc_min_trials, self.mc_max_trials],
            "cache_size": self.cache_size,
        }


SETTINGS = Settings()
