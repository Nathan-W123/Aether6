"""Sandboxed execution of the compiled simulator.

This module is the only place in the service that starts a process, and it is written so a
visitor can influence *nothing* about how that process is started:

* The executable path comes from :mod:`app.settings` (operator configuration), never a request.
* The argument vector is a fixed list of literals plus the path of a file this service wrote
  into a directory it created. ``asyncio.create_subprocess_exec`` is used, never a shell, so
  there is no string for user input to escape out of — and there is no user string anyway,
  because every request field is a bounded number or an enum.
* Each run gets a fresh ``mkdtemp`` directory, so concurrent runs cannot see or clobber each
  other's output, and the directory is removed in a ``finally`` block once the results have
  been parsed into memory.
* A semaphore bounds how many simulators may run at once and a queue timeout refuses the
  request rather than letting arrivals pile up; every process additionally runs under a
  wall-clock timeout and is killed by process group if it overruns.
"""

from __future__ import annotations

import asyncio
import contextlib
import os
import shutil
import signal
import tempfile
import time
from pathlib import Path

from .cache import LruCache
from .models import (MonteCarloRequest, MonteCarloResponse, SimulationRequest,
                     SimulationResponse)
from .results import read_montecarlo, read_states, read_summary, read_waypoints
from .scenario import assert_plain_data, build_montecarlo_scenario, build_scenario, write_scenario
from .settings import SETTINGS, Settings

#: Exit codes the simulator uses. 0 is a clean run; 1 means a safety limit tripped (a stall,
#: ground contact) which is a *legitimate* outcome with valid logs to show; 2 and above are
#: real errors (bad configuration, an exception) and become a 500.
_EXIT_OK = 0
_EXIT_UNSTABLE = 1


class RunError(RuntimeError):
    """A run that could not be served, carrying the HTTP status the API should return."""

    def __init__(self, status_code: int, detail: str) -> None:
        super().__init__(detail)
        self.status_code = status_code
        self.detail = detail


class SimulationRunner:
    """Owns the concurrency budget, the result caches and the subprocess plumbing."""

    def __init__(self, settings: Settings = SETTINGS) -> None:
        self.settings = settings
        self._semaphore = asyncio.Semaphore(settings.max_concurrency)
        self._active = 0
        self._sim_cache = LruCache(settings.cache_size)
        self._mc_cache = LruCache(max(0, settings.cache_size // 4))

    # -- introspection -----------------------------------------------------------------
    @property
    def active_runs(self) -> int:
        return self._active

    def simulator_available(self) -> bool:
        return os.access(self.settings.sim_binary, os.X_OK) and os.access(
            self.settings.mc_binary, os.X_OK)

    def cache_stats(self) -> dict:
        return {"simulate": self._sim_cache.stats(), "montecarlo": self._mc_cache.stats()}

    # -- process execution -------------------------------------------------------------
    async def _execute(self, binary: Path, args: list[str], workdir: Path,
                       timeout_s: float) -> tuple[int, str]:
        """Run ``binary`` with a fixed argument vector and return (exit code, stderr tail).

        ``args`` must contain only literals chosen by this module and paths this module
        created; it is passed as a list, so no shell is involved at any point.
        """
        if not os.access(binary, os.X_OK):
            raise RunError(503, "simulator binary is not available on this server")

        try:
            process = await asyncio.create_subprocess_exec(
                str(binary), *args,
                cwd=str(workdir),
                stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.PIPE,
                # Own process group, so a timeout can reap the worker threads with it.
                start_new_session=True,
                env={"PATH": "/usr/bin:/bin", "LC_ALL": "C", "HOME": str(workdir)},
            )
        except OSError as exc:  # pragma: no cover - only on a broken deployment
            raise RunError(503, f"could not start the simulator: {exc}") from exc

        try:
            _, stderr = await asyncio.wait_for(process.communicate(), timeout=timeout_s)
        except asyncio.TimeoutError:
            await self._terminate(process)
            raise RunError(504, f"the simulation exceeded its {timeout_s:g} s time limit")

        tail = (stderr or b"").decode("utf-8", "replace").strip()[-400:]
        return process.returncode or 0, tail

    @staticmethod
    async def _terminate(process: asyncio.subprocess.Process) -> None:
        """Kill an overrunning process and its group, then reap it."""
        for sig in (signal.SIGTERM, signal.SIGKILL):
            if process.returncode is not None:
                break
            with contextlib.suppress(ProcessLookupError, PermissionError):
                os.killpg(os.getpgid(process.pid), sig)
            with contextlib.suppress(asyncio.TimeoutError):
                await asyncio.wait_for(process.wait(), timeout=2.0)
        with contextlib.suppress(Exception):
            await process.wait()

    @contextlib.asynccontextmanager
    async def _slot(self):
        """Acquire one of the concurrency slots, or refuse the request."""
        try:
            await asyncio.wait_for(self._semaphore.acquire(),
                                   timeout=self.settings.queue_timeout_s)
        except asyncio.TimeoutError:
            raise RunError(503, "the simulation server is busy; please try again in a moment")
        self._active += 1
        try:
            yield
        finally:
            self._active -= 1
            self._semaphore.release()

    @contextlib.contextmanager
    def _workspace(self, prefix: str):
        """A unique scratch directory for one run, removed however the run ends."""
        path = Path(tempfile.mkdtemp(prefix=f"aether-{prefix}-"))
        try:
            yield path
        finally:
            shutil.rmtree(path, ignore_errors=True)

    # -- public entry points -----------------------------------------------------------
    async def simulate(self, request: SimulationRequest) -> SimulationResponse:
        key = request.cache_key()
        cached = self._sim_cache.get(key)
        if cached is not None:
            return cached.model_copy(update={"cached": True})

        started = time.monotonic()
        async with self._slot():
            # Re-check: an identical request may have completed while this one queued.
            cached = self._sim_cache.get(key)
            if cached is not None:
                return cached.model_copy(update={"cached": True})

            with self._workspace("sim") as workdir:
                scenario = build_scenario(request, self.settings, workdir)
                assert_plain_data(scenario)
                scenario_path = workdir / "scenario.yaml"
                write_scenario(scenario, scenario_path)

                code, stderr = await self._execute(
                    self.settings.sim_binary,
                    [str(scenario_path), "--quiet"],
                    workdir, self.settings.simulate_timeout_s)
                if code not in (_EXIT_OK, _EXIT_UNSTABLE):
                    raise RunError(500, _message(stderr, "the simulation failed"))

                summary_path = workdir / "summary.json"
                if not summary_path.is_file():
                    raise RunError(500, _message(stderr, "the simulation produced no results"))
                trim, metrics = read_summary(summary_path)
                series = read_states(workdir / "states.csv", self.settings.series_points)
                waypoints = read_waypoints(workdir / "waypoints.csv")

        response = SimulationResponse(
            request=request,
            cached=False,
            server_runtime_s=round(time.monotonic() - started, 3),
            trim=trim,
            metrics=metrics,
            waypoints=waypoints,
            series=series,
        )
        self._sim_cache.put(key, response)
        return response

    async def montecarlo(self, request: MonteCarloRequest) -> MonteCarloResponse:
        key = request.cache_key()
        cached = self._mc_cache.get(key)
        if cached is not None:
            return cached.model_copy(update={"cached": True})

        started = time.monotonic()
        async with self._slot():
            cached = self._mc_cache.get(key)
            if cached is not None:
                return cached.model_copy(update={"cached": True})

            with self._workspace("mc") as workdir:
                scenario = build_montecarlo_scenario(request, self.settings, workdir)
                assert_plain_data(scenario)
                scenario_path = workdir / "scenario.yaml"
                write_scenario(scenario, scenario_path)

                code, stderr = await self._execute(
                    self.settings.mc_binary,
                    [str(scenario_path)],
                    workdir, self.settings.montecarlo_timeout_s)
                if code != _EXIT_OK:
                    raise RunError(500, _message(stderr, "the Monte-Carlo campaign failed"))

                if not (workdir / "summary.json").is_file():
                    raise RunError(500, _message(stderr, "the campaign produced no results"))
                payload = read_montecarlo(workdir)

        response = MonteCarloResponse(
            request=request,
            cached=False,
            server_runtime_s=round(time.monotonic() - started, 3),
            **payload,
        )
        self._mc_cache.put(key, response)
        return response


def _message(stderr: str, fallback: str) -> str:
    """Surface the simulator's own diagnostic when it produced one, else a generic message.

    The simulator only ever prints its own text — it never echoes a request field — so this
    cannot leak user input back to another visitor through the cache.
    """
    line = stderr.strip().splitlines()[-1] if stderr.strip() else ""
    return f"{fallback}: {line}" if line else fallback
