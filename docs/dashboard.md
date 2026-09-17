# Web dashboard

A single-page dashboard that runs the real simulator on demand. It exists so the project can
be understood in about thirty seconds by someone who will not clone it: pick a scenario,
watch the aircraft fly the circuit, and see the tracking, control, estimation and robustness
results that follow from it.

The page shows *computed* results, not stored screenshots. Every trajectory, chart and number
under "Trajectory" through "LQR against classical PID" comes from a `aether_sim` process
started for that request. Only the 256-trial Monte-Carlo campaign and the modal analysis are
precomputed, and both are labelled as such on the page.

```
 React + Vite (TypeScript)          FastAPI (Python 3.11)          C++ binaries
 ────────────────────────           ─────────────────────          ────────────
  sliders, presets       ── POST ▶   validate (Pydantic)
  3D trajectory                      generate scenario YAML  ──▶   aether_sim
  SVG charts                         run in a temp directory       aether_mc
  Monte-Carlo panels     ◀ JSON ──   parse CSV/JSON, delete   ◀──  states.csv
                                     cache the result              summary.json
```

## Contents

- [Running it](#running-it)
- [What the controls do](#what-the-controls-do)
- [API](#api)
- [Security model](#security-model)
- [Container and deployment](#container-and-deployment)
- [Design decisions](#design-decisions)

## Running it

The service executes the compiled binaries, so build them first.

```bash
make build                 # the C++ simulator
make dashboard             # builds the frontend, then serves everything on :8000
```

Then open <http://localhost:8000>.

For frontend work, run the two processes separately; Vite proxies `/api` and `/health` to the
backend, so hot reload works against the real simulator:

```bash
make dashboard-dev         # uvicorn --reload on :8000, Vite on :5173
```

Tests:

```bash
make dashboard-test        # the FastAPI service's suite (49 cases)
cd web/frontend && npm run build   # `tsc -b` type-checks, then Vite builds
```

### Layout

```
web/
├── backend/
│   ├── app/
│   │   ├── main.py         FastAPI app, routes, static mount
│   │   ├── models.py       request and response models; the bounds live here
│   │   ├── settings.py     operator configuration from the environment
│   │   ├── scenario.py     validated request -> scenario YAML
│   │   ├── runner.py       sandboxed subprocess execution
│   │   ├── results.py      CSV/JSON -> browser payload (standard library only)
│   │   ├── precomputed.py  the committed campaign and modal analysis
│   │   ├── presets.py      the four one-click scenarios
│   │   └── cache.py        bounded LRU result cache
│   ├── tests/test_api.py
│   └── requirements.txt
└── frontend/
    ├── src/
    │   ├── api/            typed client and response types
    │   ├── components/     panels, 3D view, SVG chart primitives
    │   ├── lib/            palette, formatting, measurement hook
    │   ├── App.tsx         layout and run orchestration
    │   └── theme.css       design tokens
    └── package.json
```

## What the controls do

The scenario files carry roughly ninety values. The dashboard exposes six, chosen because
each one changes the answer visibly and none of them can produce a run that is merely broken.

| Control | Range | What it changes |
| --- | --- | --- |
| Wind speed | 0–15 m/s | The steady wind magnitude at the reference altitude *and* the Dryden turbulence intensity, which scales with it at 1.4 m/s of `w20` per m/s of wind. The bearing is fixed. |
| Target altitude | 60–250 m | Shifts the whole altitude profile of the circuit; the per-leg offsets (+30, −10, +10 m) ride on top. |
| Commanded airspeed | 22–32 m/s | The airspeed the mission is flown at; the per-leg offsets scale with it. |
| Sensor noise | 0.25–4× | Multiplies every sensor noise and bias sigma, and the filter's assumed measurement noise with it — a real vehicle tunes its filter for the sensors it carries. |
| Control law | LQR / PID | Servo-LQR with integral augmentation, or cascaded PID by successive loop closure. |
| Controller sees | EKF / truth | Whether the control law is closed on the navigation filter's estimate or on true states. |

The lower airspeed bound is empirical, not arbitrary: the vehicle stalls in the turns below
about 21 m/s, so the slider floor is 22 m/s.

Fixed by the service, not by the visitor: the duration (200 s), the integrator (RK4 at
250 Hz), the control rate (100 Hz), the airframe, the waypoint geometry and the wind bearing.
Those bound the cost of a request.

### The reachable envelope

Sweeping the corners and midpoints of all four sliders — 108 combinations, LQR on estimated
states, default seed — 96 complete the mission and 12 do not:

| Wind | Airspeed | Outcome |
| --- | --- | --- |
| 0–8 m/s | any | all complete |
| 10 m/s | 22 m/s | ground contact on 3 of 6 altitude/noise combinations |
| 15 m/s | 22–25 m/s | 8 of 18 end in ground contact or an overspeed dive |
| 15 m/s | ≥ 27 m/s | all complete |

That is not a defect to tune away. A 13.5 kg aircraft at 22 m/s in a 15 m/s wind with
severe turbulence has a margin of about 7 m/s over the wind, and the dashboard should let a
visitor find that. What matters is that the page says so: a truncated run raises a banner
naming the termination reason in plain language, and the summary tiles show `—` rather than
`0.00` when the run ended before the simulator's settling window opened, because at that
point the RMS figures are zero for want of samples rather than because the errors were zero.
`summary.json` reports `metrics_window_s` for exactly this reason.

## API

| Method | Path | Purpose |
| --- | --- | --- |
| `GET` | `/health` | Liveness and readiness. `status` is `ok` only when both binaries are executable. |
| `GET` | `/api/meta` | Presets, slider bounds, series units and the service's own limits. |
| `GET` | `/api/precomputed/montecarlo` | The committed 256-trial campaign. |
| `GET` | `/api/precomputed/modes` | The committed modal analysis and trim reference point. |
| `POST` | `/api/simulate` | One closed-loop run: trim, metrics, waypoints and 68 resampled series. |
| `POST` | `/api/montecarlo` | A live 8–16 trial dispersion campaign. |

Interactive documentation is at `/api/docs`, generated from the same models that validate the
requests.

A `/api/simulate` response is about 300 kB of JSON, or 115 kB over the wire with the gzip
middleware. The 200 s run is logged at 10 Hz by the simulator and resampled to 600 points per
series here, which is the resolution the charts can actually draw.

Two exit codes count as success. `0` is a clean run; `1` means the safety monitor stopped the
run — a stall, a ground contact — which is a real outcome with valid logs worth showing, and
the dashboard reports it as the termination reason. `2` and above are configuration or
runtime errors and become a 500.

## Security model

The service runs arbitrary visitors' requests against a native binary, so the boundary is
drawn tightly.

**Only validated form inputs.** Every field is a bounded number or an enum, declared in
`models.py` with `extra="forbid"`, per-field `strict=True` and `allow_inf_nan=False`. There is
no field that accepts a path, a filename, a YAML fragment or free text. The frontend fetches
the same bounds from `/api/meta`, so a slider cannot offer a value the backend would reject.
Validation failures return 422 naming the field and the reason, without echoing the payload
back.

**No user text reaches a shell.** The scenario is built as a Python dict of numbers and fixed
strings and serialised with `yaml.safe_dump`; `assert_plain_data()` refuses anything that is
not a plain scalar before it is written. The binary is started with
`asyncio.create_subprocess_exec` — never a shell — with an argument vector of literals plus
the path of a file the service just wrote. The child gets a minimal environment.

**A unique temporary directory per run.** `tempfile.mkdtemp()` per request, removed with
`shutil.rmtree` in a `finally` block once the results are parsed into memory. Concurrent runs
cannot see each other's output, and nothing survives the response.

**Execution timeouts.** 45 s per simulation and 120 s per campaign, against a measured 1.8 s
and 4.4 s. An overrun is killed by process group (SIGTERM, then SIGKILL) and returns 504.

**Concurrency limits.** An `asyncio.Semaphore` allows two simulators at once; a request that
waits more than 20 s for a slot is refused with 503 rather than queued indefinitely.

**Monte Carlo is capped.** A visitor may run 8–16 trials of 120 s. The published 256-trial
campaign is served from committed files and can never be launched from the browser.

**Identical requests are cached.** The key is a SHA-256 of the canonical JSON of the
*validated* request. Because the scenario is a pure function of those bounded fields, an equal
key really is an equal simulation. The cache holds 64 runs and 16 campaigns; the cache is
re-checked after acquiring a concurrency slot, so requests that queue behind an identical one
return without running anything.

The container runs as an unprivileged user. `/tmp` must stay writable, which is where the
per-run directories live.

## Container and deployment

`Dockerfile` builds three stages:

1. `debian:bookworm-slim` compiles the C++ binaries against the system Eigen and yaml-cpp,
   with the test suite switched off.
2. `node:22-bookworm-slim` installs from `package-lock.json` with `npm ci` and builds the
   frontend.
3. `python:3.11-slim-bookworm` installs the pinned Python requirements and copies in the
   binaries, the built bundle, the service, the airframe configuration and the precomputed
   results.

Stages 1 and 3 share a Debian release so the binaries find the glibc, libstdc++ and yaml-cpp
they were linked against. The runtime installs `libyaml-cpp-dev` rather than a versioned
runtime package, so the image does not hard-code a soname.

The image is configured entirely through environment variables (`AETHER_ROOT`,
`AETHER_BIN_DIR`, `AETHER_RESULTS_DIR`, `AETHER_AIRCRAFT`, `AETHER_STATIC_DIR`, and the
tuning values in `settings.py`). None of them can be set by a request.

The service binds `0.0.0.0` on `$PORT`, which Railway injects at start, and `railway.json`
points the platform's health check at `/health` — the same endpoint the Docker `HEALTHCHECK`
polls.

```bash
docker build -t aether6 .
docker run --rm -p 8000:8000 aether6
```

On Railway, a service pointed at this repository picks up `railway.json` and needs no further
configuration. Raise `AETHER_MAX_CONCURRENCY` above 2 only in step with the plan's CPU
allocation: a run is single-threaded and CPU-bound, so concurrency beyond the core count just
makes every visitor wait longer.

## Design decisions

**The frontend has three dependencies.** React, ReactDOM and Three.js. The charts are drawn
by about 590 lines of SVG primitives in `src/components/charts/`, because a charting library
would have brought its own colour handling and its own opinions about dual axes, and the
alternative was to fight it on both. The 3D view uses Three.js directly rather than
`@react-three/fiber`, whose peer-dependency tree pulls in Expo. The result is a 90 kB gzipped
application chunk plus a 132 kB Three.js chunk that changes between deploys only when
Three.js does.

**One y-axis per chart panel.** Quantities in different units get different panels. A second
axis on the right is the fastest way to imply a relationship between two series that has no
physical meaning.

**A fixed, colour-vision-safe series order.** Eight hues, checked for separation under
deuteranopia, protanopia and tritanopia against the dark surface, used in the same order
everywhere so a colour means the same thing across panels. The order is never cycled: a panel
that would need a ninth series is split instead.

**The estimator's axes carry the filter's own uncertainty.** The position, attitude and gyro
bias error panels shade ±1σ from the covariance diagonal. A filter whose errors sit outside
its own band is mistuned, and that is visible at a glance rather than buried in a metric.

**The page is never empty.** The nominal preset runs as soon as `/api/meta` returns, so the
first thing a visitor sees is a flown mission rather than a form.

**The 3D attitude is the simulated attitude.** NED maps to the viewer's frame as
`x = east, y = altitude, z = −north`, a proper rotation with determinant +1, and the glyph's
basis is built from the body forward, up and right axes rotated by the logged quaternion.
Nothing is mirrored or re-derived from Euler angles. The glyph is drawn at a fixed fraction
of the framed extent — the real vehicle is about 2 m across a 1 km circuit and would be a
single pixel.

**Parsing uses the standard library.** `results.py` uses `csv`, `json` and `math` rather than
pandas, which keeps roughly 100 MB out of the runtime image for a task that is a decimated
CSV read. Parsing a 2000-row log takes a few tens of milliseconds against the 1.8 s the
simulation itself costs.

## Known behaviour worth reading the charts for

The controller comparison shows something the static figures in `docs/benchmarks.md` do not:
which control law wins depends on how much wind there is.

Cross-track RMS [m] at 25 m/s airspeed, 120 m, LQR against PID, over two turbulence seeds:

| Wind | LQR | PID | Altitude RMS, LQR / PID |
| --- | --- | --- | --- |
| 0 m/s | 10.1, 11.4 | 11.7, 12.6 | 0.6–1.0 / 1.8–2.1 |
| 5 m/s | 10.4, 12.3 | 13.1, 14.6 | 0.7–1.1 / 1.9–2.1 |
| 8 m/s | 17.4, 20.1 | 18.5, 23.0 | 0.8–1.1 / 2.1–2.3 |
| 11 m/s | 14.5, 36.4 | departed, 19.9 | — |
| 14 m/s | departed, 37.9 | departed, 19.8 | — |

Up to about 8 m/s the LQR tracks the path better and the altitude two to three times better.
Beyond that, among the runs that stay airborne, its cross-track error grows faster than the
PID's while its altitude tracking stays better.

The likely mechanism is in the design model rather than the implementation: the LQR's course
kinematics use the coordinated-turn approximation `χ̇ ≈ (g/Va₀)·δφ` linearised about the
*trim* airspeed (`src/control/LqrAutopilot.cpp`), which stops being a good estimate of course
rate once the wind is a large fraction of the airspeed and the crab angle is large. The PID's
outer loop closes on measured course with integral action and makes no such approximation.
Confirming that would mean re-deriving the lateral design model about ground speed and
re-running the campaign; it is recorded here and in [limitations.md](limitations.md) rather
than tuned away, because the comparison panel is the thing that surfaces it.
