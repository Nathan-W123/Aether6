# Benchmark results

Every number below was measured by running the code in this repository. Reproduce all of it
with `./scripts/run_all.sh` (about 3 min 40 s on the machine described here).

**Measurement environment**

| | |
|---|---|
| CPU | Intel Xeon @ 2.80 GHz, 4 vCPU (container) |
| Compiler | GCC 13.3.0, `-O3` via `CMAKE_BUILD_TYPE=Release` |
| CMake / Eigen / yaml-cpp / Catch2 | 3.28.3 / 3.4.0 / 0.8.0 / 3.4.0 |
| Python | 3.11.15 with NumPy 2.4, pandas 3.0, Matplotlib 3.11 |

---

## 1. Runtime

| Stage | Wall clock | Notes |
|---|---|---|
| Configure + build (4 jobs, cold) | ~35 s | 21 translation units + 10 test files |
| Unit-test suite | **26 s** | 100 cases, 33 898 assertions (same through CTest) |
| Trim + linearisation + 19-point airspeed sweep | 0.3 s | |
| One 300 s closed-loop mission | **4.4 s** | 150 000 frames, RK4 at `dt = 0.002 s` |
| Integrator study | 1.0 s | |
| Monte-Carlo campaign, 256 × 300 s | **146.6 s** | 4 threads, 2.27 s mean per trial |
| All 16 figures (incl. the 3.0 MB GIF) | 26 s | the GIF alone is 19 s |
| **`scripts/run_all.sh` end to end** | **3 min 39 s** | |

### Real-time factor

The nominal scenario runs at **67.6× real time** single-threaded: 300 s of flight in 4.4 s,
covering 1 007 992 right-hand-side evaluations plus 60 000 EKF propagations, 1 500 GNSS
updates, 6 000 barometer updates and 15 000 magnetometer and pitot updates each.

| Scenario | Wall clock | Real-time factor |
|---|---|---|
| `nominal` (LQR, EKF feedback, wind) | 4.44 s | 67.6× |
| `ideal` (LQR, truth feedback, still air) | 3.90 s | 77.0× |
| `nominal_pid` (PID, EKF feedback, wind) | 4.43 s | 67.8× |
| `wind_disturbance` (strong wind, severe turbulence) | 4.49 s | 66.8× |

The Monte-Carlo trials run at 250 Hz instead of 500 Hz, so a trial costs 2.27 s for 300 s of
flight (**132× real time**); with 4 threads the campaign throughput is ~1.75 trials/s.

---

## 2. Integrator accuracy and cost

### RK4 convergence order (damped oscillator, analytic solution)

`ẍ + 2ζωẋ + ω²x = 0`, `ω = 2 rad/s`, `ζ = 0.05`, `T = 10 s`.

| `h` [s] | global error | observed order | evaluations | wall clock [s] |
|---|---|---|---|---|
| 0.2      | 2.094e-3  | —     | 200    | 5.1e-6 |
| 0.1      | 1.147e-4  | 4.191 | 400    | 1.0e-5 |
| 0.05     | 6.747e-6  | 4.087 | 800    | 2.0e-5 |
| 0.025    | 4.105e-7  | 4.039 | 1 600  | 4.1e-5 |
| 0.0125   | 2.534e-8  | 4.018 | 3 200  | 8.1e-5 |
| 0.00625  | 1.575e-9  | 4.008 | 6 400  | 1.6e-4 |
| 0.003125 | 9.814e-11 | 4.004 | 12 800 | 3.2e-4 |

The observed order converges to 4.00 from above, as expected for a fourth-order method whose
error constant is being resolved.

### Dormand-Prince 5(4) tolerance tracking

Same problem. Achieved error tracks the request within a factor of ~0.5 across eight decades:

| `rel_tol` | achieved error | accepted steps | rejected | evaluations | wall clock [s] |
|---|---|---|---|---|---|
| 1e-4  | 4.819e-5  | 50    | 0 | 301    | 1.4e-5 |
| 1e-6  | 4.870e-7  | 123   | 0 | 739    | 3.5e-5 |
| 1e-8  | 4.791e-9  | 316   | 2 | 1 911  | 9.0e-5 |
| 1e-10 | 4.969e-11 | 810   | 3 | 4 882  | 2.2e-4 |
| 1e-12 | 5.045e-13 | 2 057 | 4 | 12 371 | 5.7e-4 |

### Nonlinear 6-DOF aircraft against a reference solution

20 s of open-loop flight from trim with elevator and aileron doublets in a 3.6 m/s wind,
compared against Dormand-Prince at `rel_tol = 1e-12`. Controls are held over 0.5 s frames
(zero-order hold), so the right-hand side is smooth within a step.

| Method | `h` or `rel_tol` | state error | position error [m] | attitude error [rad] | evaluations | wall clock [s] |
|---|---|---|---|---|---|---|
| RK4 | 0.05   | 1.753e-3  | 6.07e-3  | 2.64e-5  | 1 600  | 3.2e-4 |
| RK4 | 0.02   | 3.937e-5  | 1.36e-4  | 6.01e-7  | 4 000  | 7.5e-4 |
| RK4 | 0.01   | 2.369e-6  | 8.21e-6  | 3.63e-8  | 8 000  | 1.6e-3 |
| RK4 | 0.005  | 1.454e-7  | 5.03e-7  | 2.23e-9  | 16 000 | 3.1e-3 |
| RK4 | 0.002  | 3.681e-9  | 1.27e-8  | 5.65e-11 | 40 000 | 7.6e-3 |
| RK4 | 0.001  | 2.271e-10 | 7.87e-10 | 3.48e-12 | 80 000 | 1.6e-2 |
| DOPRI 5(4) | 1e-4  | 3.351e-6  | 1.15e-5  | 1.16e-7  | 3 227  | 7.0e-4 |
| DOPRI 5(4) | 1e-6  | 3.231e-7  | 1.12e-6  | 2.43e-9  | 4 312  | 9.5e-4 |
| DOPRI 5(4) | 1e-8  | 2.561e-9  | 8.87e-9  | 1.81e-11 | 8 435  | 1.8e-3 |
| DOPRI 5(4) | 1e-10 | 2.266e-11 | 7.85e-11 | 0        | 19 663 | 4.2e-3 |

The error ratios for RK4 confirm fourth-order behaviour on the full nonlinear model as well:
halving `h` from 0.01 to 0.005 cuts the error by 16.3×, and the 2.5× reduction from 0.005 to
0.002 cuts it by 39.5× (`2.5⁴ = 39.1`).

**The trade-off, stated plainly.** At comparable accuracy the adaptive method is roughly
twice as cheap: RK4 at `h = 0.005` costs 16 000 evaluations for a 1.45e-7 state error, while
DOPRI at `rel_tol = 1e-8` costs 8 435 evaluations for 2.56e-9 — **57× more accurate for half
the work**. Fixed-step RK4 is nevertheless the default for the closed-loop scenarios, because
a fixed step makes the sensor and controller schedule exactly reproducible and the per-step
cost predictable, which matters more than efficiency for a 4 s run. The adaptive method is
the right choice when the trajectory has widely varying time scales, and it is available in
any scenario with `integration.integrator: dopri54`.

**Step size for the closed loop.** The scenarios use `dt = 0.002 s` (500 Hz), which the table
above puts at about 1e-8 m of integration error over 20 s — four orders of magnitude below
the 1 m scale of anything else in the problem. The Monte-Carlo campaign relaxes this to
`dt = 0.004 s` for speed, still around 1e-7 m.

---

## 3. Open-loop stability at trim

`Va = 25 m/s`, `h = 120 m`, `ρ = 1.211 kg/m³`. Trim converged in 8 iterations with an
infinity-norm residual of **6.14e-14**:

```
alpha = 4.579 deg   theta = 4.579 deg   phi = 0.009 deg
elevator = -1.371 deg   aileron = 0.143 deg   rudder = -0.041 deg   throttle = 0.671
```

(The small aileron and bank offsets trim out the propeller reaction torque.)

| Mode | Eigenvalue [1/s] | ω_n [rad/s] | ζ | Period [s] | Time to half/double [s] |
|---|---|---|---|---|---|
| Short period | −2.797 ± 6.408i | 6.991 | 0.400 | 0.981 | 0.248 |
| Phugoid | −0.0723 ± 0.5024i | 0.508 | 0.142 | 12.507 | 9.584 |
| Roll subsidence | −16.814 | 16.814 | 1.000 | — | 0.041 |
| Dutch roll | −0.5985 ± 5.0708i | 5.106 | 0.117 | 1.239 | 1.158 |
| Spiral | +0.0366 | — | — | — | 19.0 (doubling) |

All five classical modes are present with realistic values for a small UAV: a well-damped
short period, a lightly damped phugoid whose 12.5 s period is within 11% of the Lanchester
estimate `π√2 V/g = 11.3 s`, a fast roll mode, a lightly damped dutch roll, and a mildly
divergent spiral. Both reduced models are controllable (rank 4 of 4).

LQR closed-loop poles (augmented designs, `r_elevator = 300`, `r_aileron = 200`): the fastest
longitudinal pole is at −7.9 rad/s, comfortably slower than the 25 rad/s control-surface
actuator, and every pole of both designs is in the open left half plane. Riccati residuals
are ≈ 1e-13.

---

## 4. Closed-loop mission performance

Six-waypoint circuit, 3.54 km per lap, 300 s (two laps), identical seed for every scenario.
Errors are RMS over the run after a 20 s settling window.

| Scenario | Cross-track RMS / max [m] | Altitude RMS / max [m] | Airspeed RMS [m/s] | Peak bank [°] | Peak α [°] | Laps |
|---|---|---|---|---|---|---|
| `ideal` (truth feedback, still air) | **10.25** / 41.2 | **0.61** / 3.7 | 0.61 | 39.2 | 10.3 | 2 |
| `nominal` (LQR, EKF, 5 m/s wind + turbulence) | **11.81** / 48.7 | **1.05** / 5.2 | 0.65 | 40.2 | 10.5 | 2 |
| `nominal_pid` (PID, EKF, same conditions) | **15.08** / 49.5 | **1.90** / 5.2 | 0.64 | 37.0 | 10.0 | 2 |
| `wind_disturbance` (10.8 m/s wind, severe turbulence) | **31.33** / 163.9 | **1.14** / 5.3 | 0.86 | 41.2 | 12.3 | 2 |

Reading the table:

* **Estimation costs little.** Replacing perfect feedback with the EKF costs 1.6 m of
  cross-track RMS and 0.44 m of altitude RMS — the navigation solution is good enough that
  the loop barely notices.
* **LQR beats PID on the same mission by 22% in cross-track RMS and 45% in altitude RMS**, at
  the same airspeed accuracy and with a similar control effort. The LQR sees the coupled
  lateral dynamics and coordinates aileron and rudder together; the PID cascade closes each
  loop separately.
* **A 10.8 m/s wind — 43% of cruise airspeed — costs 2.7× the cross-track error but does not
  destabilise anything.** The peak of 164 m occurs where a downwind leg meets a 90° corner and
  the turn radius grows with groundspeed; altitude and airspeed tracking are essentially
  unaffected. Both laps still complete.
* **No actuator saturates** in the nominal run (0.00% of samples on every channel), and the
  35° bank limit is exceeded by 13% of samples with a 40.2° peak — the expected transient
  overshoot at the sharpest corner.

### Estimator accuracy (nominal run)

| Quantity | RMSE |
|---|---|
| Position (3-D) | **1.45 m** |
| Position (horizontal) | 1.18 m |
| Altitude | 0.84 m |
| Velocity (3-D) | 0.21 m/s |
| Attitude (rotation angle) | **1.53°** |
| Yaw | 0.88° |
| Wind (steady state, `t > 60 s`) | 0.62 / 0.58 m/s (N / E) |
| Sideslip reconstruction | 1.9° RMS |
| Airspeed | 0.24 m/s RMS |
| Final gyro-bias error | 1.6 mrad/s |
| Final accel-bias error | 0.12 m/s² |

Minimum eigenvalue of the covariance over the whole run: 2.2e-6 — positive throughout.

---

## 5. Monte-Carlo campaign

256 trials × 300 s, master seed 987654321, 4 threads, **146.6 s** wall clock.
Dispersions: mass ±8%, inertia ±12%, every aerodynamic derivative ±12%, thrust ±8%, initial
altitude ±10 m, airspeed ±1.5 m/s, attitude ±4-6°, heading ±20°, wind magnitude
5 ± 3 m/s from a uniformly random direction, turbulence `W₂₀` 7 ± 3.5 m/s, density ±5%, and a
log-normal ×e^(±0.3) scaling of every sensor noise and bias (all 1-σ).

**Outcome: 247 completed, 9 failed — a 3.5% failure rate** (7 ground contact, 2 airspeed
high).

| Metric | Unit | Mean | Std dev | Median | 95th pct | Max |
|---|---|---|---|---|---|---|
| RMS cross-track error | m | 14.56 | 4.78 | 13.62 | 25.15 | 45.81 |
| Peak cross-track error | m | 62.71 | 23.22 | 59.03 | 106.54 | 186.40 |
| RMS altitude error | m | 1.03 | 0.44 | 0.92 | 1.81 | 4.72 |
| Peak altitude error | m | 4.33 | 0.71 | 4.29 | 5.63 | 8.22 |
| RMS airspeed error | m/s | 0.73 | 0.10 | 0.71 | 0.94 | 1.13 |
| Estimator position RMSE | m | 1.89 | 0.84 | 1.71 | 3.49 | 6.35 |
| Estimator velocity RMSE | m/s | 0.245 | 0.091 | 0.239 | 0.406 | 0.576 |
| Estimator attitude RMSE | deg | 1.60 | 0.54 | 1.52 | 2.48 | 3.25 |
| Estimator yaw RMSE | deg | 1.14 | 0.43 | 1.13 | 1.89 | 2.63 |
| Final gyro-bias error | rad/s | 0.0018 | 0.0010 | 0.0016 | 0.0037 | 0.0059 |
| Final accel-bias error | m/s² | 0.165 | 0.092 | 0.147 | 0.331 | 0.556 |
| Peak bank angle | deg | 40.33 | 1.26 | 40.14 | 42.75 | 45.43 |
| Laps completed | - | 1.98 | 0.14 | 2.00 | 2.00 | 2.00 |
| Wall clock per trial | s | 2.27 | 0.26 | 2.31 | 2.37 | 2.49 |

**Where the failures are.** The failed trials average 14.6 kg against 13.6 kg overall and
`C_Lα = 4.28` against 5.03 — roughly 25% higher effective wing loading, usually combined with
a 6-12% low-density draw. The mission commands 35°-bank turns at a fixed 23-27 m/s, and those
combinations simply run out of stall margin. That is a vehicle-performance boundary, not a
controller defect: the envelope protection widens the turns until it cannot, and then the
aircraft descends. `results/figures/monte_carlo.png` plots the outcome against mass and
`C_Lα` directly.

**Peak bank is tightly controlled** (40.3 ± 1.3°) across every dispersion, which is the
gain-derived command limiting working as designed.

---

## 6. Numerical accuracy summary

| Quantity | Achieved |
|---|---|
| Trim residual (infinity norm) | 6.1e-14 |
| Riccati residual, longitudinal / lateral | 4.6e-13 / 7.6e-14 |
| Linearisation vs finite differences | second-order, < 50ε² |
| RK4 observed convergence order | 4.00 |
| Quaternion norm drift over 10 s without renormalisation | < 1e-6 |
| Angular momentum conservation, torque-free, 20 s | 1e-9 relative |
| EKF covariance minimum eigenvalue over 300 s | 2.2e-6 (positive) |
| EKF discretisation error, second-order vs exact | < 1e-4 relative |
| Dryden stationary σ vs configured | within 6% |

---

## 7. Repository footprint

| | |
|---|---|
| C++ (headers + sources + apps) | ~7 700 lines |
| Tests | ~3 200 lines |
| Python | ~1 500 lines |
| Documentation (this set, before the README) | ~1 900 lines |
| Committed results (figures + summaries + linear model + campaign tables) | 7.9 MB |
| Generated but *not* committed (bulk CSV logs) | ~53 MB, regenerated by `make run-all` |
