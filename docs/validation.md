# Validation methodology

Everything in this file is produced by code in the repository. Reproduce it with:

```bash
make build && make test          # the automated suite
make trim                        # trim, linearisation and modal analysis
make integrators                 # convergence and cost study
make run-scenarios               # the closed-loop regressions
make monte-carlo                 # the statistical campaign
```

Current status: **103 test cases, 33 916 assertions, all passing** (28 s, single-threaded,
`Release`, GCC 13.3 on an x86-64 container).

---

## 1. Why these tests

Three questions are worth separating.

* **Is the code doing what the equations say?** Checked by identities and analytic solutions:
  quaternion algebra, rotation orthogonality, free fall, torque-free rotation, ISA values,
  convergence orders, Riccati residuals.
* **Do the equations say something physically sensible?** Checked by sign conventions, static
  and dynamic stability, modal frequencies against classical approximations, and trim trends.
* **Does the whole stack hold together?** Checked by closed-loop regressions on a real
  mission, estimator consistency, and a Monte-Carlo campaign.

A test that only re-states the implementation proves nothing, so the checks below are written
against independent references — closed-form solutions, conserved quantities, published ISA
numbers, textbook modal approximations — wherever one exists.

---

## 2. Kinematics and algebra

| Property | Check | Result |
|---|---|---|
| Quaternion norm | 200 random quaternions | ‖q‖ = 1 to 1e-14 |
| Product associativity | `(a⊗b)⊗c = a⊗(b⊗c)` | 1e-13 |
| Conjugate inverts | `q ⊗ q* = 1` | 1e-13 |
| Rotation is proper orthogonal | `RRᵀ = I`, `det R = 1` | 1e-13 |
| Sandwich product = matrix product | 200 random cases | 1e-12 |
| Rotation preserves length | ‖Rv‖ = ‖v‖ | 1e-12 |
| Euler round trip | 300 random attitudes away from the singularity | 1e-11 |
| Named rotations | yaw +90° puts the nose East; pitch +90° puts it up; roll +90° puts the right wing down | exact |
| exp / log round trip | 300 rotation vectors, plus the small-angle branch | 1e-11 |
| boxplus / boxminus | `(q ⊞ δ) ⊟ q = δ` | 1e-11 |
| `wrapPi` | endpoints and multiples of 2π | exact |

## 3. Numerical linear algebra

| Property | Check | Result |
|---|---|---|
| `expm` diagonal | `e^{diag(1,−2,0.5)}` | 1e-12 relative |
| `expm` rotation generator | planar rotation by ωt | 1e-12 |
| `expm` nilpotent | exactly truncating series | 1e-12 |
| `expm` large norm | `‖A‖ = 30`, scaling and squaring | orthogonal to 1e-10 |
| Van Loan | scalar Ornstein-Uhlenbeck: `Ad = e^{−a dt}`, `Qd = q/(2a)(1−e^{−2a dt})` | 1e-11 |
| CARE solution | symmetric, PSD, residual `‖AᵀP+PA−PBR⁻¹BᵀP+Q‖_F` | < 1e-9 |
| CARE stabilises | closed loop of an open-loop-unstable plant | all poles in the LHP |
| Controllability rank | a deliberately unreachable mode | rank 1 of 2, detected |
| Numerical Jacobian | against an analytic Jacobian | < 1e-8 |
| Levenberg-Marquardt | Rosenbrock from (−1.2, 1) | converges to (1, 1) within 1e-6 |

> **A bug this caught.** The first `expm` implementation started the Padé denominator
> recurrence with the wrong sign, giving `e^1 = 2.894` instead of `2.718` — a 6.5% error that
> silently corrupted the Dryden turbulence discretisation. The analytic `expm` test found it
> immediately.

## 4. Dynamics: conventions and conservation

**Sign conventions** — each of these is a separate assertion in `tests/test_dynamics.cpp`:
positive elevator gives a nose-down moment *and* more lift; positive aileron gives a positive
rolling moment; positive rudder gives right side force *and* a nose-left yawing moment;
positive α gives a nose-down moment increment (static stability); positive β gives a
nose-right moment (weathercock) and a negative rolling moment (dihedral effect); each rate
derivative opposes its rate, and the increment is antisymmetric in the sign of the rate.

**Gravity transformation** — wings level, gravity acts along body +z; pitched 90° nose-up it
acts along body −x; rolled 90° right it acts along body +y. Each to 1e-9.

**Conservation** (vacuum, zero gravity where stated):

| Property | Setup | Result |
|---|---|---|
| Zero-force straight line | no aero, no thrust, no gravity, 10 s | NED velocity constant to 1e-9 m/s; position matches `p₀ + v₀T` to 1e-7 m |
| Free fall | vacuum, gravity on, 5 s | drop and speed match `½gT²`, `gT` to 1e-10 relative |
| Torque-free rotation | tumbling at (0.8, 0.5, −0.3) rad/s, 20 s | inertial angular momentum conserved to 1e-9 relative; rotational kinetic energy to 1e-9 |
| Quaternion norm stabilisation | start at ‖q‖ = 1.02, integrate 10 s **without** renormalisation | returns to 1 within 1e-6 |

**Robustness** — at `Va = 1e-9 m/s` with 2 rad/s body rates and full control deflections the
derivative is finite and the aerodynamic wrench is below 1e-12 N: the rate terms use a
clamped airspeed, but the dynamic pressure uses the true one, so forces vanish rather than
diverge.

**Wind enters only through the air-relative velocity** — the same air-relative state in still
air and in a 5 m/s headwind produces identical aerodynamic forces to 1e-9 N.

**Specific force excludes gravity** — an accelerometer in free fall reads zero to 1e-12.

**Actuators** — rate limiting, magnitude saturation and the first-order lag are each checked
against their closed-form response (the lag reaches `1 − e⁻¹` of a step after one time
constant, within 2%).

**Atmosphere** — against published ISA values: `T(5 km) = 255.65 K`, `p(5 km) = 54 019.9 Pa`,
`ρ(5 km) = 0.73643 kg/m³`, `T(11 km) = 216.65 K`, `p(11 km) = 22 632 Pa`, all within 2e-4
relative except density (2e-3). Density is checked monotonic from 0 to 20 km.

## 5. Integrators

**RK4 convergence order.** Global error at `T = 4 s` on `ẏ = λy` for five step sizes; the
observed order between successive halvings is required to lie in **[3.85, 4.15]** — the
measured values are 4.19, 4.09, 4.04, 4.02, 4.01, 4.00 on the oscillator problem used by
`aether_integrators`. The same check on an undamped oscillator requires [3.8, 4.2].

**Adaptive error control.** Dormand-Prince 5(4) at `rel_tol ∈ {1e-4 … 1e-10}` must achieve a
global error below `200 × rel_tol` and improve monotonically. Measured on the damped
oscillator:

| requested `rel_tol` | achieved error | accepted | rejected | evaluations |
|---|---|---|---|---|
| 1e-4  | 4.82e-5  | 50   | 0 | 301 |
| 1e-6  | 4.87e-7  | 123  | 0 | 739 |
| 1e-8  | 4.79e-9  | 316  | 2 | 1 911 |
| 1e-10 | 4.97e-11 | 810  | 3 | 4 882 |
| 1e-12 | 5.04e-13 | 2 057| 4 | 12 371 |

**Step-size adaptation.** On a problem with a sharp transient the controller must shrink and
re-grow the step (`min_dt < max_dt`) and match a very fine fixed-step solution to 1e-6.

**Rejection.** An oversized first step is rejected, the state does not advance, `dt_used = 0`
and the proposed next step is smaller.

**Structure preservation.** Pure attitude kinematics at a constant body rate for 20 s: with
the projection installed the norm stays 1 to 1e-15 and the total rotation angle matches
`|ω|T` to 1e-8.

**Cross-check on the aircraft.** RK4 at `dt = 1e-4` and DOPRI at `rel_tol = 1e-11` agree to
1e-5 m in position and 1e-8 rad in attitude after 8 s.

**API guards.** A non-positive step size and a reversed time interval both throw.

## 6. Trim

| Property | Result |
|---|---|
| Straight-and-level residual (infinity norm) | **6.14e-14**, 8 iterations |
| Packed-state derivative at trim | ‖v̇‖ < 1e-10, ‖ω̇‖ < 1e-10, ḣ < 1e-10 |
| Physical sanity | α = 4.58° > 0; θ = α (γ = 0, β = 0) to 1e-9; δ_e = −1.37° (up elevator, statically stable); δ_t = 0.671 ∈ (0, 1) |
| Airspeed trend | α decreases monotonically over 18-32 m/s; every case converges below 1e-9 |
| Climb / descent | θ = α + γ to 1e-8; achieved climb rate = `Va sin γ` to 1e-9; climb needs more throttle than level, descent less |
| Coordinated turn (15°/s) | φ > 0 and `tan φ = Vψ̇/g` within 5%; β < 1e-9 |
| Steep but reachable climb | 30 m/s at +5° converges below 1e-9 at 89% throttle |
| Unreachable condition | a 40° climb is reported **not converged** (the throttle is on its stop) |
| Bound penalties | exactly zero on a feasible point; an infeasible request still returns controls inside the actuator box and \|α\| < 20°, \|β\| < 15° |

## 7. Linearisation

**Second-order agreement.** For `ε ∈ {1e-2, 1e-3, 1e-4}` the mismatch between `A δ` and the
finite difference of the nonlinear model must fall by at least a factor of 20 per decade of
`ε`, and stay below `50ε² + 1e-9`. The same check is applied to every column of `B`.

**Trajectory agreement.** After a 0.5° elevator step, the linear and nonlinear models agree
within 2% on the velocity, attitude and rate perturbations after 1 s.

**Structure.** The attitude-rate block of `A` equals the identity to 1e-6; the attitude rows
have no dependence on position or velocity; the two horizontal position columns are zero
(flat Earth, horizontally uniform atmosphere); and the full 12-state model has exactly four
eigenvalues within 1e-6 of the origin — three position integrators and heading.

**Modes.** Computed at `Va = 25 m/s`, `h = 120 m`:

| Mode | Eigenvalue [1/s] | ω_n [rad/s] | ζ | Period [s] | Test requirement |
|---|---|---|---|---|---|
| Short period | −2.797 ± 6.408i | 6.991 | 0.400 | 0.98 | stable, 2 < ω_n < 20, 0.25 < ζ < 2 |
| Phugoid | −0.0723 ± 0.5024i | 0.508 | 0.142 | 12.51 | stable, ζ < 0.4, period within 35% of `π√2 V/g` = 11.32 s |
| Roll subsidence | −16.814 | 16.81 | 1.0 | — | stable, real, faster than −3 |
| Dutch roll | −0.5985 ± 5.0708i | 5.106 | 0.117 | 1.24 | stable, ω_n > 1, ζ > 0.02 |
| Spiral | +0.0366 | — | — | — | real, \|λ\| < 0.5 (doubling time 19.0 s) |

Both reduced models are checked controllable (rank 4 of 4).

## 8. Environment

**Dryden turbulence variance.** 4×10⁵ samples after a 200 s burn-in: achieved standard
deviations within 6% of the configured σ on all three axes, and the sample mean below four
standard errors of the autocorrelated mean estimator. Because the filters use an exact
(Van Loan) discretisation, the same check passes at `dt = 0.002 s` and `dt = 0.05 s` — the
variance does not depend on the step size, which is the whole point of doing it exactly.

**Altitude scaling.** The MIL-F-8785C low-altitude model gives `σ_w = 0.1 W₂₀` independent of
height (within 15%) while `σ_u` grows towards the ground; both are checked.

**Shear.** The logarithmic profile returns the configured wind exactly at the reference
altitude, is weaker below and stronger above it, preserves direction, and stays finite at
`h = 0`.

**Reproducibility.** Two `WindModel`s with the same seed produce identical gusts; a different
seed produces different ones; `reset()` zeroes the filter states.

## 9. Sensors

| Property | Result |
|---|---|
| Sample rates | 200/5/20/50/50 Hz over 10 s: 2000, 50, 200, 500, 500 samples (±2) |
| Rate with an incommensurate step | 200 Hz on a 500 Hz frame delivers the configured rate to 0.5%, with jitter bounded by one step; a step longer than the period resynchronises instead of building a backlog |
| Gyro white noise | 2×10⁵ samples: σ within 3% of configuration, mean below 5% of σ |
| Gyro bias random walk | 400 independent runs: RMS after 100 s within 15% of `walk·√T` |
| Magnetometer | with noise and bias disabled, reads `R(q)ᵀ m^n` exactly (1e-15); field magnitude correct; northern-hemisphere down component positive |
| Scripted GNSS outage | no fixes inside the window, fixes before and after; `enable_gps: false` yields none at all |
| Determinism | identical seeds give identical samples; different seeds differ |

## 10. Estimator

**Covariance health.** Over 4 000 propagation steps with interleaved GNSS, barometer and
magnetometer updates, the covariance is asserted symmetric (to 1e-12 relative) and positive
semi-definite (smallest eigenvalue > −1e-12) **after every single predict and update** — not
just at the end.

**Convergence.** From a deliberately wrong initial estimate (6 m, 4 m, 3 m position; 1 m/s
velocity; ~5° attitude) on a 120 s coordinated turn with 6 mrad/s gyro bias, 0.05 m/s² accel
bias and a 10 m/s wind, the filter must end within 4 m of position, 0.6 m/s of velocity and
4° of attitude, recover at least half the gyro bias, and estimate both wind components within
1.5 m/s.

**Outlier rejection.** A 5 km GNSS jump is rejected by the chi-squared gate; the accepted
count does not increase and the state does not move.

**Discretisation.** The second-order series used at run time agrees with the exact Van Loan
discretisation to **< 1e-4 relative** on the covariance after 400 steps at 200 Hz.

**Wind observability.** Flying a 120 s circle recovers the wind to < 1 m/s; flying straight
for the same time leaves a larger error *and* a larger wind covariance, because a scalar
airspeed measurement only observes the component along the current ground track. The test
asserts both.

**Wind seeding.** The one-shot wind-triangle initialisation recovers a 10 m/s wind to 0.2 m/s
from the first airspeed sample and drives the reconstructed sideslip below 0.3 m/s of lateral
air velocity; the unseeded filter's error after the same single update is more than five
times larger.

### Measured consistency in the closed loop

From `results/nominal` (300 s, `t > 20 s`), estimate error against the filter's own 1-σ:

| state | RMS error | mean σ | inside 3σ |
|---|---|---|---|
| position N [m] | 0.977 | 0.329 | 59% |
| position E [m] | 0.660 | 0.330 | 87% |
| position D [m] | 0.837 | 0.248 | 31% |
| velocity N [m/s] | 0.141 | 0.107 | 95% |
| velocity D [m/s] | 0.054 | 0.111 | 100% |
| roll [rad] | 0.013 | 0.008 | 80% |
| pitch [rad] | 0.017 | 0.007 | 62% |
| yaw [rad] | 0.016 | 0.019 | 100% |
| gyro bias x [rad/s] | 0.0011 | 0.0016 | 100% |
| accel bias x [m/s²] | 0.184 | 0.082 | 86% |
| wind N [m/s] | 0.634 | 0.133 | 52% |
| baro bias [m] | 0.832 | 0.165 | 12% |

The velocity, yaw and gyro-bias channels are consistent. The position, wind and barometer-bias
channels are **optimistic**, and the cause is known and deliberate: the GNSS truth model
includes a first-order Gauss-Markov correlated position error (σ = 1.5 m, τ = 300 s) that the
filter does not model. It cannot usefully model it either — a GNSS position bias and a
position error enter the measurement in exactly the same way, so they are not separately
observable from GNSS alone. Inflating the assumed GNSS noise (the shipped scenarios use 2.2 m
horizontal against a 1.2 m white truth) widens the envelope but cannot remove a common-mode
offset. The honest statement is that the filter's position covariance understates the error by
roughly the correlated-error magnitude; see `docs/limitations.md`.

## 11. Control

**PID primitive.** The parallel law is checked term by term; the integrator advances *after*
the output is formed (so after N steps the output carries N−1 increments and the state N);
the derivative can be supplied externally and produces no kick on a setpoint step; `reset()`
clears everything.

**Anti-windup.** With `ki = 5` and a constant error of 10 held for 10 s against a ±1 output
limit, the integrator would reach 500 without protection; conditional integration alone keeps
it below 10, and back-calculation additionally unwinds a pre-loaded integrator along the
predicted first-order decay `u_max + (I₀ − u_max)e^{−k_aw t}` (checked within 5%). Recovery
from saturation takes fewer than 20 steps.

**LQR synthesis.** Gains are finite and correctly shaped; both Riccati residuals are below
1e-8 (measured ≈ 1e-13); every augmented closed-loop eigenvalue is in the open left half
plane; and the gain-derived command limits are strictly tighter than the configured
fallbacks.

**Command limiting.** The bank at which the course terms balance the bank feedback,
`(K_χ χ_max + K_∫ ∫_max)/K_φ`, is asserted to be at most the configured `max_bank`.

**Holding trim.** Started exactly on the trim condition, both autopilots hold the trimmed
elevator to 2e-3 rad and throttle to 2e-2 after 2 s, with aileron and rudder below 0.05 rad.

**Actuator limits.** Under absurd commands (5000 m altitude, 200 m/s airspeed, 180° course
change) every channel stays inside its saturation for 2000 consecutive steps.

**Envelope protection.** Inactive below the threshold; authority ramps linearly over the
blend width; continuous at the threshold; includes pitch-rate damping; never reduces a
stronger nose-down command from the control law; can be disabled. End to end: a **30%
overweight** airframe flying the circuit at 23 m/s completes the mission with protection
(max α = 15.7°, below the 17.2° stall blend) and is **lost** without it (max α = 38.8°,
ground contact at 79.6 s).

**Guidance.** On the path the commanded course equals the path course; to the right it steers
left and vice versa; far off the path it saturates at `χ_∞`; altitude interpolates along the
leg; half-plane switching advances the waypoint and the *returned* command already belongs to
the new leg; in loop mode the index wraps to 0 so the closing leg is flown and the lap counter
increments; orbit mode commands a tangential course on the circle and turns inwards outside
it; fewer than two waypoints throws.

### Closed-loop regressions

Two regression tests fly the actual mission and assert performance, not just survival:

* `ideal.yaml` with **both** controllers: completes 200 s, ≥ 5 waypoints, cross-track RMS
  < 30 m and peak < 90 m, altitude RMS < 8 m, airspeed RMS < 2 m/s, peak bank < 50°, peak
  α < 20°, final state finite.
* `nominal.yaml` (EKF feedback, wind, turbulence): the same tracking bounds plus estimator
  position RMSE < 10 m, velocity RMSE < 1 m/s, attitude RMSE < 5°, and a positive-definite
  covariance throughout.

**Determinism.** Two `Simulator`s with the same configuration produce a bit-identical final
state; changing only the seed changes it.

## 12. Monte Carlo

**Statistics helper** — mean, standard deviation and percentiles checked against closed-form
values for 1…100, plus the empty and single-sample edge cases.

**Trial construction** — the same index reproduces the same dispersion; different indices
differ; the dispersed airframe always passes `validate()`; static stability (`C_mα < 0`),
weathercock stability (`C_nβ > 0`) and roll damping (`C_lp < 0`) survive dispersion; and the
dispersed inertia tensor is still a physically realisable rigid body (positive definite,
principal moments satisfying the triangle inequality).

**Dispersion magnitudes** — over 600 constructed trials the sample mean and standard deviation
of mass, `C_Lα`, wind speed and heading offset match their configured values within 3% and
20% respectively.

**Determinism** — a seeded campaign reproduces every trial's seed, outcome and metrics
exactly, and **4 worker threads give bit-identical results to 1 thread**. (Wall-clock time is
the one statistic excluded, since it is measured rather than computed.)

**Output** — `trials.csv` has exactly one row per trial; `summary.json` contains the trial
count, failure rate and the statistics block; all four output files exist and are non-empty.

### Campaign result

256 trials × 300 s, master seed 987654321, 4 threads, 159.9 s wall clock:

* **248 completed, 8 failed (3.1%)** — 7 ground contact, 1 airspeed high.
* The failures are concentrated in the heavy / low-lift-slope / low-density corner of the
  dispersion box (mean failed mass 14.4 kg against 13.6 kg overall, mean failed `C_Lα` 4.24
  against 5.01). Those combinations have roughly 25% higher effective wing loading, and the
  mission's 35°-bank turns at a fixed 23-27 m/s command run out of stall margin. This is a
  genuine vehicle-performance limit being found by the campaign, not a controller defect.

> **Three real defects the campaign found**, each fixed and each now covered by a test:
> 1. the LQR pitch gain was high enough that a 6° pitch error saturated the elevator, pitching
>    the aircraft into a deep stall the altitude loop then held it in (12.1% failure rate);
> 2. the first angle-of-attack protection was a hard `max()` override with no rate damping and
>    set up a bang-bang limit cycle at the short-period frequency, reaching ±40° of α (44.5%);
> 3. the EKF wind estimate started at zero, so in a 10 m/s crosswind the reconstructed
>    sideslip was wrong by 28° for the first second — enough to upset the aircraft before the
>    filter converged.
>
> The failure rate went 12.1% → 44.5% → 4.7% → 3.1% as each was diagnosed and fixed. That
> sequence is the argument for running the campaign at all.
