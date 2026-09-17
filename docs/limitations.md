# Known limitations

Written as specifically as possible: what is approximated, why, what it costs, and what it
would take to remove. Nothing here is a surprise to the code — most items are asserted or
measured somewhere in `tests/` or `results/`.

---

## 1. The aircraft parameters are synthetic

**The shipped coefficients describe no real aircraft.** They are marked `synthetic: true` in
the airframe file and printed with a `[SYNTHETIC PARAMETERS]` banner by every application.

They were derived here from classical thin-aerofoil and tail-volume relations
(`docs/model.md` §13) so that every sign and magnitude is internally consistent, and the
derivation is shown. The resulting vehicle produces all five classical flight modes with
realistic values, which is evidence that the set is *coherent* — it is not evidence that it
matches any particular airframe.

This was a deliberate choice over transcribing a published table from memory: a
misattributed coefficient set would look more authoritative and be less trustworthy. To use a
real airframe, drop its coefficients into a new file under `configs/aircraft/`; no code
changes are needed.

## 2. Flat, non-rotating Earth

No Earth rotation, no transport rate, constant gravity magnitude and direction. Over a 1 km
circuit at 25 m/s the neglected Coriolis acceleration is below 2×10⁻³ m/s², four orders of
magnitude under the aerodynamic accelerations, and the gravity variation over the 40 m
altitude band is ~1×10⁻⁴ m/s². For a small UAV at this scale the approximation is not the
limiting error. It would be wrong for a long-range or high-altitude vehicle, and the fix is
an ECEF or NED-with-transport-rate formulation in `RigidBody6DOF`.

## 3. Rigid airframe, constant mass

No aeroelasticity, no fuel burn, no CG migration, no control-surface aerodynamic loading.
Monte Carlo disperses mass and inertia between trials but they are constant *within* a trial.

## 4. Quasi-steady aerodynamics

The coefficient build-up is linear in α, β, the body rates and the deflections, with a
sigmoid blend to a flat plate past the stall. Specifically absent:

* **unsteady aerodynamics** — no Theodorsen-type lag, no dynamic-stall hysteresis, so
  post-stall behaviour is smooth and reversible where a real aircraft would show hysteresis
  and possibly a departure;
* **compressibility** — no Mach dependence, which is irrelevant at 25 m/s (M ≈ 0.07);
* **Reynolds-number dependence** — the coefficients do not vary with airspeed or altitude;
* **propeller-wash interaction** — the slipstream does not change the flow over the tail, so
  the real coupling between throttle and pitch is missing;
* **ground effect** — not modelled, which matters near the ground plane;
* **control-surface aerodynamic nonlinearity** — effectiveness is constant with deflection.

The wind-axis simplification (lift and drag rotated through α only, with the side force along
body y) is exact at β = 0 and accurate to O(β²). At the largest sideslip seen in the nominal
run (~2°) the error is below 0.1%.

## 5. No ground contact mechanics

Ground contact is **detection only**: the run stops with `termination_reason:
"ground_contact"` when the altitude crosses the ground plane. There is no landing gear, no
tyre or strut model, no ground reaction, no friction, no take-off or landing roll. Every
scenario therefore begins from a valid airborne trimmed condition, and a ground contact is
scored as a failure rather than simulated through. Adding contact mechanics would mean a
compliant-contact or constraint formulation plus wheel/brake models.

## 6. Navigation-filter consistency

Measured in `docs/validation.md` §10 and visible in `results/figures/estimator_detail.png`.
Velocity, yaw and gyro bias are consistent (95-100% of samples inside 3σ). Position, wind and
the barometer bias are **optimistic**: 59%, 52% and 12% respectively.

The cause is known and deliberate. The GNSS truth model includes a first-order Gauss-Markov
correlated position error (σ = 1.5 m, τ = 300 s) that the filter does not model, and
**cannot** usefully model: a GNSS position bias and a vehicle position error enter the
measurement identically, so they are not separately observable from GNSS alone. Inflating the
assumed GNSS noise (2.2 m horizontal against a 1.2 m white truth) widens the envelope but
cannot remove a common-mode offset. The practical consequence is that the reported position
covariance understates the true error by roughly the correlated-error magnitude — about 1 m
here. Ways to improve it: a second GNSS antenna or carrier-phase measurements; a
map/terrain-relative measurement; or modelling the correlated error with a separate absolute
position reference that breaks the ambiguity.

The x accelerometer bias is only weakly separable from pitch attitude in near-level flight
(a pitch error tilts gravity into body x exactly like an x-bias). The filter's process noise
for the accelerometer bias is therefore tuned an order of magnitude above the sensor truth
(0.04 against 0.003 m/s²/√s), which keeps σ honest at a cost of about 0.1° of attitude RMSE.
Without that inflation the accel-bias channel sits at 0% inside 3σ — confidently wrong.

## 7. Estimator scope

Not estimated: sensor scale factors, IMU axis misalignment, magnetometer soft-iron effects,
GNSS lever arm, vertical wind, or time delays and clock offsets between sensors. The
magnetometer hard-iron bias is *simulated* but not estimated, so it shows up as a small
heading bias. Vertical wind is not estimated because the pitot measurement provides only one
scalar and the horizontal components already consume its information.

Sensor measurements are also taken to be instantaneous and time-aligned with the truth state:
there is no transport delay, no sample-and-hold skew and no out-of-sequence measurement
handling, all of which a flight-grade filter must deal with.

## 8. Control scope

* The two autopilots are **fixed-gain** designs, synthesised once at the nominal trim point.
  There is no gain scheduling with airspeed or altitude, so performance degrades away from
  25 m/s; the airspeed sweep in `results/figures/trim_sweep.png` shows how much the modal
  characteristics move over 16-34 m/s.
* No explicit robustness margin analysis (gain/phase margin, µ-analysis, structured singular
  value). Robustness is demonstrated empirically by the Monte-Carlo campaign rather than
  certified analytically.
* The actuator lag is inside the *plant* but not inside the *design model*: the LQR is
  synthesised from the rigid-body linearisation only. This is why the design deliberately
  keeps the fastest closed-loop pole (−7.9 rad/s) well below the actuator pole (−25 rad/s).
  Including the actuator states in the design model would remove that constraint.
* Envelope protection covers angle of attack only. There is no explicit airspeed-low
  protection (the throttle loop handles it), no load-factor limiting and no bank-angle
  protection independent of the course clamp.
* The guidance layer follows straight-line legs and circular orbits. There is no
  Dubins-path planning, no obstacle avoidance and no altitude-rate optimisation.
* **The LQR's lateral design model assumes the trim airspeed, not ground speed.** The course
  row of the lateral `A` matrix is the coordinated-turn kinematics `χ̇ ≈ (g/Va₀)·δφ`,
  linearised about the trim airspeed `Va₀` (`src/control/LqrAutopilot.cpp`). In light wind
  that is close enough that the LQR outperforms the PID on both path and altitude tracking.
  Once the wind is a large fraction of the airspeed the crab angle is large, ground speed and
  airspeed diverge, and the assumed course-rate gain is wrong: sweeping the dashboard's wind
  slider shows the LQR's cross-track RMS growing faster than the PID's above roughly 8 m/s of
  wind at 25 m/s airspeed (see `docs/dashboard.md`), while its altitude tracking stays two to
  three times better throughout. The design model would have to be re-derived about ground
  speed — and the Monte-Carlo campaign re-run — to remove this.
* The autopilots have no wind-aware guidance: cross-track control corrects the resulting
  error rather than anticipating the drift, so a steady crosswind is rejected by the
  integrator rather than by a feed-forward crab angle.

## 9. Turbulence model

Dryden rather than von Kármán. Dryden is a rational approximation and therefore exactly
realisable as a finite-dimensional filter, which is why it is used here; von Kármán matches
measured atmospheric spectra better at high frequency but needs a fractional-order
approximation. Gusts are also treated as spatially uniform over the airframe — no gust
gradient across the span, so the rolling moment a real vehicle would see from an asymmetric
gust is missing. At a 2.6 m span in turbulence with a 50-200 m length scale this is a
reasonable approximation.

The gust is held constant over one simulation frame (2 ms) rather than being integrated
continuously with the vehicle state. At 500 Hz against a ≥ 2 s correlation time this is a
negligible zero-order-hold error.

## 10. Numerics

* The fixed-step RK4 path does no error control at all: the step size is the user's
  responsibility. `docs/benchmarks.md` §2 quantifies what each step size buys.
* The adaptive path accepts a step that is already at `min_step` even if it misses the
  tolerance, counting it in `IntegrationStats::tolerance_not_met`. This is deliberate — the
  alternative is an infinite rejection loop when the requested tolerance is below the
  round-off floor of the right-hand side — but it means a tolerance can be silently missed
  unless the caller inspects the counter.
* The CARE solver is a dense matrix-sign-function method suitable for the small systems here
  (n ≤ 16). It is not appropriate for large-scale problems, and it will fail on a Hamiltonian
  with eigenvalues on the imaginary axis (an uncontrollable or undetectable mode); it throws
  with a message naming that cause rather than returning a wrong answer.
* Everything is double precision; no interval arithmetic or error-bound propagation.

## 11. Monte-Carlo scope

* Dispersions are independent draws. Real parameter uncertainties are correlated (mass and
  inertia in particular), so the sampled corners of the box may be less likely than the
  campaign implies.
* 256 trials resolve a ~3.1% failure rate to roughly ±1.1% (1-σ binomial). Quantifying a
  rare-event probability — say 10⁻⁴ — would need importance sampling or a different method
  entirely.
* Aerodynamic coefficients are dispersed multiplicatively and independently, so a coefficient
  that is nominally zero stays zero. Adding an *additive* dispersion would exercise the
  asymmetric airframe cases (`C_l0`, `C_n0`, `C_Y0`) that the current set cannot reach.
* Every trial flies the same mission with the same waypoints; mission-level dispersion is not
  included.

## 12. Software

* Single-process, and parallel only across Monte-Carlo trials. A single trajectory is
  strictly sequential.
* Tested with GCC 13 on Linux. The code is standard C++17 with no platform-specific calls and
  should build on Clang and MSVC, but that is untested here.
* No Python packaging (`pyproject.toml`), no API/ABI stability guarantee, no installed CMake
  package config for downstream `find_package(aether)`.
* The Python analysis layer has no test suite of its own; it is exercised end to end by
  `make figures`, which fails loudly if a loader or a plot breaks.
* Doxygen comments are written throughout the public headers, but no Doxygen configuration or
  generated HTML is shipped.
