# Mathematical model and coordinate conventions

All quantities are SI throughout the C++ core: metres, kilograms, seconds, radians, newtons.
Degrees appear only in configuration files (keys ending in `_deg` / `_deg_s`) and in console
and figure output, where the conversion is explicit.

---

## 1. Reference frames

| Frame | Symbol | Definition |
|---|---|---|
| Earth / inertial | `n` | Local-level **North-East-Down** frame, flat-Earth approximation. The origin is the scenario reference point on the ground; `p_D` is positive **downwards**, so altitude is `h = -p_D`. |
| Body | `b` | **Forward-Right-Down** (FRD) frame fixed to the airframe: `x_b` out the nose, `y_b` out the right wing, `z_b` down through the belly. Origin at the centre of mass. |
| Stability / wind | - | Used only to express aerodynamic force directions; see §4. |

The flat-Earth approximation (no Earth rotation, no transport rate, constant gravity vector)
is appropriate for the vehicle scale simulated here: over a 1 km circuit at 25 m/s, the
neglected Coriolis acceleration is below 2×10⁻³ m/s², four orders of magnitude under the
aerodynamic accelerations.

### Attitude

Attitude is a **Hamilton** unit quaternion `q_nb = [q_w, q_x, q_y, q_z]ᵀ` that rotates a
*body* vector into the *NED* frame:

```
v^n = R(q_nb) v^b
```

Euler angles are the aerospace 3-2-1 sequence `(ψ, θ, φ)` applied as
`R = R_z(ψ) R_y(θ) R_x(φ)`, i.e. yaw, then pitch, then roll. They are used for display and
for the control laws; the integration state is always the quaternion, so there is no gimbal
singularity in the dynamics.

Implementation: `include/aether/math/Rotation.hpp`.

---

## 2. State vector

The nonlinear model integrates 13 states:

| Index | Symbol | Meaning | Unit |
|---|---|---|---|
| 0-2 | `p^n = [p_N, p_E, p_D]` | position in NED | m |
| 3-5 | `v^b = [u, v, w]` | velocity relative to the ground, in body axes | m/s |
| 6-9 | `q_nb = [q_w, q_x, q_y, q_z]` | attitude quaternion | - |
| 10-12 | `ω^b = [p, q, r]` | body angular rate | rad/s |

Controls are `u = [δ_e, δ_a, δ_r, δ_t]` (elevator, aileron, rudder in rad; throttle
dimensionless in [0, 1]).

For linearisation and for the navigation filter the attitude is instead carried in a
**3-parameter tangent space**: the perturbed attitude is `q = q_ref ⊗ exp(δθ/2)` where `δθ`
is a body-frame rotation vector. This removes the norm constraint from the linear model, so
the 12-state `A` matrix has no spurious constrained mode.

---

## 3. Equations of motion

```
ṗ^n  = R(q_nb) v^b                                        (translational kinematics)

v̇^b  = (1/m) F^b − ω^b × v^b                              (Newton, rotating frame)

q̇_nb = ½ q_nb ⊗ [0, ω^b]ᵀ + k_q (1 − ‖q‖²) q_nb          (attitude kinematics)

ω̇^b  = J⁻¹ ( M^b − ω^b × (J ω^b) )                        (Euler's equation)
```

* The transport term `−ω^b × v^b` in the second equation is what makes the body-frame
  formulation correct; omitting it is a common and subtle error.
* The last term of the quaternion equation is **Baumgarte norm stabilisation**. It is exactly
  zero on the unit sphere and pulls numerical drift back towards it between the explicit
  renormalisations the integrator performs. `k_q = 1 s⁻¹` by default. A unit test integrates
  the aircraft for 10 s from a deliberately non-unit quaternion **without** renormalisation
  and requires the norm to return to 1 within 10⁻⁶.
* `J` is the full inertia tensor including the x-z product of inertia, so the roll-yaw
  inertial coupling is modelled:

```
        ⎡  Jx   0  −Jxz ⎤
  J  =  ⎢   0  Jy    0  ⎥
        ⎣−Jxz   0   Jz  ⎦
```

`AircraftParameters::validate()` rejects an inertia tensor that is not positive definite or
whose principal moments violate the triangle inequality `I₁ + I₂ ≥ I₃`.

### Forces and moments

```
F^b = F_aero^b + F_prop^b + R(q_nb)ᵀ [0, 0, m g]ᵀ
M^b = M_aero^b + M_prop^b
```

Gravity is a constant `[0, 0, mg]` in NED and is rotated into body axes; it therefore
contributes to `F^b` but never to `M^b` (it acts at the centre of mass).

An ideal accelerometer measures **specific force**, i.e. the non-gravitational part:

```
f^b = (F_aero^b + F_prop^b) / m
```

so a vehicle in free fall reads zero. This is checked by a unit test.

Implementation: `src/dynamics/RigidBody.cpp`.

---

## 4. Aerodynamics

### Air data

With `W^n` the total wind (steady + shear + gust) in NED,

```
v_rel^b = v^b − R(q_nb)ᵀ W^n
V_a     = ‖v_rel^b‖
α       = atan2(w_r, u_r)
β       = arcsin( v_r / max(V_a, V_min) )
q̄       = ½ ρ V_a²
```

`atan2(0, 0)` is 0 by IEEE-754, and the `arcsin` argument is clamped to `[-1, 1]`, so the air
data stays finite at rest. `V_min = 1 m/s` is used only to bound the *non-dimensionalising*
divisions; the dynamic pressure always uses the true `V_a`, so aerodynamic forces vanish
smoothly as `V_a → 0` rather than blowing up.

### Coefficient build-up

Rate derivatives are non-dimensionalised with `ĉ = c/(2V_a)` (longitudinal) and
`b̂ = b/(2V_a)` (lateral):

```
C_L = C_L(α) + C_Lq·q ĉ + C_Lδe·δ_e
C_D = C_D0 + C_L,lin(α)²/(π e AR) + C_Dδe·|δ_e| + C_Dq·q ĉ
C_m = C_m0 + C_mα·α + C_mq·q ĉ + C_mδe·δ_e

C_Y = C_Y0 + C_Yβ·β + C_Yp·p b̂ + C_Yr·r b̂ + C_Yδa·δ_a + C_Yδr·δ_r
C_l = C_l0 + C_lβ·β + C_lp·p b̂ + C_lr·r b̂ + C_lδa·δ_a + C_lδr·δ_r
C_n = C_n0 + C_nβ·β + C_np·p b̂ + C_nr·r b̂ + C_nδa·δ_a + C_nδr·δ_r
```

### Post-stall blend

Lift uses the sigmoid blend of Beard & McLain (2012, eq. 4.9-4.10) between the linear model
and a flat plate:

```
σ(α) = (1 + e^{−M(α−α₀)} + e^{M(α+α₀)}) / ((1 + e^{−M(α−α₀)})(1 + e^{M(α+α₀)}))
C_L(α) = (1 − σ)(C_L0 + C_Lα α) + σ · 2 sign(α) sin²α cos α
```

with `α₀ = 17.2°` and `M = 50 rad⁻¹`. The exponents are clamped before `exp` so the
expression cannot overflow at large `|α|`. Induced drag deliberately uses the **linear**
`C_L` in the quadratic polar, so drag keeps growing past the stall instead of collapsing with
the blended lift.

### Body-axis forces

Lift and drag are built in stability axes and rotated into body axes through `α`; the side
force acts along body `y`:

```
f_x =  q̄S(−C_D cos α + C_L sin α)
f_y =  q̄S C_Y
f_z =  q̄S(−C_D sin α − C_L cos α)
m_x =  q̄Sb C_l,   m_y = q̄Sc C_m,   m_z = q̄Sb C_n
```

This is the standard small-UAV formulation (Beard & McLain 2012, eq. 4.18-4.19). It is exact
at `β = 0` and accurate to `O(β²)` otherwise; the alternative is a full wind-to-body rotation,
which would require the side-force derivative to be defined in wind axes instead.

Note that `f_x > 0` at positive `α` is **not** a bug: body `x` is not the relative-wind
direction, and the forward tilt of the lift vector can exceed the drag component. The unit
test checks the physically meaningful statement, namely that the aerodynamic force projected
onto the relative-wind direction is negative.

### Sign conventions

| Input | Positive means | Primary effect |
|---|---|---|
| `δ_e` | elevator trailing edge **down** | nose-down pitching moment (`C_mδe < 0`), more lift (`C_Lδe > 0`) |
| `δ_a` | right-roll command | positive (right) rolling moment (`C_lδa > 0`) |
| `δ_r` | rudder trailing edge **left** | side force to the **right** (`C_Yδr > 0`), nose-**left** yawing moment (`C_nδr < 0`) |
| `δ_t` | more power | thrust along body `x` |

Every one of these is asserted in `tests/test_dynamics.cpp`. The rudder pair is the one most
often written inconsistently in published tables: a rudder aft of the CG that pushes the tail
to the right *must* yaw the nose to the left, so `C_Yδr` and `C_nδr` have **opposite** signs.

Implementation: `src/dynamics/Aerodynamics.cpp`.

---

## 5. Propulsion

Actuator-disk (momentum theory) model:

```
T   = ½ ρ S_prop C_prop [ (k_motor δ_t)² − V_a² ]
M_x = −k_Tp (k_Ω δ_t)²
```

The thrust expression goes negative when the airspeed exceeds the slipstream exit velocity,
which models propeller windmilling drag. `M_x` is the motor reaction torque: a right-handed
propeller rolls the airframe to the left, which the trim solver compensates with a small
aileron and bank offset (0.14° aileron, 0.009° bank at the nominal condition). A thrust line
offset from the CG contributes `r × F`.

Implementation: `src/dynamics/Propulsion.cpp`.

---

## 6. Atmosphere and wind

**Atmosphere** — 1976 U.S. Standard Atmosphere. Troposphere (0-11 km): linear lapse
`T = T₀ − Lh` with `p = p₀(T/T₀)^{g/(LR)}`. Lower stratosphere (11-20 km): isothermal. The
unit test checks the model against published ISA values at 0, 5 and 11 km.

**Steady wind and shear** — a logarithmic boundary-layer profile scales the configured wind
by `ln(h/z₀) / ln(h_ref/z₀)`, with `z₀ = 0.05 m` (open grassland) by default.

**Turbulence** — Dryden shaping filters (MIL-F-8785C) driven by white noise, expressed in
body axes:

```
H_u(s) = σ_u √(2V_a/L_u) / (s + V_a/L_u)
H_{v,w}(s) = σ √(3V_a/L) (s + V_a/(√3 L)) / (s + V_a/L)²
```

Each filter is realised in state space and propagated with an **exact** discretisation (Van
Loan), so the stationary variance is `σ²` independently of the step size. A unit test
measures the achieved standard deviation over 4×10⁵ samples at two different step sizes and
requires both within 8% of the configured `σ`.

With `altitude_scaled: true` the intensities and length scales follow the MIL-F-8785C
low-altitude model (`σ_w = 0.1 W₂₀`, `L_w = h`, `L_u = L_v = h/(0.177 + 0.000823h)^1.2`, with
`h` in feet).

Implementation: `src/env/Atmosphere.cpp`, `src/env/Wind.cpp`.

---

## 7. Actuators

Each channel is a first-order lag `δ̇ = (δ_cmd − δ)/τ` whose slew is clipped to a rate limit
and whose position is clipped to the magnitude limits. Commands are saturated **before** the
lag, so the internal state never integrates an unreachable demand. Defaults: `τ = 0.04 s` and
300°/s for the surfaces, `τ = 0.20 s` and 2 s⁻¹ for the throttle.

---

## 8. Trim

`TrimSolver` solves for steady flight with Levenberg-Marquardt on the square system

* unknowns `z = [α, β, φ, θ, δ_e, δ_a, δ_r, δ_t]` (8),
* residuals `r = [u̇, v̇, ẇ, ṗ, q̇, ṙ, ḣ − V_a sin γ, β − β*]` (8).

Steady flight means the body-frame velocity and angular rate are constant, so all six dynamic
residuals vanish. For a coordinated turn the body rate is set from the requested turn rate,
`ω^b = [−ψ̇ sin θ, ψ̇ cos θ sin φ, ψ̇ cos θ cos φ]ᵀ`. Trim is always computed in still air.

The eight physical residuals are augmented with six one-sided penalties that keep the search
inside the actuator box and inside a sensible `α`/`β` envelope. They are **exactly zero** on a
feasible point, so a reachable condition converges to the same answer with or without them;
they matter only when the requested condition is not reachable. Without them the optimiser
minimises an impossible residual by wandering out of the box — a 30 m/s climb at 5° was
reported with a 44° rudder deflection and 26° of sideslip. (That condition is in fact
perfectly reachable at 89% throttle; the penalties are what let the solver find it.)

A converged trim whose controls still sit on an actuator stop is reported as **not**
converged: it is not a usable equilibrium. This is how the solver reports a genuinely
unreachable flight condition such as a 40° climb.

Achieved residual at the nominal condition: **6.1×10⁻¹⁴** (infinity norm), in 8 iterations.

---

## 9. Linearisation

`linearize()` takes central differences in the 12-dimensional tangent space
`δx = [δp^n, δv^b, δθ, δω^b]` and in the 4 control inputs. The attitude row uses the exact
tangent velocity

```
d(δθ)/dt = ω − R(exp(δθ))ᵀ ω_ref
```

which reduces to `δθ̇ = δω` about a non-rotating reference. (The right-Jacobian correction
`J_r⁻¹(δθ)` is omitted: for a straight-flight reference it contributes exactly zero, because
perturbing `δθ` alone leaves `ω = 0`, and perturbing `δω` alone leaves `J_r = I`.)

The 12-state model is then projected onto the classical decoupled sets:

* longitudinal `[δu, δw, δq, δθ_y]` with inputs `[δ_e, δ_t]`;
* lateral `[δv, δp, δr, δθ_x]` with inputs `[δ_a, δ_r]`.

Verification: a unit test compares `A δ` against the finite difference of the nonlinear model
for `ε ∈ {10⁻², 10⁻³, 10⁻⁴}` and requires the mismatch to fall by at least a factor of 20 per
decade, confirming second-order agreement. A second test integrates both models for 1 s after
a 0.5° elevator step and requires agreement within 2%.

---

## 10. Control

### Successive loop closure (PID)

```
course  → bank command  → aileron
altitude → pitch command → elevator
airspeed → throttle
sideslip → rudder, plus a washed-out yaw-rate damper
```

Six PID loops, each with output saturation, derivative filtering, and anti-windup by both
conditional integration and back-calculation. The altitude command is slew limited to 4 m/s
so a large step cannot saturate the pitch loop, and the integrators are preloaded from the
trim condition so the cascade starts out holding trim instead of ramping up to it.

### Servo-LQR

Two designs are synthesised from the reduced linear models at run time (not hard-coded gains):

* **Longitudinal**, state `[δu, δw, δq, δθ, h−h_c, ∫(h−h_c), ∫(V_a−V_{a,c})]`, inputs
  `[δ_e, δ_t]`. The altitude row is the exact linearisation of `ḣ = u sin θ − w cos θ`
  about the trim attitude; the airspeed row uses `δV_a = (u₀δu + w₀δw)/V_{a0}`.
* **Lateral**, state `[δv, δp, δr, δφ, χ−χ_c, ∫(χ−χ_c)]`, inputs `[δ_a, δ_r]`, with the
  **coordinated-turn** course kinematics `χ̇ ≈ (g/V_{a0}) δφ`.

The Riccati equations are solved with Roberts' matrix-sign-function method plus
Kleinman-Newton polishing (`aether::math::solveCare`); the achieved residuals are
`‖AᵀP + PA − PBR⁻¹BᵀP + Q‖_F ≈ 10⁻¹³`.

> **Why the course row uses bank, not yaw rate.** The kinematically obvious choice,
> `χ̇ = r/cos θ₀`, tells the optimiser that the *rudder* is the direct actuator for course.
> It then steers with the rudder, which is aerodynamically the wrong actuator for a
> fixed-wing aircraft: the first version of this code produced a 20°+ rudder deflection and a
> 9° steady sideslip in every turn, and departed controlled flight at the sharpest corner.
> Using the coordinated-turn relation makes bank the course actuator and leaves the rudder to
> damp the dutch roll and coordinate the turn.

### LQR command limiting

The error states are saturated so the feedback law cannot demand an attitude outside the
vehicle's handling limits. With the aileron near its balance point the closed loop settles
where the bank term cancels the course terms, `K_φ φ = K_χ χ_err + K_∫ ∫χ_err`, so clamping

```
|χ_err|   ≤ (1 − a) φ_max K_φ / K_χ
|∫χ_err|  ≤      a  φ_max K_φ / K_∫
```

makes that balance point exactly `φ_max`, with the fraction `a = 0.25` of the bank budget
reserved for integral (trim) authority. The altitude channel uses the same split against the
pitch budget. These limits are **computed from the synthesised gains**, not guessed, and are
tested directly. Measured peak bank in the nominal run: 40.2° against a 35° limit, i.e. 13%
transient overshoot on the sharpest corner.

### Angle-of-attack envelope protection

Above `α_max = 13.8°` a protective nose-down demand is blended in:

```
δ_e,prot = k_α (α − α_max) + k_q q,        weight = min(1, (α − α_max)/Δ)
δ_e      = δ_e + weight · max(0, δ_e,prot − δ_e)
```

and the **course error is faded out by the same weight**, so the aircraft widens the turn
rather than trading its last stall margin for course tracking (in a turn it is the load factor
that drives `α`).

Both details matter and both were found by the Monte-Carlo campaign:

* a pure proportional override without the `k_q q` term switches on and off at the
  short-period frequency and sets up a bang-bang limit cycle that reached ±40° of `α`;
* without the bank fade, 12 of 256 dispersed trials still stalled in the turns.

---

## 11. Guidance

Straight-line path following with the standard vector field (Beard & McLain 2012, Ch. 10):

```
χ_c = χ_q − χ_∞ (2/π) arctan(k_path · e_py)
```

where `e_py` is the signed cross-track error and `χ_q` the course of the active leg. The
active waypoint advances when the vehicle crosses the half-plane whose normal bisects the
incoming and outgoing legs, or when it comes within the capture radius — whichever happens
first — and the decision is taken **before** the command is computed, so there is no one-step
lag between passing a waypoint and steering towards the next one. In `loop` mode the index
wraps to 0 so the closing leg back to the first waypoint is flown as well.

Altitude is interpolated linearly along the active leg, spreading altitude changes over the
leg instead of stepping at the waypoint. Terminal behaviour is `loop`, `orbit` (loiter circle
around the final waypoint) or `hold`.

---

## 12. Navigation

An 18-state **error-state (indirect) EKF**.

Nominal state `[p^n, v^n, q_nb, b_g, b_a, w_ne, b_baro]` (19 scalars); error state
`δx = [δp, δv, δθ, δb_g, δb_a, δw, δb_baro]` (18 scalars), with the attitude error defined
multiplicatively in the **body** frame.

Propagation is driven by the IMU — the inertial measurements are *inputs*, not measurements:

```
δṗ = δv
δv̇ = −R̂ [a_m − b̂_a]_× δθ − R̂ δb_a − R̂ n_a
δθ̇ = −[ω_m − b̂_g]_× δθ − δb_g − n_g
δḃ_g = n_bg,   δḃ_a = n_ba,   δẇ = n_w,   δḃ_baro = n_b
```

Measurements:

| Source | Model | Rate |
|---|---|---|
| GNSS | `z = [p^n; v^n]` | 5 Hz |
| Barometer | `z = −p_D + b_baro` | 20 Hz |
| Magnetometer | `z = R(q)ᵀ m^n`, `H_θ = [ẑ]_×` | 50 Hz |
| Pitot | `z = ‖v^n − w^n‖` | 50 Hz |

Numerical hygiene: the Joseph form keeps the covariance symmetric and positive semi-definite
even with a suboptimal gain; every update is followed by an explicit symmetrisation; the
attitude-error reset applies the Jacobian `G = blkdiag(I, I, I − ½[δθ]_×, I, I, I)`; and a
chi-squared gate rejects gross outliers.

**Why the wind states exist.** The control laws feed back *air-relative* velocity, because
that is what the still-air trim linearisation is about. Without a wind estimate, a crosswind
crab is indistinguishable from a sideslip: in a 10 m/s crosswind the reconstructed `β` is
wrong by ~28°, which is enough to upset the aircraft within the first second. The scalar
pitot measurement makes the along-track wind component observable immediately and the
cross-track component observable as soon as the heading changes — the classical wind
triangle. The filter additionally seeds the wind from the very first airspeed reading via
`w = v^n − R(q̂)[V_a, 0, 0]ᵀ`, which is exact at `α = β = 0`.

**Why the barometer-bias state exists.** The static-pressure offset is drawn once per run
(σ = 2 m) and is otherwise unmodelled. Without a state for it the altitude estimate inherits
the offset while the covariance keeps shrinking — the filter is confidently wrong — and the
autopilot then flies to a biased altitude. Adding the state moved the closed-loop altitude
tracking error from 2.04 m RMS to **1.05 m RMS** and the estimator's vertical position RMSE
from 1.93 m to **0.84 m**.

---

## 13. Synthetic parameter derivation

> **The shipped coefficients describe no real aircraft.** They are labelled `synthetic: true`
> in `configs/aircraft/aether6_uav.yaml` and in every console banner. The *model structure*
> follows the standard textbook formulation cited below; the *numbers* were derived here, so
> that every sign and magnitude is internally consistent and no published table is
> misattributed.

Geometry chosen for a plausible 13.5 kg airframe: `S = 0.52 m²`, `b = 2.60 m`,
`c = S/b = 0.20 m`, `AR = 13.0`. Assumed tail geometry: `S_h = 0.11 m²`, `S_v = 0.055 m²`,
`l_h = l_v = 0.70 m`, `a_h = 3.9 rad⁻¹`, `a_v = 2.8 rad⁻¹`, `η = 0.95`, surface effectiveness
`τ = 0.55`, downwash `dε/dα = 0.25`, fin height above CG `z_v = 0.12 m`.

Tail volume coefficients: `V_H = S_h l_h/(S c) = 0.740`, `V_V = S_v l_v/(S b) = 0.0285`.

| Coefficient | Relation used | Value |
|---|---|---|
| `C_Lα` | finite-wing lift slope at AR 13 | 5.00 |
| `C_Lq` | `2 η a_h (S_h/S)(l_h/c)` | 5.50 |
| `C_Lδe` | `η a_h τ (S_h/S)` | 0.43 |
| `C_mα` | static margin 0.24 c | −1.20 |
| `C_mq` | `−2 η a_h (S_h/S)(l_h/c)²` | −19.0 |
| `C_mδe` | `−η a_h τ V_H` | −1.50 |
| `C_Yβ` | `−η (S_v/S) a_v` plus a fuselage contribution | −0.400 |
| `C_Yr` | `2 η (S_v/S) a_v (l_v/b)` | 0.150 |
| `C_Yδr` | `η (S_v/S) a_v τ` | 0.155 |
| `C_nβ` | `η V_V a_v` | 0.076 |
| `C_nr` | `−2 η V_V a_v (l_v/b)` plus profile drag | −0.060 |
| `C_nδr` | `−η V_V a_v τ` | −0.042 |
| `C_lδr` | `C_Yδr · z_v/b` | 0.007 |
| `C_lp` | strip theory for AR 13 | −0.520 |
| `C_lr` | `≈ C_L/4` at cruise plus the fin term | 0.120 |

`C_D0 = 0.030` and `e = 0.85` give `L/D = 15.6` at the cruise condition, which is reasonable
for a high-aspect-ratio UAV. The propulsion constants give 67% cruise throttle at 25 m/s and
a static thrust-to-weight ratio of 0.47.

The resulting airframe produces the classical flight modes with realistic values — short
period `ω_n = 6.99 rad/s, ζ = 0.40`; phugoid period 12.5 s against the Lanchester estimate
`π√2 V/g = 11.3 s`; dutch roll `ω_n = 5.11 rad/s, ζ = 0.117`; roll subsidence time constant
0.06 s; and a mildly divergent spiral with a 19 s doubling time. That agreement is the
evidence that the synthetic set is physically coherent; it is asserted in
`tests/test_trim_linearize.cpp`.

---

## References for the model *structure*

1. R. W. Beard and T. W. McLain, *Small Unmanned Aircraft: Theory and Practice*, Princeton
   University Press, 2012 — Ch. 3-5 (6-DOF model, aerodynamics, wind), Ch. 6 (autopilot),
   Ch. 10-11 (path following, waypoint switching).
2. B. L. Stevens, F. L. Lewis and E. N. Johnson, *Aircraft Control and Simulation*, 3rd ed.,
   Wiley, 2016 — Ch. 1-2 (frames, rigid-body equations), Ch. 4 (trim and linearisation).
3. MIL-F-8785C, *Flying Qualities of Piloted Airplanes*, 1980 — Dryden turbulence spectra and
   the low-altitude intensity/scale model.
4. J. Solà, *Quaternion kinematics for the error-state Kalman filter*, arXiv:1711.02508, 2017 —
   error-state formulation, injection and covariance reset.
5. E. Hairer, S. P. Nørsett and G. Wanner, *Solving Ordinary Differential Equations I*,
   2nd ed., Springer, 1993 — Dormand-Prince 5(4) coefficients and the PI step-size controller.
6. J. D. Roberts, *Linear model reduction and solution of the algebraic Riccati equation by
   use of the sign function*, Int. J. Control 32(4), 1980.
7. C. Van Loan, *Computing integrals involving the matrix exponential*, IEEE TAC 23(3), 1978.
8. G. H. Golub and C. F. Van Loan, *Matrix Computations* — Algorithm 11.3.1, scaling and
   squaring with a Padé approximant.
