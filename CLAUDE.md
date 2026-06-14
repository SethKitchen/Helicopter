# helisim — project guide

A helicopter simulator built **from first principles** in Rust. The near-term
use case is designing a model electric helicopter; the architecture must scale
up to a human-usable electric helicopter.

## Roadmap (build in this order)

Aero track:
1. **Hover BEMT solver** — *complete.* Single rotor, rigid blades, steady hover.
   Validated against Caradonna & Tung (1981) and Harrington (1951).
2. **Forward flight** — *complete.* Glauert momentum inflow + azimuthal
   blade-element integration on a rigid blade. Surfaces the advancing/retreating
   asymmetry and the resulting large uncommanded **rolling moment** (the result
   that motivates flapping). Validated by the Glauert inflow closed form and the
   power-bucket shape. Uniform inflow, reverse-flow lift nulled — no flapping yet.
3. **Blade flapping dynamics** — *complete.* Rigid-blade first-harmonic flapping
   solved by harmonic balance (3×3 linear system). The rotor is now a dynamic
   system: the rigid-blade rolling moment is reacted by flapping (hub moment → 0
   for a central hinge; residual ∝ hinge offset) and reappears as a tip-path-plane
   tilt. The gyroscopic 90° phase lag emerges on its own. Validated against the
   Lock-number closed form. Cyclic pitch inputs supported (sets up trim).
4. **Trim (5a)** — *complete.* Six-unknown / six-equation steady-flight trim
   (θ₀, θ₁c, θ₁s, tail collective, fuselage pitch & roll) by multidimensional
   Newton with a numerical Jacobian. Reuses hover BEMT, forward inflow and
   flapping as the residual. Hover and steady level forward flight. The capstone
   cross-check passes: hover trim reproduces the milestone-1 hover-BEMT collective
   and power to ~0.3%.
5. **Two-way coupling + parasite (5b)** — *complete.* Flap↔inflow converged
   together (a nested fixed point in the trim residual) + airframe parasite power.
   Fixes the high-μ collapse: trimmed power is now positive and physical past
   μ=0.25, and the *complete* power bucket (hover-high → interior minimum →
   parasite rise) appears for the first time. Prerequisite to 6-DOF (equilibria
   are now trustworthy across the speed range).
6. **Linearized dynamics (5c)** — *complete.* Stability & control derivatives by
   perturbing the trimmed equilibrium, assembled into the longitudinal system
   matrix; eigenvalues from a std-only solver. Reproduces the open-loop hover
   instability (unstable oscillatory pitch–speed mode) without it being put there.
7. **Nonlinear longitudinal time-marching (5d)** — *complete.* RK4 on the
   nonlinear longitudinal EOM with the rotor solved quasi-statically *inside the
   integration loop*. Validated against the pre-computed 5c eigenvalues: a
   perturbed hover reproduces the predicted period (7.1 vs 6.97 s) and growth
   (σ 0.503 vs 0.505), tracks the linear model in the small, and departs into
   nonlinearity as the unstable oscillation grows. Longitudinal only.
8. **Lateral-directional oracle (5e-i, corrected in 5f)** — *complete.* Perturb
   hover in `[v,p,r,φ]`; the **tail rotor enters as a dynamic element** (Nr, Yv,
   Nv). Eigenvalues: an oscillatory instability (lateral phugoid) + stable
   roll/yaw subsidences. Signs (Lp<0, Nr<0, Yv<0) and the roll-sideslip cubic pass.
9. **Coupled 8-state (5e-ii)** — *complete.* 8×8 `[u,w,q,θ,v,p,r,φ]`. Decouple
   gate: zeroing the cross-blocks reproduces the 5c ∪ 5e-i eigenvalues exactly;
   coupling shifts every mode (pitch-roll) and a longitudinal disturbance excites
   lateral motion (linear time-march confirms it).
10. **Rotation-based lateral aero (5f)** — *complete.* The main-rotor lateral
    response is the exact rotation of the *validated longitudinal* response —
    **velocity perturbations rotate +90°, rate perturbations −90°**, so
    `Lv=−Mu, Yv=Xu, Lp=Mq`. This FIXED a sign bug: the original axisymmetry-by-
    assertion used `Lv=+Mu`, which had made the lateral mode look like a
    divergence; corrected, it's the oscillatory lateral phugoid. Validated: the
    rotation path reproduces the longitudinal oracle across amplitudes.
11. **General-state aero fix (5g precondition)** — *complete.* full_aero is now
    fully equivariant for *combined* states (the fy and hub-roll signs were the
    bug); validated by the combined-axis rotation test (rotate a state with
    simultaneous v,w,p,q → response rotates, to 1e-6).
12. **Nonlinear coupled 8-state march (5g)** — *complete.* All `[u,w,q,θ,v,p,r,φ]`
    integrated with both rotors in the loop. 6-variable hover equilibrium (incl.
    roll φ_e for the tail side force) is an exact fixed point (drift 2.7e-11/6 s);
    the nonlinear EOM's numerical Jacobian matches the coupled 8×8 model (two
    independent routes); a Δu=0.1 m/s perturbation tracks the 8-D linear
    prediction <5% through ~4 s then departs as both instabilities compound.
13. **Pitt–Peters dynamic inflow (5h)** — *complete.* The rotor inflow is now a
    three-state finite-state model `ν=[λ₀,λ₁s,λ₁c]` with its own dynamics
    `[M]ν̇+[L]⁻¹ν=C` (apparent mass `[M]=diag(8/3π,−16/45π,−16/45π)`; Pitt–Peters
    `[L]` with wake-skew coupling). **Architecture shift:** rotor inflow moves
    from an *inner fixed-point* (solved inside the aero call) to *outer integrated
    state* — three extra states to march. Two clean gates (the inflow states are
    internal, no standalone oracle): **(1)** zeroing the cyclic states recovers
    the 5g uniform-inflow baseline bit-for-bit, and marching with lag→0 collapses
    onto the quasi-static fixed point (Δν 4e-13); **(2)** the cyclic inflow flips
    the sign of the off-axis cyclic response `∂My/∂θ1c` (−3.2 frozen → +0.5
    solved) — the documented "wrong sign of off-axis response to cyclic". Gravest
    inflow mode τ≈0.085 s (~2 revs), the literature's O(1-rev) lag scale. `helisim
    inflow`.
14. **Control-input time histories (5i)** — *complete.* The aircraft is now
    *driven*: an 11-state march `[u,w,q,θ,v,p,r,φ,λ₀,λ₁s,λ₁c]` with the Pitt–Peters
    inflow **in the loop** (the reason 5h came first) and time-varying controls
    (collective/lat-cyc/lon-cyc/pedal as deltas from trim, behind a
    `ControlSchedule` trait: `Step`/`Pulse`/`Trim`). Validation shifts from
    "matches a number" to "responds correctly": **(1)** control effectiveness `B`
    has the right signs (collective→climb, +lat-cyc→right-roll, lon-cyc→pitch,
    pedal→yaw) and ~20:1 on-axis dominance; **(2)** the off-axis cyclic response
    `∂My/∂θ1c` flips from −3.18 (inflow frozen at t=0) to +0.53 (inflow developed)
    — the 5h sign flip now a *time-domain* effect; **(3)** open-loop the aircraft
    diverges to a control pulse (controls released, never returns — the 5j setup),
    with `u` tracking the linear model in a bounded analytic window. The 11-state
    modes preserve the body modes (0.481±0.878i, 0.701±1.331i ≈ 8-state) and add
    three fast stable inflow modes. **Named finding:** the hover divergence runs
    *faster* than the hover-linearized rate because the wake skew χ(μ) is
    non-analytic at μ=0 (μ=|V|≥0 rectified) — the SAME λ₀↔λ₁c coupling behind the
    off-axis flip escapes the hover Jacobian. `helisim fly`.
15. **Stability augmentation (5j)** — *complete.* A rate-feedback SCAS damper
    (p→lat-cyclic, q→lon-cyclic, r→pedal) on the 11-state system. Designed and
    validated in **three layers around the wake-skew seam** (the 5i lesson made a
    design principle): **(1) off the seam** at small forward speed (5 m/s, μ≈0.04)
    the Jacobian is differentiable and linear↔nonlinear agree, so closed-loop
    eigenvalues in the LHP are a *trustworthy* gate (open 0.54 → closed −0.005;
    nonlinear tracks the closed-loop linear model in the pitch-rate channel to
    <1%); **(2) hover linear** the damper collapses the violent instability (0.70 →
    +0.024, doubling 1 s → 30 s) but a small positive residual remains — rate
    feedback is necessary, not sufficient (the slow speed/phugoid mode it can't
    reach); **(3) across the seam** the same gains turn the open-loop hover
    divergence (blows up to NaN) into a bounded nonlinear response (attitude <3.5°
    over 8 s), *including* the pitch/lateral-rate channel the hover Jacobian could
    not see. A damper, not a hold (named scope cap). `helisim sas`.
16. **Attitude hold (5k)** — *complete.* A proportional outer attitude loop
    (θ→lon-cyclic, φ→lat-cyclic) wrapping the 5j rate damper — the standard
    inner-rate/outer-attitude cascade, kept to *regulate-to-trim* (NOT command
    tracking or guidance — named scope cap). New validation character: **regulation**
    ("drive an error to zero and hold it"). **Pre-computed target hit:** the slow
    phugoid the rate damper left at +0.024 at hover moves to **−0.188** once the
    attitude loop closes — the loop with authority over it. Seam discipline's
    *second* application: **(off-seam, trustworthy)** at 5 m/s the closed loop stays
    LHP and the nonlinear march RETURNS to trim and holds (θ 5°→<0.4°), the damper
    doesn't; **(across the seam, honest)** at hover it beats the damper (which
    diverges to NaN) and holds pitch/roll bounded, but a slow residual drift remains
    (surfaces in yaw) — the SAME wake-skew coupling the hover Jacobian can't see.
    Sustained-disturbance: regulated to a bounded offset where the damper diverges
    (proportional ⇒ residual offset; integral would zero it). `helisim attitude`.
17. **PI attitude hold (5l)** — *complete.* Integral action closing 5k's residual
    steady-state error — a correctness fix to the attitude loop, done *before* the
    velocity loop so the outer loop isn't built on an inner loop with a known
    standing error. Two integrator states `z=[∫(θ−θ_e),∫(φ−φ_e)]` (state vector →
    13). **Falsifiable oracle hit:** under a sustained 0.6 N·m disturbance the
    proportional standing offset (1.69°) goes to **≈0 (0.09°)** with integral
    action — textbook zero-steady-state-error. Off-seam the 13-state augmented loop
    stays stable but only **marginally** (max Re ≈ −0.001 — the integrator's own
    near-origin pole); firm damping margin is the velocity loop's job (5m). **Scope
    boundary made concrete:** attitude error → 0 but the forward speed *drifts*
    (~1.6 m/s) — the disturbance-countering thrust tilt accelerates the aircraft;
    attitude hold ≠ velocity hold. That residual drift is exactly what 5m closes.
    Anti-windup named (not needed at these amplitudes). `helisim attitude`.
    ⚠ The marginal value was found only after the eigensolver moved char-poly→QR
    (item below); the char-poly's −0.062 and the "kI lower-bound" reading were
    artifacts. Lesson banked: *don't trust the char-poly eigensolver past ~10×10.*
18. **QR eigensolver (5m precondition)** — *complete.* Eigenvalues now come from the
    QR algorithm (Hessenberg `elmhes` + Francis double-shift `hqr`,
    `dynamics/schur.rs`) instead of the characteristic-polynomial route
    (Faddeev–LeVerrier + Durand–Kerner), which is ill-conditioned past ~10×10 and
    gave *wrong* eigenvalues for the 13–15-state augmented control systems.
    Validated against a known 15×15 spectrum (reals over 3 decades + complex pairs,
    one unstable). `eigenvalues` routes through it; `eigenvalues_via_char_poly` kept
    for the small analytic anchors.
19. **Velocity/position hold (5m)** — *complete.* The outermost cascade: velocity
    error → attitude command → the 5k/5l attitude loop → the 5j rate loop. For
    hover-hold the velocity-error integrator IS position (15 states: plant 11 +
    attitude integrators 2 + velocity/position integrators 2). **Timescale
    separation (named before tuning):** three eigenvalue clusters — rate/inflow
    |λ|≈8–74, attitude |λ|≈1.3, velocity |λ|≈0.22 — separated **6.7× and 5.8×**
    (≥3× target; the loops don't fight). **Pre-computed tracking target hit:** the
    ~1.6 m/s drift 5l left under 0.6 N·m is driven to **≈0** (0.019 m/s). **Capstone
    (across the seam):** at hover, from a velocity kick, the *open-loop-unstable-in-
    both-axes* aircraft arrests the drift, returns to station (pos→≈0) and holds it
    hands-off, and the yaw/wake-skew seam-residual that survived 5j/5k/5l no longer
    runs away — position feedback finally has authority over the slow drift the
    inner loops were blind to. Anti-windup still not needed (θ_cmd ≈ 0.8°, modest).
    Scope: hold/steady-command only. `helisim hover`.
20. **External validation (Milestone 6)** — *next; a CATEGORY CHANGE.* Every gate
    so far has been *internal* (closed forms, reduction-to-known-case, derivative
    signs, route-vs-route). The stack is internally coherent end-to-end — earned,
    but distinct from *external accuracy*. Scope: match published data for ONE
    documented aircraft (Bo-105 or UH-60 — pick the one whose FULL parameter set
    *and* results are findable in the open literature: Padfield, GARTEUR/ADS-33).
    Order by least-approximated-first so a match is meaningful and a mismatch
    diagnostic: **(1) trimmed control positions vs. airspeed** (forgiving of inflow
    detail; tests the force/moment balance validated hardest); **(2) stability
    derivatives** (Mu, Mq, Zw, Nr…) vs. published, where κ/uniform-inflow error
    first shows. **Predict before comparing** which derivatives match (force/moment,
    clean) vs. carry error (power-related via κ; high-μ via uniform inflow; anything
    needing elastic-blade modes) — turning the comparison into a falsifiable test of
    our understanding of the model's OWN error structure. A mismatch is not failure:
    it *measures* the deliberately-scoped approximations (κ, uniform inflow, rigid
    blade, first-harmonic flap), each named and isolated so the result is diagnostic
    rather than a single uninterpretable "off by 20%". **Hard rule applies: never
    fabricate oracle values — source and cite, or say so.**
    *Status:* predictions recorded before comparison (`validation/MILESTONE6_PREDICTIONS.md`);
    aircraft chosen = **UH-60A**, dataset sourced from one public citable
    apples-to-apples report — NASA TM 85890 (GENHEL), holding both parameters and the
    oracle (Table 4 trim positions, Tables 12+ derivatives). Captured:
    `validation/UH60_GENHEL_TM85890.md` (+ UH-60-specific caveats: canted TR,
    stabilator, SC1095). Rejected the flybar R-50 (unrepresentable) and the
    export-restricted Fletcher TM.
    *First comparison DONE — hover longitudinal derivatives* (`Aircraft::uh60()`,
    `dynamics/tests/uh60_external_validation.rs`, results in `validation/MILESTONE6_RESULTS.md`):
    all four signs match the real aircraft (incl. Mu>0); **Zw lands at 12%** (within
    the predicted band — core heave physics externally confirmed) and **Mu is right
    sign + order** (speed-stability physics confirmed). **The finding: Mq (predicted
    clean) is ~65× too small** — right sign, badly under-predicted — caught by external
    data where internal sign/self-consistency couldn't; attributed (with humility) to
    the quasi-static first-harmonic flap discarding the flap-lag/precession part of
    hover pitch damping. The per-approximation error budget the methodology promised:
    uniform inflow *cheap* for Zw, quasi-static flap *expensive* for Mq.
    *Lateral/directional discriminator DONE* (pre-registered prediction, confirmed):
    **Nr = 1.5%** (tail-rotor-based — definitively rules out a units/assembly bug, since
    Nr uses no main-rotor flap), Yv 6%, Nv 13%; **Lp ~17× too small, the SAME deficit as
    Mq, measured cleanly** (Lp oracle non-degenerate, roll has no stabilator confound).
    **Final diagnosis:** signs all correct and magnitudes within ~6–35% everywhere EXCEPT
    the main-rotor-flap RATE damping (Mq, Lp), under-predicted ~15–30× and *travelling
    with the flap across axes*. Decomposition localizes it to the in-phase β1c response
    to body rate (`dβ1c/dq̄=−0.071` vs `dβ1s/dq̄≈−1`); leading (un-asserted) hypothesis:
    missing gyroscopic/kinematic body-rate coupling in the flap equation. The external
    comparison localized a real model gap to one term in one sub-model — the diagnostic
    resolution the internal chain was built for.
    *Flap-damping fix DERIVED + IMPLEMENTED + VALIDATED* (pre-registered in
    `validation/MILESTONE6_FLAP_FIX_PREREG.md`): the missing **gyroscopic
    "rotor-follows-shaft" coupling** of hub rate into flap (`FlapProperties.gyro_rate`,
    added to `aero.rs`/`flap_general.rs` forcing). Derivation: spin angular momentum
    rotated by hub rate gives an O(rate) sinψ/cosψ forcing feeding the in-phase β1c/β1s.
    Coefficient **2** (textbook, NOT fitted → dβ1c/dq̄=−1.87); sign **−2** mandated by
    gyroscopic-damping physics (+2 gives anti-damping, so sign is physical not flipped).
    Result: **Lp −0.19 → −3.25 vs oracle −3.35 = 3%** (clean axis, was ~17× short); the
    **regression is bit-for-bit perfect** (Zw, Mu, Xu, Nr, Yv, Nv all unchanged — the
    term touches only rate damping). Mq→−0.45 (residual vs −1.03 = the omitted
    stabilator). **Adoption: `gyro_rate` defaults to 0** so all prior milestones are
    unchanged (150 tests pass); `uh60()` uses −2. Universal default = a deliberate
    5c–5m revalidation step (it changes the demo dynamics). NEXT: trim-position
    comparison (needs control rigging); optionally adopt gyro_rate=−2 globally + revalidate.

Electric-powertrain track — **COMPLETE & trustworthy** (don't revisit before aero):
- **Cell → pack → powertrain → thermal → mission.** Couples BEMT shaft power
  through a constant-η driveline into a Thévenin cell/pack, solves the coupled
  constant-power current/voltage (same bisection pattern as the inflow),
  integrates SoC to hover endurance, and runs a lumped-mass thermal model
  (I²R heat in, convective cooling out) to peak cell temperature. Answers both
  *can this pack hover this aircraft, at what C-rate, for how long* and *does a
  sustained climb cook the cells.* Validated against the Samsung INR18650-25R
  datasheet (capacity + voltage sag) and 18650 thermal characterization
  (specific heat, 75 °C protection behaviour).

Safety & design track — **COMPLETE** (the power-off + noise + sizing layer the
powered-flight stack omitted; built off the validated cores, never modifies them):
- **Autorotation (power-off).** The whole aero/control stack assumes a *driven*
  rotor; this adds the regime it never visits — air coming *up* through the disk.
  Descent-regime inflow: exact momentum closed forms for the climb and
  windmill-brake states (each validated to 1e-9 against its own momentum quadratic)
  + the **measured** vortex-ring/turbulent-wake curve (cited Leishman quartic),
  because momentum theory is *physically invalid* there and real autorotation sits
  in that band. Steady **vertical** descent by energy balance `V_d = v_i + P₀/T`
  (monotone bisection, adaptive bracket — a draggy rotor sits arbitrarily deep in
  the windmill state, so a fixed ceiling would silently clamp it); validated
  against the measured ideal band `V_d/v_h∈[1.7,2.0]` (Harrington-style band
  oracle, not a closed form). **Forward** autorotation: the glide polar
  `RoD(V)=P_req(V)/W` and the min-sink / best-glide speeds — the realistic,
  survivable case (roughly halves the vertical rate at full scale). Flare energy
  `½IΩ²` + autorotation index. **Findings:** vertical autorotation is fast
  (~3300 fpm full-scale rep.); small model rotors are profile-drag-heavy, so they
  autorotate ABOVE the ideal band and gain little from forward flight — flare
  energy is their binding margin. **Flare survivability** (`survivability.rs`):
  composes the steady descent rate and the flare energy into the go/no-go energy
  bound — flare margin `M=E_flare/½m(V_d²−V_safe²)` must exceed 1 (necessary, NOT
  sufficient; transient entry/flare dynamics deliberately omitted as the
  "looks-right-quietly-wrong" integrator trap). Energy-bound only, all assumptions
  named (Ω_min frac, reaction delay, safe touchdown). **Height-velocity envelope**
  (`height_velocity.rs`): the low-speed "dead man's curve" by the ENERGY method
  `h_crit(V)=h_crit_hover−V²/2g` (forward KE as a height equivalent) — NO free
  parameter, anchored to the validated vertical critical height; the high-speed
  lobe is deferred to a dynamic flare model, *not* faked (the transient has no
  clean oracle — the integrator trap, avoided on purpose). `helisim` (feeds the
  design study).
- **Rotor-speed decay (dynamic entry, `rotor_decay.rs`).** The one transient
  piece, made honest by an EXACT oracle: the rotor EOM `IΩΩ̇=−P` integrates in
  closed form under constant power to `t_decay=½I(Ω₀²−Ω_min²)/P_h=E_flare/P_h` —
  the worst-case seconds before RPM is unrecoverable after power loss. An RK4 march
  (descent relief relaxing the drain) is **gated against that closed form in the
  constant-power limit** (exact) + a step-size check — satisfying the "never trust
  a time-integrator without a pre-computed oracle" rule precisely where the full
  vortex-ring-coupled entry aero (deliberately omitted) would have none. **Finding
  (in `helisim design`):** the 3.5 kg model has only **~0.5 s** of decay time vs
  ~7–8 s full-scale — a small electric helicopter needs AUTOMATIC power-loss
  detection + instant collective drop; no human reacts that fast.
- **★ EXTERNAL validation (autorotation, R22).** The first *external* check of
  this track (a category change, like Milestone 6 for the core aero) — predictions
  + parameter mapping LOCKED in `validation/AUTOROTATION_EXTERNAL_PREREG.md` before
  the oracle was sourced; results in `..._RESULTS.md`; test
  `crates/autorotation/tests/r22_external_validation.rs`. Oracle: Robinson R22 POH
  (best glide **75 KIAS**, ~**4:1** ratio; min-RoD **53 KIAS**), sourced + cited,
  not fabricated. **Result:** the clean calibration-free claims PASS exactly
  (best-glide speed > min-sink speed; forward < vertical), and the power-derived
  magnitudes land within ~9–16% with the error in the **pre-registered direction**
  (best-glide speed over-predicted by the assumed flat-plate area f; glide RATIO —
  least calibration-sensitive — matches to 11%). Two inputs (C_d0, f) are stated
  assumptions, so it's a "right order + right ordering + error attributable to
  named inputs" validation, not a precision match — declared up front.
- **Acoustics (rotor noise).** Electric removes engine/exhaust noise → the rotor
  dominates, so quiet design lives in the rotor model already built. **Gutin**
  rotational (loading) noise closed form on a std-only Bessel `J_n` (implemented
  here, validated vs tabulated zeros/values/recurrence). Validated internally:
  on-axis null, off-axis directivity peak (the torque term flips the bracket sign
  near the disk plane — peak is off-axis, NOT in-plane, a corrected assumption),
  harmonic decay, dB energy-sum, and the `∝M_tip³` tip-speed lever (10% V_tip cut
  ≈2.7 dB). **Honest scope:** tonal loading noise only; broadband + full Farassat-1A
  thickness + an external measured-SPL oracle are NAMED and deferred — never faked
  (the external-SPL match is a Milestone-6-style sourcing task).
- **Sizing study (`design`).** Composes the validated cores (BEMT hover trim ←
  mission, autorotation, acoustics) into the priority vector — **no new physics**;
  its tests are composition-consistency + trade-direction, not a new oracle.
  `helisim design`. **Load-bearing finding (the sweep falsified the first
  narrative; believed the disagreement per the ★ rule):** at fixed tip speed,
  hover power/endurance and the autorotation descent rate are NOT monotone in
  rotor radius — they have a **sweet spot ≈R 0.65–0.7 m** for the 3.5 kg model and
  worsen as bigger blades grow draggy; FM *falls* with radius (wrong airtime metric
  here); only noise is monotone (bigger/slower = quieter). **Sharper safety
  constraint:** the flare-margin column does the OPPOSITE of the others — at fixed
  V_tip a bigger disk spins slower (Ω=V_tip/R) so stored flare energy ½IΩ² falls,
  and the energy bound FAILS at R≳0.7 (a hard 'NO' cliff right where airtime is
  best). Safety's two metrics (descent rate ↓, flare margin ↓) pull opposite ways
  on radius, so the disk can't just grow — recommendation comes from the priority
  ORDER, not a single fabricated objective.
  **Recommender (`recommend.rs`) — the project's purpose: SUGGEST targets, don't
  consume them.** Searches the rotor geometry grid (blades × radius × tip speed ×
  solidity; chord derived from σ, rotor inertia *estimated from blade geometry* so
  the safety constraint responds physically), rejects anything that can't hover or
  fails the **safety floor** (flare margin ≥ threshold — a hard constraint, priority
  1), then ranks survivors by rank-weighted, min-max-normalised priority metrics
  (vert-integ → cost → airtime → efficiency → noise). Returns the winner + full
  ranked list + rationale, and **flags grid-edge optima** (honest: the true optimum
  may lie outside the searched range). **It beats the hand-picked `model()`:**
  recommends 3 blades, R=0.70, V_tip=90 → flare margin 2.36 (vs 1.30), endurance
  27.5 min (vs 18.9), 23.5 dB (vs 47) — safer, longer, quieter at once. Cost +
  vertical-integration are now IN the design report (so the recommender honours
  priorities #2/#3). `helisim design` leads with the recommendation. **Next: emit
  manufacturing geometry + build steps from the recommended spec (the stated end
  goal — "3D-print this shape / cut this block into this shape").**
- **Manufacture (`manufacture`) — design → buildable geometry (the end goal, started).**
  Turns the recommended [`DesignCandidate`] into real dimensioned part geometry,
  beginning with the blade: exact **NACA 4-digit section coordinates** (validated
  against published 0012 ordinates — y_t(0.30)=0.0600, TE(1.0)=0.00126), a
  dimensioned **BladeSpec** (span/chord/max-thickness), the **raw stock block** to
  start from (with machining allowance), and **step-by-step shaping instructions**.
  Geometry is exact math → geometric oracles, not fabricated numbers. `helisim
  design` now ends with the recommended blade's build steps (e.g. "Obtain stock
  654×44×5 mm balsa; shape NACA 0012, chord 36.7 mm, max thickness 4.40 mm @ 30%").
  **COMPLETE part system + assembly + export.** Every part is its own
  [`BuildPart`] (trait = polymorphism boundary) **physically sized from the
  design**, not guessed: blade (NACA section), **mast** (torsion `d=(16Q/πτ)^⅓`
  from the actual hover torque), **hub/grips** (from blade root + mast bore),
  **swashplate** (∝ rotor), **tail boom** (bending — root moment = main torque
  exactly, `M=Q`), **powertrain tray** (pack footprint). [`build_package`]
  assembles all six + a 10-step assembly sequence (ending with the power-loss
  safety check). [`export`] writes **STL** (printable extruded blade solid) and
  **DXF** (cuttable closed NACA-section polyline) — hand-written, zero deps, tests
  check well-formedness (facet/vertex counts, headers). `helisim build` runs the
  whole chain (recommend → size every part → assembly → write `build_output/*.stl
  + *.dxf`). Tests are geometric/engineering oracles (published NACA ordinates,
  the torsion/bending stress limits), never fabricated. Sizing is a first cut —
  the build output says so (confirm critical parts before flight).
  **Fidelity round (all 4 refinements):** (1) **lofted blade** — taper + twist
  interpolated over spanwise stations into a true tapered/twisted STL solid, plus
  a **root fitting** part (tang + retention bolt); (2) **structural proof**
  (`structural.rs`) — real flight-load margins of safety, the dominant load being
  **blade centrifugal tension** `F_cf=ω²m_blade·r_cg`, plus mast torsion + boom
  bending margins (finding: the tail BOOM is the most marginal first-cut part,
  MS≈+0.3); (3) **fuselage pod** + a **whole-aircraft assembly STL** (a `mesh.rs`
  triangle toolkit — cylinder/ellipsoid/loft + rotate/translate — positions
  fuselage+mast+blades+boom into one solid) + a valid **STEP wireframe** export
  (ISO-10303-21 section polylines; B-rep solid named/deferred); (4) **tail rotor**
  sub-assembly sized for anti-torque (`T_tr=Q/L_boom`), a miniature of the main
  rotor reusing the blade spec. `build_package` now emits 9 parts; `helisim build`
  prints the structural margins and writes blade.stl (lofted) + aircraft.stl
  (assembly) + blade.step + blade_section.dxf.
  **Heavy round (3 standalone efforts, each with a real oracle):**
  (1) **B-rep STEP solid** (`step_brep.rs`) — the blade mesh as a true
  `MANIFOLD_SOLID_BREP` (shared VERTEX_POINTs/EDGE_CURVEs, one ADVANCED_FACE per
  triangle, CLOSED_SHELL), NOT a wireframe. Validated **topologically**: closed
  genus-0 ⇒ Euler `V−E+F=2`, every edge used exactly twice (manifold), all #id refs
  resolve. (Full AP203 product conformance + CAD round-trip = named, not claimed.)
  (2) **FEA** (new crate `fea` + `fea_structural.rs`) — std-only Euler-Bernoulli
  beam FEM (assemble K, Gaussian solve, recover M & σ), validated against
  closed-form beam theory (cantilever `PL³/3EI` EXACT for cubic elements;
  simply-supported `PL³/48EI`; distributed `qL⁴/8EI` converges). Upgrades the
  section check: solves the tail boom (cantilever, tail-thrust tip load) and blade
  flap (distributed lift), reports the **deflection** the `M/Z` check couldn't and
  cross-checks FE vs closed-form stress (independent routes agree to 0.1 MPa).
  **Finding:** model boom tip deflects 62 mm, blade tip 82 mm — both pass on
  STRESS but are flexible, so **stiffness, not strength, may govern** (the FE adds
  exactly what the first-cut missed). (3) **Fastener/bearing selection**
  (`fasteners.rs`) — metric bolt (class 8.8) + deep-groove bearing catalogues;
  selects the **smallest standard part whose rated capacity ≥ load×SF** (validation:
  chosen passes, next size down fails). Hardware schedule: blade-retention M2
  (363 N centrifugal, double shear), 626 mast bearings, 623 grip bearings.
  `helisim build` now also prints the FEA + hardware schedule and writes the B-rep
  blade.step.
  **Deep round (4 more standalone efforts, each oracle-backed):**
  (1) **Whole-assembly B-rep** — `aircraft_to_step_ap203` emits every main solid
  (fuselage/mast/blades/boom), positioned, as separate MANIFOLD_SOLID_BREPs;
  required fixing the primitive meshes (ellipsoid pole fans) to be clean closed
  manifolds — each validated V−E+F=2. (2) **Full AP203 conformance**
  (`assembly_to_step_ap203`) — proper product structure (APPLICATION_CONTEXT →
  PRODUCT → PRODUCT_DEFINITION → SHAPE_DEFINITION_REPRESENTATION) +
  ADVANCED_BREP_SHAPE_REPRESENTATION with a mm-unit GEOMETRIC_REPRESENTATION_CONTEXT;
  validated by required-entity presence + all #refs resolve. (3) **Geometric
  (tension) stiffening** in the beam FEA — element geometric stiffness `Kg` with
  per-element axial tension; validated by the **taut-string limit** (`EI→0 ⇒
  qL²/8T`) and `T→0` recovering the beam. Applied to the blade with centrifugal
  tension `T(r)=ω²μ(R²−r²)/2`: **the floppy 82 mm static flap deflection becomes
  ~11 mm spun-up** (the real rotating-blade stiffness; static FEA over-predicts 7×).
  (4) **True 2-D continuum FE** — a plane-stress CST (`fea/cst.rs`), validated by
  the FE **patch test** (uniform stress reproduced exactly) + a uniaxial bar
  (`σ=F/A`, `δ=FL/AE` to machine precision) + Poisson sign. `helisim build` writes
  aircraft.step (whole-aircraft AP203 B-rep). Deferred: plate-bending/curved-shell
  elements; NEXT_ASSEMBLY_USAGE_OCCURRENCE component tree; CAD round-trip check.
- **Cost + buildability (`cost`).** Priorities #2 (vertical integration) and #3
  (cost), the two the aero/safety stack didn't touch. A bill of materials from a
  coarse mass/power/energy spec, every line tagged with a **buildability** taxonomy
  (raw-stock / fabricated / assembled / purchased) → a **vertical-integration
  index** (cost-weighted self-build fraction) + the irreducible buy-list.
  **Provenance honesty applied to money:** costs are a PARAMETRIC model with named,
  overridable [`UnitCosts`] inputs (representative defaults flagged as assumptions,
  NOT sourced facts); only the relative breakdown + buildability split are findings.
  Tests are accounting consistency/monotonicity, not a cost oracle. **Finding:** at
  model scale the COTS flight-controller + sensors ($-flat) dominate and are
  unbuildable, pinning the self-build index ~25%; vertical integration improves
  with scale as self-made structure/rotor grow against ~flat avionics. `helisim
  design` (cost section).

Each milestone is added as new crate(s); never break the existing cores.

## Hard rules (always follow)

- **Zero external dependencies.** `std` only, everywhere. No crates in any
  `Cargo.toml`. If something seems to need a dependency, implement it.
- **Latest stable Rust, edition 2024.** Set edition once in
  `[workspace.package]`; crates inherit with `edition.workspace = true`.
- **One concept per file.** A file holds a single struct/trait/idea. Split
  rather than grow. See the file map below.
- **≤ 500 lines per file.** If a file approaches this, extract a concept.
- **Object-oriented + polymorphism.** Model behaviour as traits; depend on
  `&dyn Trait` (or `Box<dyn Trait>`) at boundaries so implementations are
  swappable. Concrete examples: `Airfoil`, `ValidationCase`.
- **Validations are tests.** Every published-oracle comparison lives in a
  `#[test]` (see `crates/validation/tests/`). A claim of "it matches" must be a
  passing test, runnable with `cargo test`.
- **Use the known data.** Validate against published experimental/CFD data, not
  invented numbers. Cite the source in a doc comment. Never fabricate oracle
  values — if a number can't be sourced, say so and pick a defensible model
  instead (see Harrington: FM band, not a faked C_T table).
- **Be honest about model error.** When a method has a known limitation
  (e.g. BEMT over-predicts thrust vs. a contracted wake), document the size and
  cause in code, and encode the *expected* behaviour in the test — do not
  silently fudge a parameter to force a match.

## Oracle coverage (documented example numbers for every module)

`validation/ORACLE_COVERAGE.md` is the **coverage map**: every validated quantity,
the documented example number it matches, the source, and the oracle type
(A external / B closed-form / C self-consistency / D structural). After an audit
that found several modules validating only by self-consistency where a documented
number was readily sourceable, documented-number tests were added so a reader can
hand-check each: **pack** (6S2P 25R → 21.6 V / 5 Ah / 108 Wh / 63 mΩ / 540 g / 40 A,
Samsung datasheet), **powertrain** (0.85×0.95=0.80; 1000 W→1250 W), **thermal**
(convection h in the Incropera Nu·k/D bands), **airfoil** (NACA0012 a₀=5.73/rad,
C_lmax 1.4, C_d0 0.0065 — Abbott & von Doenhoff / Prouty), **manufacture**
(bolt areas = ISO 724, working shear = ISO 898-1 0.6·800/2.4; boom Z≈0.058D³ =
Roark; Al allowables MMPDS/ASM). Honest gaps that remain (no clean external number
without Milestone-6-style sourcing) are listed at the bottom of that file — chiefly
the **acoustics external-SPL** anchor. The rule still holds: a "match" is a passing
`#[test]`, and a number with no source is never fabricated — it is named as a gap.

## Validation lessons (hard-won; violate at your peril)

- **★ A cross-check's only value is that it CAN disagree — and you must believe
  the disagreement (the load-bearing rule).** For every claim, name the two routes
  and confirm they don't share the component that could be wrong. Two routes that
  share a fallible part can't catch its error: the 5f lateral-sign bug slipped
  through because the axisymmetry assertion and full_aero shared the same
  conceptual sign. Two routes that DON'T share it do catch it: 5l's "stable
  −0.062" (char-poly eigensolver) disagreed with a provably-stable nonlinear march
  that did NOT use the eigensolver — the disagreement was right and the reported
  number was wrong (true −0.001, char-poly ill-conditioned past ~10×10; fixed with
  the QR solver, `dynamics/schur.rs`). The result had been *endorsed*; the
  cross-check overturned it anyway. When two independent routes disagree, the
  disagreement is the finding — investigate it, don't reconcile to the number you
  already published.
- **Two independent routes only validate if GENUINELY independent (5f).** The
  original lateral `Lv=+Mu` axisymmetry assertion and full_aero shared the same
  conceptual sign error, so they "agreed" while both wrong. Derive (rotation),
  don't assert. (The sharper, general form is the ★ rule above.)
- **The first external comparison is the one IRREVERSIBLE epistemic moment — lock
  everything before you see the oracle (Milestone 6).** Internal milestones can be
  re-run and re-validated; the model meeting ground truth cannot be un-seen, and the
  comparison's value depends entirely on the predictions AND the parameter mapping
  being fixed beforehand. Predictions live in `validation/MILESTONE6_PREDICTIONS.md`;
  parameter-mapping decisions (every place the real aircraft doesn't map one-to-one
  onto the model — canted tail rotor, stabilator, cross-inertia, airfoil, twist,
  parasite) are decided on PHYSICS and locked in `validation/MILESTONE6_PARAMETER_MAPPING.md`.
  Why this is a lesson and not just a note: the parameter-entry step is where the lock
  silently leaks — "correcting" a parameter during the build while glancing at Table 4
  is model-fitting you'd never detect afterward. So the build-and-compare runs as its
  OWN session, never rushed at the tail of another. Stop *because* of this, not just
  *that* you stopped. (Sibling rule: reject compromised-provenance data even when it's
  the best — the export-restricted Fletcher TM — and reject data whose error channels
  the model can't represent — the flybar R-50 — because an external mismatch is only
  diagnostic if every error source is attributable to a named cause.)
- **Eigenvalue-dimension audit (so "5c–5k stand" is checked, not asserted).** The
  char-poly eigensolver was wrong only past ~10×10. Per-milestone matrix size fed
  to `eigenvalues`: 5c long 4×4, 5e-i lat 4×4, 5e-ii/5f/5g coupled & linearize8
  8×8, 5i linearize11 / 5j & 5k closed_loop_matrix 11×11 — all ≤11, unaffected;
  5l augmented **13×13** (where the bug bit), 5m linearize15 **15×15**. Falsifiable
  evidence: switching `eigenvalues` to QR changed the result of EXACTLY ONE test
  (the 13-state 5l gate); every ≤11 milestone test passed unchanged. Both 13/15
  systems now use QR.
- **Where the linearization sits on a non-differentiable point of the model, the
  linear design/oracle is valid ONLY for the modes that survive the linearization
  (5i→5j).** The Pitt–Peters wake skew χ(μ) is non-analytic at μ=0 (μ=|V|≥0 is
  rectified), so the hover Jacobian is blind to the λ₀↔λ₁c coupling that's live in
  the nonlinear march — making it both the wrong validation gate (5i) and the
  wrong design oracle (5j) for the pitch/lateral-rate channels. The fix: design
  and validate OFF the seam (small forward speed, χ differentiable, linear↔nonlinear
  agree — the trustworthy oracle), then CONFIRM across the seam on the nonlinear
  case. Distinct from "derive don't assert": this is about *where you are allowed
  to trust a linear oracle at all.*

## Workspace layout

```
crates/
  airfoil/      sectional aerodynamics (Cl, Cd)
    airfoil.rs    trait Airfoil           <- polymorphism boundary
    linear.rs     LinearAirfoil           (analytic NACA0012: lift slope,
                                           stall, P-G compressibility, drag polar)
    table.rs      TableAirfoil            (interpolated measured polar)
  rotor/        geometry + operating point
    rotor.rs      Rotor                   (geometry as functions of x = r/R)
    operating.rs  Operating               (RPM / tip-Mach, density, sound speed)
  bemt/         hover BEMT solver
    config.rs     Config
    tip_loss.rs   prandtl_tip_loss
    station.rs    Station                 (per-station converged state + dCT/dx)
    solution.rs   HoverSolution
    solver.rs     solve_hover             (per-station inflow bisection + span integral)
  forward/      forward-flight BEMT (rigid blade)
    condition.rs  ForwardCondition (advance ratio μ + disk tilt)
    config.rs     ForwardConfig
    inflow.rs     Glauert momentum inflow + analytic closed form
    solver.rs     solve_forward: outer inflow bisection + azimuth×radius integral
    solution.rs   ForwardSolution (C_T, C_P, C_roll, reverse-flow fraction, …)
    tests/forward_validation.rs  Glauert closed form + power bucket + roll moment
  coupled/      two-way flap↔inflow coupling (forward flight)
    config.rs     CoupledConfig
    loads.rs      blade-element thrust/power integral WITH flapping in u_P
    solver.rs     solve_coupled — flap↔inflow relaxed fixed point (λ bounded)
    solution.rs   CoupledSolution (+ rotor_power_w: physical induced+profile)
    tests/coupled_validation.rs  convergence + loading equalisation
  flapping/     rigid-blade first-harmonic flapping
    properties.rs FlapProperties (Lock number, hinge offset, ν_β, gyro_rate [5h/M6 rate-damping])
    controls.rs   Controls (cyclic pitch θ1c/θ1s)
    config.rs     FlapConfig
    linalg.rs     solve3 — 3×3 linear solve (NEW solver shape, not bisection)
    harmonics.rs  build_system: harmonic-balance forcing vector F + response matrix G
    closed_form.rs analytic (β0,β1c,β1s) oracle
    solver.rs     solve_flapping → FlapSolution
    solution.rs   FlapSolution (coning, cyclic flap, hub moments, phase lag)
    tests/flapping_validation.rs  Lock closed form + 90° lag + moment→tilt
  sim/          nonlinear time-marching (5d longitudinal, 5g coupled 8-state, 5i driven 11-state)
    rk4.rs        fixed-step RK4 integrator (+ rk4_step_t time-aware, for control inputs)
    eom.rs        nonlinear longitudinal EOM (rotor-in-the-loop, quasi-static)
    coupled_march.rs  nonlinear 8-state EOM + 6-var equilibrium + linearize8 (5g)
    control.rs    ControlSchedule trait (Step/Pulse/Trim) + Channel; control conventions (5i)
    driven_march.rs  driven 11-state EOM (inflow in loop) + linearize11[_at] + control_matrix11[_at] (5i)
    driven_equilibrium.rs  trimmed 11-state equilibrium at a prescribed velocity (hover + off-seam) + model11[_at]
    sas.rs        RateSas rate damper + closed_loop_matrix (A+BK) + simulate11_sas[_dist] (5j)
    attitude_hold.rs  attitude_hold: layer θ→lon-cyc, φ→lat-cyc onto the rate damper (5k)
    pi_attitude.rs  PiAttitudeHold (integral action) + augmented_matrix (13-state) + simulate13 (5l)
    velocity_hold.rs  VelocityHold cascade (15-state) + deriv15 + simulate15 + linearize15 (5m)
    simulate.rs   simulate_hover_longitudinal + simulate_linear[_nd] (for the gates)
    analysis.rs   fit_growing_oscillation (period/growth from peaks)
    tests/sim_validation.rs           longitudinal fixed point + linear-match gate
    tests/coupled_march_validation.rs 8-D fixed point + Jacobian↔coupled8 + track/depart
    tests/driven_validation.rs        5i: control effectiveness + off-axis time-domain flip + open-loop divergence
    tests/sas_validation.rs           5j: off-seam trustworthy gate + hover damping/residual + nonlinear hold
    tests/attitude_hold_validation.rs 5k: phugoid→LHP + off-seam regulation + hover seam-residual + sustained-disturbance
    tests/pi_attitude_validation.rs   5l: integrator marginal-stable off-seam + zero steady-state attitude error + velocity-drift boundary
    tests/velocity_hold_validation.rs 5m: timescale-separation clusters + drift→0 + hover position-hold capstone
    tests/uh60_external_validation.rs  5/6: UH-60 hover derivs vs GENHEL (Zw 12% match, Mu sign+order, Mq ~65× under = the finding)
  dynamics/     linearized stability & control derivatives + modes
    complex.rs    minimal Complex
    eigen.rs      char_poly (Faddeev–LeVerrier) + roots (Durand–Kerner); small-matrix anchors
    schur.rs      QR eigensolver (elmhes Hessenberg + Francis hqr) — eigenvalues for n≳10 (5m)
    tests/schur_validation.rs  known 15×15 spectrum (reals over 3 decades + complex pairs)
    aero.rs       perturbable main-rotor longitudinal forces/moments (u,w,q)
    derivatives.rs  longitudinal stability derivatives (central differences)
    model.rs      assemble A [u,w,q,θ], eigenvalues, classify modes; hovering_cubic
    full_aero.rs  generalized main-rotor aero (Forces6, uniform inflow) + rotate6
    flap_general.rs first-harmonic flap harmonic balance, general flow + linear inflow (5h)
    pitt_peters.rs Pitt–Peters 3-state inflow: [M],[L] matrices, steady solve, ν̇ (5h)
    inflow_coupling.rs couples full_aero↔pitt_peters: main_rotor_with_inflow, quasi_static, march (5h)
    lateral.rs    lateral derivs via rotation of longitudinal (5f) + tail; A[v,p,r,φ]
    coupled8.rs   8×8 [u,w,q,θ,v,p,r,φ], cross-coupling via rotation; decouple/couple switch
    tests/dynamics_validation.rs   longitudinal: signs + unstable-osc + cubic anchor
    tests/lateral_validation.rs    lateral: signs + oscillatory-unstable + cubic anchor
    tests/rotation_validation.rs   5f: rotation path reproduces longitudinal oracle across amplitudes
    tests/coupled_validation.rs    decouple→oracle union, couple→shifted modes
    tests/pitt_peters_validation.rs 5h: τ→0 baseline recovery + off-axis sign flip
  trim/         steady-flight trim (6-DOF force/moment balance)
    newton.rs     solve_newton — multidim Newton + numerical Jacobian (NEW shape)
    aircraft.rs   Aircraft / TailRotor (rotors, geometry, mass)
    condition.rs  TrimCondition (hover / forward speed)
    residual.rs   the six force/moment residuals (reuses bemt/forward/flapping)
    solver.rs     trim — Newton with speed continuation
    solution.rs   TrimResult
    tests/trim_validation.rs  hover cross-check vs standalone BEMT + fwd trends
  validation/   published-oracle cases
    oracle.rs        trait ValidationCase + run_case  <- polymorphism boundary
    caradonna_tung.rs  primary oracle (C_T vs collective)
    harrington.rs      secondary check (figure-of-merit band)
    tests/validation.rs  the validation suite as tests
  cell/         battery cell equivalent-circuit
    cell.rs       trait Cell                <- polymorphism boundary
    thevenin.rs   TheveninCell (+ samsung_25r oracle)
    tests/discharge.rs  datasheet discharge validation
  pack/         series/parallel pack
    pack.rs       Pack (S×P scaling of voltage/capacity/resistance/mass)
  powertrain/   motor + ESC
    powertrain.rs          trait Powertrain  <- polymorphism boundary
    constant_efficiency.rs ConstantEfficiency
  thermal/      lumped-mass cell thermal
    cooling.rs    trait Cooling + Convective  <- polymorphism boundary
    lumped.rs     LumpedThermalCell (C dT/dt = Qgen - Qcool)
    limits.rs     ThermalLimits / ThermalStatus (safe/warn/over-temp band)
    tests/thermal_validation.rs  18650 thermal oracle
  mission/      end-to-end coupling
    electrical.rs     coupled constant-power current solve (bisection)
    hover_trim.rs     find collective for thrust = weight (bisection)
    endurance.rs      SoC + temperature discharge integrator
    hover_mission.rs  analyze_hover -> HoverReport (incl. hover peak temp)
    climb.rs          analyze_climb -> ClimbReport (sustained-climb thermal check)
    tests/end_to_end.rs  chain + design-tension + thermal-safety tests
  autorotation/ power-off descent (safety; std + helisim-rotor only)
    inflow.rs     descent-regime v_i/v_h: climb + windmill closed forms + measured VRS curve
    descent.rs    steady vertical autorotation V_d=v_i+P₀/T (bisection, adaptive bracket)
    forward.rs    forward glide polar RoD(V)=P_req/W → min-sink + best-glide
    index.rs      rotor KE ½IΩ² + flare-height equiv + autorotation index
    survivability.rs flare energy bound: flare margin + critical hover height (go/no-go)
    height_velocity.rs low-speed dead-man's curve h_crit(V)=h_crit_hover−V²/2g (energy method)
    rotor_decay.rs dynamic entry RPM decay t=E_flare/P_h + RK4 gated vs analytic
    solution.rs   AutorotationSolution
    tests/autorotation_validation.rs  momentum-quadratic anchors + measured ideal band [1.7,2.0] + forward glide
    tests/r22_external_validation.rs  EXTERNAL: R22 POH glide speeds (locked prereg, cited oracle)
  acoustics/    rotor harmonic noise (priority: minimal sound)
    bessel.rs     integer-order J_n(x), std-only (validated vs tabulated zeros/values)
    rotational.rs Gutin rotational (loading) noise harmonic pressure
    thickness.rs  ∝M_tip³ tip-speed noise lever (relative indicator)
    spl.rs        dB re 20µPa + energy-sum spectrum assembly
    solution.rs   NoiseSpectrum / Harmonic
    tests/acoustics_validation.rs  directivity + tip-speed master knob
  design/       model-scale sizing study (composes the validated cores; NO new physics)
    candidate.rs  DesignCandidate (builder knobs: geometry, tip speed, pack, parasite)
    report.rs     DesignReport (consequences by priority: safety/airtime/efficiency/noise)
    metrics.rs    evaluate — BEMT trim + autorotation + acoustics + cost → report
    sweep.rs      sweep_radius — the disk-loading trade at fixed tip speed
    recommend.rs  recommend — search + safety-constrained priority-ranked suggestion
    tests/design_validation.rs  composition-consistency + trade-direction + recommender
  manufacture/  recommended design → buildable geometry + step-by-step (the end goal)
    part.rs       trait BuildPart (polymorphism boundary) + Source taxonomy
    materials.rs  allowable-stress constants (Al shear/bending, conservative)
    airfoil_coords.rs NACA 4-digit section coords (validated vs published 0012 ordinates)
    blade.rs      BladeSpec from a design: dimensions, raw stock, shaping instructions
    hub.rs        HubSpec — teetering/articulated head + grips from blade root
    mast.rs       MastSpec — drive shaft, torsion-sized from hover torque
    swashplate.rs SwashplateSpec — control plates, ∝ rotor + mast bore
    boom.rs       BoomSpec — tail boom, bending-sized (root moment = main torque)
    mount.rs      MountSpec — powertrain tray from pack footprint
    root_fitting.rs RootFitting — blade root tang + retention bolt
    fuselage.rs   FuselageSpec — ellipsoidal pod + canopy
    tail_rotor.rs TailRotorSpec — anti-torque sub-rotor (T_tr=Q/L_boom), reuses BladeSpec
    structural.rs check_structure — flight-load margins (centrifugal + torsion + bending)
    mesh.rs       triangle toolkit (cylinder/ellipsoid/lofted-blade + transforms)
    structural.rs check_structure — section margins (centrifugal/torsion/bending)
    fea_structural.rs run_fea — beam-FEM boom+blade (deflection + FE-vs-closed-form)
    fasteners.rs  bolt/bearing catalogues + select-smallest-adequate + hardware_schedule
    assembly.rs   BuildPackage — all 9 parts + the assembly sequence
    export.rs     blade_to_stl/lofted_blade_to_stl (printable) + airfoil_to_dxf (cuttable)
    assembly_export.rs aircraft_to_stl (whole-aircraft) + aircraft_to_step (STEP wireframe)
    step_brep.rs  mesh_to_step_brep/blade_to_step_brep — real MANIFOLD_SOLID_BREP solid
    (tests inline) geometric oracles (NACA ordinates) + stress limits + Euler V-E+F=2 + STL/DXF/STEP
  fea/          minimal std-only finite-element analysis (validated vs theory)
    linsolve.rs   dense Ax=b (Gaussian elimination, partial pivoting)
    beam.rs       Euler-Bernoulli beam FEM + geometric (tension) stiffening Kg
    cst.rs        plane-stress constant-strain triangle (2-D continuum FE)
    tests/beam_validation.rs  cantilever PL³/3EI (exact) + string limit qL²/8T + distributed
    tests/cst_validation.rs   FE patch test + uniaxial bar (σ=F/A, δ=FL/AE exact)
  cost/         parametric cost + buildability (priorities #2 vert-integ, #3 cost)
    component.rs  Component + Buildability taxonomy (raw-stock/fabricated/assembled/purchased)
    costs.rs      UnitCosts — named, overridable cost inputs (representative defaults)
    bom.rs        AircraftSpec → Bom (bill of materials)
    report.rs     summarize → CostReport (vertical-integration index + buy-list)
    tests/cost_validation.rs  accounting consistency + monotonicity + taxonomy order
  cli/          command-line driver (report/study/mission_cli/design_cli formatting)
```

## Physics conventions (BEMT core)

- Nondimensional radial station `x = r/R`; velocities scaled by tip speed `ΩR`.
- Inflow ratio `λ = v_i/(ΩR)`; inflow angle `φ = atan2(λ, x)`; AoA `α = θ(x) − φ`.
- Blade element: `dC_T/dx = (σ/2)(x²+λ²)(Cl cosφ − Cd sinφ)`.
- Momentum (hover): `dC_T/dx = 4 F λ² x`, `F` = Prandtl tip loss.
- Per-station `λ` solved by **bisection** on the (monotone) thrust-balance
  residual — robust, no derivatives, no divergence.

### Solver vocabulary

Six solver shapes are now in use — pick by problem structure:
1. **Monotone-residual bisection** wrapping an integral — hover/forward inflow,
   hover thrust-trim, coupled pack current. Use for a 1-D root of a monotone
   residual: robust, derivative-free.
2. **Small linear-system solve** — flapping harmonic balance (`flapping/linalg.rs`
   `solve3`). Use when the unknown is a vector of coefficients and the response is
   *linear* in them: assemble and solve, don't root-find.
3. **Multidimensional Newton with numerical Jacobian** — trim (`trim/newton.rs`
   `solve_newton`). Use for a coupled *nonlinear* vector root (forces & moments):
   finite-difference the Jacobian, damped/backtracking step, and — for hard cases
   — parameter continuation (trim marches up in speed from hover).
4. **Eigenvalue extraction** — for the modes of a linear system matrix. TWO
   implementations: the characteristic polynomial (Faddeev–LeVerrier + Durand–Kerner,
   `dynamics/eigen.rs`) for *small* matrices (validated against the analytically-
   rootable hovering cubic), and the **QR algorithm** (Hessenberg + Francis
   double-shift, `dynamics/schur.rs`) for everything else. `eigenvalues` routes
   through QR: the char-poly route is numerically ill-conditioned past ~10×10 and
   silently returned WRONG eigenvalues for the 13–15-state augmented control systems
   (caught when a "stable" 13-state read −0.062 but was really −0.001). Lesson:
   form-the-poly-then-root is a trap at scale; work on the matrix. (The perturbation
   engine for the *derivatives* is the numerical-Jacobian machinery from shape 3.)
5. **Fixed-step RK4 time integration** — sim (`sim/rk4.rs`). The integrator is
   trivial; the architectural shift is the **rotor model as a callee inside the
   integration loop** (re-solved quasi-statically each substep), the first time
   the rotor isn't solved once per condition. CAUTION: a time-integrator can look
   right while quietly wrong (wrong damping, energy leak, too-big step) — always
   gate it against a pre-computed oracle (here the 5c eigenvalues) AND check more
   than one step size.
6. **Outer-state inflow integration (Pitt–Peters, 5h)** — `dynamics/pitt_peters.rs`
   + `inflow_coupling.rs`. The rotor inflow stops being an *inner fixed-point*
   (re-solved each aero call) and becomes *outer integrated state*: three states
   `ν=[λ₀,λ₁s,λ₁c]` with their own ODE `[M]ν̇+[L]⁻¹ν=C`. Use when a sub-model has
   real dynamics (a lag) that the quasi-static fixed point throws away. CAUTION:
   the states are internal — no standalone oracle — so gate on the τ→0 reduction
   to the validated quasi-static model (exact) AND a documented qualitative
   signature (the off-axis sign flip).
- `C_P == C_Q`; figure of merit `FM = C_T^{3/2}/(√2 C_P)`.
- Solidity `σ = N_b c/(πR)`.

## Forward-flight conventions (forward crate)

- Advance ratio `μ = V cosα/(ΩR)`; tangential velocity `u_T = x + μ sinψ`
  (advancing ψ≈90°, retreating ψ≈270°); through-disk inflow `u_P = λ` (uniform).
- Momentum (Glauert): `C_T = 2 λ_i √(μ²+λ²)`, `λ_i = λ − μ tanα`.
- Coefficients are azimuth-averaged: `C = (σ/2)(1/2π)∫₀^2π∫ (u_T²+u_P²)(…) dx dψ`.
- Same outer-bisection-on-λ wrapping an inner integral as hover; hover is μ=0.
- Reverse flow where `u_T<0` (inboard retreating, x<μ|sinψ|): lift nulled,
  area fraction reported.
- `c_roll` = lateral (advancing-side) lift asymmetry → rolling moment; `c_pitch`
  ≈0 for uniform inflow (fore-aft symmetric). Moment coeffs `M/(ρA(ΩR)²R)`.

## Flapping conventions (flapping crate)

- Flap eqn `β'' + ν_β² β = (γ/2)∫₀¹ x(u_T²θ − u_T u_P)dx`, `u_P = λ + xβ' + μβcosψ`.
- `β = β₀ − β₁c cosψ − β₁s sinψ`, ψ=0 downstream. `β₁c>0` = rearward blow-back.
- Lock number `γ = ρacR⁴/I_β`; `ν_β²=1+1.5e/(1−e)` (e = hinge offset). Central
  hinge ν_β=1 (resonant), hub moment 0; offset → residual hub moment ∝ (ν²−1).
- Pitch `θ(x,ψ)=θ₀+θ_tw x+θ1c cosψ+θ1s sinψ`. Linear lift, reverse flow NOT
  nulled, cutout neglected (0→1) — all to match the analytic oracle.
- One-way coupling: flapping uses the forward-flight inflow λ (does not re-couple).

## Trim conventions (trim crate)

- Body axes x fwd / y right / z down. Unknowns `[θ₀, θ₁c, θ₁s, θ₀_tr, pitch, roll]`,
  six residuals = force (X,Y,Z) + moment (roll,pitch,yaw) balance.
- Main thrust ⟂ TPP, tilted from shaft (body −z) by flapping (β1c,β1s); central
  hinge → no hub moment, offset → hub moment balances pitch. Hub at height `h`
  above CG; tail rotor at (−arm,0,−height) producing yaw `T_tr·arm = Q`.
- Hover uses hover BEMT (so it matches milestone 1 exactly); forward uses forward
  BEMT + flapping. Speed continuation from hover for forward trim robustness.
- Scope 5a: hover + steady level forward flight only.
- 5b: forward main rotor uses the two-way coupled solve ([`coupled`]); forward
  power = `rotor_power_w(κ)` (physical induced+profile, κ calibrated at hover so
  the cross-check stays exact) + parasite `½ρV³f` + tail. Hover still uses hover
  BEMT directly. The λ-bounded coupled fixed point keeps high-μ trim physical.

## Control conventions (5i — `sim/control.rs`)

Controls are **deltas from trim**, in **radians** of blade pitch, behind the
`ControlSchedule` trait. Each channel's sign is pinned to a physical effect and
validated against the control-effectiveness matrix `B = ∂ẋ/∂u` (and the trusted
derivative signs). Body axes x-fwd/y-right/z-down; φ right-down +, θ nose-up +,
r nose-right +.

- **`Collective` Δθ₀** — positive raises main thrust → climb (`ẇ < 0`, w is body-down).
- **`LatCyclic` Δθ1c** — positive → +roll moment → right roll (`ṗ > 0`; matches
  `∂Mx/∂θ1c > 0`).
- **`LonCyclic` Δθ1s** — positive → pitch moment (`q̇ > 0`).
- **`Pedal` Δθ₀_tail** — positive raises tail thrust → yaw reaction about the arm.
- On-axis dominates off-axis ~20:1. The off-axis `∂My/∂θ1c` is the diagnostic
  one: −3.18 with inflow frozen (the first instant of a step) → +0.53 once the
  inflow develops — the 5h sign flip as a time-domain effect.

## Validation status (run `cargo test`, or `cargo run` for the report)

- **Solver correctness:** per-station inflow reproduces the analytical BEMT
  closed form to < 2% (no tip loss, no drag, incompressible).
- **Figure of merit (calibration-free match):** Harrington Rotor 1 peak FM
  ≈ 0.71, inside the published [0.62, 0.75] band.
- **C_T vs collective (Caradonna & Tung):** correct trend and magnitude; BEMT
  over-predicts the CFD-validated experiment by ~20–27% at design collective
  (largest at low collective) — the documented momentum-vs-contracted-wake
  limitation. The solver's C_T agrees with other published BEMT codes.
- **Trim (hover cross-check):** the full six-equation force/moment Newton solve
  lands on the same main-rotor collective (~0.07°) and power (~0.3%) as the
  independent hover-BEMT thrust=weight inversion — two independent routes agreeing.
  Forward flight shows the classic trends (collective down, longitudinal stick
  forward, nose-down attitude with speed).
- **Two-way coupling + parasite (5b):** the flap↔inflow fixed point converges and
  equalises the advancing/retreating loading; with the physical power
  decomposition (`κ·C_T·λ + profile`, κ calibrated to hover BEMT) + parasite
  `½ρV³f`, trimmed power is positive and physical past μ=0.25 and the **complete
  power bucket** appears (hover ~587 W → min ~318 W at ~15 m/s → parasite rise).
  This fixed the high-μ collapse that drove power negative with frozen inflow.
- **Linearized hover dynamics (5c):** all longitudinal derivative signs match
  theory — Mu>0 (destabilizing), Mq<0, Zw<0, Xu<0 — and the eigenvalues show the
  textbook hover signature: an **unstable oscillatory mode** (~0.64±1.17i, period
  ~5 s) + two stable subsidences. The instability emerged from the derivatives,
  unprompted. The new eigenvalue routine matches the analytic hovering cubic to
  4 digits. These derivatives are force/moment-based → clean of the κ caveat.
- **Nonlinear time-march (5d):** trim is an exact fixed point (drift ~1e-12);
  a perturbed hover reproduces the 5c eigenvalue (period 7.1 vs 6.97 s, σ 0.503
  vs 0.505), the nonlinear trajectory coincides with `ẋ=Ax` through the linear
  regime then departs as the instability grows — verified at multiple step sizes.
  Equilibrium is the self-consistent *dynamics* hover (uniform-inflow thrust =
  weight, cyclic 0), so 5c and 5d describe the same fixed point.
- **Lateral hover (5e-i / 5f):** Lp<0, Nr<0, Yv<0 (textbook). The main rotor's
  lateral response is the **exact rotation of the validated longitudinal aero** —
  velocity perturbations rotate +90° (`Lv=−Mu, Yv=Xu`), angular-rate perturbations
  rotate −90° (`Lp=Mq, Yp=−Xq`). The +90/−90 distinction is the subtle point:
  it makes `Lv` *negative*, so the lateral hover is **oscillatory-unstable** (a
  lateral phugoid mirroring the longitudinal), NOT the aperiodic divergence the
  pre-5f `Lv=+Mu` sign error produced. The **tail rotor is a dynamic element**
  with height included (named decision): `v_axial = v + p·h_tr − r·l_tr` → Nr,
  Yv, Nv and the roll-yaw coupling. The roll-sideslip cubic matches the 4×4.
  ⚠ Lesson: "two independent routes" only validates if they're *genuinely*
  independent — the original axisymmetry assertion and full_aero shared the same
  conceptual sign error. The rotation construction (derived, not asserted) is the
  trustworthy route; it reproduces the longitudinal oracle across amplitudes.
- **Coupled 8-state (5e-ii / 5f):** the decouple gate is exact — zeroing the cross
  blocks reproduces the 5c ∪ 5e-i eigenvalues; coupling shifts every mode and a
  longitudinal Δu excites lateral motion. The cross-coupling is built by **exact
  rotation** of the validated longitudinal response (not the sign-prone in-place
  lateral path). After the 5f fix, both instabilities are oscillatory.
- **General-state aero + nonlinear 8-state (5g):** full_aero is now exactly
  equivariant for *combined* states — the bug was the assembly signs (`fy=+Tβ1s`
  and `hub_roll=+Kβ1s` must both be negated so force/moment rotate as proper
  vectors with the flap); validated by rotating a combined (v,w,p,q) state (1e-6).
  The nonlinear 8-state march: a 6-variable hover equilibrium (incl. roll φ_e for
  the tail side force) is an exact fixed point (drift 2.7e-11/6 s though the
  equilibrium is unstable); the EOM's numerical Jacobian matches the coupled 8×8
  model; a Δu=0.1 m/s perturbation tracks the 8-D linear prediction (<5% to ~4 s)
  then departs as both instabilities compound — a narrower window than 4-D, with
  amplitude/window named in the test rather than incidental.
- **Pitt–Peters dynamic inflow (5h):** inflow is now three integrated states
  `ν=[λ₀,λ₁s,λ₁c]` (architecture shift: inner-fixed-point → outer-integrated-state).
  Inflow states are *internal* (no standalone oracle), so two clean gates carry it:
  **(1)** zeroing the cyclic states recovers the 5g uniform baseline bit-for-bit and
  lag→0 collapses onto the quasi-static fixed point (Δν 4e-13, exact & falsifiable);
  **(2)** the cyclic inflow flips the sign of the off-axis cyclic response
  `∂My/∂θ1c` (−3.2 frozen → +0.5 solved) — the documented "wrong sign of off-axis
  response to cyclic" the model is famous for correcting, emerging on its own from
  the `[L]` λ₀↔λ₁c wake-skew coupling (not a tuned target). Gravest mode τ≈0.085 s
  (~2 revs), the literature's O(1-rev) lag scale. `helisim inflow`.
- **Control-input time histories (5i):** the driven 11-state march (rigid body +
  inflow, both rotors, time-varying controls). Validation is *response correctness*,
  not a scalar: **(1)** control effectiveness `B=∂ẋ/∂u` has the pinned physical
  signs and ~20:1 on-axis dominance; **(2)** the off-axis `∂My/∂θ1c` flips −3.18
  (inflow frozen at t=0) → +0.53 (inflow developed) — the 5h flip now in the time
  domain; **(3)** open-loop the aircraft diverges to a control pulse and never
  returns (the 5j setup), with `u` tracking the linear model in a bounded window.
  The 11-state modes preserve the body modes (≈8-state) + three fast STABLE inflow
  modes. **Named limitation/finding:** the hover divergence is *faster* than the
  hover-linearized rate — the wake skew χ(μ) is non-analytic at μ=0 (rectified
  μ=|V|), so the off-axis-flip coupling escapes the hover Jacobian; the pitch/lateral
  rates depart immediately while `u` (analytic channel) tracks. Documented, not
  fudged. `helisim fly`.
- **Stability augmentation (5j):** a rate-feedback SCAS damper, validated in three
  layers around the wake-skew seam. **(1)** OFF the seam (5 m/s, μ≈0.04) the
  Jacobian is differentiable and nonlinear tracks the closed-loop linear model in
  the *pitch-rate channel* (the one hover misses) to <1% — so closed-loop
  eigenvalues in the LHP (open 0.54 → closed −0.005) are a TRUSTWORTHY gate there.
  **(2)** Hover LINEAR: the damper collapses the instability (0.70 → +0.024) but a
  small positive residual remains (the slow phugoid rate feedback can't reach) —
  necessary, not sufficient. **(3)** ACROSS the seam: the same gains turn the
  open-loop hover divergence (NaN) into a bounded nonlinear response (<3.5° over
  8 s), including the seam-hidden channel. A damper not a hold (named scope cap).
  `helisim sas`.
- **Attitude hold (5k):** a proportional outer attitude loop (θ→lon-cyclic,
  φ→lat-cyclic, gains 0.1) on the 5j rate damper — first **regulation** check
  ("drive error to zero, hold it"). **Pre-computed target:** the hover phugoid the
  rate damper left at +0.024 → −0.188 once the attitude loop closes (the loop with
  authority over it). Seam discipline, 2nd application: **off-seam (5 m/s,
  trustworthy)** the nonlinear march returns to trim and holds (θ 5°→<0.4°), the
  damper doesn't; **across the seam (hover, honest)** it beats the damper (which
  diverges to NaN) and holds pitch/roll bounded, but a slow residual drift remains
  in yaw — the same wake-skew coupling the hover Jacobian can't see, confirmed and
  documented not fudged. Sustained disturbance: regulated to a bounded offset where
  the damper diverges (proportional ⇒ residual; integral would zero it — done in
  5l). Scope: regulate-to-trim, NOT command tracking/guidance. `helisim attitude`.
- **PI attitude hold (5l):** integral action (two integrator states, vector → 13)
  closing 5k's residual — a correctness fix to the attitude loop *before* the
  velocity loop is built on it. **Falsifiable oracle:** the proportional standing
  offset under a sustained 0.6 N·m disturbance (1.69°) → **≈0 (0.09°)** with the
  integrator — zero steady-state error (nonlinear, the real 5l result). The 13-state
  augmented loop is stable but only **marginally** (max Re ≈ −0.001, the integrator's
  near-origin pole, gain-independent); firm margin is 5m's job. **Scope boundary made
  concrete:** attitude → 0 but forward speed drifts ~1.6 m/s (thrust tilt to counter
  the moment accelerates the aircraft) — attitude hold ≠ velocity hold; that drift
  is what 5m closes. Anti-windup named, not yet needed. `helisim attitude`.
  ⚠ A char-poly eigensolver artifact (below) had reported −0.062 and a spurious
  "kI lower-bound"; the QR solver corrected both. Lesson: don't trust char-poly >10×10.
- **QR eigensolver:** `eigenvalues` now uses the QR algorithm (Hessenberg + Francis
  double-shift, `dynamics/schur.rs`) instead of char-poly + Durand–Kerner, which is
  ill-conditioned past ~10×10 and gave *wrong* eigenvalues for the 13–15-state
  augmented control systems (it agreed at ≤11, so 5c–5k stand). Validated against a
  known 15×15 spectrum. This is what makes the 5m timescale-separation gate trustworthy.
- **Velocity/position hold (5m):** the outermost cascade (velocity error → attitude
  command → 5k/5l → 5j), 15 states. **Timescale separation named before tuning and
  confirmed:** three eigenvalue clusters (rate/inflow |λ|≈8–74, attitude ≈1.3,
  velocity ≈0.22) separated 6.7× and 5.8× (≥3× — loops don't fight); all LHP
  (−0.098). **Tracking target hit:** the ~1.6 m/s drift 5l left under 0.6 N·m → ≈0
  (0.019 m/s). **Capstone across the seam:** at hover the open-loop-unstable-both-axes
  aircraft arrests a velocity kick, returns to station (pos→≈0) and holds hands-off,
  and the yaw/wake-skew seam-residual that survived 5j/5k/5l no longer runs away
  (position feedback has authority over the slow drift the inner loops couldn't reach).
  Anti-windup still not needed (θ_cmd≈0.8°). `helisim hover`.
  **⚠ Calibration caveat (validation ledger):** forward-flight *power* now carries
  one tuned constant — the induced factor κ, fixed at hover so the cross-check
  stays exact. The bucket *shape* and high-μ *positivity* are real, but absolute
  forward power is anchored to a hover calibration, NOT an independent emergent
  result. Do not treat any power-derived quantity as an independent check. The
  force/moment residuals (thrust=weight, the moments) carry no such constant and
  remain independent — so anything derived from forces/moments (e.g. the 5c
  stability derivatives Mu, Mq, Zw, Xu) is a clean check; power-derived is not.
- **Cell discharge (Samsung INR18650-25R):** OCV-SoC + R fitted to the low-rate
  curve and the 20 A energy point (7.83 Wh); the model then *predicts* 0.5/5/10 A
  delivered capacity (~2500/2480/2460 mAh), reproducing the cell's flat capacity
  and monotonic voltage sag. Fitted R ≈ 21 mΩ (DC IR 14.8 mΩ + lumped
  polarisation).
- **Forward flight (Glauert):** the inflow solver reproduces the Glauert
  closed-form induced inflow `λ_i = √((−μ²+√(μ⁴+C_T²))/2)` to <1e-6 (hover and
  high-speed limits too). At constant thrust, rotor power falls below hover
  (induced-power collapse); overlaying a representative airframe parasite
  `0.5·(f/A)·μ³` recovers the classic dip-then-rise power bucket. Headline: a
  rigid blade at μ=0.3 makes the advancing half carry ~3.7× the retreating
  half's thrust → a large uncommanded rolling moment (≈245 N·m on the C&T rotor),
  pitching ≈0 for uniform inflow. Reverse-flow disk tiny at low μ, growing with
  speed; its lift is nulled.
- **Flapping (Lock number):** the harmonic-balance coefficients match the textbook
  closed form `β0,β1c,β1s(γ,μ,θ,λ)` (<1e-3 rad). The 90° phase lag emerges
  unprompted — hover cosine cyclic θ1c → pure sine flap (β1c≈0, β1s≈−θ1c). The
  rigid rolling moment is reacted by flapping: hub moment 0 for a central hinge,
  residual ∝ hinge offset; the moment reappears as TPP blow-back (β1c).
- **Cell thermal (18650):** specific heat 900 J/(kg·K) (literature 800–1100);
  with natural convection, a 20 A discharge hits the 75 °C protection limit
  before emptying while 10 A stays cooler and empties on voltage — matching the
  Batemo free-convection test's which-limit-terminates behaviour. Heat = I²R
  using the cell's own R (consistent with the voltage sag, so it equals the
  dissipated electrical energy); entropic term neglected.

## Electric-hover chain conventions (mission)

- Hover is a constant-power load (controller holds RPM). `P_elec = P_mech / η`.
- Coupled solve: `P_elec = V·I`, `V = OCV(SoC) − I·R` → bisection on the monotone
  power-balance residual over `I ∈ [0, OCV/2R]`; infeasible if `P_elec > OCV²/4R`.
- C-rate (1/h) = cell current / cell capacity (Ah); check vs continuous rating.
- Pack scaling: `V = S·cell`, `Ah = P·cell`, `R = (S/P)·cell`, mass ∝ S·P.
- Thermal: per-cell `C dT/dt = I_cell²·R − h·A·(T−T_amb)`. Climb power modelled as
  `P_hover + W·V_climb` (energy bound; proper climb BEMT arrives with forward
  flight). Key finding: in a sustained climb the **75 °C thermal limit bites
  before the 8C current limit** — the C-rate check alone is not a safety check.

## Commands

- `cargo build` / `cargo test` — build / run all tests.
- `cargo run` — full validation report.
- `cargo run -- spanwise` — C&T θ=8° spanwise loading/inflow dump.
- `cargo run -- harrington` — Harrington figure-of-merit sweep.
- `cargo run -- study` — C_T sensitivity diagnostic (a0 / compressibility / tip loss).
- `cargo run -- forward` — forward-flight sweep over advance ratio: power bucket
  + the rigid-blade rolling moment.
- `cargo run -- flapping` — blade flapping: rolling moment → tip-path-plane tilt,
  hinge-offset residual, and the gyroscopic 90° phase lag.
- `cargo run -- trim` — steady-flight trim: hover cross-check + forward sweep.
- `cargo run -- dynamics` — hover stability derivatives + eigenvalue modes
  (shows the open-loop hover instability).
- `cargo run -- sim` — nonlinear time-march of a perturbed hover vs the linear
  eigenvalue gate (matches, then departs into nonlinearity).
- `cargo run -- lateral` — lateral-directional hover oracle + coupled 8-state
  decouple/couple gate.
- `cargo run -- coupled` — nonlinear 8-state march: fixed point, Jacobian↔coupled8
  eigenvalues, and track-then-depart vs the 8-D linear prediction.
- `cargo run -- inflow` — Pitt–Peters dynamic inflow (5h): τ→0 recovers the
  quasi-static baseline, and the off-axis cyclic response sign flip.
- `cargo run -- fly` — control-input time histories (5i): control effectiveness +
  conventions, the off-axis time-domain sign flip, and open-loop divergence to a
  control pulse.
- `cargo run -- sas` — stability augmentation (5j): off-seam trustworthy design,
  hover damping with residual, and the nonlinear hover hold across the seam.
- `cargo run -- attitude` — attitude hold (5k) + PI integral action (5l): phugoid
  → LHP, off-seam regulation, hover seam-residual, and zero steady-state error.
- `cargo run -- hover` — velocity/position hold (5m): timescale-separation clusters,
  drift→0, and the hands-off hover position-hold capstone.
- `cargo run -- mission` — end-to-end electric hover: power → C-rate → endurance,
  plus a disk-loading design-tension sweep.
- `cargo run -- design` — model-scale sizing study: the priority vector (safety →
  airtime → efficiency → noise) at a starter point + a radius/disk-loading sweep
  showing the sweet spot (not a monotone "bigger is better"). Composes the
  autorotation, acoustics and BEMT-trim cores.
- `cargo run -- build` — the end goal: recommend a design, then emit the COMPLETE
  build package — every part sized from the design (mast by torsion, boom by
  bending, etc.), the assembly sequence, and exported STL (printable blade) + DXF
  (cuttable section) files to `build_output/`.
