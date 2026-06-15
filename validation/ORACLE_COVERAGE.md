# Oracle coverage map

Every validated quantity, the **documented example number** it is checked against,
and the source. Oracle types:
**A** = external published number · **B** = closed-form / analytic identity ·
**C** = self-consistency (route-vs-route, monotonicity, composition) ·
**D** = structural.

A claim of "it matches" is a passing `#[test]`; run `cargo test`.

## Aerodynamics & dynamics core

| Module | Type | Documented number(s) matched | Source |
|--------|------|------------------------------|--------|
| bemt / validation | A | C&T hover C_T: 0.00213 (5°), 0.00474 (8°), 0.00796 (12°) @ M_tip 0.439; BEMT over-predicts (documented) | Caradonna & Tung 1981, NASA TM-81232 |
| bemt / validation | A | Harrington Rotor-1 peak figure of merit ∈ [0.62, 0.75] | Harrington 1951, NACA TN-2318 |
| validation | B | per-station inflow vs analytic BEMT closed form, <2% | Leishman |
| airfoil | A | NACA0012: a₀ = 5.73/rad (0.10/deg), C_lmax ≈ 1.4, C_d0 ≈ 0.0065 | Abbott & von Doenhoff; Prouty |
| forward | A/B | Glauert inflow closed form λ_i=√((−μ²+√(μ⁴+C_T²))/2), <1e-6; power bucket | Glauert |
| flapping | A/B | Lock-number closed form (β₀,β₁c,β₁s); 90° phase lag | Leishman/Johnson/Prouty |
| trim | B/C | hover trim = standalone BEMT (two routes, ~0.3% power); force/moment balance | internal cross-check |
| dynamics | A | UH-60A hover derivs vs GENHEL: Zw 12%, Nr 1.5%, Lp 3% (post gyro-fix), Mu sign+order | NASA TM 85890 (GENHEL) |
| dynamics | B | eigenvalues vs analytic hovering cubic; known 15×15 spectrum (QR) | analytic |
| sim | C | nonlinear march reproduces 5c eigenvalue (period 7.1 vs 6.97 s, σ 0.503 vs 0.505) | route-vs-route |

## Electric powertrain

| Module | Type | Documented number(s) matched | Source |
|--------|------|------------------------------|--------|
| cell | A | Samsung 25R: 7.83 Wh @ 20 A; predicts 2500/2480/2460 mAh @ 0.5/5/10 A; R≈21 mΩ | Samsung INR18650-25R datasheet |
| pack | A/B | 6S2P 25R: 21.6 V, 5.0 Ah, **108 Wh**, 63 mΩ, 540 g, 40 A (8C) | datasheet + S/P scaling |
| powertrain | A/B | η = 0.85 motor × 0.95 ESC = 0.80; 1000 W mech → 1250 W elec | T-Motor / ESC benchmarks |
| thermal | A | specific heat 900 J/(kg·K) ∈ [800,1100]; 20 A hits 75 °C limit, 10 A doesn't | 18650 characterization; Batemo |
| thermal | A/B | convection h: natural 7.5 ∈ [5,10], forced 40 ∈ [30,60] (Nu·k/D) | Incropera & DeWitt |
| mission | A/B | hover cool / sustained climb over-temps; endurance = Wh/P | composition of above |

## Safety, FEA & manufacturing

| Module | Type | Documented number(s) matched | Source |
|--------|------|------------------------------|--------|
| autorotation | A | R22: best-glide 75 KIAS (model 87, +16%), min-sink 53 KIAS (48, −9%), glide ratio 4:1 (3.6, −11%) | Robinson R22 POH (pre-registered) |
| autorotation | A/B | ideal vertical autorotation V_d/v_h ∈ [1.7,2.0]; windmill momentum quadratic 1e-9 | Leishman / Prouty |
| acoustics | A | Bessel J_n vs tabulated: J₀(1)=0.7652, J₁ max 0.5819, zeros 2.4048/3.8317; J₅(5)=0.2611 | Abramowitz & Stegun |
| acoustics | B | Gutin on-axis null, off-axis directivity peak, ∝M_tip³ tip-speed lever | Gutin 1936 |
| fea (beam) | B | cantilever PL³/3EI (exact); simply-supp PL³/48EI; distributed qL⁴/8EI; string qL²/8T | Euler-Bernoulli theory |
| fea (cst) | B | patch test (uniform σ exact); uniaxial bar σ=F/A, δ=FL/AE | CST theory |
| manufacture (mast) | B | torsion d=(16Q/πτ)^⅓; τ=16Q/πd³ at allowable | Shigley |
| manufacture (boom) | B | thin-tube Z = π·0.5904·D³/32 ≈ 0.058 D³ (wall 0.1D) | Roark |
| manufacture (fasteners) | A | bolt stress areas = ISO 724 (M3 5.03 mm²…); working shear = 0.6·800/2.4 MPa | ISO 724 / ISO 898-1 |
| manufacture (struct) | B | blade centrifugal F_cf=ω²·m·r_cg; margins; Al allowables (MMPDS/ASM) | mechanics + materials data |
| step B-rep | B | closed genus-0 solid Euler V−E+F=2; every edge used 2× (manifold) | topology |

## Honest remaining gaps (named, not faked)

These rely on self-consistency / closed-form where a clean **external measured**
number is genuinely hard to source without the careful-sourcing discipline of
Milestone 6 — listed so coverage is not over-claimed:

- **control gains (sim 5j–5m)**: ✅ ADDRESSED — `closed_loop_damping_vs_ads33_level1`
  checks ζ against the published **ADS-33E / MIL-F-9490D Level-1 ζ≥0.35**. Finding:
  the velocity/position modes meet it (ζ 0.45–0.76); the body modes sit at ζ≈0.10
  (documented limitation — rate gains tuned for timescale separation, not inner
  damping; raising them is the named next step).
- **acoustics**: ⏳ OPEN — no external *measured-SPL* datapoint (only Bessel tables
  + Gutin closed form + directivity). Sourcing attempted (NASA NTRS 19700005920
  "A Review of Aerodynamic Noise From Propellers" is a scanned PDF, not reliably
  text-extractable via available tools; CEAS/Springer flyover papers are paywalled).
  Per the no-fabrication rule, NOT anchored with an invented number — this stays a
  Milestone-6-style careful-sourcing task (find one published Gutin worked example
  or measured rotor SPL with full geometry, pre-register, then compare).
- **forward / coupled**: advancing-vs-retreating split and the flap↔inflow loop
  are validated by closed forms + consistency, not wind-tunnel load distributions.
- **design / cost**: compositions — no external design/cost baseline (appropriate;
  they delegate to validated cores). Cost unit prices are named inputs, not quotes.
- **FEA**: ✅ ADDRESSED at assembly level — `combined_loads_superpose_exactly`
  (linear FE: combined = Σ individual = PL³/3EI + qL⁴/8EI) and
  `a_soft_outboard_segment_adds_tip_compliance` (per-element EI assembly). Remaining
  element types (plate-bending / curved shell) named as future work.

- **battery / bms (battery-bms branch)**: ✅ cell models for four 21700 cells
  (Molicel P50B, Ampace JP40, BAK 45D, EVE 40PL) from sourced datasheets + Battery
  Mooch measured DCIR. **EXTERNAL validation** (`crates/bms/tests/battery_external_validation.rs`,
  prereg `validation/BATTERY_EXTERNAL_PREREG.md`, results `..._RESULTS.md`): **P1
  capacity retention at 10C 96–98 % vs sourced ≥95 % — clean match**; **P4 emergent
  continuous: steady-state still-air surface limit reproduces JP40 (47 vs 45 A) and
  P50B (36 vs 35 A) to ~4 %, emergent** (temperature-dependent `R` load-bearing),
  BAK over-predicted (45 vs 30, its rating conservative — believed not patched). The
  prereg's P4 was FALSIFIED (surface criterion meaningless at high rate — skin lags
  core) and the disagreement believed, surfacing the 2-node core-vs-surface finding.
  Named GAPS (no fabrication): per-rate delivered mAh / loaded-voltage-sag curves
  (paywalled / datasheet-graph only) → P3 left open; **per-cell measured OCV curves**
  (About:Energy login-gated) → shared representative NMC curve retained; **published
  eVTOL pack** spec (proprietary) → pack-level external validation deferred.

## Test coverage

`cargo llvm-cov` (a dev tool, not a project dependency): **99.28% line / 99.88%
function / 99.06% region**, up from 81.3% after a CLI lib/bin split + a smoke test
running every subcommand, trait-default and accessor tests, and defensive-branch
tests (infeasible / None / edge paths) with confirmed-number assertions. The
remaining ~63 lines are the irreducible kind: the binary entry point `fn main`
(not run by `cargo test`), defensive non-convergence/singular safety nets that
valid inputs do not reach, and CLI narrative branches the fixed demos never hit.
Pushing to a literal 100% would require testing unreachable code, so it is left
honestly named rather than gamed.
