# Autorotation external validation — pre-registration (LOCK BEFORE ORACLE)

Scope: external check of the **autorotation crate** (a safety side-track), NOT the
core-aero Milestone 6. Same discipline applies: predictions + parameter mapping
are fixed here, *before* any published autorotation result is looked at. Hard rule
in force: **never fabricate an oracle — source and cite, or don't claim a match.**

## Aircraft & parameter mapping (decided on physics, locked)

Robinson **R22** — chosen because its autorotation reference speeds are public
(POH / type literature) and its geometry is widely published. Parameter mapping
(every value an explicit assumption, since we are not fitting):

| Parameter | Value | Basis / assumption |
|-----------|-------|--------------------|
| Gross mass | 621 kg (~1370 lb) | published MGTOW |
| Rotor radius R | 3.84 m (25.2 ft dia) | published |
| Air density ρ | 1.225 kg/m³ | sea-level standard |
| Tip speed ΩR | 213 m/s (~530 rpm) | published rotor RPM × R |
| Solidity σ | 0.060 | 2 blades, ~0.18 m chord |
| Mean profile C_d0 | 0.010 | **assumed** (no authoritative source) |
| Flat-plate area f | 0.50 m² | **assumed** (light-heli range 0.3–0.6 m²) |

Two parameters (C_d0, f) are *assumptions*, not sourced — so this is a "right
order + right ordering + error attributable to the named coarse inputs"
validation, **not** a precision match. That is stated up front, not as an excuse
after the fact.

## Model predictions (computed from the code above, LOCKED)

- **Vertical autorotation:** V_d = 14.6 m/s = **2871 fpm**, V_d/v_h = 1.99.
- **Forward min-sink:** **1904 fpm @ 24.5 m/s (48 kt)**.
- **Best-glide:** angle **15.6°** @ 45.0 m/s (**87 kt**), RoD 2466 fpm.

## Predicted comparison to the (not-yet-seen) oracle — falsifiable claims

1. **Min-sink RoD** will be the right order (1500–2500 fpm) but **over-predicted**
   by ~10–25%, because the assumed profile power (≈41 kW → a 6.75 m/s descent-rate
   floor) is the dominant term and C_d0 is guessed high-ish. This is the same
   power-calibration caveat the rest of the project carries — power-derived
   quantities are not clean checks.
2. **Best-glide speed** will be **over-predicted** (model 87 kt vs an expected
   published value nearer 60–70 kt), because the assumed flat-plate area f sets
   where the parasite term turns the bucket up; a high f pushes best-glide fast.
3. **Ordering will be correct** (clean, force/kinematics-based, no caveat):
   best-glide speed > min-sink speed, and forward min-sink RoD < vertical RoD.
4. **Vertical V_d/v_h ≈ 2.0** sits at the top of the measured ideal band — expected,
   since the R22 is mildly profile-heavy.

A mismatch on (1)/(2) is **not failure** — it measures the assumed C_d0/f and the
profile-power approximation, each named. A mismatch on (3) WOULD be a real failure
(it needs no calibrated input). That asymmetry is the point of pre-registering.

## What would falsify the model
- Best-glide speed ≤ min-sink speed → kinematic/derivation bug (serious).
- Forward min-sink ≥ vertical descent rate → glide-polar sign error (serious).
- RoD off by >2× → more than the coarse-input story can carry.
