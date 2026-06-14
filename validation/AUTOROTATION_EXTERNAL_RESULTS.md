# Autorotation external validation — results (R22)

Comparison run **after** locking `AUTOROTATION_EXTERNAL_PREREG.md`. Oracle: Robinson
R22 POH / type literature (best glide **75 KIAS**, ~**4:1** glide ratio; minimum
rate of descent **53 KIAS**). Test: `crates/autorotation/tests/r22_external_validation.rs`.

## Result table

| Quantity | Model (locked) | Published R22 | Error | Pre-registered? |
|----------|----------------|---------------|-------|-----------------|
| Min-sink airspeed | 24.5 m/s (48 kt) | 53 KIAS (27.3 m/s) | **−9%** | ✓ right order |
| Best-glide airspeed | 45.0 m/s (87 kt) | 75 KIAS (38.6 m/s) | **+16%** (over) | ✓ predicted over-prediction |
| Best-glide ratio | 3.58 : 1 | ~4 : 1 | **−11%** | ✓ clean |
| Best-glide > min-sink | yes | yes | exact | ✓ clean claim holds |
| Forward min-sink < vertical | yes (1904 < 2871 fpm) | (physical) | exact | ✓ clean claim holds |
| Vertical V_d/v_h | 1.99 | (top of measured band) | — | ✓ |

## Reading

**The clean, calibration-free claims passed exactly.** Speed ordering
(best-glide > min-sink) and forward-slower-than-vertical need no assumed input;
they are kinematic/force facts of the glide polar, and they match — the part that
would have exposed a real derivation bug.

**The power-derived magnitudes landed within ~10–16%, with the error in the
pre-registered direction.** Best-glide speed over-predicted exactly as called
(the assumed flat-plate area f sets where the parasite term turns the bucket up).
Min-sink speed came out *better* than the conservative prereg band (−9%). The
glide RATIO — relatively insensitive to absolute power calibration — matched to
11%, the tightest magnitude agreement, which is the expected pattern.

**Error attribution (the methodology's promise).** The two assumed inputs (C_d0,
f) and the profile-power approximation own the residual; nothing is fudged. A
better C_d0/f (sourced, not guessed) would tighten the magnitudes; the ordering
needs nothing.

## Honest caveats / scope
- Two input parameters (C_d0, f) are assumptions, not sourced — this is a
  "right order + right ordering + error attributable to named inputs" validation,
  not a precision match. Stated in the prereg before the comparison.
- KIAS treated as ≈ TAS at low altitude (calibration/altitude effects ignored).
- This validates the **steady** glide polar only; the dynamic flare and the
  height-velocity envelope are separate (next).
- Power-derived quantities carry the project's standing power-calibration caveat;
  the ordering/force claims do not.
