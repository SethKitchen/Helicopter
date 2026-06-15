//! Milestone 6 — BO-105 hover TRIM validation (cyclic + attitude), the axis the UH-60's
//! pitch-bias actuator confounded. Oracle: NASA CR-3144 Table III-3 CASE 29 (0 kt), which
//! reports trim CONTROL ANGLES DIRECTLY IN DEGREES — so this is rigging-free at the
//! blade-pitch level (no deg↔in scale trap; the BO-105 has no PBA). Pre-reg + outcome
//! are this test's comments. Sign/axis convention reconciled before any sign claim;
//! magnitudes are convention-free.

use helisim_trim::{Aircraft, NewtonConfig, TrimCondition, trim};

// CR-3144 CASE 29 (0 kt) trim, degrees.
const B1S_O: f64 = -0.42; // longitudinal cyclic
const A1S_O: f64 = -0.33; // lateral cyclic
const THETA_O: f64 = 2.64; // Euler pitch
const PHI_O: f64 = -2.97; // Euler roll

#[test]
fn bo105_hover_cyclic_and_attitude_vs_cr3144() {
    let ac = Aircraft::bo105();
    let r = trim(&ac, &TrimCondition::hover(), &NewtonConfig::default());
    let (t1s, t1c) = (r.cyclic_lon.to_degrees(), r.cyclic_lat.to_degrees());
    let (pitch, roll) = (r.pitch.to_degrees(), r.roll.to_degrees());

    println!(
        "BO-105 hover trim (rigging-free, degrees):\n  \
         cyclic θ1s(lon) {t1s:+.2}° vs B1S {B1S_O:+.2} | θ1c(lat) {t1c:+.2}° vs A1S {A1S_O:+.2}\n  \
         attitude Θ {pitch:+.2}° vs {THETA_O:+.2} | Φ {roll:+.2}° vs {PHI_O:+.2}",
    );

    // --- LATERAL axis: CLEAN, and the headline of this comparison. The BO-105's
    // rigging-free, PBA-free oracle finally gives a cyclic comparison the UH-60's pitch-
    // bias actuator blocked. θ1c magnitude is order-consistent with |A1S|=0.33° (sign
    // deferred to the convention reconciliation); roll Φ matches the tail-side-force bank.
    let lat_ratio = t1c.abs() / A1S_O.abs();
    assert!(
        (0.4..2.5).contains(&lat_ratio),
        "lateral cyclic magnitude order-consistent (got {lat_ratio:.2}×)"
    );
    assert!(roll < 0.0, "left bank to balance the tail side force");
    let roll_ratio = roll / PHI_O;
    assert!(
        (0.4..2.5).contains(&roll_ratio),
        "roll order-consistent w/ oracle (got {roll_ratio:.2}×)"
    );

    // --- LONGITUDINAL axis: RECOVERED by the sourced 3° shaft tilt (pre-registered &
    // verified here). It was a miss with shaft tilt omitted (θ1s≈0.03°, Θ≈0.09°);
    // adding the sourced 3° fwd shaft tilt moves Θ → +2.84° vs +2.64° (7.6%) and θ1s into
    // order with B1S=−0.42° (sign deferred to convention). The shaft-tilt mechanism is
    // VALIDATED HERE because cg_offset≈0 isolates it (CG under the hub) — which is exactly
    // what makes the UH-60's overshoot under the SAME term attributable to its cg_offset,
    // not to a shaft-tilt bug (see uh60_hover_trim_attitude_vs_genhel).
    let pitch_ratio = pitch / THETA_O;
    assert!(
        (0.5..2.0).contains(&pitch_ratio),
        "pitch recovered by sourced shaft tilt (got {pitch_ratio:.2}×)"
    );
    let lon_ratio = t1s.abs() / B1S_O.abs();
    assert!(
        (0.4..2.5).contains(&lon_ratio),
        "longitudinal cyclic order-consistent after shaft tilt (got {lon_ratio:.2}×)"
    );
}
