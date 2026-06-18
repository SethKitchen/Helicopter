//! Resonance (Campbell / fan-plot) check — the gap a flexible printed structure
//! makes critical: a part whose natural frequency lands on a rotor per-rev harmonic
//! is driven to large amplitude and fails, even with positive static margins.
//!
//! The beam FEA is static (no mass matrix), so the fundamental bending frequency is
//! taken from the **closed-form cantilever** result (the clean oracle):
//!
//! `f₁ = (β₁²/2π)·√(EI / (μ L⁴))`,  `β₁ = 1.875104` (first clamped-free eigenvalue),
//!
//! with `μ` the mass per unit length. For the spinning blade the centrifugal tension
//! raises the flap frequency — the **Southwell** relation `f_rot² = f₀² + K·(Ω/2π)²`
//! (K ≈ 1 for the first flap mode). The aircraft is excited at the rotor harmonics
//! `n·Ω/2π` (n = 1…N_blades+1; 1/rev imbalance and N/rev blade passage are the worst);
//! a part is flagged if its frequency sits within ±10 % of any harmonic.
//!
//! If a piece is infeasible (resonant, or a boom fundamental below 1/rev), the report
//! says so and names the feasible fix (stiffer section / carbon-fibre tube).

use crate::fea_structural::as_built_blade_modulus;
use crate::materials::{E_AL, RHO_AL, SIGMA_ALLOW_AL};
use crate::naca_section::flap_inertia;
use crate::sizing::{BOOM_TARGET_PER_REV, TUBE_A_COEFF, TUBE_I_COEFF, boom_governing_od};
use helisim_design::{DesignCandidate, DesignReport};
use std::f64::consts::PI;

/// First clamped-free (cantilever) eigenvalue βL.
const BETA1: f64 = 1.875_104;
/// Southwell coefficient for the first flap mode (centrifugal stiffening ≈ +1/rev²).
const SOUTHWELL_K: f64 = 1.0;
/// Resonance band: a frequency within ±this fraction of a harmonic is flagged.
const BAND: f64 = 0.10;

/// Closed-form cantilever fundamental bending frequency, Hz.
pub fn cantilever_fundamental_hz(ei: f64, mu: f64, length: f64) -> f64 {
    if mu <= 0.0 || length <= 0.0 {
        return 0.0;
    }
    BETA1 * BETA1 / (2.0 * PI) * (ei / (mu * length.powi(4))).sqrt()
}

/// Rotating-blade flap frequency by the Southwell relation, Hz.
pub fn rotating_flap_hz(f0_hz: f64, omega_rad_s: f64) -> f64 {
    let f_rev = omega_rad_s / (2.0 * PI);
    (f0_hz * f0_hz + SOUTHWELL_K * f_rev * f_rev).sqrt()
}

/// True if `f` sits within ±`BAND` of any integer harmonic `n_lo..=n_hi` of `rev_hz`.
/// (The blade FLAP is excluded from 1/rev — a flap mode is ~1/rev BY DESIGN, the very
/// flapping the aero stack models; the danger is 2/rev and up.)
fn is_resonant(f_hz: f64, rev_hz: f64, n_lo: usize, n_hi: usize) -> bool {
    if rev_hz <= 0.0 {
        return false;
    }
    let nu = f_hz / rev_hz; // per-rev order
    (n_lo..=n_hi).any(|n| (nu - n as f64).abs() <= BAND)
}

/// The resonance report.
#[derive(Clone, Debug)]
pub struct ResonanceReport {
    /// Rotor speed, 1/rev, Hz.
    pub rotor_hz: f64,
    /// Number of blades (sets the blade-passage harmonic).
    pub n_blades: usize,
    /// Rotating blade flap frequency, Hz, and its per-rev order.
    pub blade_flap_hz: f64,
    pub blade_per_rev: f64,
    /// Tail-boom fundamental bending frequency, Hz, and per-rev order.
    pub boom_hz: f64,
    pub boom_per_rev: f64,
    /// Resonance flags.
    pub blade_resonant: bool,
    pub boom_resonant: bool,
    /// Everything clear of the harmonics?
    pub feasible: bool,
    /// Findings + feasible-fix recommendations.
    pub notes: Vec<String>,
}

/// Run the resonance check for a design (uses the AS-BUILT blade modulus and the
/// stiffness-governed boom OD, so the frequencies match the parts actually built).
pub fn analyze_resonance(c: &DesignCandidate, report: &DesignReport) -> ResonanceReport {
    let omega = c.omega();
    let rev_hz = omega / (2.0 * PI);
    let n_blades = c.n_blades;

    // --- blade flap (rotating) ---
    let span = c.radius_m - c.root_cutout * c.radius_m;
    let (_e_mat, e_eff, _f_wall) = as_built_blade_modulus(c);
    let ei_blade = e_eff * flap_inertia(c.chord_m);
    let mu_blade = c.blade_areal_density_kg_m2 * c.chord_m;
    let f0_blade = cantilever_fundamental_hz(ei_blade, mu_blade, span);
    let blade_flap_hz = rotating_flap_hz(f0_blade, omega);
    let blade_per_rev = if rev_hz > 0.0 {
        blade_flap_hz / rev_hz
    } else {
        0.0
    };
    // The flap is ~1/rev by design (the flapping the aero stack models); the dangerous
    // resonances are 2/rev … (N+1)/rev.
    let blade_resonant = is_resonant(blade_flap_hz, rev_hz, 2, n_blades + 1);

    // --- tail boom (non-rotating cantilever, excited by the main rotor harmonics) ---
    let torque = if report.hover_shaft_power_w.is_finite() && omega > 0.0 {
        report.hover_shaft_power_w / omega
    } else {
        1.0
    };
    let boom_len = 1.15 * c.radius_m;
    let target_hz = BOOM_TARGET_PER_REV * rev_hz;
    let od = boom_governing_od(
        torque,
        boom_len,
        E_AL,
        RHO_AL,
        SIGMA_ALLOW_AL,
        0.02,
        target_hz,
    );
    let ei_boom = E_AL * TUBE_I_COEFF * od.powi(4);
    let mu_boom = RHO_AL * TUBE_A_COEFF * od * od;
    let boom_hz = cantilever_fundamental_hz(ei_boom, mu_boom, boom_len);
    let boom_per_rev = if rev_hz > 0.0 { boom_hz / rev_hz } else { 0.0 };
    let boom_resonant = is_resonant(boom_hz, rev_hz, 1, n_blades + 1);

    // --- findings + feasible fixes ---
    let mut notes = Vec::new();
    if blade_resonant {
        notes.push(format!(
            "⚠ Blade flap at {:.1}/rev is within ±{:.0}% of a higher harmonic — RETUNE: change root \
             chord/thickness (raises f) or blade count so the flap order avoids 2…{}/rev.",
            blade_per_rev,
            BAND * 100.0,
            n_blades + 1
        ));
    } else {
        notes.push(format!(
            "Blade flap {:.0} Hz ({:.1}/rev) — ~1/rev is the design flap; clear of 2…{}/rev.",
            blade_flap_hz,
            blade_per_rev,
            n_blades + 1
        ));
    }
    if boom_resonant || boom_per_rev < 1.0 {
        notes.push(format!(
            "⚠ Tail boom fundamental {:.0} Hz ({:.1}/rev) is {} — REPLACE the aluminium tube with a \
             carbon-fibre tube (E/ρ ~3× → ~1.7× higher frequency for the same OD) or step the OD up \
             a size; re-check until it clears the band.",
            boom_hz,
            boom_per_rev,
            if boom_per_rev < 1.0 { "below 1/rev (too floppy)" } else { "near a harmonic" }
        ));
    } else {
        notes.push(format!(
            "Tail boom {:.0} Hz ({:.1}/rev) — clear of the main-rotor harmonics.",
            boom_hz, boom_per_rev
        ));
    }

    let feasible = !blade_resonant && !boom_resonant && boom_per_rev >= 1.0;
    ResonanceReport {
        rotor_hz: rev_hz,
        n_blades,
        blade_flap_hz,
        blade_per_rev,
        boom_hz,
        boom_per_rev,
        blade_resonant,
        boom_resonant,
        feasible,
        notes,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use helisim_airfoil::LinearAirfoil;
    use helisim_bemt::Config;
    use helisim_design::evaluate;

    /// ORACLE: the closed-form cantilever fundamental matches the published formula
    /// `f₁ = (β₁²/2π)√(EI/μL⁴)` — check a steel ruler-ish beam against a hand value.
    /// A 1 m steel beam, EI = 200e9 · (0.02·0.005³/12) = 41.67 N·m², μ = 7850·0.02·0.005
    /// = 0.785 kg/m → f₁ = (1.875104²/2π)·√(41.67/0.785) ≈ 4.07 Hz.
    #[test]
    fn cantilever_frequency_matches_closed_form() {
        let e = 200.0e9;
        let i = 0.02 * 0.005_f64.powi(3) / 12.0;
        let mu = 7850.0 * 0.02 * 0.005;
        let f = cantilever_fundamental_hz(e * i, mu, 1.0);
        assert!((f - 4.07).abs() < 0.1, "got {f} Hz");
    }

    #[test]
    fn rotation_raises_the_flap_frequency() {
        let f0 = 5.0;
        let f_static = rotating_flap_hz(f0, 0.0);
        let f_spun = rotating_flap_hz(f0, 100.0); // 100 rad/s ≈ 955 rpm
        assert!((f_static - f0).abs() < 1e-9);
        assert!(
            f_spun > f0,
            "Southwell stiffening must raise the flap frequency"
        );
    }

    #[test]
    fn resonance_detected_on_a_harmonic_and_cleared_off_it() {
        let rev = 20.0; // 1/rev = 20 Hz
        assert!(is_resonant(40.5, rev, 1, 4), "40.5 Hz ≈ 2/rev → resonant");
        assert!(!is_resonant(50.0, rev, 1, 4), "50 Hz = 2.5/rev → clear");
        // 1/rev excluded for the blade flap (n_lo = 2): 20 Hz is the design flap, not a fault.
        assert!(!is_resonant(20.0, rev, 2, 4), "1/rev excluded when n_lo=2");
        assert!(
            is_resonant(20.0, rev, 1, 4),
            "1/rev caught when n_lo=1 (boom)"
        );
    }

    #[test]
    fn report_runs_and_flags_feasibility() {
        let c = DesignCandidate::model();
        let r = evaluate(&c, &LinearAirfoil::naca0012(), &Config::default());
        let res = analyze_resonance(&c, &r);
        assert!(res.rotor_hz > 0.0);
        assert!(res.blade_flap_hz > 0.0 && res.boom_hz > 0.0);
        assert_eq!(res.notes.len(), 2);
        // feasible iff neither part is resonant and the boom clears 1/rev.
        assert_eq!(
            res.feasible,
            !res.blade_resonant && !res.boom_resonant && res.boom_per_rev >= 1.0
        );
    }
}
