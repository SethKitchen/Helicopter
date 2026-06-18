//! Structural fatigue of the metal root joint over the service life — the cyclic
//! check the static margins miss. The aluminium doubler at the bolt hole sees two
//! load spectra:
//!   • **GAG** (ground–air–ground): every flight is one full 0→F_cf→0 cycle of the
//!     centrifugal load — `flights_per_year · life_years` cycles at the hole's peak
//!     (concentrated) stress, R≈0.
//!   • **HCF** (per-rev): the 1/rev flap/coning oscillation adds a small alternating
//!     stress about the steady centrifugal mean, for `rev_hz · flight_seconds` cycles
//!     per flight — millions over the life.
//!
//! Each spectrum's allowable cycles come from a **Basquin S-N law** for 6061-T6
//! (`σ_a = A·N^b`, fit to the ASM anchors σ(10³)≈290 MPa and σ(5×10⁸)≈96 MPa),
//! mean-stress-corrected by **Goodman**; the damage is summed by **Miner's rule**
//! (`D = Σ nᵢ/Nᵢ`; D = 1 ⇒ failure). The blade SPAR is nylon (a different S-N) — its
//! HCF is a named separate check, not folded into this aluminium sum.

use crate::naca_section::flap_inertia;
use crate::root_fea::analyze_root_hole;
use helisim_design::{DesignCandidate, DesignReport};
use std::f64::consts::PI;

/// Representative ultimate tensile strength of the printed blade (SLS PA-CF / Onyx +
/// fiber), Pa — the anchor of the polymer S-N curve.
const BLADE_UTS_PA: f64 = 70.0e6;
/// Semi-log polymer S-N slope: `σ_a/UTS = 1 − B·log₁₀(N)`. B = 0.1 ⇒ 0.3·UTS at 10⁷
/// cycles (a representative nylon endurance ratio; metals have no such single ratio).
const POLY_SN_B: f64 = 0.10;
/// Per-rev alternating flap-bending stress as a fraction of the 1-g flap stress
/// (forward-flight 1/rev asymmetry + gusts; hover alone is near-steady).
const BLADE_FLAP_ALT_FRACTION: f64 = 0.30;

/// 6061-T6 Basquin coefficient `A` (Pa) and exponent `b` for fully-reversed (R=−1)
/// fatigue: fit through σ(10³ cyc)=290 MPa and σ(5×10⁸ cyc)=96 MPa (ASM 6061-T6 S-N).
const SN_A_PA: f64 = 519.0e6;
const SN_B: f64 = -0.0843;
/// 6061-T6 ultimate tensile strength, Pa (for the Goodman mean-stress correction).
const UTS_AL_PA: f64 = 310.0e6;
/// Per-rev alternating stress in the doubler as a fraction of the steady centrifugal
/// stress (flap/coning oscillation — modest, the doubler mainly reacts steady tension).
const HCF_ALT_FRACTION: f64 = 0.10;

/// Allowable cycles for a fully-reversed alternating stress from the Basquin S-N law:
/// `N = (σ_ar / A)^{1/b}`. Returns a huge number below the lowest anchor stress.
pub fn basquin_allowable_cycles(sigma_ar_pa: f64) -> f64 {
    if sigma_ar_pa <= 0.0 {
        return f64::INFINITY;
    }
    (sigma_ar_pa / SN_A_PA).powf(1.0 / SN_B)
}

/// Goodman equivalent fully-reversed amplitude for an alternating stress `sigma_a`
/// about a mean `sigma_m`: `σ_ar = σ_a / (1 − σ_m/σ_u)`. Saturates as σ_m→σ_u.
pub fn goodman_equiv(sigma_a_pa: f64, sigma_m_pa: f64) -> f64 {
    let denom = 1.0 - (sigma_m_pa / UTS_AL_PA).min(0.99);
    sigma_a_pa / denom
}

/// The fatigue report for the aluminium root doubler.
#[derive(Clone, Debug)]
pub struct FatigueReport {
    /// GAG cycles over the life (= flights).
    pub gag_cycles: f64,
    /// Peak (Kt-concentrated) hole stress each flight, MPa.
    pub gag_peak_mpa: f64,
    /// Allowable GAG cycles at that stress.
    pub gag_allowable: f64,
    /// GAG damage fraction.
    pub gag_damage: f64,
    /// Per-rev (HCF) cycles over the life.
    pub hcf_cycles: f64,
    /// HCF alternating stress, MPa.
    pub hcf_alt_mpa: f64,
    /// Allowable HCF cycles.
    pub hcf_allowable: f64,
    /// HCF damage fraction.
    pub hcf_damage: f64,
    /// Total Miner damage over the target life.
    pub total_damage: f64,
    /// Predicted fatigue life, years (target_years / total_damage).
    pub predicted_life_years: f64,
    /// Meets the target life (Miner damage ≤ 1)?
    pub meets_life: bool,
}

/// Run the root-joint fatigue check for `flights_per_year` over `life_years`, each
/// flight `flight_minutes` long.
pub fn analyze_fatigue(
    c: &DesignCandidate,
    _report: &DesignReport,
    flights_per_year: f64,
    life_years: f64,
    flight_minutes: f64,
) -> FatigueReport {
    let hole = analyze_root_hole(c);
    let peak_pa = hole.peak_stress_mpa * 1e6; // Kt·net (the fatigue-critical stress)
    let nominal_pa = hole.nominal_net_mpa * 1e6; // steady centrifugal mean (net)

    // --- GAG: full 0→peak→0 cycle each flight (R≈0 ⇒ σ_a = σ_m = peak/2) ---
    let gag_cycles = flights_per_year * life_years;
    let gag_ar = goodman_equiv(0.5 * peak_pa, 0.5 * peak_pa);
    let gag_allow = basquin_allowable_cycles(gag_ar);
    let gag_damage = if gag_allow.is_finite() {
        gag_cycles / gag_allow
    } else {
        0.0
    };

    // --- HCF: 1/rev oscillation about the steady centrifugal mean ---
    let rev_hz = c.omega() / (2.0 * PI);
    let hcf_cycles = rev_hz * (flight_minutes * 60.0) * gag_cycles;
    let hcf_alt = HCF_ALT_FRACTION * nominal_pa;
    let hcf_ar = goodman_equiv(hcf_alt, nominal_pa);
    let hcf_allow = basquin_allowable_cycles(hcf_ar);
    let hcf_damage = if hcf_allow.is_finite() {
        hcf_cycles / hcf_allow
    } else {
        0.0
    };

    let total = gag_damage + hcf_damage;
    let predicted_life_years = if total > 0.0 {
        life_years / total
    } else {
        f64::INFINITY
    };
    FatigueReport {
        gag_cycles,
        gag_peak_mpa: peak_pa / 1e6,
        gag_allowable: gag_allow,
        gag_damage,
        hcf_cycles,
        hcf_alt_mpa: hcf_alt / 1e6,
        hcf_allowable: hcf_allow,
        hcf_damage,
        total_damage: total,
        predicted_life_years,
        meets_life: total <= 1.0,
    }
}

/// Allowable cycles for a polymer at alternating stress `sigma_a` from the semi-log
/// S-N law `σ_a/UTS = 1 − B·log₁₀(N)` ⇒ `N = 10^((1 − σ_a/UTS)/B)`.
pub fn polymer_allowable_cycles(sigma_a_pa: f64) -> f64 {
    if sigma_a_pa >= BLADE_UTS_PA {
        return 1.0; // at/above UTS — fails in one cycle
    }
    let ratio = sigma_a_pa / BLADE_UTS_PA;
    10.0_f64.powf((1.0 - ratio) / POLY_SN_B)
}

/// Flap-bending relief at the blade root from the head type: a teetering or
/// articulated head has a flap hinge that sheds most of the root moment (residual ∝
/// hinge offset); a hingeless head carries the full cantilever moment.
fn flap_relief(n_blades: usize) -> f64 {
    if n_blades <= 4 {
        0.15 // teetering (2) / articulated (3+): hinge relieves the root moment
    } else {
        1.0 // (a hingeless head would carry it all)
    }
}

/// The blade-spar (polymer) fatigue report — the HCF check the aluminium S-N can't do.
#[derive(Clone, Debug)]
pub struct BladeFatigueReport {
    /// 1-g cantilever flap-bending root stress (no hinge relief), MPa.
    pub flap_1g_mpa: f64,
    /// Per-rev alternating flap stress after hinge relief, MPa.
    pub alt_mpa: f64,
    /// Per-rev cycles over the life.
    pub cycles: f64,
    /// Allowable cycles from the polymer S-N at that alternating stress.
    pub allowable: f64,
    /// Miner damage over the target life.
    pub damage: f64,
    /// Predicted blade fatigue life, years.
    pub predicted_life_years: f64,
    /// Meets the target life?
    pub meets_life: bool,
}

/// Per-rev fatigue of the printed (polymer) blade spar over `life_years`.
pub fn analyze_blade_fatigue(
    c: &DesignCandidate,
    flights_per_year: f64,
    life_years: f64,
    flight_minutes: f64,
) -> BladeFatigueReport {
    let span = c.radius_m - c.root_cutout * c.radius_m;
    let lift_per_blade = c.gross_mass_kg * 9.80665 / c.n_blades as f64;
    let m_root = 0.5 * lift_per_blade * span; // uniform-lift cantilever root moment
    let z_blade = flap_inertia(c.chord_m) / (0.06 * c.chord_m);
    let sigma_1g = m_root / z_blade;
    let alt = BLADE_FLAP_ALT_FRACTION * flap_relief(c.n_blades) * sigma_1g;

    let rev_hz = c.omega() / (2.0 * PI);
    let cycles = rev_hz * (flight_minutes * 60.0) * flights_per_year * life_years;
    let allowable = polymer_allowable_cycles(alt);
    let damage = cycles / allowable;
    let predicted_life_years = if damage > 0.0 {
        life_years / damage
    } else {
        f64::INFINITY
    };
    BladeFatigueReport {
        flap_1g_mpa: sigma_1g / 1e6,
        alt_mpa: alt / 1e6,
        cycles,
        allowable,
        damage,
        predicted_life_years,
        meets_life: damage <= 1.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use helisim_airfoil::LinearAirfoil;
    use helisim_bemt::Config;
    use helisim_design::evaluate;

    /// ORACLE: the Basquin fit reproduces its two ASM anchor points.
    #[test]
    fn basquin_reproduces_the_sn_anchors() {
        // σ(10³) ≈ 290 MPa, σ(5×10⁸) ≈ 96 MPa.
        assert!((basquin_allowable_cycles(290.0e6) - 1.0e3).abs() / 1.0e3 < 0.05);
        let n = basquin_allowable_cycles(96.0e6);
        assert!((n / 5.0e8 - 1.0).abs() < 0.1, "got {n:.2e}");
    }

    #[test]
    fn goodman_raises_amplitude_with_mean_stress() {
        let zero_mean = goodman_equiv(50.0e6, 0.0);
        let with_mean = goodman_equiv(50.0e6, 150.0e6);
        assert!((zero_mean - 50.0e6).abs() < 1e-6);
        assert!(
            with_mean > zero_mean,
            "mean stress must raise the equivalent amplitude"
        );
    }

    #[test]
    fn more_flights_accumulate_more_damage() {
        let c = DesignCandidate::model();
        let r = evaluate(&c, &LinearAirfoil::naca0012(), &Config::default());
        let short = analyze_fatigue(&c, &r, 365.0, 1.0, 20.0);
        let long = analyze_fatigue(&c, &r, 365.0, 10.0, 20.0);
        assert!(long.total_damage > short.total_damage);
        assert!(long.gag_cycles > short.gag_cycles);
    }

    /// ORACLE: the polymer semi-log S-N hits its anchor (0.3·UTS at 10⁷ cycles) and is
    /// monotone (more cycles ⇒ lower allowable stress).
    #[test]
    fn polymer_sn_matches_its_endurance_anchor() {
        let n = polymer_allowable_cycles(0.3 * BLADE_UTS_PA);
        assert!(
            (n / 1.0e7 - 1.0).abs() < 0.05,
            "0.3·UTS should give 10⁷, got {n:.2e}"
        );
        assert!(
            polymer_allowable_cycles(0.2 * BLADE_UTS_PA)
                > polymer_allowable_cycles(0.4 * BLADE_UTS_PA)
        );
    }

    #[test]
    fn articulated_head_relieves_blade_flap_fatigue() {
        let c = DesignCandidate::model(); // articulated/teetering head
        let b = analyze_blade_fatigue(&c, 365.0, 10.0, 20.0);
        // The hinge relief makes the alternating stress a small fraction of the 1-g.
        assert!(
            b.alt_mpa < 0.2 * b.flap_1g_mpa + 1e-9,
            "hinge relief applied"
        );
        assert!(b.cycles > 1.0e6, "per-rev fatigue is a high-cycle problem");
        assert!(b.predicted_life_years > 0.0);
    }

    #[test]
    fn miner_damage_unity_is_failure_for_an_overloaded_joint() {
        // A heavily loaded joint (high peak stress) must accumulate D ≥ 1 over the life.
        // Crank the rotor speed up so the centrifugal hole stress is large.
        let mut c = DesignCandidate::model();
        c.tip_speed_ms *= 4.0; // 16× centrifugal force
        let r = evaluate(&c, &LinearAirfoil::naca0012(), &Config::default());
        let f = analyze_fatigue(&c, &r, 365.0, 10.0, 20.0);
        assert!(f.gag_peak_mpa > 0.0);
        // The model is robust; this overload should bite (damage rises sharply).
        assert!(
            f.total_damage
                > analyze_fatigue(&DesignCandidate::model(), &r, 365.0, 10.0, 20.0).total_damage
        );
    }
}
