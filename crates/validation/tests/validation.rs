//! Validation suite: the published-oracle checks expressed as tests.
//!
//! These encode exactly what a *correct hover BEMT* should satisfy:
//!   * the analytical BEMT closed form is reproduced (solver correctness);
//!   * thrust rises monotonically with collective;
//!   * Caradonna & Tung C_T lands in the documented BEMT over-prediction band;
//!   * the Harrington Rotor 1 peak figure of merit matches published data with
//!     no calibration (the headline, calibration-free validation);
//!   * the spanwise loading has the right shape (peaks outboard, →0 at the tip).

use helisim_airfoil::LinearAirfoil;
use helisim_bemt::{Config, solve_hover};
use helisim_rotor::{Operating, Rotor};
use helisim_validation::{CaradonnaTung, HarringtonRotor1, ValidationCase, run_case};

/// The solver's per-station inflow must reproduce the analytical BEMT closed
/// form `lambda(x) = (sigma*a/16)(sqrt(1 + 32*theta*x/(sigma*a)) - 1)`
/// (Leishman) under its assumptions: no tip loss, no drag, incompressible,
/// linear lift. This proves the inflow root-find is a correct BEMT, independent
/// of any experimental oracle.
#[test]
fn bemt_reproduces_analytical_closed_form() {
    let a0 = 5.73;
    let theta = 8f64.to_radians();
    let rotor = Rotor::rectangular(2, 1.143, 0.191, theta, 0.05);
    let op = Operating::from_tip_mach(0.439, rotor.radius);
    let airfoil = LinearAirfoil {
        a0,
        alpha0: 0.0,
        cl_max: 1.0e9, // disable stall
        cd0: 0.0,
        cd1: 0.0,
        cd2: 0.0,
        compressible: false,
    };
    let cfg = Config {
        n_stations: 400,
        tip_loss: false,
        ..Config::default()
    };
    let sol = solve_hover(&rotor, &op, &airfoil, &cfg);

    let sigma = rotor.solidity(); // constant chord => local == nominal
    let mut max_rel = 0.0_f64;
    for s in &sol.stations {
        if !(0.2..=0.95).contains(&s.x) {
            continue; // small-angle approx degrades very near root/tip
        }
        let lam_exact =
            (sigma * a0 / 16.0) * ((1.0 + 32.0 * theta * s.x / (sigma * a0)).sqrt() - 1.0);
        let rel = (s.lambda - lam_exact).abs() / lam_exact;
        max_rel = max_rel.max(rel);
    }
    assert!(
        max_rel < 0.02,
        "per-station inflow deviates from analytical BEMT by {:.3}%",
        max_rel * 100.0
    );
}

/// Thrust coefficient must rise monotonically with collective pitch.
#[test]
fn caradonna_tung_ct_monotonic() {
    let results = run_case(&CaradonnaTung::default(), &Config::default());
    for w in results.windows(2) {
        assert!(
            w[1].ct_pred > w[0].ct_pred,
            "C_T must increase with collective: {} -> {}",
            w[0].ct_pred,
            w[1].ct_pred
        );
    }
}

/// Caradonna & Tung C_T must land within the documented BEMT over-prediction
/// band: predicted >= experimental (BEMT over-predicts) and within each point's
/// tolerance. This is the trend/magnitude check; the calibration-free
/// quantitative match is carried by figure of merit below.
#[test]
fn caradonna_tung_ct_in_bemt_band() {
    let results = run_case(&CaradonnaTung::default(), &Config::default());
    assert_eq!(results.len(), 3);
    for r in &results {
        assert!(
            r.pass,
            "θ={:.0}°: C_T {:.5} vs exp {:.5} ({:+.1}%) outside band {:.0}%",
            r.point.collective_deg,
            r.ct_pred,
            r.point.ct_expected,
            r.rel_err * 100.0,
            r.point.tol_frac * 100.0
        );
        // BEMT over-predicts this rotor — the sign of the error is itself a
        // documented, expected property.
        assert!(
            r.rel_err > -0.05,
            "θ={:.0}°: BEMT should not under-predict C_T here",
            r.point.collective_deg
        );
    }
}

/// Headline, calibration-free validation: the Harrington Rotor 1 hover peak
/// figure of merit must fall in the published band [0.62, 0.75].
#[test]
fn harrington_peak_fm_matches_published_band() {
    let harr = HarringtonRotor1::default();
    let airfoil = harr.airfoil();
    let cfg = Config::default();

    let mut peak_fm = 0.0_f64;
    for step in 0..=20 {
        let theta_deg = 2.0 + step as f64 * 0.6;
        let rotor = harr.build_rotor(theta_deg.to_radians());
        let op = Operating::from_tip_mach(harr.tip_mach, rotor.radius);
        let sol = solve_hover(&rotor, &op, airfoil.as_ref(), &cfg);
        peak_fm = peak_fm.max(sol.figure_of_merit);
    }

    let (lo, hi) = harr.expected_peak_fm();
    assert!(
        (lo..=hi).contains(&peak_fm),
        "Harrington peak FM {peak_fm:.3} outside published band [{lo:.2}, {hi:.2}]"
    );
}

/// Caradonna & Tung figure of merit must be physical and improve with disk
/// loading (higher collective → higher FM in this regime).
#[test]
fn caradonna_tung_fm_physical_and_rising() {
    let results = run_case(&CaradonnaTung::default(), &Config::default());
    for r in &results {
        assert!(
            r.fm_pred > 0.3 && r.fm_pred < 0.85,
            "FM {:.3} not physical at θ={:.0}°",
            r.fm_pred,
            r.point.collective_deg
        );
    }
    assert!(
        results[2].fm_pred > results[0].fm_pred,
        "FM should rise with collective in this thrust range"
    );
}

/// The spanwise loading must peak in the outboard third and be driven to near
/// zero at the tip by the tip-loss factor.
#[test]
fn spanwise_loading_shape() {
    let ct = CaradonnaTung::default();
    let rotor = ct.build_rotor(8f64.to_radians());
    let op = Operating::from_tip_mach(0.439, rotor.radius);
    let sol = solve_hover(&rotor, &op, ct.airfoil().as_ref(), &Config::default());

    let (peak_i, peak) = sol
        .stations
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.dct_dx.partial_cmp(&b.1.dct_dx).unwrap())
        .map(|(i, s)| (i, s.dct_dx))
        .unwrap();
    let peak_x = sol.stations[peak_i].x;
    assert!(
        peak_x > 0.66,
        "loading should peak outboard, peaked at x={peak_x:.3}"
    );

    let tip = sol.stations.last().unwrap();
    assert!(
        tip.dct_dx < 0.1 * peak,
        "tip loading should collapse: tip {:.5} vs peak {:.5}",
        tip.dct_dx,
        peak
    );
    assert!(
        tip.tip_loss < 0.05,
        "tip-loss factor should vanish at the tip"
    );
}

/// A minimal ValidationCase that does NOT override `notes()`, exercising the
/// trait's default (returns `None`) and the rest of the trait surface.
struct MinimalCase;
impl ValidationCase for MinimalCase {
    fn name(&self) -> &str {
        "minimal"
    }
    fn description(&self) -> &str {
        "a bare validation case"
    }
    fn build_rotor(&self, collective_rad: f64) -> Rotor {
        Rotor::rectangular(2, 1.0, 0.1, collective_rad, 0.2)
    }
    fn airfoil(&self) -> Box<dyn helisim_airfoil::Airfoil> {
        Box::new(LinearAirfoil::naca0012())
    }
    fn oracle_points(&self) -> Vec<helisim_validation::OraclePoint> {
        vec![helisim_validation::OraclePoint {
            collective_deg: 8.0,
            tip_mach: 0.4,
            ct_expected: 0.006,
            tol_frac: 0.5,
        }]
    }
}

#[test]
fn default_notes_is_none_and_run_case_works_on_a_minimal_case() {
    let c = MinimalCase;
    assert!(c.notes().is_none()); // the trait default
    assert!(!c.name().is_empty() && !c.description().is_empty());
    let results = run_case(&c, &Config::default());
    assert_eq!(results.len(), 1);
}
