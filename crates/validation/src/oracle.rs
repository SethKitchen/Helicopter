//! The [`ValidationCase`] trait and a generic runner that compares BEMT
//! predictions against published data.

use helisim_airfoil::Airfoil;
use helisim_bemt::{Config, solve_hover};
use helisim_rotor::{Operating, Rotor};

/// One published operating point and its expected thrust coefficient.
#[derive(Clone, Copy, Debug)]
pub struct OraclePoint {
    /// Collective pitch, degrees.
    pub collective_deg: f64,
    /// Tip Mach number of the test.
    pub tip_mach: f64,
    /// Published / accepted thrust coefficient `C_T`.
    pub ct_expected: f64,
    /// Acceptance tolerance as a fraction of `ct_expected` (e.g. 0.10 = 10%).
    pub tol_frac: f64,
}

/// A benchmark rotor with published hover data. Implementors describe the
/// geometry, the airfoil, and the oracle points; the runner does the rest.
pub trait ValidationCase {
    /// Short name, e.g. `"Caradonna & Tung (1981)"`.
    fn name(&self) -> &str;

    /// One-line description of the configuration.
    fn description(&self) -> &str;

    /// Build the rotor geometry at a given collective pitch (radians).
    fn build_rotor(&self, collective_rad: f64) -> Rotor;

    /// The sectional aerodynamics, as a trait object.
    fn airfoil(&self) -> Box<dyn Airfoil>;

    /// The published operating points to validate against. May be empty for
    /// cases whose published data is not a C_T-vs-collective table (e.g.
    /// figure-of-merit benchmarks); see [`Self::notes`].
    fn oracle_points(&self) -> Vec<OraclePoint>;

    /// Free-form notes about the case (e.g. how to interpret its data). Default
    /// is none.
    fn notes(&self) -> Option<String> {
        None
    }
}

/// Outcome of validating a single oracle point.
#[derive(Clone, Copy, Debug)]
pub struct PointResult {
    /// The oracle point being checked.
    pub point: OraclePoint,
    /// BEMT-predicted thrust coefficient.
    pub ct_pred: f64,
    /// BEMT-predicted figure of merit.
    pub fm_pred: f64,
    /// BEMT-predicted power coefficient.
    pub cp_pred: f64,
    /// Signed relative error `(pred - expected) / expected`.
    pub rel_err: f64,
    /// Whether `|rel_err|` is within the point's tolerance.
    pub pass: bool,
}

/// Run every oracle point of `case` through the BEMT solver and report results.
pub fn run_case(case: &dyn ValidationCase, cfg: &Config) -> Vec<PointResult> {
    let airfoil = case.airfoil();
    case.oracle_points()
        .into_iter()
        .map(|point| {
            let rotor = case.build_rotor(point.collective_deg.to_radians());
            let op = Operating::from_tip_mach(point.tip_mach, rotor.radius);
            let sol = solve_hover(&rotor, &op, airfoil.as_ref(), cfg);
            let rel_err = (sol.ct - point.ct_expected) / point.ct_expected;
            PointResult {
                point,
                ct_pred: sol.ct,
                fm_pred: sol.figure_of_merit,
                cp_pred: sol.cp,
                rel_err,
                pass: rel_err.abs() <= point.tol_frac,
            }
        })
        .collect()
}
