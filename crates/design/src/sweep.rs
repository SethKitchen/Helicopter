//! Radius sweep at fixed tip speed — surfacing the central sizing trade.
//!
//! Holding gross mass and tip speed fixed and growing the rotor radius lowers the
//! disk loading. That is the single most consequential model-helicopter sizing
//! choice, and it pulls the priorities in tension: a bigger disk hovers on less
//! power (more airtime, more efficient), radiates less noise, and carries more
//! autorotation margin — but it is larger, heavier and harder to build, and its
//! RPM drops. The sweep makes that trade quantitative so the recommended point
//! can be argued from the priority order rather than guessed.

use crate::candidate::DesignCandidate;
use crate::metrics::evaluate;
use crate::report::DesignReport;
use helisim_airfoil::Airfoil;
use helisim_bemt::Config;

/// One swept point: the radius and its evaluated report.
#[derive(Clone, Copy, Debug)]
pub struct SweepPoint {
    /// Rotor radius, m.
    pub radius_m: f64,
    /// Evaluated consequences at this radius.
    pub report: DesignReport,
}

/// Sweep rotor radius across `radii` (m) at the candidate's fixed tip speed,
/// returning the evaluated report at each.
pub fn sweep_radius(
    base: &DesignCandidate,
    radii: &[f64],
    airfoil: &dyn Airfoil,
    cfg: &Config,
) -> Vec<SweepPoint> {
    radii
        .iter()
        .map(|&r| {
            let c = base.with_radius(r);
            SweepPoint {
                radius_m: r,
                report: evaluate(&c, airfoil, cfg),
            }
        })
        .collect()
}
