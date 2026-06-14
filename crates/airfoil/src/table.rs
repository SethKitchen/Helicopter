//! Tabulated airfoil: piecewise-linear interpolation of a measured polar.

use crate::airfoil::Airfoil;
use std::f64::consts::PI;

/// Piecewise-linear interpolation of a measured `(alpha, Cl, Cd)` polar. Stored
/// sorted by ascending angle of attack (radians); outside the table the nearest
/// endpoint is held.
#[derive(Clone, Debug)]
pub struct TableAirfoil {
    points: Vec<(f64, f64, f64)>,
}

impl TableAirfoil {
    /// Build from `(alpha_deg, Cl, Cd)` rows. Rows are sorted internally so the
    /// caller need not pre-order them.
    pub fn from_deg(rows: &[(f64, f64, f64)]) -> Self {
        let mut points: Vec<(f64, f64, f64)> = rows
            .iter()
            .map(|&(a_deg, cl, cd)| (a_deg * PI / 180.0, cl, cd))
            .collect();
        points.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap());
        TableAirfoil { points }
    }
}

impl Airfoil for TableAirfoil {
    fn cl_cd(&self, alpha: f64, _mach: f64) -> (f64, f64) {
        let pts = &self.points;
        if pts.is_empty() {
            return (0.0, 0.0);
        }
        if alpha <= pts[0].0 {
            return (pts[0].1, pts[0].2);
        }
        let last = pts[pts.len() - 1];
        if alpha >= last.0 {
            return (last.1, last.2);
        }
        let i = pts.partition_point(|p| p.0 < alpha);
        let (a0, cl0, cd0) = pts[i - 1];
        let (a1, cl1, cd1) = pts[i];
        let t = (alpha - a0) / (a1 - a0);
        (cl0 + t * (cl1 - cl0), cd0 + t * (cd1 - cd0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interpolates_midpoint() {
        let t = TableAirfoil::from_deg(&[(0.0, 0.0, 0.01), (10.0, 1.0, 0.05)]);
        let (cl, cd) = t.cl_cd(5.0_f64.to_radians(), 0.0);
        assert!((cl - 0.5).abs() < 1e-9);
        assert!((cd - 0.03).abs() < 1e-9);
    }

    #[test]
    fn holds_endpoints() {
        let t = TableAirfoil::from_deg(&[(0.0, 0.0, 0.01), (10.0, 1.0, 0.05)]);
        assert!((t.cl(20.0_f64.to_radians(), 0.0) - 1.0).abs() < 1e-9);
        assert!((t.cl(-5.0_f64.to_radians(), 0.0) - 0.0).abs() < 1e-9);
    }
}
