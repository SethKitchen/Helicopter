//! NACA 4-digit airfoil coordinates — the actual cross-section shape to cut.
//!
//! The aero stack uses the NACA 0012 section; to *build* a blade we need its real
//! coordinates. The NACA 4-digit family has a closed-form geometry, so this is
//! exact, not a fit. For digits `MPXX` (max camber `M`%, at `P` tenths of chord,
//! thickness `XX`%), the half-thickness distribution is
//!
//! `y_t(x) = 5t (0.2969√x − 0.1260x − 0.3516x² + 0.2843x³ − 0.1015x⁴)`,
//!
//! with `x, y` normalised by chord. For a symmetric section (`M=0`) the surface is
//! `±y_t`; camber shifts and rotates it. This is the published definition (NACA
//! Report 460 / Abbott & von Doenhoff, *Theory of Wing Sections*), so the tests
//! check it against the tabulated 0012 ordinates.

/// NACA 4-digit half-thickness `y_t(x)` for thickness fraction `t` (e.g. 0.12 for
/// 0012) at chordwise station `x ∈ [0,1]`. Uses the −0.1015 (open trailing edge)
/// coefficient, matching the published ordinate tables.
pub fn naca4_half_thickness(t: f64, x: f64) -> f64 {
    5.0 * t
        * (0.2969 * x.sqrt() - 0.1260 * x - 0.3516 * x * x + 0.2843 * x.powi(3)
            - 0.1015 * x.powi(4))
}

/// A point on the section contour, chord-normalised.
#[derive(Clone, Copy, Debug)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

/// Generate the closed contour of a **symmetric** NACA 00XX section as
/// chord-normalised points, ordered upper trailing edge → leading edge → lower
/// trailing edge. `n` is the number of stations per surface; cosine spacing
/// clusters points at the leading and trailing edges where curvature is highest.
pub fn naca00xx_contour(t: f64, n: usize) -> Vec<Point> {
    let mut upper = Vec::with_capacity(n);
    let mut lower = Vec::with_capacity(n);
    for i in 0..n {
        // Cosine spacing in [0,1].
        let beta = std::f64::consts::PI * i as f64 / (n as f64 - 1.0);
        let x = 0.5 * (1.0 - beta.cos());
        let yt = naca4_half_thickness(t, x);
        upper.push(Point { x, y: yt });
        lower.push(Point { x, y: -yt });
    }
    // Upper TE→LE, then lower LE→TE (skip the shared LE point).
    let mut contour: Vec<Point> = upper.into_iter().rev().collect();
    contour.extend(lower.into_iter().skip(1));
    contour
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Tabulated NACA 0012 ordinates (Abbott & von Doenhoff): the half-thickness
    /// at x=0.30 is 0.06002, and the finite trailing edge at x=1.0 is 0.00126.
    #[test]
    fn matches_published_0012_ordinates() {
        assert!((naca4_half_thickness(0.12, 0.30) - 0.06002).abs() < 1e-4);
        assert!((naca4_half_thickness(0.12, 1.00) - 0.00126).abs() < 1e-5);
        // Leading edge closes to zero thickness.
        assert!(naca4_half_thickness(0.12, 0.0).abs() < 1e-12);
    }

    /// Max thickness of a 0012 is 12% of chord (full), reached near x≈0.30.
    #[test]
    fn max_thickness_is_twelve_percent() {
        let mut max_full = 0.0_f64;
        let mut at = 0.0;
        for i in 0..=1000 {
            let x = i as f64 / 1000.0;
            let full = 2.0 * naca4_half_thickness(0.12, x);
            if full > max_full {
                max_full = full;
                at = x;
            }
        }
        assert!((max_full - 0.12).abs() < 1e-3, "max thickness {max_full}");
        assert!((at - 0.30).abs() < 0.05, "max at x={at}");
    }

    /// The contour is closed-ish (starts and ends near the trailing edge) and
    /// symmetric about the chord line.
    #[test]
    fn contour_is_symmetric_and_te_to_te() {
        let c = naca00xx_contour(0.12, 80);
        assert!(c.first().unwrap().x > 0.99 && c.last().unwrap().x > 0.99);
        // Leading edge (min x) sits on the chord line.
        let le = c.iter().min_by(|a, b| a.x.total_cmp(&b.x)).unwrap();
        assert!(le.x.abs() < 1e-6 && le.y.abs() < 1e-9);
    }
}
