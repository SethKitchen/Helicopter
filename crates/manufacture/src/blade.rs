//! From a recommended design to a **buildable blade**: real dimensions, the raw
//! stock to start from, and step-by-step shaping instructions.
//!
//! This is the first piece of the stated end goal — turning a chosen design into
//! "get a block of this size and cut it into this shape". It reads the geometry
//! off a [`DesignCandidate`] and pairs it with the section coordinates from
//! [`crate::airfoil_coords`]. Material/method are scale-appropriate suggestions
//! (small chords → carve/laminate from stock; large chords → spar + skin), stated
//! as recommendations, not the only route.

use crate::airfoil_coords::{naca00xx_contour, Point};
use crate::part::{BuildPart, Source};
use helisim_design::DesignCandidate;

/// NACA 0012 thickness fraction (the section the aero stack is built around).
const THICKNESS_FRAC: f64 = 0.12;

/// A buildable blade specification with real (SI) dimensions.
#[derive(Clone, Debug)]
pub struct BladeSpec {
    /// Airfoil designation.
    pub airfoil: &'static str,
    /// Number of blades to make.
    pub n_blades: usize,
    /// Inboard (root cutout) radius, m.
    pub root_radius_m: f64,
    /// Tip radius, m.
    pub tip_radius_m: f64,
    /// Lifting span (tip − root), m — the length to shape.
    pub span_m: f64,
    /// Root chord, m (the representative chord; equals tip for a rectangular blade).
    pub chord_m: f64,
    /// Tip chord, m (`= chord_m` for a rectangular blade; `< chord_m` if tapered).
    pub tip_chord_m: f64,
    /// Linear twist (washout) root→tip, degrees (applied tip-down for loft/export).
    pub twist_deg: f64,
    /// Maximum section thickness, m (`0.12 · chord`).
    pub max_thickness_m: f64,
    /// Recommended raw stock block per blade `(length, width, thickness)`, mm,
    /// with a small machining allowance.
    pub stock_block_mm: (f64, f64, f64),
    /// Suggested material + construction method.
    pub method: &'static str,
}

impl BladeSpec {
    /// The dimensional section contour at this blade's chord, in mm — the profile
    /// to cut. `n` points per surface.
    pub fn section_contour_mm(&self, n: usize) -> Vec<Point> {
        naca00xx_contour(THICKNESS_FRAC, n)
            .into_iter()
            .map(|p| Point {
                x: p.x * self.chord_m * 1000.0,
                y: p.y * self.chord_m * 1000.0,
            })
            .collect()
    }

    /// Chord (m) at span fraction `s ∈ [0,1]` (root→tip), linearly interpolated.
    pub fn local_chord_m(&self, s: f64) -> f64 {
        self.chord_m + (self.tip_chord_m - self.chord_m) * s
    }

    /// Geometric twist (deg) at span fraction `s`: linear washout, 0 at the root
    /// to `−twist_deg` at the tip (leading-edge-down toward the tip).
    pub fn local_twist_deg(&self, s: f64) -> f64 {
        -self.twist_deg * s
    }

    /// Whether the blade is tapered or twisted (so a loft is needed, not a simple
    /// extrusion).
    pub fn is_lofted(&self) -> bool {
        (self.tip_chord_m - self.chord_m).abs() > 1e-9 || self.twist_deg.abs() > 1e-9
    }

    /// Step-by-step shaping instructions for one blade.
    pub fn instructions(&self) -> Vec<String> {
        let (l, w, t) = self.stock_block_mm;
        vec![
            format!(
                "1. Obtain stock: {l:.0} × {w:.0} × {t:.0} mm of {} (one per blade, ×{}).",
                self.method, self.n_blades
            ),
            format!(
                "2. Mark the chord line along the {:.0} mm length; mark stations every \
                 {:.0} mm.",
                self.span_m * 1000.0,
                self.span_m * 1000.0 / 10.0
            ),
            format!(
                "3. Shape the NACA {} section (chord {:.1} mm, max thickness {:.2} mm at \
                 30% chord) constant along the span.",
                self.airfoil.trim_start_matches("NACA "),
                self.chord_m * 1000.0,
                self.max_thickness_m * 1000.0
            ),
            if self.twist_deg.abs() > 1e-6 {
                format!(
                    "4. Apply linear washout: {:.1}° leading-edge-down from root to tip.",
                    self.twist_deg.abs()
                )
            } else {
                "4. No twist (untwisted blade) — keep the chord plane flat.".to_string()
            },
            format!(
                "5. Leave the inboard {:.0} mm (root cutout) as the root attachment; drill \
                 the hub bolt pattern there.",
                self.root_radius_m * 1000.0
            ),
            "6. Balance each blade spanwise and chordwise against the others before assembly."
                .to_string(),
        ]
    }
}

/// Derive a buildable blade spec from a design candidate. `twist_deg` is the
/// linear washout to apply (0 for the untwisted designs the aero stack uses).
/// Rectangular planform (`tip_chord = chord`); use [`blade_from_design_tapered`]
/// for a taper.
pub fn blade_from_design(c: &DesignCandidate, twist_deg: f64) -> BladeSpec {
    blade_from_design_tapered(c, twist_deg, 1.0)
}

/// As [`blade_from_design`] but with a tip-to-root chord `taper_ratio` (1.0 =
/// rectangular, e.g. 0.6 = a 60%-tip taper). The stock block is sized on the root
/// (largest) chord.
pub fn blade_from_design_tapered(c: &DesignCandidate, twist_deg: f64, taper_ratio: f64) -> BladeSpec {
    let root_radius = c.root_cutout * c.radius_m;
    let span = c.radius_m - root_radius;
    let max_thickness = THICKNESS_FRAC * c.chord_m;
    // Stock block: span + 10% length allowance, chord + 20% width, thickness +20%.
    let stock = (
        span * 1000.0 * 1.10,
        c.chord_m * 1000.0 * 1.20,
        max_thickness * 1000.0 * 1.20,
    );
    // Scale-appropriate method by chord size.
    let method = if c.chord_m < 0.08 {
        "balsa/basswood laminate or hard-foam core"
    } else if c.chord_m < 0.20 {
        "foam core with glass/carbon skin"
    } else {
        "carbon spar + skinned foam/honeycomb core"
    };
    BladeSpec {
        airfoil: "NACA 0012",
        n_blades: c.n_blades,
        root_radius_m: root_radius,
        tip_radius_m: c.radius_m,
        span_m: span,
        chord_m: c.chord_m,
        tip_chord_m: c.chord_m * taper_ratio,
        twist_deg,
        max_thickness_m: max_thickness,
        stock_block_mm: stock,
        method,
    }
}

impl BuildPart for BladeSpec {
    fn name(&self) -> &str {
        "main-rotor blades"
    }
    fn material(&self) -> &str {
        self.method
    }
    fn source(&self) -> Source {
        Source::RawStock
    }
    fn key_dimensions_mm(&self) -> Vec<(&'static str, f64)> {
        vec![
            ("span", self.span_m * 1000.0),
            ("chord", self.chord_m * 1000.0),
            ("max thickness", self.max_thickness_m * 1000.0),
        ]
    }
    fn build_steps(&self) -> Vec<String> {
        self.instructions()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec() -> BladeSpec {
        blade_from_design(&DesignCandidate::model(), 0.0)
    }

    #[test]
    fn dimensions_are_consistent_with_the_design() {
        let c = DesignCandidate::model();
        let s = spec();
        assert!((s.tip_radius_m - c.radius_m).abs() < 1e-12);
        assert!((s.root_radius_m - c.root_cutout * c.radius_m).abs() < 1e-12);
        assert!((s.span_m - (c.radius_m - c.root_cutout * c.radius_m)).abs() < 1e-12);
        assert!((s.max_thickness_m - 0.12 * c.chord_m).abs() < 1e-12);
        assert_eq!(s.n_blades, c.n_blades);
    }

    #[test]
    fn stock_block_has_machining_allowance() {
        let s = spec();
        // Stock must be at least as big as the finished part.
        assert!(s.stock_block_mm.0 >= s.span_m * 1000.0);
        assert!(s.stock_block_mm.1 >= s.chord_m * 1000.0);
        assert!(s.stock_block_mm.2 >= s.max_thickness_m * 1000.0);
    }

    #[test]
    fn section_contour_scales_to_chord() {
        let s = spec();
        let contour = s.section_contour_mm(60);
        // Max |y| of the dimensional contour is the half-thickness in mm.
        let max_y = contour.iter().map(|p| p.y.abs()).fold(0.0_f64, f64::max);
        assert!((max_y - 0.06 * s.chord_m * 1000.0).abs() < 0.05);
        // Chordwise extent equals the chord (mm).
        let max_x = contour.iter().map(|p| p.x).fold(0.0_f64, f64::max);
        assert!((max_x - s.chord_m * 1000.0).abs() < 0.5);
    }

    #[test]
    fn instructions_are_nonempty_and_mention_stock() {
        let s = spec();
        let steps = s.instructions();
        assert!(steps.len() >= 5);
        assert!(steps[0].contains("stock"));
    }
}
