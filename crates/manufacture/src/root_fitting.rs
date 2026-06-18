//! Blade root fitting — the clamp/tang that joins the blade to the grip.
//!
//! The aerodynamic blade is shaped from the cutout to the tip; the inboard end
//! needs a solid fitting that the hub grip clamps and a bolt passes through. It is
//! sized from the blade root chord/thickness and the centrifugal load it carries
//! (the bolt diameter is checked in [`crate::structural`]).

use crate::part::{BuildPart, Source};

/// A blade root-fitting specification (metres).
#[derive(Clone, Debug)]
pub struct RootFitting {
    /// Number of fittings (one per blade).
    pub count: usize,
    /// Length of the fitting along the span (clamp overlap), m.
    pub length_m: f64,
    /// Width (≈ root chord), m.
    pub width_m: f64,
    /// Thickness (≈ blade root thickness), m.
    pub thickness_m: f64,
    /// Retention bolt diameter, m.
    pub bolt_diameter_m: f64,
}

/// Build a root fitting from blade root geometry and a chosen bolt size.
pub fn root_fitting_for(
    n_blades: usize,
    root_chord_m: f64,
    root_thickness_m: f64,
    bolt_diameter_m: f64,
) -> RootFitting {
    RootFitting {
        count: n_blades,
        length_m: 1.5 * root_chord_m,
        width_m: root_chord_m,
        thickness_m: root_thickness_m.max(bolt_diameter_m * 1.8),
        bolt_diameter_m,
    }
}

impl BuildPart for RootFitting {
    fn name(&self) -> &str {
        "blade root fittings"
    }
    fn material(&self) -> &str {
        "2× 6061 aluminium doubler plates (cut from flat bar) + a steel bolt bushing"
    }
    fn source(&self) -> Source {
        Source::Fabricated
    }
    fn key_dimensions_mm(&self) -> Vec<(&'static str, f64)> {
        vec![
            ("length", self.length_m * 1000.0),
            ("width", self.width_m * 1000.0),
            ("thickness", self.thickness_m * 1000.0),
            ("bolt Ø", self.bolt_diameter_m * 1000.0),
        ]
    }
    fn build_steps(&self) -> Vec<String> {
        let bolt_mm = self.bolt_diameter_m * 1000.0;
        let plates = 2 * self.count;
        vec![
            format!(
                "1. Cut {plates} doubler plates ({} per blade) from the 6061 aluminium flat bar: each \
                 {:.0} × {:.0} × ~2 mm. Use the listed mini-hacksaw + needle files, OR send \
                 `blade_section.dxf`-style rectangles to a laser/water-jet service (both are in the \
                 shopping list with links). Deburr.",
                2,
                self.length_m * 1000.0,
                self.width_m * 1000.0,
            ),
            format!(
                "2. Cut {} steel bushing(s) (one per blade) to the root thickness ({:.1} mm) from the \
                 listed bushing/standoff stock; the bolt will bear on this steel, never on plastic.",
                self.count,
                self.thickness_m * 1000.0
            ),
            format!(
                "3. The printed root already carries the Ø{:.1} mm PILOT hole (printed in, not drilled) \
                 on the pitch axis (~25% chord). REAM the pilot to Ø{bolt_mm:.1} mm; drill the two \
                 aluminium doublers undersize then ream the stack together so all three are concentric \
                 (do NOT drill the plastic root from solid — it delaminates). BOND the steel bushing \
                 into the reamed bore with structural epoxy (NOT press-fit: a polymer interference fit \
                 stress-relaxes and loosens). This bolt is the flap/feather PIVOT and carries the \
                 blade centrifugal force in double shear (sized in the structural check).",
                bolt_mm - 1.0
            ),
            "4. Bond the two doublers to the root faces with structural epoxy (scuff + degrease, \
             clamp, FULL cure — not 5-min epoxy). The bonded doublers carry the centrifugal load into \
             metal; the bonded bushing keeps the bolt bearing on steel; the printed root only locates."
                .to_string(),
            format!(
                "5. Ream the grip jaws to the SAME Ø{bolt_mm:.1} mm; fit the pitch bearings, pass the \
                 bolt through grip + doubler/root/doubler + grip, and fit the nyloc nut — snug, free \
                 to pivot, threadlocked."
            ),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fitting_scales_with_root_and_has_min_thickness_for_bolt() {
        let f = root_fitting_for(3, 0.04, 0.002, 0.004);
        assert_eq!(f.count, 3);
        assert!((f.width_m - 0.04).abs() < 1e-12);
        // Thin blade root → thickness driven up to fit the bolt (1.8×Ø).
        assert!(f.thickness_m >= 0.004 * 1.8 - 1e-12);
    }
}
