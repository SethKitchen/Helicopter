//! 3D-print material database — Markforged **Onyx Pro** compatible materials.
//!
//! The control-path parts (rotor blade, swashplate, pitch links) are printed on a
//! Markforged Onyx Pro, which lays **Onyx** (micro-carbon-filled nylon) base and
//! reinforces it with **continuous Fiberglass** only — carbon fiber and Kevlar
//! need the Mark Two / X7. The Onyx Pro also prints Precise PLA (prototyping) and
//! Smooth TPU 95A (flexible), but neither is a flight-structure material, so the
//! structural database below carries the two load-bearing options whose mechanical
//! properties are sourced from the Markforged Composites Datasheet.
//!
//! Values (Markforged Composites Datasheet, static.markforged.com):
//! * **Onyx** — flexural modulus 3.0 GPa, flexural strength 71 MPa, ρ 1.2 g/cm³.
//! * **Onyx + Fiberglass** (continuous-fiber coupon) — flexural modulus 22 GPa,
//!   flexural strength 200 MPa (tensile 590 MPa / 21 GPa), ρ ≈ 1.5 g/cm³.

/// A printable structural material.
#[derive(Clone, Copy, Debug)]
pub struct PrintMaterial {
    /// Material name.
    pub name: &'static str,
    /// Flexural (bending) modulus, GPa — the stiffness that sets deflection.
    pub flex_modulus_gpa: f64,
    /// Flexural strength, MPa — the working failure stress in bending.
    pub flex_strength_mpa: f64,
    /// Density, g/cm³.
    pub density_g_cm3: f64,
    /// Poisson's ratio (for the shear modulus / torsional wind-up).
    pub poisson: f64,
    /// Sourcing note (datasheet provenance).
    pub note: &'static str,
}

impl PrintMaterial {
    /// Young's / flexural modulus, Pa.
    pub fn e_pa(&self) -> f64 {
        self.flex_modulus_gpa * 1e9
    }
    /// Flexural strength, Pa.
    pub fn strength_pa(&self) -> f64 {
        self.flex_strength_mpa * 1e6
    }
    /// Density, kg/m³.
    pub fn density_kg_m3(&self) -> f64 {
        self.density_g_cm3 * 1000.0
    }
    /// Shear modulus `G = E / 2(1+ν)`, Pa.
    pub fn shear_modulus_pa(&self) -> f64 {
        self.e_pa() / (2.0 * (1.0 + self.poisson))
    }
}

/// Neat Onyx (micro-carbon-filled nylon).
pub fn onyx() -> PrintMaterial {
    PrintMaterial {
        name: "Onyx",
        flex_modulus_gpa: 3.0,
        flex_strength_mpa: 71.0,
        density_g_cm3: 1.2,
        poisson: 0.4,
        note: "Markforged Composites Datasheet: flex 3.0 GPa / 71 MPa, ρ 1.2",
    }
}

/// Onyx reinforced with continuous Fiberglass (the Onyx Pro's only fiber).
pub fn onyx_fiberglass() -> PrintMaterial {
    PrintMaterial {
        name: "Onyx+Fiberglass",
        flex_modulus_gpa: 22.0,
        flex_strength_mpa: 200.0,
        density_g_cm3: 1.5,
        poisson: 0.3,
        note: "Markforged Composites Datasheet FG coupon: flex 22 GPa / 200 MPa (tensile 590 MPa/21 GPa), ρ ≈1.5",
    }
}

/// The Onyx Pro **structural** materials, lightest first (so "lightest adequate"
/// selection prefers neat Onyx and steps up to Fiberglass only when needed).
pub fn onyx_pro_structural() -> Vec<PrintMaterial> {
    vec![onyx(), onyx_fiberglass()]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// DATASHEET ORACLE — the two structural materials match the Markforged
    /// Composites Datasheet, and Fiberglass is the stiffer/stronger/heavier one.
    #[test]
    fn datasheet_values_and_ordering() {
        let o = onyx();
        assert_eq!(
            (o.flex_modulus_gpa, o.flex_strength_mpa, o.density_g_cm3),
            (3.0, 71.0, 1.2)
        );
        let fg = onyx_fiberglass();
        assert_eq!(
            (fg.flex_modulus_gpa, fg.flex_strength_mpa, fg.density_g_cm3),
            (22.0, 200.0, 1.5)
        );
        // Fiberglass buys ~7× stiffness, ~2.8× strength, at ~1.25× the weight.
        assert!((fg.e_pa() / o.e_pa() - 7.33).abs() < 0.1);
        assert!((fg.strength_pa() / o.strength_pa() - 2.82).abs() < 0.05);
        assert!((fg.density_kg_m3() / o.density_kg_m3() - 1.25).abs() < 0.01);
    }

    #[test]
    fn structural_db_is_lightest_first() {
        let db = onyx_pro_structural();
        for w in db.windows(2) {
            assert!(w[0].density_g_cm3 <= w[1].density_g_cm3);
        }
    }

    #[test]
    fn shear_modulus_from_e_and_poisson() {
        // Onyx G = 3.0/(2·1.4) = 1.07 GPa.
        assert!((onyx().shear_modulus_pa() - 3.0e9 / 2.8).abs() < 1e3);
    }
}
