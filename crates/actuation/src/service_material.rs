//! Manufacturing options for the control surfaces — **in-house print vs an
//! outsourced service** — as one unified, sourced material+source catalogue.
//!
//! Outsourcing (Protolabs, Xometry, …) trades the Onyx Pro's fixed two-material
//! menu for a wide one (SLS/MJF nylon, glass- and carbon-filled nylon, CNC metal)
//! and removes the printer capex, at quote-based per-part pricing. Each option
//! pairs a [`PrintMaterial`] (sourced structural specs — so it runs through the
//! same control-surface analysis) with **where to make it** and a **cost level**.
//!
//! Headline (the analysis surfaces it): the service SLS/MJF *polymers* are all
//! **less stiff** than Markforged continuous Fiberglass — chopped/filled powders
//! (PA12 ~1.5, glass-filled ~3, carbon-filled ~5 GPa flexural) vs continuous-fiber
//! Onyx+Fiberglass (22 GPa). Only **CNC metal** (Al 6061, 69 GPa) beats it. So the
//! stiffness-critical surfaces favour in-house Fiberglass or outsourced CNC metal;
//! the low-load parts are cheapest as outsourced SLS nylon.
//!
//! Specs are sourced (datasheets noted per entry); ⚠ vendor TDS values are
//! "typical, not for specification" — confirm the exact grade/process before build.

use crate::material::{PrintMaterial, onyx, onyx_fiberglass};

/// Rough cost tier — exact cost is quote-based (geometry/volume dependent).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CostLevel {
    /// Lowest cost (budget services / commodity SLS nylon).
    Budget,
    /// Mid (mainstream SLS/MJF; in-house print once the printer is owned).
    Mid,
    /// Premium (fast in-house service bureaus; CNC metal; filled grades).
    Premium,
}

impl CostLevel {
    pub fn label(&self) -> &'static str {
        match self {
            CostLevel::Budget => "$",
            CostLevel::Mid => "$$",
            CostLevel::Premium => "$$$",
        }
    }
}

/// One way to make a control-surface part: a material + process + where + cost.
#[derive(Clone, Copy, Debug)]
pub struct ServiceMaterial {
    /// The structural material (drives the control-surface analysis).
    pub material: PrintMaterial,
    /// Process, e.g. "Markforged CFR (in-house)", "SLS", "MJF", "CNC".
    pub process: &'static str,
    /// Where it can be made (in-house, or example services).
    pub where_made: &'static str,
    /// Cost tier.
    pub cost: CostLevel,
    /// Cost/sourcing note (sourced hints; pricing is quote-based).
    pub note: &'static str,
}

/// All manufacturing options, in-house and outsourced (sorted by material density,
/// lightest first — so "lightest adequate" prefers light options).
///
/// Sourced specs (flexural modulus / flexural strength / density):
/// * **SLS/MJF PA12** 1.5 GPa / 58 MPa / 1.01 — SLS Nylon PA12 datasheet (tensile
///   1.7 GPa/48 MPa; MJF isotropic, ρ 1.01). The commodity baseline.
/// * **SLS Nylon-12 CF** 5.0 GPa / 70 MPa / 1.05 — carbon-filled PA12 (grade-
///   dependent: FDM/SLS 2.4–5.5 GPa; representative mid value, ⚠ verify TDS).
/// * **SLS PA12 40% Glass-Filled (PA 3200 GF)** 3.0 GPa / 55 MPa / 1.30 — EOS PA
///   3200 GF (flex modulus 2.6–3.1 GPa, flex strength 37–73 MPa, tensile mod 3.2).
/// * **Markforged Onyx / Onyx+Fiberglass** — from [`crate::material`] (in-house).
/// * **CNC Aluminium 6061-T6** 69 GPa / 276 MPa / 2.70 — standard wrought Al
///   (E 68.9 GPa, yield 276 MPa); machined, not printed.
pub fn manufacturing_options() -> Vec<ServiceMaterial> {
    let sls_pa12 = PrintMaterial {
        name: "SLS PA12 (nylon)",
        flex_modulus_gpa: 1.5,
        flex_strength_mpa: 58.0,
        density_g_cm3: 1.01,
        poisson: 0.4,
        note: "SLS Nylon PA12 datasheet (flex 1.5 GPa/58 MPa; MJF isotropic ~same)",
    };
    let sls_cf = PrintMaterial {
        name: "SLS Nylon-12 CF",
        flex_modulus_gpa: 5.0,
        flex_strength_mpa: 70.0,
        density_g_cm3: 1.05,
        poisson: 0.4,
        note: "carbon-filled PA12 (grade-dependent 2.4–5.5 GPa; ⚠ verify TDS)",
    };
    let pa12_gf = PrintMaterial {
        name: "SLS PA12-GF (glass)",
        flex_modulus_gpa: 3.0,
        flex_strength_mpa: 55.0,
        density_g_cm3: 1.30,
        poisson: 0.4,
        note: "EOS PA 3200 GF (flex 2.6–3.1 GPa / 37–73 MPa, tensile mod 3.2)",
    };
    let cnc_al = PrintMaterial {
        name: "CNC Aluminium 6061-T6",
        flex_modulus_gpa: 69.0,
        flex_strength_mpa: 276.0,
        density_g_cm3: 2.70,
        poisson: 0.33,
        note: "wrought Al 6061-T6 (E 68.9 GPa, yield 276 MPa); machined, not printed",
    };

    let mut opts = vec![
        ServiceMaterial {
            material: sls_pa12,
            process: "SLS / MJF",
            where_made: "Protolabs, Xometry, Sculpteo, Shapeways, PCBWay/JLC3DP",
            cost: CostLevel::Budget,
            note: "commodity nylon; cheapest service polymer; quote-based per part",
        },
        ServiceMaterial {
            material: sls_cf,
            process: "SLS (carbon-filled)",
            where_made: "Xometry, Sculpteo, Shapeways",
            cost: CostLevel::Mid,
            note: "stiffer & light; grade-dependent; not on every service",
        },
        ServiceMaterial {
            material: onyx(),
            process: "Markforged CFR (in-house Onyx Pro)",
            where_made: "in-house print",
            cost: CostLevel::Mid,
            note: "you own the printer; no per-part quote",
        },
        ServiceMaterial {
            material: pa12_gf,
            process: "SLS (glass-filled)",
            where_made: "Protolabs, Xometry, Sculpteo",
            cost: CostLevel::Mid,
            note: "stiff & dimensionally stable but heavier & more brittle",
        },
        ServiceMaterial {
            material: onyx_fiberglass(),
            process: "Markforged CFR (in-house Onyx Pro)",
            where_made: "in-house print",
            cost: CostLevel::Premium,
            note: "continuous fiberglass — stiffest polymer here (22 GPa); in-house only",
        },
        ServiceMaterial {
            material: cnc_al,
            process: "CNC machining",
            where_made: "Protolabs, Xometry, PCBWay/JLC3DP",
            cost: CostLevel::Premium,
            note: "metal-grade stiffness/strength; heavy; the realistic swashplate route",
        },
    ];
    opts.sort_by(|a, b| {
        a.material
            .density_g_cm3
            .total_cmp(&b.material.density_g_cm3)
    });
    opts
}

/// The options as bare [`PrintMaterial`]s (for the control-surface analysis).
pub fn options_as_materials() -> Vec<PrintMaterial> {
    manufacturing_options()
        .into_iter()
        .map(|o| o.material)
        .collect()
}

/// Look up the source (process/where/cost) for a chosen material by name.
pub fn source_for(name: &str) -> Option<ServiceMaterial> {
    manufacturing_options()
        .into_iter()
        .find(|o| o.material.name == name)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// DATASHEET ORACLE — the sourced service-material specs (a reader can check
    /// each against the cited datasheet) and the stiffness ordering that drives the
    /// recommendation: commodity nylon < glass-filled < carbon-filled < continuous
    /// fiberglass < CNC aluminium.
    #[test]
    fn service_material_specs_and_stiffness_order() {
        let by = |n: &str| {
            manufacturing_options()
                .into_iter()
                .find(|o| o.material.name == n)
                .unwrap()
        };
        assert_eq!(by("SLS PA12 (nylon)").material.flex_modulus_gpa, 1.5);
        assert_eq!(by("SLS PA12-GF (glass)").material.flex_modulus_gpa, 3.0);
        assert_eq!(by("CNC Aluminium 6061-T6").material.flex_modulus_gpa, 69.0);
        // The continuous-fiber in-house option out-stiffens every service polymer.
        let fg = by("Onyx+Fiberglass").material.flex_modulus_gpa;
        assert!(fg > by("SLS Nylon-12 CF").material.flex_modulus_gpa);
        assert!(fg > by("SLS PA12-GF (glass)").material.flex_modulus_gpa);
    }

    #[test]
    fn options_include_inhouse_and_service() {
        let opts = manufacturing_options();
        assert!(opts.iter().any(|o| o.where_made == "in-house print"));
        assert!(opts.iter().any(|o| o.where_made.contains("Protolabs")));
        // Lightest-first ordering.
        for w in opts.windows(2) {
            assert!(w[0].material.density_g_cm3 <= w[1].material.density_g_cm3);
        }
    }

    #[test]
    fn source_lookup_round_trips() {
        let s = source_for("CNC Aluminium 6061-T6").unwrap();
        assert_eq!(s.process, "CNC machining");
        assert_eq!(s.cost, CostLevel::Premium);
        assert!(source_for("nonexistent").is_none());
    }
}
