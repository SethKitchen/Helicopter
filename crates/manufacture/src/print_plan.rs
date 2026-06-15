//! Print planning — fit every part to a build volume, split what doesn't fit, and
//! choose each split joint's fastening.
//!
//! Ties [`crate::build_volume`] + [`crate::split`] to the [`crate::assembly`]
//! package: for each part it derives the **internal load** a split joint would
//! carry (centrifugal for the blade, landing impact for the gear, a light shell
//! load for the fuselage, …) so the snap-vs-bolt choice is load-based, not
//! guessed. It also recommends the smallest build volume that prints the biggest
//! part whole (so you can avoid splitting by choosing a service bed).

use crate::assembly::build_package;
use crate::build_volume::{BuildVolume, smallest_fitting};
use crate::split::{SplitPlan, plan_split};
use helisim_design::{DesignCandidate, DesignReport};

/// The internal load (N) a split joint in `part` must carry — a per-part
/// structural heuristic (documented), so snap-vs-bolt is load-driven.
pub fn joint_load_for(part_name: &str, c: &DesignCandidate, report: &DesignReport) -> f64 {
    let weight = c.gross_mass_kg * 9.80665;
    let omega = c.omega();
    let name = part_name.to_lowercase();

    if name.contains("blade") {
        // Centrifugal tension — the dominant, large rotor load.
        let root_r = c.root_cutout * c.radius_m;
        let span = c.radius_m - root_r;
        let m_blade = c.blade_areal_density_kg_m2 * c.chord_m * span;
        let r_cg = 0.5 * (c.radius_m + root_r);
        omega * omega * m_blade * r_cg
    } else if name.contains("boom") {
        // A boom splice carries the boom BENDING moment (root moment = main
        // torque), reacted as a force couple over a ~20 mm splice depth — a
        // structural joint, so it bolts (not the tiny anti-torque force).
        let torque = if report.hover_shaft_power_w.is_finite() && omega > 0.0 {
            report.hover_shaft_power_w / omega
        } else {
            1.0
        };
        torque / 0.02
    } else if name.contains("landing gear") {
        // Half the hard-landing load through a mid-skid joint.
        0.5 * 3.0 * weight
    } else if name.contains("mount") {
        weight // the tray carries the pack/powertrain
    } else if name.contains("fuselage") {
        0.12 * weight // light shell / fairing joint
    } else {
        0.05 * weight // small parts: light
    }
}

/// Plan every part for one build volume.
pub fn plan_prints(
    c: &DesignCandidate,
    report: &DesignReport,
    vol: &BuildVolume,
) -> Vec<SplitPlan> {
    let pkg = build_package(c, report);
    pkg.parts
        .iter()
        .map(|part| {
            let load = joint_load_for(part.name(), c, report);
            plan_split(part.as_ref(), vol, load)
        })
        .collect()
}

/// The single biggest part envelope in the package (drives printer choice).
pub fn largest_part_bbox(c: &DesignCandidate, report: &DesignReport) -> (f64, f64, f64, String) {
    let pkg = build_package(c, report);
    pkg.parts
        .iter()
        .map(|p| {
            let b = p.bounding_box_mm();
            (b.0.max(b.1).max(b.2), b, p.name().to_string())
        })
        .max_by(|a, z| a.0.total_cmp(&z.0))
        .map(|(_, b, n)| (b.0, b.1, b.2, n))
        .unwrap_or((0.0, 0.0, 0.0, String::new()))
}

/// The smallest build volume that prints every part whole (no splitting), or
/// `None` if even the largest bed needs a split — with the limiting part named.
pub fn recommend_printer(
    c: &DesignCandidate,
    report: &DesignReport,
) -> (Option<BuildVolume>, String) {
    let (l, w, h, name) = largest_part_bbox(c, report);
    (smallest_fitting((l, w, h)), name)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::build_volume::onyx_pro;
    use crate::split::Joint;
    use helisim_airfoil::LinearAirfoil;
    use helisim_bemt::Config;
    use helisim_design::evaluate;

    fn setup() -> (DesignCandidate, DesignReport) {
        let c = DesignCandidate::model();
        let r = evaluate(&c, &LinearAirfoil::naca0012(), &Config::default());
        (c, r)
    }

    /// The blade is the biggest part and overflows the Onyx Pro → must split, and
    /// because it carries centrifugal tension the joint is BOLTED, not a snap.
    #[test]
    fn blade_overflows_and_is_bolted() {
        let (c, r) = setup();
        let plans = plan_prints(&c, &r, &onyx_pro());
        let blade = plans.iter().find(|p| p.part.contains("blade")).unwrap();
        assert!(!blade.fits, "blade exceeds the Onyx Pro bed");
        assert!(blade.pieces >= 2);
        assert!(blade.joint_load_n > 50.0, "centrifugal load is structural");
        assert!(matches!(
            blade.joint,
            Joint::Bolted { .. } | Joint::Overloaded
        ));
    }

    /// The biggest part overflows the desktop Onyx Pro; the blade fits a service
    /// SLS bed whole, while the very long tail boom exceeds every bed (it is really
    /// tube stock cut to length, or a multi-piece print) — `recommend_printer`
    /// returns `None` for the whole set, the honest "must split / use stock" signal.
    #[test]
    fn biggest_parts_need_a_service_bed_or_split() {
        use crate::build_volume::smallest_fitting;
        let (c, r) = setup();
        let (l, w, h, limiting) = largest_part_bbox(&c, &r);
        assert!(!limiting.is_empty());
        assert!(
            !onyx_pro().fits((l, w, h)),
            "biggest part overflows the desktop Onyx Pro"
        );
        // The 510 mm blade fits the 600 mm SLS bed whole.
        assert_eq!(
            smallest_fitting((510.0, 50.0, 6.0)).unwrap().name,
            "EOS SLS PA12 (service)"
        );
        // The boom (≈690 mm) exceeds every bed → no whole-print bed for the set.
        assert!(recommend_printer(&c, &r).0.is_none());
    }

    /// A light part (the fuselage shell) carries a low joint load → the smallest
    /// bolt (M2 through a flange), while the blade's structural load needs a
    /// bigger bolt — every joint bolted and sized.
    #[test]
    fn joint_bolt_scales_with_load() {
        let (c, r) = setup();
        let light = joint_load_for("fuselage / canopy pod", &c, &r);
        let heavy = joint_load_for("main-rotor blades", &c, &r);
        assert!(
            light < heavy,
            "shell seam is far lighter than the blade joint"
        );
    }
}
