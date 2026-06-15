//! The complete build package: every part, sized from one design, plus the order
//! to assemble them.
//!
//! This is where the parts come together. [`build_package`] takes a design and its
//! evaluated report and produces the full set of [`BuildPart`]s (blade, hub, mast,
//! swashplate, boom, powertrain tray) — each physically sized from the same
//! design — followed by the assembly sequence that joins them into a flying
//! machine.

use crate::blade::blade_from_design;
use crate::boom::boom_for;
use crate::fuselage::fuselage_for;
use crate::hub::hub_from_blade;
use crate::landing_gear::landing_gear_for;
use crate::mast::mast_for_torque;
use crate::materials::SIGMA_ALLOW_AL;
use crate::mount::mount_for;
use crate::part::BuildPart;
use crate::root_fitting::root_fitting_for;
use crate::swashplate::swashplate_for;
use crate::tail_rotor::tail_rotor_for;
use helisim_design::{DesignCandidate, DesignReport};

/// A complete, ordered build: all the parts plus the assembly sequence.
pub struct BuildPackage {
    /// Every part to make or buy, sized from the design.
    pub parts: Vec<Box<dyn BuildPart>>,
    /// The order in which to assemble them.
    pub assembly_steps: Vec<String>,
}

/// Build the full package from a design and its evaluated report (which supplies
/// the hover power → torque used to size the mast and boom).
pub fn build_package(c: &DesignCandidate, report: &DesignReport) -> BuildPackage {
    let omega = c.omega();
    let torque = if report.hover_shaft_power_w.is_finite() && omega > 0.0 {
        report.hover_shaft_power_w / omega
    } else {
        1.0
    };
    let head_height = 0.20 * c.radius_m + 0.05;
    let pack_mass = 0.25 * c.gross_mass_kg;

    let blade = blade_from_design(c, 0.0);
    let mast = mast_for_torque(torque, head_height);
    let hub = hub_from_blade(
        c.n_blades,
        blade.chord_m,
        blade.max_thickness_m,
        blade.root_radius_m,
        mast.diameter_m,
    );
    let swash = swashplate_for(c.radius_m, mast.diameter_m, c.n_blades);
    let boom = boom_for(torque, c.radius_m);
    let mount = mount_for(pack_mass, c.radius_m);
    let fuselage = fuselage_for(c.gross_mass_kg, c.radius_m);
    let landing_gear = landing_gear_for(
        c.gross_mass_kg,
        fuselage.length_m,
        fuselage.width_m,
        SIGMA_ALLOW_AL,
    );
    // Retention bolt sized for the centrifugal load (double shear, 200 MPa working).
    let omega2 = omega * omega;
    let span = c.radius_m - blade.root_radius_m;
    let m_blade = c.blade_areal_density_kg_m2 * c.chord_m * span;
    let r_cg = 0.5 * (c.radius_m + blade.root_radius_m);
    let f_cf = omega2 * m_blade * r_cg;
    let bolt_d = (2.0 * f_cf / (std::f64::consts::PI * 200.0e6))
        .sqrt()
        .max(0.003);
    let root_fit = root_fitting_for(c.n_blades, blade.chord_m, blade.max_thickness_m, bolt_d);
    let tail = tail_rotor_for(torque, c.radius_m, c.tip_speed_ms);

    let parts: Vec<Box<dyn BuildPart>> = vec![
        Box::new(blade),
        Box::new(root_fit),
        Box::new(hub),
        Box::new(mast),
        Box::new(swash),
        Box::new(tail),
        Box::new(boom),
        Box::new(fuselage),
        Box::new(landing_gear),
        Box::new(mount),
    ];

    let assembly_steps = vec![
        "1. Bolt the powertrain tray + motor to the airframe core.".to_string(),
        "2. Install the mast in its bearings and couple it to the motor/gearbox.".to_string(),
        "3. Slide the swashplate onto the mast; mount the servos and link them (120° CCPM)."
            .to_string(),
        "4. Fit the hub to the mast top; connect pitch links from the swashplate to the grips."
            .to_string(),
        "5. Bolt the blades into the grips; set zero pitch and track the blades.".to_string(),
        "6. Fit the tail boom + tail rotor; set the anti-torque control.".to_string(),
        "7. Install the battery on the tray; set the CG on the rotor shaft axis.".to_string(),
        "8. Bench-test all controls for direction and travel; balance + track the rotor."
            .to_string(),
        "9. Tethered spin-up; check vibration and control response before free flight.".to_string(),
        "10. SAFETY: verify the power-loss response (instant collective drop / governor) — the \
             rotor-decay window is short at model scale."
            .to_string(),
    ];

    BuildPackage {
        parts,
        assembly_steps,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::part::Source;
    use helisim_airfoil::LinearAirfoil;
    use helisim_bemt::Config;
    use helisim_design::evaluate;

    fn pkg() -> BuildPackage {
        let c = DesignCandidate::model();
        let report = evaluate(&c, &LinearAirfoil::naca0012(), &Config::default());
        build_package(&c, &report)
    }

    #[test]
    fn package_has_all_parts_with_steps() {
        let p = pkg();
        assert_eq!(p.parts.len(), 10);
        for part in &p.parts {
            // Exercise the full BuildPart surface on every concrete part type.
            assert!(!part.name().is_empty());
            assert!(!part.material().is_empty());
            assert!(!part.source().label().is_empty());
            assert!(
                !part.build_steps().is_empty(),
                "{} has no steps",
                part.name()
            );
            assert!(!part.key_dimensions_mm().is_empty());
        }
        // Every Source variant has a label.
        for s in [
            Source::RawStock,
            Source::Fabricated,
            Source::Assembled,
            Source::Purchased,
        ] {
            assert!(!s.label().is_empty());
        }
        assert!(p.assembly_steps.len() >= 8);
    }

    #[test]
    fn includes_the_blade_and_mast_and_boom() {
        let p = pkg();
        let names: Vec<&str> = p.parts.iter().map(|x| x.name()).collect();
        assert!(names.iter().any(|n| n.contains("blade")));
        assert!(names.iter().any(|n| n.contains("mast")));
        assert!(names.iter().any(|n| n.contains("boom")));
        assert!(names.iter().any(|n| n.contains("tail rotor")));
        assert!(names.iter().any(|n| n.contains("fuselage")));
        assert!(names.iter().any(|n| n.contains("landing gear")));
    }
}
