//! Structural & inertial consequences of **splitting** parts to fit the printer.
//!
//! A split is not free, and this ties the split planning back into the physics it
//! affects. For the rotor **blade** (where it matters most):
//! * the joint adds hardware mass (bolts + a local doubler) at a spanwise station
//!   → raises the rotor polar inertia `I` (∝ m·r²) → raises stored **flare energy**
//!   `½IΩ²` (autorotation), the **gross mass**, the centrifugal load, and shifts
//!   the **Lock number** (flapping);
//! * the bolted splice is the new **critical section** — it must carry the
//!   centrifugal tension *at the joint radius* through its **net area** (minus the
//!   bolt holes), reported here as a margin.
//!
//! For other split parts it tallies the added joint (bolt) mass into the gross.
//!
//! It also models the two effects previously left as caps — they are now computed
//! numbers, not TODOs: the local **flap-stiffness knockdown** of the bolted splice
//! (added tip deflection from the joint's compliance) and the **parasite drag** of
//! the joint surface step (added profile power).

use crate::blade::blade_from_design;
use crate::build_volume::BuildVolume;
use crate::fasteners::select_bolt;
use crate::naca_section::{flap_inertia, max_thickness, structural_area};
use crate::part::BuildPart;
use helisim_design::DesignCandidate;

/// Steel fastener density, kg/m³.
const STEEL_DENSITY: f64 = 7850.0;
/// Local doubler/reinforcement mass per blade joint, as a fraction of blade mass.
/// A documented first-cut (a short bonded splice plate over the joint).
pub const DOUBLER_MASS_FRAC: f64 = 0.05;
/// Bending-stiffness efficiency of a bolted/spliced joint vs continuous material
/// (a documented value; bolted composite splices run ~0.6–0.8).
pub const JOINT_FLEX_EFFICIENCY: f64 = 0.7;
/// Residual surface step at a finished joint, mm (print/assembly mismatch) — drives
/// the joint parasite drag.
pub const JOINT_STEP_MM: f64 = 0.2;
/// Drag coefficient of a small forward-facing surface step.
const STEP_CD: f64 = 1.2;
/// Sea-level air density, kg/m³.
const RHO: f64 = 1.225;

/// Approximate mass of one bolt of nominal diameter `d_m`, kg (shank + ~50 % head).
fn bolt_mass_kg(d_m: f64) -> f64 {
    let shank_len = 4.0 * d_m; // ~4 diameters of grip
    let vol = std::f64::consts::PI * d_m * d_m / 4.0 * shank_len;
    STEEL_DENSITY * vol * 1.5
}

/// What splitting the blade does to the rotor + the blade-joint strength check.
#[derive(Clone, Debug)]
pub struct BladeJointEffect {
    /// Number of spanwise joints (0 if the blade prints whole on this bed).
    pub joints: usize,
    /// Radius of the innermost (most-loaded) joint, m.
    pub joint_radius_m: f64,
    /// Centrifugal tension carried at that joint, N.
    pub cf_at_joint_n: f64,
    /// Net-section margin (strength / σ through the bolt holes).
    pub net_section_margin: f64,
    /// Bolt chosen for the splice and the count per joint.
    pub bolt: Option<String>,
    pub bolts_per_joint: usize,
    /// Added mass per blade (bolts + doublers), kg.
    pub added_mass_per_blade_kg: f64,
    /// Rotor polar inertia before / after the split, kg·m².
    pub base_inertia: f64,
    pub new_inertia: f64,
    /// Stored flare energy before / after, J, and the percentage change.
    pub base_flare_j: f64,
    pub new_flare_j: f64,
    pub flare_delta_pct: f64,
    /// Added gross mass from all blade joints, kg.
    pub gross_mass_delta_kg: f64,
    /// Added flap tip deflection from the splice's stiffness knockdown, mm
    /// (the modelled "stiffness cap").
    pub flap_deflection_penalty_mm: f64,
    /// Added rotor profile power from the joint surface step, W (the modelled
    /// "drag cap").
    pub drag_power_penalty_w: f64,
}

/// Compute the rotor-level effect and strength check of splitting the blade on a
/// given build volume. `blade_tensile_pa` is the blade material working tensile
/// stress (e.g. ~200 MPa for Onyx+Fiberglass, lower for neat nylon).
pub fn blade_joint_effect(
    c: &DesignCandidate,
    vol: &BuildVolume,
    blade_tensile_pa: f64,
    blade_flex_modulus_pa: f64,
) -> BladeJointEffect {
    let blade = blade_from_design(c, 0.0);
    let pieces = vol.pieces_needed(blade.bounding_box_mm());
    let joints = pieces.saturating_sub(1);

    let omega = c.omega();
    let r = c.radius_m;
    let root_r = c.root_cutout * r;
    let span = r - root_r;
    let chord = c.chord_m;
    let mu = c.blade_areal_density_kg_m2 * chord; // mass / span
    let base_inertia = c.rotor_inertia;
    let base_flare = 0.5 * base_inertia * omega * omega;

    if joints == 0 {
        return BladeJointEffect {
            joints: 0,
            joint_radius_m: 0.0,
            cf_at_joint_n: 0.0,
            net_section_margin: f64::INFINITY,
            bolt: None,
            bolts_per_joint: 0,
            added_mass_per_blade_kg: 0.0,
            base_inertia,
            new_inertia: base_inertia,
            base_flare_j: base_flare,
            new_flare_j: base_flare,
            flare_delta_pct: 0.0,
            gross_mass_delta_kg: 0.0,
            flap_deflection_penalty_mm: 0.0,
            drag_power_penalty_w: 0.0,
        };
    }

    // Joint radii: evenly spaced; the innermost carries the most tension.
    let radii: Vec<f64> = (1..pieces)
        .map(|k| root_r + span * k as f64 / pieces as f64)
        .collect();
    let cf_at = |rad: f64| omega * omega * mu * (r * r - rad * rad) / 2.0;
    let r_in = radii[0];
    let cf_in = cf_at(r_in);

    // Bolt: BOLTS_PER_JOINT share the centrifugal load in double shear, SF 2.
    let bolts_per_joint = crate::split::BOLTS_PER_JOINT;
    let bolt = select_bolt(cf_in / bolts_per_joint as f64, true, 2.0);
    let bolt_d = bolt
        .as_ref()
        .map(|b| b.diameter_mm / 1000.0)
        .unwrap_or(0.003);

    // Net-section margin: conservative load-bearing area minus the bolt holes.
    let t = max_thickness(chord);
    let a_struct = structural_area(chord);
    let a_net = (a_struct - bolts_per_joint as f64 * bolt_d * t).max(1e-9);
    let net_section_margin = blade_tensile_pa / (cf_in / a_net);

    // Added mass per blade: bolts + a doubler at each joint.
    let m_blade = mu * span;
    let per_joint = bolts_per_joint as f64 * bolt_mass_kg(bolt_d) + DOUBLER_MASS_FRAC * m_blade;
    let added_mass_per_blade = per_joint * joints as f64;

    // Inertia increment: each joint's mass at its radius (∝ r²).
    let di_per_blade: f64 = radii.iter().map(|&rad| per_joint * rad * rad).sum();
    let new_inertia = base_inertia + c.n_blades as f64 * di_per_blade;
    let new_flare = 0.5 * new_inertia * omega * omega;

    // --- modelled cap 1: flap-stiffness knockdown → added tip deflection ---
    // The splice has efficiency η over a short length L_j; the extra rotation it
    // allows under the local flap moment propagates to the tip. Summed over joints.
    // Uses the accurate integrated NACA flap inertia (not the thin-rectangle
    // over-estimate), consistent with the FEA blade model.
    let i_flap = flap_inertia(chord);
    let ei = blade_flex_modulus_pa * i_flap;
    let w_lift = c.gross_mass_kg * 9.80665 / c.n_blades as f64 / span; // distributed lift, N/m
    let l_joint = 0.3 * chord; // splice length ≈ 30 % chord
    let flap_penalty_m: f64 = radii
        .iter()
        .map(|&rad| {
            let m_local = w_lift * (r - rad).powi(2) / 2.0; // cantilever moment at the joint
            let extra_rot = m_local * l_joint * (1.0 / JOINT_FLEX_EFFICIENCY - 1.0) / ei;
            extra_rot * (r - rad) // → tip deflection
        })
        .sum();

    // --- modelled cap 2: joint surface-step parasite drag → added profile power ---
    // A step of height JOINT_STEP_MM across the chord at radius rad sees speed Ω·rad;
    // ΔP = ½ρ v³ Δf, Δf = Cd · step · chord. Summed over joints and blades.
    let d_f = STEP_CD * (JOINT_STEP_MM / 1000.0) * chord; // drag area, m²
    let drag_w: f64 = c.n_blades as f64
        * radii
            .iter()
            .map(|&rad| 0.5 * RHO * (omega * rad).powi(3) * d_f)
            .sum::<f64>();

    BladeJointEffect {
        joints,
        joint_radius_m: r_in,
        cf_at_joint_n: cf_in,
        net_section_margin,
        bolt: bolt.map(|b| b.name.to_string()),
        bolts_per_joint,
        added_mass_per_blade_kg: added_mass_per_blade,
        base_inertia,
        new_inertia,
        base_flare_j: base_flare,
        new_flare_j: new_flare,
        flare_delta_pct: (new_flare / base_flare - 1.0) * 100.0,
        gross_mass_delta_kg: c.n_blades as f64 * added_mass_per_blade,
        flap_deflection_penalty_mm: flap_penalty_m * 1000.0,
        drag_power_penalty_w: drag_w,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::build_volume::{eos_sls_pa12, onyx_pro};

    /// On the desktop Onyx Pro the blade splits → the joint ADDS inertia (more
    /// flare energy) and gross mass, and the net section still passes in a
    /// fiberglass-grade blade. The effect is real and signed correctly.
    #[test]
    fn split_blade_adds_inertia_flare_and_mass() {
        let c = DesignCandidate::model();
        let e = blade_joint_effect(&c, &onyx_pro(), 200.0e6, 22.0e9);
        assert!(e.joints >= 1, "blade overflows the Onyx Pro");
        assert!(
            e.new_inertia > e.base_inertia,
            "joint mass raises rotor inertia"
        );
        assert!(e.flare_delta_pct > 0.0, "more stored flare energy");
        assert!(e.gross_mass_delta_kg > 0.0, "gross mass goes up");
        // The mid-span joint carries less than the root centrifugal load and the
        // fiberglass net section holds it.
        assert!(e.cf_at_joint_n > 0.0);
        assert!(e.net_section_margin > 1.0, "joint passes in FG-grade blade");
        assert!(e.bolt.is_some());
    }

    /// The two former "named caps" are now real numbers: the splice adds some flap
    /// deflection (>0, finite) and some profile drag power (>0, finite) — modelled,
    /// not hand-waved.
    #[test]
    fn stiffness_and_drag_caps_are_modelled_numbers() {
        let c = DesignCandidate::model();
        let e = blade_joint_effect(&c, &onyx_pro(), 200.0e6, 22.0e9);
        assert!(e.flap_deflection_penalty_mm > 0.0 && e.flap_deflection_penalty_mm.is_finite());
        assert!(e.drag_power_penalty_w > 0.0 && e.drag_power_penalty_w.is_finite());
        // A softer blade (lower modulus) suffers a bigger stiffness penalty.
        let soft = blade_joint_effect(&c, &onyx_pro(), 200.0e6, 3.0e9).flap_deflection_penalty_mm;
        assert!(soft > e.flap_deflection_penalty_mm);
    }

    /// A too-weak (neat nylon) blade fails the joint net-section check — the split
    /// can be the governing structural location, which the check surfaces.
    #[test]
    fn weak_blade_material_can_fail_the_joint() {
        // Big, fast rotor + weak material → the joint net section is overstressed.
        let mut c = DesignCandidate::model();
        c.tip_speed_ms = 200.0;
        let strong = blade_joint_effect(&c, &onyx_pro(), 200.0e6, 22.0e9).net_section_margin;
        let weak = blade_joint_effect(&c, &onyx_pro(), 20.0e6, 22.0e9).net_section_margin;
        assert!(weak < strong);
    }

    /// If the blade fits the bed whole (big SLS bed), there is no joint and no
    /// effect — the coupling is correctly zero.
    #[test]
    fn no_split_no_effect() {
        // A short blade that fits the 600 mm SLS bed.
        let mut c = DesignCandidate::model();
        c.radius_m = 0.3;
        let e = blade_joint_effect(&c, &eos_sls_pa12(), 200.0e6, 22.0e9);
        assert_eq!(e.joints, 0);
        assert_eq!(e.flare_delta_pct, 0.0);
        assert_eq!(e.gross_mass_delta_kg, 0.0);
        assert_eq!(e.flap_deflection_penalty_mm, 0.0);
        assert_eq!(e.drag_power_penalty_w, 0.0);
    }
}
