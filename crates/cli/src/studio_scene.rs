//! Fully itemised 3D scene for the design studio.
//!
//! The studio consumes `manufacture::aircraft_parts` directly, so the rendered
//! model is the same mesh exported to `aircraft.step` / `aircraft.stl` and used by
//! mass-property calculations. This module only adds UI-facing IDs, display
//! names, group labels, and smooth-shading hints.

use helisim_design::{DesignCandidate, DesignReport};
use helisim_manufacture::aircraft_parts;
use helisim_manufacture::mesh::Tri;

/// One pickable component: a stable id, display name, group (drives colour +
/// material), a smooth-shading hint, and the triangle mesh (mm, z-up).
pub struct ScenePart {
    pub id: String,
    pub name: String,
    pub group: &'static str,
    pub smooth: bool,
    pub tris: Vec<Tri>,
}

fn group_for_export_name(name: &str) -> &'static str {
    match name {
        "tail boom" => "boom",
        "tail boom fairing" | "canopy" => "fuselage",
        "horizontal_stab" | "tail_fin" => "tail_fin",
        "blade_grips" => "hub",
        "blade_root_fittings" => "hub",
        "powertrain_tray" => "motor",
        "landing_gear" => "landing_gear",
        "tail_rotor" => "tail_rotor",
        "swashplate" => "swashplate",
        "battery" => "battery",
        "motor" => "motor",
        "esc" => "esc",
        "avionics" => "avionics",
        "blade" => "blade",
        "mast" => "mast",
        "hub" => "hub",
        "fuselage" => "fuselage",
        _ => "other",
    }
}

fn display_name(name: &str, blade_index: usize) -> String {
    match name {
        "blade" => format!("main-rotor blade #{blade_index}"),
        "tail boom" => "tail boom".to_string(),
        "tail boom fairing" => "tail boom fairing".to_string(),
        "horizontal_stab" => "horizontal stabilizer".to_string(),
        "tail_fin" => "tail fin".to_string(),
        "landing_gear" => "landing gear (attached skids)".to_string(),
        "tail_rotor" => "tail rotor (anti-torque)".to_string(),
        "blade_grips" => "blade root grips".to_string(),
        "blade_root_fittings" => "blade root bushings + doublers".to_string(),
        "powertrain_tray" => "powertrain tray".to_string(),
        "battery" => "battery pack".to_string(),
        "motor" => "motor (BLDC)".to_string(),
        "esc" => "ESC (speed controller)".to_string(),
        "avionics" => "avionics (flight controller + IMU)".to_string(),
        "fuselage" => "fuselage / canopy pod".to_string(),
        "canopy" => "canopy blister".to_string(),
        other => other.replace('_', " "),
    }
}

fn is_smooth(name: &str) -> bool {
    matches!(
        name,
        "fuselage"
            | "canopy"
            | "tail boom fairing"
            | "mast"
            | "hub"
            | "swashplate"
            | "motor"
            | "blade"
            | "tail boom"
            | "tail_rotor"
    )
}

/// Build the full studio scene for a design.
pub fn studio_scene(c: &DesignCandidate, report: &DesignReport) -> Vec<ScenePart> {
    let mut blade_count = 0usize;
    aircraft_parts(c, report)
        .into_iter()
        .map(|(name, tris)| {
            if name == "blade" {
                blade_count += 1;
            }
            let id = if name == "blade" {
                if blade_count == 1 {
                    "blade".to_string()
                } else {
                    format!("blade_{blade_count}")
                }
            } else {
                name.replace(' ', "_")
            };
            ScenePart {
                id,
                name: display_name(name, blade_count),
                group: group_for_export_name(name),
                smooth: is_smooth(name),
                tris,
            }
        })
        .collect()
}
