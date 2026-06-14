//! Build the bill of materials from a coarse aircraft specification.
//!
//! Maps the handful of quantities a sizing study produces (mass split, motor
//! power, pack energy) onto concrete components, each tagged with its cost and
//! buildability. The component *taxonomy* — which parts are raw-stock vs bought —
//! is the load-bearing modelling content; the costs ride on [`UnitCosts`].

use crate::component::{Buildability, Component};
use crate::costs::UnitCosts;

/// Coarse aircraft specification — the inputs a sizing study already produces.
#[derive(Clone, Copy, Debug)]
pub struct AircraftSpec {
    /// Number of main-rotor blades.
    pub n_blades: usize,
    /// Mass of one blade, kg.
    pub blade_mass_kg: f64,
    /// Rotor hub / head mass, kg.
    pub hub_mass_kg: f64,
    /// Airframe / structure mass, kg.
    pub structure_mass_kg: f64,
    /// Motor + ESC mass, kg.
    pub powertrain_mass_kg: f64,
    /// Installed motor power, kW (peak/continuous design point).
    pub motor_power_kw: f64,
    /// Battery pack energy, Wh.
    pub pack_energy_wh: f64,
    /// Battery pack mass, kg.
    pub pack_mass_kg: f64,
}

/// A complete bill of materials.
#[derive(Clone, Debug)]
pub struct Bom {
    /// The line items.
    pub items: Vec<Component>,
}

/// Assemble the bill of materials for a spec at the given unit costs.
pub fn build_bom(spec: &AircraftSpec, costs: &UnitCosts) -> Bom {
    let blade_mass = spec.n_blades as f64 * spec.blade_mass_kg;
    let motor_w = spec.motor_power_kw;
    // Split powertrain mass roughly motor:esc = 3:1.
    let motor_mass = spec.powertrain_mass_kg * 0.75;
    let esc_mass = spec.powertrain_mass_kg * 0.25;

    let items = vec![
        Component {
            name: "main-rotor blades",
            subsystem: "rotor",
            mass_kg: blade_mass,
            cost: blade_mass * (costs.blade_per_kg + costs.fabrication_per_kg),
            buildability: Buildability::RawStock,
        },
        Component {
            name: "rotor hub / head",
            subsystem: "rotor",
            mass_kg: spec.hub_mass_kg,
            cost: spec.hub_mass_kg * (costs.structure_per_kg + costs.fabrication_per_kg),
            buildability: Buildability::Fabricated,
        },
        Component {
            name: "airframe / structure",
            subsystem: "structure",
            mass_kg: spec.structure_mass_kg,
            cost: spec.structure_mass_kg * (costs.structure_per_kg + costs.fabrication_per_kg),
            buildability: Buildability::RawStock,
        },
        Component {
            name: "motor (magnets+copper+laminations)",
            subsystem: "powertrain",
            mass_kg: motor_mass,
            cost: motor_w * costs.motor_per_kw,
            buildability: Buildability::Assembled,
        },
        Component {
            name: "ESC / controller",
            subsystem: "powertrain",
            mass_kg: esc_mass,
            cost: motor_w * costs.esc_per_kw,
            buildability: Buildability::Purchased,
        },
        Component {
            name: "battery cells",
            subsystem: "energy",
            mass_kg: spec.pack_mass_kg * 0.8,
            cost: spec.pack_energy_wh * costs.cell_per_wh,
            buildability: Buildability::Purchased,
        },
        Component {
            name: "pack assembly (BMS, case)",
            subsystem: "energy",
            mass_kg: spec.pack_mass_kg * 0.2,
            cost: spec.pack_energy_wh * costs.pack_assembly_per_wh,
            buildability: Buildability::Assembled,
        },
        Component {
            name: "flight controller + sensors",
            subsystem: "avionics",
            mass_kg: 0.1,
            cost: costs.avionics_flat,
            buildability: Buildability::Purchased,
        },
    ];
    Bom { items }
}
