//! Unit costs — the **named, overridable inputs** of the parametric cost model.
//!
//! ⚠ These are a MODEL PARAMETERISATION, not sourced market facts. The defaults
//! are representative small-scale / hobby order-of-magnitude figures (early-2020s
//! USD) so the model produces a sensible shape out of the box; **replace them with
//! real quotes** for any decision. The project rule is to never present a
//! fabricated number as an oracle — so absolute costs here are explicitly
//! model-with-inputs, and only the *relative* breakdown and the buildability
//! split are treated as findings.

/// Per-unit cost inputs for the bill of materials. Override any field with a real
/// quote; the defaults are representative, not authoritative.
#[derive(Clone, Copy, Debug)]
pub struct UnitCosts {
    /// Frame / structure material, currency per kg (e.g. aluminium/CF stock).
    pub structure_per_kg: f64,
    /// Blade material, currency per kg (composite / ply / foam core).
    pub blade_per_kg: f64,
    /// Fabrication overhead added per kg of self-made part (tooling/consumables).
    pub fabrication_per_kg: f64,
    /// Motor build cost, currency per kW (magnets + copper + laminations).
    pub motor_per_kw: f64,
    /// ESC / motor controller, currency per kW (power semiconductors — bought).
    pub esc_per_kw: f64,
    /// Battery cells, currency per Wh (the dominant irreducible buy-item).
    pub cell_per_wh: f64,
    /// Pack assembly (BMS, interconnect, case), currency per Wh.
    pub pack_assembly_per_wh: f64,
    /// Flight controller + sensors, flat currency (bought).
    pub avionics_flat: f64,
}

impl Default for UnitCosts {
    /// Representative small-scale defaults (early-2020s USD, order-of-magnitude).
    /// Documented as assumptions — override with real quotes.
    fn default() -> Self {
        UnitCosts {
            structure_per_kg: 20.0,
            blade_per_kg: 40.0,
            fabrication_per_kg: 30.0,
            motor_per_kw: 50.0,
            esc_per_kw: 30.0,
            cell_per_wh: 0.30,
            pack_assembly_per_wh: 0.05,
            avionics_flat: 200.0,
        }
    }
}
