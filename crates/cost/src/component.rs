//! One line item of the bill of materials, with its buildability.
//!
//! The buildability classification is the heart of the vertical-integration
//! priority: it records how much of each component you can realistically make
//! yourself versus must buy. It is a documented modelling taxonomy (the
//! `self_fraction` values are a transparent choice, not measured data), so the
//! resulting index is a *relative* guide to where self-fabrication effort pays
//! off — not an absolute claim.

/// How a component can be sourced, from most to least self-fabricable.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Buildability {
    /// Made directly from raw stock (wood, foam, metal bar, sheet): blades,
    /// frame, skids. The high-self-integration ideal.
    RawStock,
    /// Machined / 3D-printed / laid-up from stock with tooling: hub, mounts.
    Fabricated,
    /// Hand-assembled from purchased sub-parts you cannot economically make
    /// (motor from magnets+wire+laminations; pack from cells): you add the labour
    /// and integration, but the core parts are bought.
    Assembled,
    /// Must be purchased — no realistic home route: battery cells, rare-earth
    /// magnets, power semiconductors / ESC, MEMS sensors, the flight controller.
    Purchased,
}

impl Buildability {
    /// Fraction of this component's value you contribute by self-fabrication
    /// (the rest is bought-in material/parts). A modelling choice, documented:
    /// raw stock 0.95, fabricated 0.80, assembled 0.40, purchased 0.0.
    pub fn self_fraction(&self) -> f64 {
        match self {
            Buildability::RawStock => 0.95,
            Buildability::Fabricated => 0.80,
            Buildability::Assembled => 0.40,
            Buildability::Purchased => 0.0,
        }
    }

    /// Short label for tables.
    pub fn label(&self) -> &'static str {
        match self {
            Buildability::RawStock => "raw-stock",
            Buildability::Fabricated => "fabricated",
            Buildability::Assembled => "assembled",
            Buildability::Purchased => "PURCHASED",
        }
    }
}

/// One bill-of-materials line item.
#[derive(Clone, Debug)]
pub struct Component {
    /// Human-readable name.
    pub name: &'static str,
    /// Subsystem grouping (e.g. "rotor", "powertrain", "structure", "avionics").
    pub subsystem: &'static str,
    /// Mass, kg.
    pub mass_kg: f64,
    /// Estimated cost, in the [`crate::UnitCosts`] currency (USD by default).
    pub cost: f64,
    /// How it is sourced.
    pub buildability: Buildability,
}

impl Component {
    /// The self-fabricated share of this component's cost.
    pub fn self_built_cost(&self) -> f64 {
        self.cost * self.buildability.self_fraction()
    }
}
