//! Roll a bill of materials up into cost + vertical-integration findings.

use crate::bom::Bom;
use crate::component::Buildability;

/// Aggregated cost and buildability summary of a [`Bom`].
#[derive(Clone, Debug)]
pub struct CostReport {
    /// Total estimated cost (currency of the unit costs).
    pub total_cost: f64,
    /// Total mass, kg.
    pub total_mass_kg: f64,
    /// Cost-weighted self-build fraction `Σ cost·self_fraction / Σ cost` — the
    /// **vertical-integration index** (1 = entirely self-made, 0 = all bought).
    pub vertical_integration_index: f64,
    /// Cost that must be spent on purchased (irreducible buy) items.
    pub purchased_cost: f64,
    /// Fraction of total cost that is purchased buy-items.
    pub purchased_cost_fraction: f64,
    /// (subsystem, cost) pairs, for the breakdown table.
    pub by_subsystem: Vec<(&'static str, f64)>,
    /// The irreducible buy-items (Purchased), as (name, cost), highest first.
    pub buy_items: Vec<(&'static str, f64)>,
}

/// Summarise a bill of materials.
pub fn summarize(bom: &Bom) -> CostReport {
    let total_cost: f64 = bom.items.iter().map(|c| c.cost).sum();
    let total_mass_kg: f64 = bom.items.iter().map(|c| c.mass_kg).sum();
    let self_built: f64 = bom.items.iter().map(|c| c.self_built_cost()).sum();
    let vii = if total_cost > 0.0 { self_built / total_cost } else { 0.0 };

    let purchased_cost: f64 = bom
        .items
        .iter()
        .filter(|c| c.buildability == Buildability::Purchased)
        .map(|c| c.cost)
        .sum();

    // Group costs by subsystem, preserving first-seen order.
    let mut by_subsystem: Vec<(&'static str, f64)> = Vec::new();
    for c in &bom.items {
        if let Some(entry) = by_subsystem.iter_mut().find(|(s, _)| *s == c.subsystem) {
            entry.1 += c.cost;
        } else {
            by_subsystem.push((c.subsystem, c.cost));
        }
    }

    let mut buy_items: Vec<(&'static str, f64)> = bom
        .items
        .iter()
        .filter(|c| c.buildability == Buildability::Purchased)
        .map(|c| (c.name, c.cost))
        .collect();
    buy_items.sort_by(|a, b| b.1.total_cmp(&a.1));

    CostReport {
        total_cost,
        total_mass_kg,
        vertical_integration_index: vii,
        purchased_cost,
        purchased_cost_fraction: if total_cost > 0.0 { purchased_cost / total_cost } else { 0.0 },
        by_subsystem,
        buy_items,
    }
}
