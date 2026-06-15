//! Outsourced manufacturing **services** — where to get the control surfaces made.
//!
//! A small catalogue of on-demand 3D-printing / CNC services with their business
//! model, processes, an instant-quote URL, and a cost tier. **Pricing is
//! quote-based** (geometry / material / volume / lead-time dependent) — exact cost
//! comes from uploading the CAD to each service's instant-quote engine, so this
//! records the *pricing model* and relative tier, plus the few sourced anchors
//! (e.g. PCBWay metal ≈ $100/kg SS316L), never a fabricated per-part price.

use crate::service_material::CostLevel;

/// One manufacturing service.
#[derive(Clone, Copy, Debug)]
pub struct Service {
    /// Service name.
    pub name: &'static str,
    /// Business model.
    pub model: &'static str,
    /// Processes relevant to these parts.
    pub processes: &'static str,
    /// Instant-quote URL (upload CAD → price + lead time).
    pub quote_url: &'static str,
    /// Relative cost tier.
    pub cost: CostLevel,
    /// Pricing-model / positioning note (sourced).
    pub note: &'static str,
}

/// The service catalogue (sourced from each vendor's site, 2026-06).
pub fn services() -> Vec<Service> {
    vec![
        Service {
            name: "Protolabs",
            model: "in-house bureau",
            processes: "SLS, MJF, SLA, DMLS, CNC, injection moulding",
            quote_url: "https://www.protolabs.com/services/3d-printing/",
            cost: CostLevel::Premium,
            note: "fastest (1–3 day), in-house; often cheaper than Xometry for small/simple parts",
        },
        Service {
            name: "Xometry",
            model: "marketplace (10,000+ shops)",
            processes: "SLS (PA12/GF/CF), MJF, FDM, CNC, sheet metal",
            quote_url: "https://www.xometry.com/capabilities/3d-printing-service/",
            cost: CostLevel::Mid,
            note: "AI instant-quote; widest material menu incl. carbon-filled nylon; ISO/AS9100",
        },
        Service {
            name: "Fictiv",
            model: "managed marketplace",
            processes: "FDM, SLS, SLA, MJF, PolyJet, CNC",
            quote_url: "https://www.fictiv.com/",
            cost: CostLevel::Premium,
            note: "vetted suppliers + strong DFM support; higher price, fewer 3DP processes",
        },
        Service {
            name: "Sculpteo",
            model: "service bureau (BASF)",
            processes: "SLS (PA11/PA12/GF/CF), MJF, CNC",
            quote_url: "https://www.sculpteo.com/en/",
            cost: CostLevel::Mid,
            note: "material-science focus; good for advanced nylon grades",
        },
        Service {
            name: "Shapeways",
            model: "marketplace",
            processes: "SLS nylon (PA12/CF), MJF, +metals; 55+ materials",
            quote_url: "https://www.shapeways.com/",
            cost: CostLevel::Mid,
            note: "broadest material/finish menu; per-part instant pricing",
        },
        Service {
            name: "Craftcloud",
            model: "aggregator / price comparison",
            processes: "routes to partner SLS/MJF/CNC/metal",
            quote_url: "https://craftcloud3d.com/",
            cost: CostLevel::Budget,
            note: "compares partner prices (save ~50%), no service fee / no minimums",
        },
        Service {
            name: "PCBWay / JLC3DP",
            model: "low-cost bureau (Asia)",
            processes: "SLS PA12, MJF, resin, CNC metal",
            quote_url: "https://www.pcbway.com/rapid-prototyping/3d-printing/",
            cost: CostLevel::Budget,
            note: "cheapest; metal ≈ $100/kg SS316L; longer shipping; 2025 polymer price cuts",
        },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn catalogue_is_well_formed() {
        let s = services();
        assert!(s.len() >= 6);
        for svc in &s {
            assert!(
                svc.quote_url.starts_with("https://"),
                "{} bad url",
                svc.name
            );
            assert!(!svc.note.is_empty() && !svc.processes.is_empty());
        }
    }

    #[test]
    fn spans_budget_to_premium() {
        let s = services();
        assert!(s.iter().any(|x| x.cost == CostLevel::Budget));
        assert!(s.iter().any(|x| x.cost == CostLevel::Premium));
        // The named anchors are present.
        assert!(s.iter().any(|x| x.name == "Protolabs"));
        assert!(s.iter().any(|x| x.name == "Xometry"));
    }
}
