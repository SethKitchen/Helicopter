//! Purchasable **hardware + consumables** for the build — every fitting the
//! instructions call for, as a buyable line with a retailer, a link, and a
//! representative price, so "are these easy to buy?" is answered with a shopping list.
//!
//! The fasteners/bearings come from the same selectors as [`crate::hardware_schedule`]
//! (smallest-adequate standard parts); the consumables (structural epoxy,
//! threadlocker, aluminium bar for the tangs, a carbon spar tube for split blades, a
//! blade balancer, a reamer) are the items the blade/root/grip steps require.
//!
//! **Price/provenance honesty (same rule as the rest of the repo):** prices are
//! representative figures at common retailers, NOT live quotes; links are listing /
//! search URLs to find the part, not an endorsement of a specific seller. Standard
//! metric hardware (socket-cap screws, nyloc nuts, deep-groove bearings) and the
//! consumables are all commodity items on Amazon / McMaster / a hobby shop.

use crate::fasteners::{retention_bolt, select_bearing};
use helisim_design::{DesignCandidate, DesignReport};

/// One purchasable line.
#[derive(Clone, Debug)]
pub struct BuyItem {
    /// What to buy (specific enough to search).
    pub item: String,
    /// Quantity (parts, or 1 for a kit/tube).
    pub qty: f64,
    /// Representative unit price, USD (NOT a quote).
    pub usd: f64,
    /// Where (retailer / marketplace).
    pub retailer: &'static str,
    /// A listing / search URL to find it.
    pub url: String,
    /// Why it's in the build.
    pub note: &'static str,
}

impl BuyItem {
    /// Line total, USD.
    pub fn line_total(&self) -> f64 {
        self.qty * self.usd
    }
}

/// The full hardware + consumables shopping list for a design's build.
pub fn shopping_list(c: &DesignCandidate, report: &DesignReport) -> Vec<BuyItem> {
    let nb = c.n_blades as f64;
    let mut v = Vec::new();

    // --- Selected fasteners / bearings (smallest-adequate, from the load) ---
    let omega = c.omega();
    let span = c.radius_m - c.root_cutout * c.radius_m;
    let m_blade = c.blade_areal_density_kg_m2 * c.chord_m * span;
    let r_cg = 0.5 * (c.radius_m + c.root_cutout * c.radius_m);
    let f_cf = omega * omega * m_blade * r_cg; // blade centrifugal force

    let amzn = |q: &str| format!("https://www.amazon.com/s?k={}", q.replace(' ', "+"));
    // Direct product link when the selected size matches the captured product (the
    // default design hits M3/623/626); otherwise fall back to an exact-size search so
    // the link is always correct. Direct links captured 2026-06-17 (live price at link).
    let pick = |direct: &str, is_match: bool, search: String| -> String {
        if is_match {
            direct.to_string()
        } else {
            amzn(&search)
        }
    };
    {
        let b = retention_bolt(f_cf);
        let m = b.name.to_lowercase();
        let is_m3 = b.name == "M3";
        v.push(BuyItem {
            item: format!("{} socket-cap screws, A2 stainless (assortment kit)", b.name),
            qty: 1.0,
            usd: 13.0,
            retailer: "Amazon",
            url: pick(
                "https://www.amazon.com/Assortment-Washers-Stainless-Printer-Projects/dp/B0G35DLTHY",
                is_m3,
                format!("{m} socket head cap screw stainless assortment"),
            ),
            note: "blade retention bolt = flap/feather pivot (one per blade + spares)",
        });
        v.push(BuyItem {
            item: format!("{} nyloc lock nuts, 304 stainless (100-pack)", b.name),
            qty: 1.0,
            usd: 9.0,
            retailer: "Amazon",
            url: pick(
                "https://www.amazon.com/Nylon-Insert-Lock-Stainless-Steel/dp/B07TBQHJL5",
                is_m3,
                format!("{m} nyloc nut stainless"),
            ),
            note: "locks each retention/pivot bolt",
        });
    }
    // Grip pitch (feather) bearings — two per blade.
    if let Some(brg) = select_bearing(3.0, f_cf, 1.5) {
        v.push(BuyItem {
            item: format!("{}ZZ deep-groove ball bearings (10-pack)", brg.name),
            qty: 1.0,
            usd: 9.0,
            retailer: "Amazon",
            url: pick(
                "https://www.amazon.com/XiKe-623ZZ-Pre-Lubricated-Performance-Cost-Effective/dp/B07JHT7D8R",
                brg.name == "623",
                format!("{}ZZ bearing 10 pack", brg.name),
            ),
            note: "grip pitch/feather bearings (2 per blade; carries centrifugal load)",
        });
    }
    // Mast bearings (×2) and swashplate bearing — size from the mast bore.
    let torque = if report.hover_shaft_power_w.is_finite() && omega > 0.0 {
        report.hover_shaft_power_w / omega
    } else {
        1.0
    };
    let mast_d_mm = crate::sizing::round_up_mm(crate::sizing::mast_min_dia_for_torsion(
        torque,
        crate::materials::TAU_ALLOW_AL,
    )) * 1000.0;
    let weight = c.gross_mass_kg * 9.80665;
    if let Some(brg) = select_bearing(mast_d_mm, weight, 2.0) {
        v.push(BuyItem {
            item: format!(
                "{}ZZ deep-groove ball bearings (mast ×2 + swashplate; 10-pack)",
                brg.name
            ),
            qty: 1.0,
            usd: 9.0,
            retailer: "Amazon",
            url: pick(
                "https://www.amazon.com/6x19x6-Bearing-bearings-Skateboard-Bearings/dp/B06WGXM7WB",
                brg.name == "626",
                format!("{}ZZ bearing 10 pack", brg.name),
            ),
            note: "two mast bearings + one swashplate bearing (bore on the mast)",
        });
    }

    // --- Consumables + tools the blade/root/grip steps require (direct product links,
    //     captured 2026-06-17; click for the live price). ---
    v.push(BuyItem {
        item: "J-B Weld 8265S Original cold-weld structural epoxy (2 oz)".to_string(),
        qty: 1.0,
        usd: 6.0,
        retailer: "Amazon",
        url: "https://www.amazon.com/J-B-Weld-8265S-Cold-Weld-Reinforced/dp/B0006O1ICE".to_string(),
        note: "bonds the aluminium doublers to the blade-root faces (the load path)",
    });
    v.push(BuyItem {
        item: "Loctite 243 medium-strength blue threadlocker (10 mL)".to_string(),
        qty: 1.0,
        usd: 12.0,
        retailer: "Amazon",
        url: "https://www.amazon.com/LOCTITE-Corp-Medium-Strength-Blue-Threadlocker/dp/B07K7YYCW9"
            .to_string(),
        note: "all rotating-head and control fasteners",
    });
    {
        let b = retention_bolt(f_cf);
        v.push(BuyItem {
            item: format!(
                "{} stainless standoff/spacers (bolt bushings; assortment)",
                b.name
            ),
            qty: 1.0,
            usd: 13.0,
            retailer: "Amazon",
            url: pick(
                "https://www.amazon.com/DANA-FRED-Standoff-Assortment/dp/B0BWRMFTC2",
                b.name == "M3",
                format!(
                    "{} stainless steel standoff spacer assortment",
                    b.name.to_lowercase()
                ),
            ),
            note: "press a steel spacer into each reamed root hole so the bolt bears on metal",
        });
    }
    v.push(BuyItem {
        item: "6061-T6511 aluminium flat bar, 1/8\" × 1\" × 12\" (root doublers)".to_string(),
        qty: 1.0,
        usd: 12.0,
        retailer: "Amazon (Remington)",
        url: "https://www.amazon.com/Aluminum-General-Purpose-Lengths-Available-Extruded/dp/B08DL4H2TC".to_string(),
        note: "cut 2 doubler plates per blade from this (the metal root load path)",
    });
    v.push(BuyItem {
        item: "TOOLAN mini saw + needle-file set (cut/finish the doublers)".to_string(),
        qty: 1.0,
        usd: 16.0,
        retailer: "Amazon",
        url: "https://www.amazon.com/TOOLAN-8-1-Ergonomic-Mechanism/dp/B0DXBR4THC".to_string(),
        note: "TOOL to cut + deburr the doubler plates from the flat bar",
    });
    v.push(BuyItem {
        item: "Laser/water-jet cut SERVICE (instead of cutting by hand)".to_string(),
        qty: 1.0,
        usd: 29.0,
        retailer: "SendCutSend",
        url: "https://sendcutsend.com/".to_string(),
        note: "instant quote — upload the doubler outline; they cut + ship the plates (~$29 min order)",
    });
    v.push(BuyItem {
        item: "STASRC magnetic blade/prop balancer (250–800 heli)".to_string(),
        qty: 1.0,
        usd: 16.0,
        retailer: "Amazon",
        url: "https://www.amazon.com/Magnetic-Multirotor-Propeller-Helicopter-Quadcopter/dp/B09MS35Q41".to_string(),
        note: "TOOL: balance the matched blade set (how-to in the blade build steps); mandatory",
    });
    v.push(BuyItem {
        item: "Utoolmart 3 mm HSS chucking reamer (close pivot fit)".to_string(),
        qty: 1.0,
        usd: 8.0,
        retailer: "Amazon",
        url: "https://www.amazon.com/Utoolmart-Straight-Chucking-Numerical-Indexable/dp/B0811B871K"
            .to_string(),
        note: "TOOL: ream the root + grip pivot hole for a slop-free flap/feather bolt",
    });

    // Whole-printing a large blade: the SLS service IS the purchase of the part.
    let span_mm = (c.radius_m - c.root_cutout * c.radius_m) * 1000.0;
    if span_mm > 320.0 && span_mm <= 750.0 {
        v.push(BuyItem {
            item: format!("SLS-print SERVICE — whole blade in PA-CF nylon ({span_mm:.0} mm span)"),
            qty: nb,
            usd: 60.0,
            retailer: "PCBWay / Xometry",
            url: "https://www.pcbway.com/rapid-prototyping/3d-printing/".to_string(),
            note: "upload blade.stl for an instant quote; they print + ship the whole blade",
        });
    }

    v
}

/// Total of the shopping list, USD.
pub fn shopping_total(items: &[BuyItem]) -> f64 {
    items.iter().map(|i| i.line_total()).sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use helisim_airfoil::LinearAirfoil;
    use helisim_bemt::Config;
    use helisim_design::evaluate;

    fn setup() -> (DesignCandidate, DesignReport) {
        let c = DesignCandidate::model();
        let r = evaluate(&c, &LinearAirfoil::naca0012(), &Config::default());
        (c, r)
    }

    #[test]
    fn list_covers_fasteners_bearings_and_consumables_with_links() {
        let (c, r) = setup();
        let list = shopping_list(&c, &r);
        // Every line has a non-empty item, retailer and a URL.
        for it in &list {
            assert!(!it.item.is_empty());
            assert!(!it.retailer.is_empty());
            assert!(it.url.starts_with("http"), "{} has no link", it.item);
            assert!(it.usd > 0.0);
        }
        let joined = list
            .iter()
            .map(|i| i.item.as_str())
            .collect::<Vec<_>>()
            .join(" | ");
        assert!(joined.contains("epoxy"), "epoxy listed");
        assert!(joined.contains("balancer"), "balancer listed");
        assert!(joined.contains("bearing"), "bearings listed");
        assert!(
            joined.to_lowercase().contains("screw"),
            "retention bolt listed"
        );
        assert!(shopping_total(&list) > 0.0);
    }
}
