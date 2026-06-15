//! Purchasable component catalog with **sourced, dated, overridable** unit prices.
//!
//! ## Provenance honesty (the project rule applied to money)
//! Every price here is a REPRESENTATIVE figure sourced from a named retailer on a
//! stated date, and is **overridable** — exactly the stance the `cost` crate takes.
//! Battery and hobby-electronics prices fluctuate weekly (sales, stock, bulk
//! breaks), so these are starting points to be confirmed against the live listing,
//! NOT quotes. The *structure* of the bill of materials (what you need and how many)
//! is the durable finding; the dollar totals carry the price caveat.
//!
//! Prices captured **2026-06-15** (USD). Where a retailer's sale price differed
//! from list, the more stable list-ish figure is used and the sale noted.

/// A sourced unit price. `url` is the product/category page; `as_of` is the
/// capture date; both make the number checkable and overridable.
#[derive(Clone, Copy, Debug)]
pub struct UnitPrice {
    pub usd: f64,
    pub retailer: &'static str,
    pub url: &'static str,
    pub as_of: &'static str,
}

impl UnitPrice {
    const fn new(usd: f64, retailer: &'static str, url: &'static str) -> Self {
        UnitPrice {
            usd,
            retailer,
            url,
            as_of: "2026-06-15",
        }
    }
}

/// Self-build taxonomy (parallels `helisim_cost::Buildability`): how a line item
/// enters the pack. Purchased items are the irreducible buy-list.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Buildability {
    /// Bought as-is (cells, BMS, connectors, fuse).
    Purchased,
    /// Bought as raw stock and cut/formed (nickel strip, wire, insulation).
    RawStock,
    /// A tool you buy once, not consumed per pack.
    Tool,
}

/// One line of the bill of materials.
#[derive(Clone, Debug)]
pub struct BomLine {
    pub item: String,
    pub qty: f64,
    pub unit: &'static str,
    pub unit_price: UnitPrice,
    pub buildability: Buildability,
    pub note: String,
}

impl BomLine {
    pub fn line_total_usd(&self) -> f64 {
        self.qty * self.unit_price.usd
    }
}

// ---- Sourced default prices (representative, 2026-06-15, overridable) ----

/// Per-cell price by display name. Sources: 18650 Battery Store / IMR Batteries
/// product pages (sale prices often lower than these list-ish figures).
pub fn cell_price(name: &str) -> UnitPrice {
    match name {
        "Molicel P50B" => UnitPrice::new(
            8.99,
            "18650 Battery Store",
            "https://www.18650batterystore.com/products/molicel-21700-p50b-5000mah-50a-battery",
        ),
        "Ampace JP40" => UnitPrice::new(
            5.99,
            "IMR Batteries",
            "https://imrbatteries.com/products/ampace-jp40-21700-4000mah-70a-battery",
        ),
        "BAK 45D" => UnitPrice::new(
            5.99,
            "18650 Battery Store",
            "https://www.18650batterystore.com/products/bak-45d-21700-4500mah-60a-battery",
        ),
        "EVE 40PL" => UnitPrice::new(
            5.99,
            "IMR Batteries",
            "https://imrbatteries.com/products/eve-40pl-21700-4000mah-70a-battery",
        ),
        _ => UnitPrice::new(7.00, "(generic 21700)", ""),
    }
}

/// A single integrated smart BMS (Li-ion variant) for small series counts. Daly
/// 7–8S 40A smart-BMS class. ~$37–75 on the official store; $45 representative.
pub const SMART_BMS: UnitPrice = UnitPrice::new(
    45.0,
    "Daly (official store)",
    "https://bmsdaly.com/collections/smart",
);

/// A distributed-BMS slave module (~16 channels) for large series strings.
/// Rougher estimate — flagged; confirm against a vendor quote.
pub const BMS_SLAVE_MODULE: UnitPrice =
    UnitPrice::new(60.0, "(distributed BMS, representative)", "");
/// Distributed-BMS master controller.
pub const BMS_MASTER: UnitPrice = UnitPrice::new(90.0, "(distributed BMS, representative)", "");
/// Main HV contactor for a large pack.
pub const CONTACTOR: UnitPrice = UnitPrice::new(40.0, "(HV contactor, representative)", "");
/// Hall current sensor for a large pack.
pub const CURRENT_SENSOR: UnitPrice = UnitPrice::new(25.0, "(Hall sensor, representative)", "");

/// Pure nickel strip, 0.15 mm × 8 mm, 5 m roll, 99.6 %.
pub const NICKEL_ROLL_5M: UnitPrice = UnitPrice::new(
    15.0,
    "DIY500AMP",
    "https://diy500amp.com/products/5m-roll-pure-nickel-strips-99-6-purity-16ft-0-15mm-x-8mm",
);
/// Balance-lead wiring kit (covers a small series count).
pub const BALANCE_LEAD_KIT: UnitPrice =
    UnitPrice::new(6.0, "(silicone balance wire, representative)", "");
/// Main lead: 12 AWG silicone wire + XT90 connector pigtail set (≤ ~90 A).
pub const MAIN_LEAD_XT90: UnitPrice = UnitPrice::new(
    8.0,
    "ReadyMadeRC",
    "https://www.readymaderc.com/products/details/86593-xt90-male-female-adapters-with-12awg-10cm-leads",
);
/// Heavy main cable + high-current connector (e.g. 2 AWG + Anderson/HV) for packs
/// drawing more than an XT90/12 AWG lead can carry. Representative.
pub const MAIN_LEAD_HEAVY: UnitPrice =
    UnitPrice::new(35.0, "(heavy gauge + HV connector, representative)", "");
/// Copper busbar interconnect set for high-current traction packs where 0.15 mm
/// nickel strip cannot carry the series-link current. Representative.
pub const COPPER_BUSBAR_SET: UnitPrice =
    UnitPrice::new(120.0, "(copper busbar / thick interconnect, representative)", "");
/// Inline fuse + holder (ANL/bolt class, rated to pack peak current).
pub const FUSE_AND_HOLDER: UnitPrice = UnitPrice::new(
    13.0,
    "Renogy (ANL fuse set)",
    "https://www.renogy.com/20a-30a-40a-60a-80a-100a-200a-300a-400a-anl-fuse-set-w-fuse/",
);
/// Fish/barley paper insulation roll.
pub const FISH_PAPER_ROLL: UnitPrice = UnitPrice::new(
    10.0,
    "Battery Hookup",
    "https://batteryhookup.com/products/5-ft-of-barley-fish-paper-battery-insulation-tape",
);
/// Kapton tape roll.
pub const KAPTON_ROLL: UnitPrice = UnitPrice::new(7.0, "(Kapton tape, representative)", "");
/// 21700 cell holder / spacer, per cell.
pub const CELL_HOLDER_EACH: UnitPrice =
    UnitPrice::new(0.20, "(21700 spacer, representative)", "");

// ---- Power distribution between the pack and the actuators ----

/// Brushless heli ESC that drives the motor off the pack (size ≥ motor continuous
/// current × headroom). Representative — confirm against a vendor part at the
/// motor's current/cell rating.
pub const ESC: UnitPrice = UnitPrice::new(40.0, "(brushless heli ESC, representative)", "");
/// Switching HV BEC that powers the digital control-surface servos off the pack
/// (size ≥ servo peak current). Representative.
pub const HV_BEC: UnitPrice = UnitPrice::new(20.0, "(HV switching BEC, representative)", "");

// ---- One-time tools (not consumed per pack) ----

/// Battery spot welder (kWeld-class / Sunkko 709A class).
pub const SPOT_WELDER: UnitPrice =
    UnitPrice::new(130.0, "kWeld (keenlab) / Sunkko", "https://kweld.keenlab.de/");
/// Smart balance charger for first charge / balancing.
pub const BALANCE_CHARGER: UnitPrice =
    UnitPrice::new(60.0, "SkyRC / ISDT (representative)", "https://www.skyrc.com/");
/// Multimeter for cell matching and checks.
pub const MULTIMETER: UnitPrice = UnitPrice::new(25.0, "(multimeter, representative)", "");

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn line_total_multiplies_qty_by_price() {
        let line = BomLine {
            item: "cell".into(),
            qty: 14.0,
            unit: "ea",
            unit_price: cell_price("EVE 40PL"),
            buildability: Buildability::Purchased,
            note: String::new(),
        };
        assert!((line.line_total_usd() - 14.0 * 5.99).abs() < 1e-9);
    }

    #[test]
    fn every_cell_has_a_price_and_known_cells_have_a_link() {
        for name in ["Molicel P50B", "Ampace JP40", "BAK 45D", "EVE 40PL"] {
            let p = cell_price(name);
            assert!(p.usd > 0.0);
            assert!(!p.url.is_empty(), "{name} missing link");
            assert_eq!(p.as_of, "2026-06-15");
        }
    }
}
