//! Charging EQUIPMENT — the circuit gear each source needs to actually deliver
//! charge power toward a **1:1 charge:flight** target, with a representative BOM.
//!
//! 1:1 means charging at flight power (`ratio = P_flight / P_charge`). This module
//! sizes each source's power and lists the install hardware, so the bill of
//! materials reflects how the pack gets charged — not just the pack itself.
//!
//! ## Provenance
//! Install costs (electrician circuits, EVSE, DC-charger $/kW, PV hardware) are
//! REPRESENTATIVE and overridable — the durable output is *what equipment each
//! path needs and the charge:flight ratio it reaches*, not the exact dollars.
//! A 120/240 V branch is hard-capped by code (NEC 80% continuous); only DC fast
//! charge (or a large PV array at full sun) can reach flight power for a big pack.

/// Per-watt / per-unit representative prices (USD, overridable).
const CHARGER_USD_PER_W_AC: f64 = 0.05; // AC→DC charger, ~$50 per kW
const DC_CHARGER_USD_PER_KW: f64 = 400.0; // DC fast charger hardware, ~$/kW
const CIRCUIT_120V_USD: f64 = 200.0; // dedicated 20 A/120 V circuit + outlet (electrician)
const CIRCUIT_240V_USD: f64 = 600.0; // 240 V branch + breaker (electrician)
const HV_CONNECTOR_USD: f64 = 300.0; // HV charge inlet/connector for DC fast
const PANEL_USD: f64 = 120.0; // 400 W panel (~$0.30/W)
const PANEL_W: f64 = 400.0;
const MPPT_USD_PER_W: f64 = 0.06; // MPPT controller, ~$/W of array
const PV_BOS_USD_PER_W: f64 = 0.10; // mounting + wiring (balance of system), $/W

/// One equipment line.
#[derive(Clone, Debug)]
pub struct EquipLine {
    pub item: String,
    pub qty: f64,
    pub unit_price_usd: f64,
    pub note: String,
}

impl EquipLine {
    pub fn total_usd(&self) -> f64 {
        self.qty * self.unit_price_usd
    }
}

/// A charging install for one source: the power it delivers, the charge:flight
/// ratio that buys, and the equipment to build it.
#[derive(Clone, Debug)]
pub struct ChargeKit {
    pub source: String,
    pub charge_power_w: f64,
    /// charge:flight time ratio = P_flight / P_charge (≤1 is at/under 1:1).
    pub charge_flight_ratio: f64,
    pub equipment: Vec<EquipLine>,
    pub note: String,
}

impl ChargeKit {
    pub fn equipment_total_usd(&self) -> f64 {
        self.equipment.iter().map(EquipLine::total_usd).sum()
    }
    /// Does this install reach ~1:1 (charge at least flight power)?
    pub fn reaches_unity(&self) -> bool {
        self.charge_flight_ratio <= 1.05
    }
}

fn ratio(flight_power_w: f64, charge_power_w: f64) -> f64 {
    if charge_power_w > 0.0 {
        flight_power_w / charge_power_w
    } else {
        f64::INFINITY
    }
}

/// 120 V kit — a dedicated 20 A/120 V circuit (NEC 80% → 16 A) through a charger.
/// Power is hard-capped ~1.7 kW; for a big pack the ratio is far from 1:1.
pub fn kit_120v(flight_power_w: f64) -> ChargeKit {
    let ac = 120.0 * 20.0 * 0.80;
    let dc = ac * 0.90;
    ChargeKit {
        source: "120 V / 20 A residential".into(),
        charge_power_w: dc,
        charge_flight_ratio: ratio(flight_power_w, dc),
        equipment: vec![
            EquipLine {
                item: "Dedicated 20 A/120 V circuit + outlet".into(),
                qty: 1.0,
                unit_price_usd: CIRCUIT_120V_USD,
                note: "electrician; code-capped at 1.92 kW AC".into(),
            },
            EquipLine {
                item: format!("AC→DC charger ~{:.0} W", dc),
                qty: 1.0,
                unit_price_usd: (dc * CHARGER_USD_PER_W_AC).max(80.0),
                note: "sized to the circuit".into(),
            },
        ],
        note: "Branch power is code-limited — no circuit beats ~1.9 kW on 120 V.".into(),
    }
}

/// 240 V Level-2 kit — a 240 V branch (up to ~80 A → ~17 kW DC). The biggest AC
/// option; reaches 1:1 only for modest flight powers.
pub fn kit_240v(flight_power_w: f64) -> ChargeKit {
    let max_dc = 240.0 * 80.0 * 0.80 * 0.90; // ~13.8 kW at 80 A
    let dc = flight_power_w.min(max_dc);
    let breaker_a = (dc / 0.90 / 0.80 / 240.0).ceil();
    ChargeKit {
        source: "240 V Level-2".into(),
        charge_power_w: dc,
        charge_flight_ratio: ratio(flight_power_w, dc),
        equipment: vec![
            EquipLine {
                item: format!("240 V circuit + {breaker_a:.0} A breaker"),
                qty: 1.0,
                unit_price_usd: CIRCUIT_240V_USD,
                note: "electrician".into(),
            },
            EquipLine {
                item: format!("EVSE / AC→DC charger ~{:.1} kW", dc / 1000.0),
                qty: 1.0,
                unit_price_usd: (dc * CHARGER_USD_PER_W_AC).max(300.0),
                note: "sized to flight power (capped ~17 kW)".into(),
            },
        ],
        note: if dc < flight_power_w {
            "Capped below flight power — best AC can do; DC fast needed for 1:1.".into()
        } else {
            "Reaches flight power → ~1:1.".into()
        },
    }
}

/// DC fast-charge kit — a charger sized to flight power, so it reaches ~1:1. The
/// only path to 1:1 for a high-power (human-scale) pack.
pub fn kit_dc_fast(flight_power_w: f64) -> ChargeKit {
    // Size to flight power, rounded up to a 10 kW increment (min 10 kW).
    let kw = (flight_power_w / 1000.0 / 10.0).ceil() * 10.0;
    let kw = kw.max(10.0);
    let dc = kw * 1000.0;
    let mut equipment = vec![
        EquipLine {
            item: format!("DC fast charger {kw:.0} kW"),
            qty: 1.0,
            unit_price_usd: kw * DC_CHARGER_USD_PER_KW,
            note: "grid→DC; needs a suitable service feed".into(),
        },
        EquipLine {
            item: "HV charge inlet + connector".into(),
            qty: 1.0,
            unit_price_usd: HV_CONNECTOR_USD,
            note: "matched to pack voltage".into(),
        },
    ];
    if kw >= 50.0 {
        equipment.push(EquipLine {
            item: "Charge-side cooling (liquid)".into(),
            qty: 1.0,
            unit_price_usd: 0.10 * dc,
            note: "fast charge heats the cells (I²R) — cool them".into(),
        });
    }
    ChargeKit {
        source: format!("DC fast charge ({kw:.0} kW)"),
        charge_power_w: dc,
        charge_flight_ratio: ratio(flight_power_w, dc),
        equipment,
        note: "Sized to flight power → ~1:1. Charge at ≤ flight C-rate to stay gentle on life."
            .into(),
    }
}

/// Solar kit — panels sized so the array, at full sun, delivers flight power (the
/// 1:1 condition). Returns the (often large) panel count + MPPT + balance-of-system.
pub fn kit_solar(flight_power_w: f64) -> ChargeKit {
    let per_panel_dc = PANEL_W * 0.80 * 0.97; // derate × MPPT, at full sun
    let panels = (flight_power_w / per_panel_dc).ceil().max(1.0);
    let array_w = panels * PANEL_W;
    let dc = panels * per_panel_dc;
    ChargeKit {
        source: format!("Solar ({panels:.0}× {PANEL_W:.0} W, {:.1} kW)", array_w / 1000.0),
        charge_power_w: dc,
        charge_flight_ratio: ratio(flight_power_w, dc),
        equipment: vec![
            EquipLine { item: format!("{PANEL_W:.0} W PV panel"), qty: panels, unit_price_usd: PANEL_USD, note: "~$0.30/W".into() },
            EquipLine { item: "MPPT charge controller(s)".into(), qty: 1.0, unit_price_usd: array_w * MPPT_USD_PER_W, note: "sized to array".into() },
            EquipLine { item: "Mounting + wiring (BOS)".into(), qty: 1.0, unit_price_usd: array_w * PV_BOS_USD_PER_W, note: "racking, combiner, cable".into() },
        ],
        note: "1:1 only at full sun; intermittent. For daily ops size to DAILY ENERGY (far fewer panels).".into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// DC fast charge sized to flight power reaches ~1:1; a 120 V branch cannot for
    /// a high-power pack.
    #[test]
    fn dc_fast_reaches_unity_120v_cannot() {
        let p_flight = 130_000.0; // human-scale hover
        let dc = kit_dc_fast(p_flight);
        assert!(
            dc.reaches_unity(),
            "DC fast ratio {}",
            dc.charge_flight_ratio
        );
        let wall = kit_120v(p_flight);
        assert!(!wall.reaches_unity());
        assert!(wall.charge_flight_ratio > 50.0); // ~100:1
    }

    /// For a small (model) flight power, even 120 V over-delivers (≤1:1).
    #[test]
    fn small_flight_power_unity_on_120v() {
        let kit = kit_120v(300.0);
        assert!(kit.reaches_unity());
    }

    /// 240 V is capped below a big flight power (best AC can do, still not 1:1).
    #[test]
    fn level2_capped_below_big_flight_power() {
        let kit = kit_240v(130_000.0);
        assert!(kit.charge_power_w < 130_000.0);
        assert!(!kit.reaches_unity());
    }

    /// Solar 1:1 for a big pack needs a large array; equipment totals are positive.
    #[test]
    fn solar_unity_array_is_large() {
        let kit = kit_solar(130_000.0);
        // ~130 kW / 310 W ≈ 420 panels.
        let panels = kit
            .equipment
            .iter()
            .find(|e| e.item.contains("panel"))
            .unwrap()
            .qty;
        assert!(panels > 300.0, "panels {panels}");
        assert!(kit.equipment_total_usd() > 0.0);
    }
}
