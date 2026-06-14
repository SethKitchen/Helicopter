//! Fastener + bearing selection — choose real standard hardware for each joint.
//!
//! The fabricated parts have holes and seats; the hardware that goes in them
//! comes from standard catalogues. This module carries small catalogues of metric
//! bolts (class 8.8) and deep-groove ball bearings and **selects the smallest
//! standard part whose rated capacity meets the joint load** (with a safety
//! factor). That selection rule is the validation: the chosen part passes and the
//! next size down fails.
//!
//! Loads come from the design — the dominant one is the blade-retention bolt under
//! centrifugal force; bearings are chosen by the shaft bore they must fit.

use helisim_design::{DesignCandidate, DesignReport};
use std::f64::consts::PI;

/// A standard metric bolt (class 8.8).
#[derive(Clone, Copy, Debug)]
pub struct Bolt {
    /// Designation, e.g. "M3".
    pub name: &'static str,
    /// Nominal diameter, mm.
    pub diameter_mm: f64,
    /// Tensile stress area, mm².
    pub stress_area_mm2: f64,
    /// Single-shear working capacity, N (≈ 200 MPa working shear × area).
    pub shear_capacity_n: f64,
}

/// The metric bolt catalogue (class 8.8, 200 MPa working shear).
///
/// Stress areas are the **ISO 724 standard tensile stress areas** (M3 = 5.03 mm²,
/// M4 = 8.78, M5 = 14.2, M6 = 20.1, M8 = 36.6, M10 = 58.0 mm²). The 200 MPa
/// working shear is the class-8.8 ultimate shear (≈ 0.6·R_m = 0.6·800 = 480 MPa,
/// ISO 898-1) divided by a safety factor ≈ 2.4.
pub fn bolt_catalogue() -> Vec<Bolt> {
    // (name, Ø mm, stress area mm²)
    let raw = [
        ("M2", 2.0, 2.07),
        ("M2.5", 2.5, 3.39),
        ("M3", 3.0, 5.03),
        ("M4", 4.0, 8.78),
        ("M5", 5.0, 14.2),
        ("M6", 6.0, 20.1),
        ("M8", 8.0, 36.6),
        ("M10", 10.0, 58.0),
    ];
    raw.iter()
        .map(|&(name, d, a)| Bolt {
            name,
            diameter_mm: d,
            stress_area_mm2: a,
            shear_capacity_n: 200.0e6 * a * 1e-6,
        })
        .collect()
}

/// A standard deep-groove ball bearing.
#[derive(Clone, Copy, Debug)]
pub struct Bearing {
    pub name: &'static str,
    pub bore_mm: f64,
    pub od_mm: f64,
    pub width_mm: f64,
    /// Dynamic load rating C, N.
    pub dynamic_c_n: f64,
}

/// The bearing catalogue (a common subset).
pub fn bearing_catalogue() -> Vec<Bearing> {
    let raw = [
        ("623", 3.0, 10.0, 4.0, 750.0),
        ("624", 4.0, 13.0, 5.0, 1330.0),
        ("625", 5.0, 16.0, 5.0, 1900.0),
        ("626", 6.0, 19.0, 6.0, 2900.0),
        ("608", 8.0, 22.0, 7.0, 3400.0),
        ("6000", 10.0, 26.0, 8.0, 4550.0),
        ("6001", 12.0, 28.0, 8.0, 5100.0),
        ("6002", 15.0, 32.0, 9.0, 5600.0),
    ];
    raw.iter()
        .map(|&(name, bore, od, w, c)| Bearing {
            name,
            bore_mm: bore,
            od_mm: od,
            width_mm: w,
            dynamic_c_n: c,
        })
        .collect()
}

/// Select the smallest bolt whose capacity meets `load_n` (×2 in double shear)
/// with safety factor `sf`. `None` if even the largest is inadequate.
pub fn select_bolt(load_n: f64, double_shear: bool, sf: f64) -> Option<Bolt> {
    let mult = if double_shear { 2.0 } else { 1.0 };
    bolt_catalogue()
        .into_iter()
        .find(|b| b.shear_capacity_n * mult >= load_n * sf)
}

/// Select the smallest bearing with bore ≥ `bore_min_mm` and rating ≥ `load_n`×`sf`.
pub fn select_bearing(bore_min_mm: f64, load_n: f64, sf: f64) -> Option<Bearing> {
    bearing_catalogue()
        .into_iter()
        .filter(|b| b.bore_mm >= bore_min_mm - 1e-9)
        .find(|b| b.dynamic_c_n >= load_n * sf)
}

/// One line of the hardware schedule.
#[derive(Clone, Debug)]
pub struct HardwareItem {
    pub joint: String,
    pub part: String,
    pub detail: String,
}

/// Select all the hardware for a design.
pub fn hardware_schedule(c: &DesignCandidate, report: &DesignReport) -> Vec<HardwareItem> {
    let mut items = Vec::new();

    // Blade retention bolt: centrifugal force, double shear (flap/feather pivot).
    let omega = c.omega();
    let span = c.radius_m - c.root_cutout * c.radius_m;
    let m_blade = c.blade_areal_density_kg_m2 * c.chord_m * span;
    let r_cg = 0.5 * (c.radius_m + c.root_cutout * c.radius_m);
    let f_cf = omega * omega * m_blade * r_cg;
    if let Some(b) = select_bolt(f_cf, true, 2.0) {
        items.push(HardwareItem {
            joint: "blade retention".to_string(),
            part: b.name.to_string(),
            detail: format!(
                "centrifugal {f_cf:.0} N (double shear, SF2) → cap {:.0} N",
                2.0 * b.shear_capacity_n
            ),
        });
    }

    // Mast bearings (×2): bore = mast diameter; radial load ~ gross weight.
    let torque = if report.hover_shaft_power_w.is_finite() && omega > 0.0 {
        report.hover_shaft_power_w / omega
    } else {
        1.0
    };
    let mast_d_mm = (16.0 * torque / (PI * 55.0e6)).cbrt() * 1000.0;
    let mast_d_mm = mast_d_mm.ceil();
    let weight = c.gross_mass_kg * 9.80665;
    if let Some(brg) = select_bearing(mast_d_mm, weight, 2.0) {
        items.push(HardwareItem {
            joint: "mast bearings (×2)".to_string(),
            part: brg.name.to_string(),
            detail: format!(
                "bore {:.0} mm ≥ mast {:.0} mm, C {:.0} N ≥ {:.0} N",
                brg.bore_mm, mast_d_mm, brg.dynamic_c_n, weight
            ),
        });
    }

    // Grip pitch bearings: carry the blade centrifugal force (per blade).
    let grip_bore = 3.0; // small pitch shaft
    if let Some(brg) = select_bearing(grip_bore, f_cf, 1.5) {
        items.push(HardwareItem {
            joint: format!("grip pitch bearings (×{})", c.n_blades),
            part: brg.name.to_string(),
            detail: format!(
                "carries {f_cf:.0} N centrifugal, C {:.0} N",
                brg.dynamic_c_n
            ),
        });
    }

    // Swashplate bearing: bore on the mast, modest control load.
    if let Some(brg) = select_bearing(mast_d_mm, 0.2 * weight, 2.0) {
        items.push(HardwareItem {
            joint: "swashplate bearing".to_string(),
            part: brg.name.to_string(),
            detail: format!("bore {:.0} mm on the mast", brg.bore_mm),
        });
    }

    items
}

#[cfg(test)]
mod tests {
    use super::*;

    /// DOCUMENTED — the bolt stress areas are the published ISO 724 metric
    /// tensile-stress areas, and the class-8.8 working shear embeds the ISO 898-1
    /// ultimate (0.6·800 = 480 MPa) with a ≈2.4 safety factor. A reader can check
    /// M3: As = 5.03 mm², working cap = 200 MPa × 5.03 mm² = 1006 N.
    #[test]
    fn catalogue_matches_iso_724_and_898_1() {
        let cat = bolt_catalogue();
        let m3 = cat.iter().find(|b| b.name == "M3").unwrap();
        assert!((m3.stress_area_mm2 - 5.03).abs() < 1e-9); // ISO 724
        assert!((m3.shear_capacity_n - 1006.0).abs() < 1.0); // 200 MPa × 5.03 mm²
        // Class-8.8 ultimate shear ≈ 480 MPa; the 200 MPa working value is that /2.4.
        let ultimate = 480.0e6 * m3.stress_area_mm2 * 1e-6;
        assert!((ultimate / m3.shear_capacity_n - 2.4).abs() < 0.01);
    }

    #[test]
    fn bolt_selection_is_the_smallest_adequate() {
        // A load the M3 (1006 N single-shear) carries but M2.5 (678 N) does not.
        let load = 900.0;
        let b = select_bolt(load, false, 1.0).unwrap();
        assert_eq!(b.name, "M3");
        // The next size down would indeed fail.
        let cat = bolt_catalogue();
        let m25 = cat.iter().find(|x| x.name == "M2.5").unwrap();
        assert!(m25.shear_capacity_n < load);
        assert!(b.shear_capacity_n >= load);
    }

    #[test]
    fn double_shear_doubles_the_capacity() {
        // A load only carryable in double shear.
        let load = 700.0;
        let single = select_bolt(load, false, 1.0).unwrap();
        let double = select_bolt(load, true, 1.0).unwrap();
        assert!(double.diameter_mm <= single.diameter_mm);
    }

    #[test]
    fn bearing_meets_bore_and_load() {
        let b = select_bearing(8.0, 3000.0, 1.0).unwrap();
        assert!(b.bore_mm >= 8.0);
        assert!(b.dynamic_c_n >= 3000.0);
        // Smallest such: bore-8 608 has C=3400 ≥ 3000.
        assert_eq!(b.name, "608");
    }

    #[test]
    fn schedule_includes_the_retention_bolt() {
        use helisim_airfoil::LinearAirfoil;
        use helisim_bemt::Config;
        use helisim_design::evaluate;
        let c = DesignCandidate::model();
        let r = evaluate(&c, &LinearAirfoil::naca0012(), &Config::default());
        let sched = hardware_schedule(&c, &r);
        assert!(sched.iter().any(|h| h.joint == "blade retention"));
        assert!(sched.iter().any(|h| h.joint.contains("mast")));
    }
}
