//! Upsizing for a service-life duty cycle — find the aircraft scale that can
//! actually carry a long-life (deeply-oversized) battery for daily flying.
//!
//! ## Why a small airframe can't (the closure condition)
//! In hover, momentum theory gives the electrical power per unit weight
//! `p = κ_profile · g · √(DL / 2ρ) / (FM · η)`, where `DL` is disk loading. The
//! energy a sortie needs per kg is `p · t_flight`, and a pack stores
//! `dod · e_spec` Wh/kg of *usable, life-limited* energy (the `dod` comes from the
//! cycle-life sizing — daily deep cycling forces a shallow `dod`, i.e. a big pack).
//! So the pack must be this fraction of gross weight:
//!
//! ```text
//! required_pack_fraction = p · t_flight / (dod · e_spec)
//! ```
//!
//! The aircraft closes only if `empty_fraction + required_pack_fraction < 1` (room
//! left for the payload). A 3.5 kg model fails not because it is small per se, but
//! because (a) it was built battery-light and (b) bolting on the big life-pack at a
//! fixed rotor spikes `DL`, raising `p` — the mass spiral. The cure: design for a
//! high pack fraction AND keep `DL` low (grow the rotor with the aircraft).
//!
//! All inputs are explicit, representative assumptions (FM, empty fraction, cell
//! specific energy) — a first-cut momentum sizing, not a structural weights model.

use std::f64::consts::PI;

const G: f64 = 9.80665;
const RHO: f64 = 1.225;

/// A sized, life-capable aircraft.
#[derive(Clone, Copy, Debug)]
pub struct UpsizeResult {
    pub gross_kg: f64,
    pub rotor_radius_m: f64,
    pub disk_loading_n_m2: f64,
    pub hover_power_elec_w: f64,
    pub pack_energy_wh: f64,
    pub pack_mass_kg: f64,
    /// Pack mass as a fraction of gross weight.
    pub pack_fraction: f64,
    pub flight_time_h: f64,
}

/// Electrical hover power per unit weight, W/kg, from momentum theory at disk
/// loading `dl` (N/m²). `profile_factor` (~1.15) accounts for profile power above
/// the induced ideal.
pub fn hover_power_per_kg(dl: f64, fm: f64, eta: f64, profile_factor: f64) -> f64 {
    profile_factor * G * (dl / (2.0 * RHO)).sqrt() / (fm * eta)
}

/// The (representative, overridable) physics + mission assumptions a sizing pass
/// holds fixed while disk loading / payload vary.
#[derive(Clone, Copy, Debug)]
pub struct UpsizeParams {
    /// Hover figure of merit.
    pub fm: f64,
    /// Driveline (motor + ESC) efficiency.
    pub eta: f64,
    /// Profile-power factor over the induced ideal (~1.15).
    pub profile_factor: f64,
    /// Sortie duration, hours.
    pub flight_time_h: f64,
    /// Depth of discharge per flight (from the cell-life sizing).
    pub dod: f64,
    /// Pack specific energy, Wh/kg.
    pub specific_energy_wh_per_kg: f64,
}

/// The pack mass fraction a daily-life duty cycle demands at disk loading `dl`.
pub fn required_pack_fraction(dl: f64, p: &UpsizeParams) -> f64 {
    let power = hover_power_per_kg(dl, p.fm, p.eta, p.profile_factor);
    power * p.flight_time_h / (p.dod * p.specific_energy_wh_per_kg)
}

/// Size the smallest aircraft (at disk loading `dl`) that carries `payload_kg`
/// plus a daily-life pack, with `empty_fraction` of gross in structure/motor/
/// avionics. Returns `None` if the duty cycle can't close at this `dl` (the pack +
/// empty leave no room for payload — lower the disk loading).
pub fn size_for_daily_life(
    payload_kg: f64,
    empty_fraction: f64,
    dl: f64,
    p: &UpsizeParams,
) -> Option<UpsizeResult> {
    let req = required_pack_fraction(dl, p);
    let room = 1.0 - empty_fraction - req;
    if room <= 1e-3 {
        return None; // pack + structure use the whole aircraft — no payload fits
    }
    let gross = payload_kg / room;
    let radius = (gross * G / (PI * dl)).sqrt();
    let power = hover_power_per_kg(dl, p.fm, p.eta, p.profile_factor);
    let pack_mass = req * gross;
    Some(UpsizeResult {
        gross_kg: gross,
        rotor_radius_m: radius,
        disk_loading_n_m2: dl,
        hover_power_elec_w: power * gross,
        pack_energy_wh: pack_mass * p.specific_energy_wh_per_kg,
        pack_mass_kg: pack_mass,
        pack_fraction: req,
        flight_time_h: p.flight_time_h,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> UpsizeParams {
        UpsizeParams {
            fm: 0.72,
            eta: 0.85,
            profile_factor: 1.15,
            flight_time_h: 1.0 / 3.0,
            dod: 0.30,
            specific_energy_wh_per_kg: 180.0,
        }
    }

    /// Required pack fraction rises with disk loading (a draggier hover needs more
    /// battery per kg) — the lever that makes a big rotor essential.
    #[test]
    fn required_fraction_grows_with_disk_loading() {
        let p = params();
        let f = |dl| required_pack_fraction(dl, &p);
        assert!(f(60.0) > f(30.0));
        assert!(f(30.0) > f(15.0));
    }

    /// Low disk loading closes (room for payload); very high disk loading does not.
    #[test]
    fn closes_at_low_dl_fails_at_high_dl() {
        let p = params();
        assert!(size_for_daily_life(1.0, 0.40, 30.0, &p).is_some());
        assert!(
            size_for_daily_life(1.0, 0.40, 200.0, &p).is_none(),
            "200 N/m² should not close"
        );
    }

    /// The sized aircraft is self-consistent: pack + empty + payload = gross, and
    /// the rotor gives the stated disk loading.
    #[test]
    fn sizing_is_self_consistent() {
        let r = size_for_daily_life(1.0, 0.40, 30.0, &params()).unwrap();
        let dl = r.gross_kg * G / (PI * r.rotor_radius_m * r.rotor_radius_m);
        assert!((dl - 30.0).abs() < 1e-6);
        let sum = 0.40 * r.gross_kg + r.pack_mass_kg + 1.0; // empty + pack + payload
        assert!(
            (sum - r.gross_kg).abs() < 1e-6,
            "mass doesn't close: {sum} vs {}",
            r.gross_kg
        );
    }

    /// A bigger payload needs a bigger aircraft (at the same disk loading).
    #[test]
    fn heavier_payload_bigger_aircraft() {
        let p = params();
        let small = size_for_daily_life(1.0, 0.40, 30.0, &p).unwrap();
        let big = size_for_daily_life(3.0, 0.40, 30.0, &p).unwrap();
        assert!(big.gross_kg > small.gross_kg);
        assert!(big.rotor_radius_m > small.rotor_radius_m);
    }
}
