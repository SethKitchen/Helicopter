//! Mission- AND life-driven gross-weight closure for a candidate **geometry** — the
//! integrated sizing the recommender solves *while* it optimizes (gaps 3+4 plus the
//! 10-year-life / 1:1-charge constraint, all folded into one fixed point, no fallback).
//!
//! For a geometry the gross mass is the fixed point of
//! `W = empty_fraction·W + fixed + payload + W_battery(W)`, where `W_battery` is set
//! by **two coupled requirements solved together each iteration**:
//!   1. the mission energy (from the trimmed hover FM + analytic forward power at `W`);
//!   2. the **service-life depth-of-discharge** — daily flying for `life_years` forces
//!      a shallow DoD (via [`helisim_bms::size_for_life`] on the cell aging model), so
//!      the pack is oversized `1/DoD` (~3×). That oversize is what makes the design
//!      battery-heavy and pushes a daily-life machine well past a naive one-flight size.
//!
//! The same oversize drops the flight C-rate, so a 1:1 charge (charge power = flight
//! power) becomes a gentle low-C charge — life and fast turnaround reinforce each other.
//!
//! Solved by **damped fixed-point iteration**; geometries whose spiral diverges, or
//! that cannot hover, return `None` and are dropped — so the recommender's wider grid
//! naturally selects the bigger, lower-disk-loading rotor that *does* carry the
//! life-pack. There is no separate "upsize fallback": closing the life-pack IS the
//! search. Validated in `tests/sizing_validation.rs`.

use crate::candidate::DesignCandidate;
use crate::mission_profile::{AircraftPower, Mission};
use helisim_airfoil::Airfoil;
use helisim_autorotation::{G, profile_power};
use helisim_bemt::Config;
use helisim_bms::size_for_life;
use helisim_cell::DegradationModel;
use helisim_mission::trim_hover_collective;

/// The service-life duty cycle whose cycle-fade sets the per-flight depth of
/// discharge (and hence the pack oversize).
#[derive(Clone, Debug)]
pub struct LifeRequirement {
    /// Flights per year (e.g. 365 for daily).
    pub flights_per_year: f64,
    /// Target service life before the pack hits end-of-life, years.
    pub life_years: f64,
    /// Storage/ambient temperature for calendar fade, °C.
    pub storage_temp_c: f64,
    /// The cell cycle+calendar aging model.
    pub degradation: DegradationModel,
}

impl LifeRequirement {
    /// Daily flying (365/yr) for `years` at 25 °C with the default aging model.
    pub fn daily(years: f64) -> Self {
        LifeRequirement {
            flights_per_year: 365.0,
            life_years: years,
            storage_temp_c: 25.0,
            degradation: DegradationModel::default(),
        }
    }
}

/// How to size a candidate: the fixed (non-rotor) masses, the pack's specific
/// energy, the mission whose energy must be flown, and the service-life requirement
/// that sets the pack oversize.
#[derive(Clone, Debug)]
pub struct SizingPolicy {
    /// Useful load to carry, kg.
    pub payload_kg: f64,
    /// Empty-structure mass as a fraction of gross (structure + powertrain + rotor).
    pub empty_fraction: f64,
    /// Non-scaling fixed mass (avionics/flight controller), kg.
    pub fixed_mass_kg: f64,
    /// Pack specific energy, Wh/kg.
    pub specific_energy_wh_kg: f64,
    /// The mission whose electrical energy each flight must supply.
    pub mission: Mission,
    /// The service-life duty cycle (sets the depth-of-discharge / pack oversize).
    pub life: LifeRequirement,
    /// Gross-mass cap, kg — exceeding it means the spiral diverges (no closed design).
    pub max_gross_kg: f64,
}

/// A geometry sized to the mission AND the service life, with the weight spiral closed.
#[derive(Clone, Debug)]
pub struct SizedCandidate {
    /// Converged gross mass, kg.
    pub gross_kg: f64,
    /// Battery (pack) mass at closure, kg.
    pub battery_kg: f64,
    /// Rotor-group mass (blades + head + mast + boom), kg — grows with radius.
    pub rotor_group_kg: f64,
    /// Empty mass at closure (non-rotor structure + fixed + rotor group), kg.
    pub empty_kg: f64,
    /// Mission electrical energy per flight at the converged gross, Wh.
    pub mission_energy_wh: f64,
    /// Life-limited depth of discharge per flight (≤1).
    pub dod: f64,
    /// Pack oversize vs a one-flight (full-DoD) pack, `1/DoD`.
    pub oversize: f64,
    /// Nameplate pack capacity, Wh (`mission_energy / DoD`).
    pub pack_capacity_wh: f64,
    /// Flight (discharge) C-rate of the sized pack.
    pub flight_c_rate: f64,
    /// Predicted capacity fade over the service life, fraction.
    pub fade_over_life: f64,
    /// Fixed-point iterations used.
    pub iters: usize,
}

impl SizingPolicy {
    /// Trim the rotor to hover at `gross_kg` and return its figure of merit. `None` if
    /// it cannot hover (the feasibility gate). This is the expensive (BEMT) step, so
    /// the closure calls it only a few times, not every fixed-point iteration.
    fn trim_fm(
        &self,
        geom: &DesignCandidate,
        gross_kg: f64,
        af: &dyn Airfoil,
        cfg: &Config,
    ) -> Option<f64> {
        let op = geom.operating();
        let (_, sol) = trim_hover_collective(&geom.rotor(), &op, af, gross_kg * G, cfg)?;
        Some(sol.figure_of_merit)
    }

    /// Cheap analytic flight-power model at `gross_kg` for a FIXED figure of merit
    /// (no BEMT trim) — the inner-loop power used while the weight spiral converges.
    fn power_with_fm(&self, geom: &DesignCandidate, gross_kg: f64, fm: f64) -> AircraftPower {
        let op = geom.operating();
        let prof = profile_power(
            op.rho,
            geom.disk_area(),
            geom.tip_speed_ms,
            geom.solidity(),
            geom.blade_cd0,
        );
        AircraftPower {
            gross_mass_kg: gross_kg,
            rho: op.rho,
            disk_area_m2: geom.disk_area(),
            figure_of_merit: fm,
            flat_plate_area_m2: geom.flat_plate_area_m2,
            profile_power_w: prof,
            powertrain_eta: geom.powertrain_eta,
        }
    }

    /// Life-sized battery for a given per-flight energy + duration, returning
    /// `(battery_mass_kg, dod, oversize, capacity_wh, flight_c_rate, fade)`.
    fn life_battery(
        &self,
        flight_energy_wh: f64,
        flight_time_h: f64,
    ) -> (f64, f64, f64, f64, f64, f64) {
        let ls = size_for_life(
            &self.life.degradation,
            flight_energy_wh,
            flight_time_h.max(1e-3),
            self.life.flights_per_year,
            self.life.life_years,
            self.life.storage_temp_c,
            self.specific_energy_wh_kg,
        );
        (
            ls.pack_mass_kg,
            ls.dod,
            ls.oversize,
            ls.capacity_wh,
            ls.flight_c_rate,
            ls.fade_over_life,
        )
    }

    /// Close the gross weight for a candidate geometry by damped fixed-point
    /// iteration, with the life-pack oversize folded in. `None` if it cannot hover or
    /// the spiral diverges past `max_gross_kg`.
    pub fn close(
        &self,
        geom: &DesignCandidate,
        af: &dyn Airfoil,
        cfg: &Config,
    ) -> Option<SizedCandidate> {
        if self.empty_fraction >= 1.0 {
            return None;
        }
        // Rotor-group mass is geometry-fixed (independent of gross), so compute once.
        let rotor_group = geom.estimate_rotor_group_mass();
        let mut gross =
            (self.payload_kg + self.fixed_mass_kg + rotor_group + 0.5).max(geom.gross_mass_kg);

        // Trim the figure of merit only a few times (it varies slowly with gross):
        // an OUTER loop re-trims FM, an INNER cheap fixed-point converges the spiral at
        // that FM. The FINAL outer trim also gates hover feasibility at the closed mass.
        let mut fm = self.trim_fm(geom, gross, af, cfg)?;
        let mut total_iters = 0;
        let mut last = (0.0_f64, 1.0_f64, 1.0_f64, 0.0_f64, 0.0_f64, 0.0_f64); // batt,dod,over,cap,c,fade
        let mut energy_wh = 0.0;
        for _outer in 0..8 {
            for _ in 0..400 {
                total_iters += 1;
                let power = self.power_with_fm(geom, gross, fm);
                let energy = self.mission.elec_energy_wh(&power);
                let flight_time_h = self.mission.total_time_s(&power) / 3600.0;
                let life = self.life_battery(energy, flight_time_h);
                let target = self.empty_fraction * gross
                    + self.fixed_mass_kg
                    + rotor_group
                    + self.payload_kg
                    + life.0;
                last = life;
                energy_wh = energy;
                if target > self.max_gross_kg {
                    return None; // divergent spiral — can't carry the life-pack
                }
                let converged = (target - gross).abs() < 1e-4;
                gross = 0.5 * (gross + target);
                if converged {
                    break;
                }
            }
            // Re-trim FM at the converged mass; stop when it stops moving (the final
            // trim is also the feasibility gate at the closed mass).
            let new_fm = self.trim_fm(geom, gross, af, cfg)?;
            let settled = (new_fm - fm).abs() < 1e-3;
            fm = new_fm;
            if settled {
                break;
            }
        }
        let (battery, dod, oversize, cap, c_rate, fade) = last;
        Some(SizedCandidate {
            gross_kg: gross,
            battery_kg: battery,
            rotor_group_kg: rotor_group,
            empty_kg: self.empty_fraction * gross + self.fixed_mass_kg + rotor_group,
            mission_energy_wh: energy_wh,
            dod,
            oversize,
            pack_capacity_wh: cap,
            flight_c_rate: c_rate,
            fade_over_life: fade,
            iters: total_iters,
        })
    }

    /// Apply a closure to a geometry, returning the candidate with its gross mass,
    /// pack energy and per-flight usable fraction (= life DoD) set, ready for
    /// [`crate::evaluate`].
    pub fn sized_candidate(
        &self,
        geom: &DesignCandidate,
        af: &dyn Airfoil,
        cfg: &Config,
    ) -> Option<(DesignCandidate, SizedCandidate)> {
        let s = self.close(geom, af, cfg)?;
        let cand = DesignCandidate {
            gross_mass_kg: s.gross_kg,
            pack_energy_wh: s.pack_capacity_wh,
            usable_fraction: s.dod, // one sortie uses only the life-DoD of the oversized pack
            ..*geom
        };
        Some((cand, s))
    }
}
