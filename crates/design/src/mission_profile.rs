//! Mission-profile energy budget — size and optimize for a *mission*, not just hover.
//!
//! Everything upstream optimizes hover endurance. A real "best" design is best for a
//! defined mission: climb out, cruise to a waypoint, loiter, return. This module
//! integrates the energy of a segmented profile through analytic flight-power
//! relations (the same momentum-induced + parasite forms the forward-flight
//! milestone validated), turning a `Mission` into the energy / pack-mass an
//! optimizer or the weight closure can target.
//!
//! Validated (`tests/mission_profile_validation.rs`) by closed forms: the segment
//! energy is exactly `Σ Pᵢ·tᵢ`; the cruise range obeys the Breguet-style identity
//! `range = E / D_equiv` (energy = equivalent-drag × distance); the forward power
//! bucket has an interior minimum the [`helisim_optimize`] solver recovers.

use helisim_autorotation::G;
use helisim_optimize::{FnObjective, NmOptions, minimize};

/// Analytic flight-power model for one fixed aircraft. Hover/climb use momentum +
/// energy-rate; forward flight uses the high-speed induced approximation
/// `P_i = W²/(2ρA V)` plus parasite `½ρV³f` plus a (speed-independent to first order)
/// rotor profile power — the standard power-bucket decomposition.
#[derive(Clone, Copy, Debug)]
pub struct AircraftPower {
    /// Gross mass, kg.
    pub gross_mass_kg: f64,
    /// Air density, kg/m³.
    pub rho: f64,
    /// Rotor disk area, m².
    pub disk_area_m2: f64,
    /// Hover figure of merit.
    pub figure_of_merit: f64,
    /// Equivalent flat-plate parasite area `f`, m².
    pub flat_plate_area_m2: f64,
    /// Rotor profile power (roughly speed-independent), W.
    pub profile_power_w: f64,
    /// Electrical→shaft efficiency.
    pub powertrain_eta: f64,
}

impl AircraftPower {
    /// Weight, N.
    pub fn weight_n(&self) -> f64 {
        self.gross_mass_kg * G
    }

    /// Hover shaft power `W·v_h/FM`, W.
    pub fn hover_shaft_power_w(&self) -> f64 {
        let w = self.weight_n();
        let v_h = (w / (2.0 * self.rho * self.disk_area_m2)).sqrt();
        w * v_h / self.figure_of_merit
    }

    /// Steady-climb shaft power: hover plus the rate of work against gravity.
    pub fn climb_shaft_power_w(&self, rate_mps: f64) -> f64 {
        self.hover_shaft_power_w() + self.weight_n() * rate_mps
    }

    /// Level forward-flight shaft power at airspeed `v` (m/s, > 0), W.
    pub fn forward_shaft_power_w(&self, v: f64) -> f64 {
        let w = self.weight_n();
        let induced = w * w / (2.0 * self.rho * self.disk_area_m2 * v);
        let parasite = 0.5 * self.rho * v * v * v * self.flat_plate_area_m2;
        induced + parasite + self.profile_power_w
    }

    /// Equivalent drag `D = P/V` in forward flight, N — the force that, times
    /// distance, equals the shaft energy (the basis of the range identity).
    pub fn forward_equiv_drag_n(&self, v: f64) -> f64 {
        self.forward_shaft_power_w(v) / v
    }

    /// Minimum-power (best-loiter) airspeed in `[lo, hi]` m/s, found with the simplex
    /// — composes the optimizer with the mission power model.
    pub fn min_power_speed_mps(&self, lo: f64, hi: f64) -> f64 {
        let obj = FnObjective::bounded(1, vec![(lo, hi)], |v: &[f64]| {
            self.forward_shaft_power_w(v[0])
        });
        minimize(&obj, &[0.5 * (lo + hi)], &NmOptions::default()).x[0]
    }

    /// Best-range airspeed in `[lo, hi]` m/s: minimizes equivalent drag `P/V` (max
    /// distance per unit energy). Always faster than the min-power speed.
    pub fn best_range_speed_mps(&self, lo: f64, hi: f64) -> f64 {
        let obj = FnObjective::bounded(1, vec![(lo, hi)], |v: &[f64]| {
            self.forward_equiv_drag_n(v[0])
        });
        minimize(&obj, &[0.5 * (lo + hi)], &NmOptions::default()).x[0]
    }
}

/// One mission leg. Each knows its own power draw and duration given an aircraft.
#[derive(Clone, Copy, Debug)]
pub enum Segment {
    /// Hover in place for `duration_s`.
    Hover { duration_s: f64 },
    /// Climb at `rate_mps` to gain `height_m` (duration = height/rate).
    Climb { rate_mps: f64, height_m: f64 },
    /// Cruise `distance_m` at `speed_mps` (duration = distance/speed).
    Cruise { speed_mps: f64, distance_m: f64 },
    /// Loiter forward at `speed_mps` for `duration_s`.
    Loiter { speed_mps: f64, duration_s: f64 },
}

impl Segment {
    /// Shaft power and duration of this leg under aircraft `p`.
    pub fn power_and_time(&self, p: &AircraftPower) -> (f64, f64) {
        match *self {
            Segment::Hover { duration_s } => (p.hover_shaft_power_w(), duration_s),
            Segment::Climb { rate_mps, height_m } => {
                (p.climb_shaft_power_w(rate_mps), height_m / rate_mps)
            }
            Segment::Cruise {
                speed_mps,
                distance_m,
            } => (p.forward_shaft_power_w(speed_mps), distance_m / speed_mps),
            Segment::Loiter {
                speed_mps,
                duration_s,
            } => (p.forward_shaft_power_w(speed_mps), duration_s),
        }
    }

    /// Shaft energy of this leg, Wh.
    pub fn shaft_energy_wh(&self, p: &AircraftPower) -> f64 {
        let (power, dt) = self.power_and_time(p);
        power * dt / 3600.0
    }
}

/// A full mission as an ordered list of legs.
#[derive(Clone, Debug, Default)]
pub struct Mission {
    /// The legs, flown in order.
    pub segments: Vec<Segment>,
}

impl Mission {
    /// Total shaft energy over the mission, Wh (`Σ Pᵢ·tᵢ`).
    pub fn shaft_energy_wh(&self, p: &AircraftPower) -> f64 {
        self.segments.iter().map(|s| s.shaft_energy_wh(p)).sum()
    }

    /// Total electrical energy the pack must supply, Wh.
    pub fn elec_energy_wh(&self, p: &AircraftPower) -> f64 {
        self.shaft_energy_wh(p) / p.powertrain_eta
    }

    /// Total mission time, s.
    pub fn total_time_s(&self, p: &AircraftPower) -> f64 {
        self.segments.iter().map(|s| s.power_and_time(p).1).sum()
    }

    /// Can a pack with `usable_energy_wh` of usable energy fly the mission?
    pub fn feasible(&self, p: &AircraftPower, usable_energy_wh: f64) -> bool {
        self.elec_energy_wh(p) <= usable_energy_wh
    }

    /// Pack mass needed to fly the mission at a given usable specific energy, kg —
    /// the mission-driven sizing quantity (feeds the weight closure).
    pub fn required_pack_mass_kg(&self, p: &AircraftPower, usable_specific_wh_kg: f64) -> f64 {
        self.elec_energy_wh(p) / usable_specific_wh_kg
    }
}
