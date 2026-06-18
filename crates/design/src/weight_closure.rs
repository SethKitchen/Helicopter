//! Gross-weight (sizing) convergence — the preliminary-design fixed point.
//!
//! A design is only physically *closed* when its gross weight carries its own empty
//! structure, its payload, AND the battery whose size is itself set by the gross
//! weight it has to lift:
//!
//! ```text
//!   W_gross = W_empty(W_gross) + payload + W_battery(W_gross)
//! ```
//!
//! This is the helicopter weight-spiral: a heavier aircraft needs a bigger battery,
//! which makes it heavier still. It closes only when the marginal mass growth is
//! < 1 kg per kg; otherwise it diverges (no buildable design — the result this
//! crate's `upsizing` study hit for a 10-year daily pack on a 3.5 kg airframe).
//!
//! Solver: **bisection on the monotone residual** `r(W)=W−W_empty−payload−W_batt`
//! (solver shape 1) with an adaptive upper bracket; divergence is reported as
//! `None`, not silently clamped. Validated (`tests/weight_closure_validation.rs`)
//! against the AFFINE closed form `W = (payload+fixed)/(1−e−f)` — exact — plus the
//! divergence threshold and the spiral-amplification factor `1/(1−e−f) > 1`.

use helisim_autorotation::G;

/// The battery-mass demand as a function of gross weight — the term that makes the
/// spiral nonlinear. The polymorphism boundary, so the disk-loading *policy* (which
/// fixes whether hover power scales as `W` or `W^1.5`) is swappable.
pub trait BatteryDemand {
    /// Battery mass needed to fly the mission at this gross weight, kg.
    fn battery_mass_kg(&self, gross_kg: f64) -> f64;
}

/// **Fixed disk loading** (the rotor grows with the aircraft): hover induced power
/// is `W·v_h/FM` with `v_h=√(DL/2ρ)` *constant*, so power — and therefore battery
/// mass — is LINEAR in gross weight. Battery is then a constant fraction of gross,
/// giving the affine closed-form fixed point used as the oracle. The design-clean
/// scaling policy.
pub struct FixedDiskLoading {
    /// Disk loading `W/A`, N/m².
    pub disk_loading_n_m2: f64,
    /// Air density, kg/m³.
    pub rho: f64,
    /// Hover figure of merit.
    pub figure_of_merit: f64,
    /// Electrical→shaft efficiency.
    pub powertrain_eta: f64,
    /// Hover/mission duration, h.
    pub flight_time_h: f64,
    /// Pack specific energy, Wh/kg.
    pub specific_energy_wh_kg: f64,
    /// Usable fraction of pack energy (depth of discharge).
    pub usable_fraction: f64,
}

impl FixedDiskLoading {
    /// The constant battery fraction `f = m_batt / m_gross` (independent of weight —
    /// this is what makes the closure affine). Exposed so the oracle can form the
    /// closed-form fixed point directly.
    pub fn battery_fraction(&self) -> f64 {
        let v_h = (self.disk_loading_n_m2 / (2.0 * self.rho)).sqrt();
        // m_batt = (W·v_h/FM/η)·t / (e_spec·usable), W = m·g ⇒ fraction = …/m.
        G * v_h * self.flight_time_h
            / (self.figure_of_merit
                * self.powertrain_eta
                * self.specific_energy_wh_kg
                * self.usable_fraction)
    }
}

impl BatteryDemand for FixedDiskLoading {
    fn battery_mass_kg(&self, gross_kg: f64) -> f64 {
        self.battery_fraction() * gross_kg
    }
}

/// **Fixed rotor** (the airframe is frozen; you are adding payload/battery to a given
/// disk): `v_h=√(W/2ρA)` grows with weight, so hover power ∝ `W^1.5` and the battery
/// demand is genuinely nonlinear — the general spiral with diminishing returns.
pub struct FixedRotor {
    /// Disk area, m² (fixed).
    pub disk_area_m2: f64,
    /// Air density, kg/m³.
    pub rho: f64,
    /// Hover figure of merit.
    pub figure_of_merit: f64,
    /// Electrical→shaft efficiency.
    pub powertrain_eta: f64,
    /// Hover/mission duration, h.
    pub flight_time_h: f64,
    /// Pack specific energy, Wh/kg.
    pub specific_energy_wh_kg: f64,
    /// Usable fraction of pack energy.
    pub usable_fraction: f64,
}

impl BatteryDemand for FixedRotor {
    fn battery_mass_kg(&self, gross_kg: f64) -> f64 {
        let w = gross_kg * G;
        let v_h = (w / (2.0 * self.rho * self.disk_area_m2)).sqrt();
        let p_shaft = w * v_h / self.figure_of_merit; // ∝ W^1.5
        let p_elec = p_shaft / self.powertrain_eta;
        let energy_wh = p_elec * self.flight_time_h;
        energy_wh / (self.specific_energy_wh_kg * self.usable_fraction)
    }
}

/// A gross-weight closure problem. Empty weight is affine in gross —
/// `W_empty = empty_fraction·W + fixed_mass` — where `fixed_mass` is the
/// non-scaling avionics/flight-controller (the cost study's flat term).
pub struct WeightClosure<'a> {
    /// Useful load to carry, kg.
    pub payload_kg: f64,
    /// Empty-structure mass as a fraction of gross (structure + powertrain + rotor).
    pub empty_fraction: f64,
    /// Non-scaling fixed mass (avionics), kg.
    pub fixed_mass_kg: f64,
    /// The battery-mass demand model.
    pub battery: &'a dyn BatteryDemand,
}

/// A converged (closed) design weight breakdown.
#[derive(Clone, Debug)]
pub struct ClosureResult {
    /// Converged gross mass, kg.
    pub gross_kg: f64,
    /// Empty-structure mass at closure, kg.
    pub empty_kg: f64,
    /// Battery mass at closure, kg.
    pub battery_kg: f64,
    /// Payload (echoed), kg.
    pub payload_kg: f64,
    /// Bisection iterations used.
    pub iters: usize,
}

impl WeightClosure<'_> {
    /// Closure residual `r(W) = W − W_empty(W) − payload − W_battery(W)`. A root is a
    /// closed design; `r` increases with `W` whenever the spiral converges.
    pub fn residual(&self, gross_kg: f64) -> f64 {
        gross_kg
            - (self.empty_fraction * gross_kg + self.fixed_mass_kg)
            - self.payload_kg
            - self.battery.battery_mass_kg(gross_kg)
    }

    /// Solve the closure by bisection on the residual with an adaptive upper bracket.
    /// Returns `None` if no closed design exists in `[m_floor, max_gross_kg]` — the
    /// mass spiral diverges (marginal growth ≥ 1 kg/kg), which is a real, reportable
    /// design outcome, not a solver failure.
    pub fn solve(&self, max_gross_kg: f64) -> Option<ClosureResult> {
        // Lower bracket: just the dead mass (fixed + payload). r < 0 here (empty
        // fraction + battery add positive mass with nothing to balance them yet).
        let lo0 = self.fixed_mass_kg + self.payload_kg + 1e-9;
        if self.residual(lo0) > 0.0 {
            // Degenerate: already closed at the floor (no scaling demand).
            let g = lo0;
            return Some(self.breakdown(g, 0));
        }
        // Grow the upper bracket until the residual turns positive (closure exists)
        // or we exceed the cap (divergent spiral).
        let mut hi = (lo0 * 2.0).max(1.0);
        let mut steps = 0;
        while self.residual(hi) < 0.0 {
            hi *= 1.6;
            steps += 1;
            if hi > max_gross_kg || steps > 200 {
                return None; // spiral does not close within the cap
            }
        }

        let mut a = lo0;
        let mut b = hi;
        let mut iters = 0;
        let mut mid = 0.5 * (a + b);
        for _ in 0..200 {
            iters += 1;
            mid = 0.5 * (a + b);
            let r = self.residual(mid);
            if r.abs() < 1e-10 || (b - a) < 1e-10 {
                break;
            }
            if r < 0.0 {
                a = mid;
            } else {
                b = mid;
            }
        }
        Some(self.breakdown(mid, iters))
    }

    fn breakdown(&self, gross_kg: f64, iters: usize) -> ClosureResult {
        ClosureResult {
            gross_kg,
            empty_kg: self.empty_fraction * gross_kg + self.fixed_mass_kg,
            battery_kg: self.battery.battery_mass_kg(gross_kg),
            payload_kg: self.payload_kg,
            iters,
        }
    }
}
