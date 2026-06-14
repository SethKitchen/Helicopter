//! The lumped-mass cell heat balance and its time step.

use crate::cooling::Cooling;

/// A single cell treated as one isothermal lump with heat capacity
/// `C = m·c_p` and a cooling surface of area `surface_area_m2`, sitting in an
/// ambient at `ambient_c`.
#[derive(Clone, Copy, Debug)]
pub struct LumpedThermalCell {
    /// Lumped heat capacity `m·c_p`, J/K.
    pub heat_capacity_j_per_k: f64,
    /// Cooling surface area, m².
    pub surface_area_m2: f64,
    /// Ambient temperature, °C.
    pub ambient_c: f64,
}

impl LumpedThermalCell {
    /// Build from cell heat capacity, surface area and ambient temperature.
    pub fn new(heat_capacity_j_per_k: f64, surface_area_m2: f64, ambient_c: f64) -> Self {
        LumpedThermalCell {
            heat_capacity_j_per_k,
            surface_area_m2,
            ambient_c,
        }
    }

    /// Advance the temperature one explicit-Euler step.
    /// `C dT/dt = Q_gen − Q_cool`. Returns the new temperature (°C).
    pub fn step(&self, temp_c: f64, heat_gen_w: f64, cooling: &dyn Cooling, dt_s: f64) -> f64 {
        let q_cool = cooling.heat_removed(temp_c, self.ambient_c, self.surface_area_m2);
        temp_c + (heat_gen_w - q_cool) / self.heat_capacity_j_per_k * dt_s
    }

    /// Steady-state temperature for a constant heat input `heat_gen_w`, i.e.
    /// where generation balances convection. For [`crate::Convective`] this is
    /// `T_ambient + Q/(h·A)`.
    pub fn steady_state_temp(&self, heat_gen_w: f64, cooling: &dyn Cooling) -> f64 {
        // Invert Q = q_cool(T): probe the cooling model at a unit overtemp to
        // recover h·A (linear in ΔT), then solve. Robust for any linear model.
        let per_degree =
            cooling.heat_removed(self.ambient_c + 1.0, self.ambient_c, self.surface_area_m2);
        if per_degree <= 0.0 {
            return f64::INFINITY;
        }
        self.ambient_c + heat_gen_w / per_degree
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cooling::Convective;

    #[test]
    fn heats_up_when_generation_exceeds_cooling() {
        let lump = LumpedThermalCell::new(40.0, 0.004, 25.0);
        let cooling = Convective::new(10.0);
        let t1 = lump.step(25.0, 8.0, &cooling, 1.0);
        assert!(t1 > 25.0);
    }

    #[test]
    fn relaxes_to_steady_state() {
        let lump = LumpedThermalCell::new(40.0, 0.004, 25.0);
        let cooling = Convective::new(10.0);
        let q = 5.0;
        let ss = lump.steady_state_temp(q, &cooling); // 25 + 5/(10*0.004)=25+125
        // Integrate long enough to approach steady state.
        let mut t = 25.0;
        for _ in 0..200_000 {
            t = lump.step(t, q, &cooling, 0.1);
        }
        assert!((t - ss).abs() < 0.5, "t={t} ss={ss}");
        assert!((ss - 150.0).abs() < 1e-9);
    }
}
