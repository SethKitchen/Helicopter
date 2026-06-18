//! Epoxy-bond creep over the service life — polymers under SUSTAINED load creep and
//! eventually creep-rupture, so the doubler bond's static lap-shear allowable must be
//! derated for (a) cumulative loaded time and (b) temperature. The centrifugal load is
//! steady whenever the rotor spins, so the bond sees a constant shear for the
//! cumulative flight hours of its life.
//!
//! Model (each factor cited as a representative engineering derate, not a fit):
//!   • **Time** — `C_t = 1 − 0.10·log₁₀(hours)` (≈ −0.1 per decade of loaded hours, a
//!     common epoxy creep-rupture slope), floored at 0.3.
//!   • **Temperature** — retention vs T toward the epoxy glass transition (interp of a
//!     representative structural-epoxy table).
//! The sustained allowable is `τ_static · C_t · C_T`; the steady bond shear must stay
//! under it. (A full viscoelastic creep-compliance solution is the named next step.)

use crate::materials::TAU_ALLOW_EPOXY;
use crate::thermal_softening::equilibrium_blade_temp_c;
use helisim_design::DesignCandidate;

/// Cumulative-loaded-time creep factor `C_t = 1 − 0.10·log₁₀(hours)`, floored at 0.3.
pub fn creep_time_factor(loaded_hours: f64) -> f64 {
    let h = loaded_hours.max(1.0);
    (1.0 - 0.10 * h.log10()).max(0.3)
}

/// Linear-interpolate a retention table at `t`.
fn interp(table: &[(f64, f64)], t: f64) -> f64 {
    if t <= table[0].0 {
        return table[0].1;
    }
    if t >= table[table.len() - 1].0 {
        return table[table.len() - 1].1;
    }
    for w in table.windows(2) {
        if t <= w[1].0 {
            let (t0, f0) = w[0];
            let (t1, f1) = w[1];
            return f0 + (f1 - f0) * (t - t0) / (t1 - t0);
        }
    }
    table[table.len() - 1].1
}

/// Structural-epoxy strength retention vs temperature (fraction of the 23 °C value),
/// falling toward the cure/glass-transition region.
pub fn epoxy_temp_factor(temp_c: f64) -> f64 {
    interp(
        &[(23.0, 1.0), (60.0, 0.70), (100.0, 0.30), (140.0, 0.10)],
        temp_c,
    )
}

/// The bond-creep report.
#[derive(Clone, Debug)]
pub struct BondCreepReport {
    /// Steady bond shear stress, MPa.
    pub sustained_shear_mpa: f64,
    /// Cumulative loaded (flight) hours over the life.
    pub loaded_hours: f64,
    /// Bond temperature (from the blade solar balance), °C.
    pub temp_c: f64,
    /// Time derate factor.
    pub time_factor: f64,
    /// Temperature derate factor.
    pub temp_factor: f64,
    /// Sustained (creep-rupture) allowable, MPa.
    pub creep_allowable_mpa: f64,
    /// Margin of safety against creep rupture.
    pub margin_of_safety: f64,
    /// Survives the sustained life?
    pub ok: bool,
}

/// Check the root doubler bond against creep rupture over the service life.
pub fn analyze_bond_creep(
    c: &DesignCandidate,
    ambient_c: f64,
    flights_per_year: f64,
    life_years: f64,
    flight_minutes: f64,
) -> BondCreepReport {
    // Steady centrifugal force and the doubler bond area (both plates).
    let omega = c.omega();
    let span = c.radius_m - c.root_cutout * c.radius_m;
    let m_blade = c.blade_areal_density_kg_m2 * c.chord_m * span;
    let r_cg = 0.5 * (c.radius_m + c.root_cutout * c.radius_m);
    let f_cf = omega * omega * m_blade * r_cg;
    let doubler_l = 1.5 * c.chord_m;
    let doubler_w = c.chord_m;
    let bond_area = 2.0 * doubler_l * doubler_w;
    let tau = f_cf / bond_area;

    let loaded_hours = flights_per_year * life_years * flight_minutes / 60.0;
    let temp = equilibrium_blade_temp_c(ambient_c);
    let c_t = creep_time_factor(loaded_hours);
    let c_temp = epoxy_temp_factor(temp);
    let allow = TAU_ALLOW_EPOXY * c_t * c_temp;
    let ms = if tau > 0.0 {
        allow / tau - 1.0
    } else {
        f64::INFINITY
    };
    BondCreepReport {
        sustained_shear_mpa: tau / 1e6,
        loaded_hours,
        temp_c: temp,
        time_factor: c_t,
        temp_factor: c_temp,
        creep_allowable_mpa: allow / 1e6,
        margin_of_safety: ms,
        ok: ms >= 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn time_factor_falls_about_a_tenth_per_decade_and_floors() {
        assert!((creep_time_factor(1.0) - 1.0).abs() < 1e-9); // ≤1 hr: no derate
        assert!((creep_time_factor(10.0) - 0.9).abs() < 1e-9); // one decade → 0.9
        assert!((creep_time_factor(1000.0) - 0.7).abs() < 1e-9); // three decades → 0.7
        assert!(creep_time_factor(1.0e12) >= 0.3 - 1e-9); // floored
    }

    #[test]
    fn temp_factor_is_unity_cool_and_falls_hot() {
        assert!((epoxy_temp_factor(23.0) - 1.0).abs() < 1e-9);
        assert!(epoxy_temp_factor(100.0) < epoxy_temp_factor(60.0));
    }

    #[test]
    fn longer_hotter_service_lowers_the_creep_margin() {
        let c = DesignCandidate::model();
        let mild = analyze_bond_creep(&c, 15.0, 365.0, 1.0, 20.0);
        let harsh = analyze_bond_creep(&c, 45.0, 365.0, 10.0, 20.0);
        assert!(harsh.loaded_hours > mild.loaded_hours);
        assert!(harsh.creep_allowable_mpa < mild.creep_allowable_mpa);
        assert!(harsh.margin_of_safety < mild.margin_of_safety);
    }
}
