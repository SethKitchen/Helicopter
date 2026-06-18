//! Thermal softening of the printed blade — a polymer (SLS PA-CF / Markforged Onyx)
//! loses stiffness AND strength as it warms toward its glass transition, so a dark
//! blade baking in the sun can be far weaker than its room-temperature spec.
//!
//! Two pieces, each with a cited basis:
//!   • **Equilibrium temperature**: a lumped solar energy balance — the blade absorbs
//!     `α·G` (G ≈ 1000 W/m² peak sun) and sheds it by convection `h·ΔT`, so
//!     `T_blade ≈ T_ambient + α·G/h`. (Conservative — neglects re-radiation and
//!     spanwise conduction, both of which lower T.)
//!   • **Retention vs temperature**: nylon modulus/strength fall from ~100 % at 23 °C
//!     toward the glass transition; representative PA-12/Onyx knockdowns are tabulated
//!     and linearly interpolated (datasheet trend, NOT a single fitted curve).
//!
//! The blade's centrifugal-tension margin is then re-checked at the hot strength.

use helisim_design::DesignCandidate;

/// Peak solar irradiance, W/m².
const SOLAR_FLUX: f64 = 1000.0;
/// Solar absorptivity of a dark printed surface.
const ABSORPTIVITY: f64 = 0.9;
/// Convective coefficient on a blade in still-to-moderate air, W/m²·K.
const H_CONV: f64 = 30.0;
/// Room-temperature blade tensile working allowable (printed PA-CF / Onyx), Pa.
const STRENGTH_23C_PA: f64 = 50.0e6;

/// Equilibrium blade temperature in sun, °C: `T_amb + α·G/h`.
pub fn equilibrium_blade_temp_c(ambient_c: f64) -> f64 {
    ambient_c + ABSORPTIVITY * SOLAR_FLUX / H_CONV
}

/// Linear-interpolate a retention table `[(temp_c, factor)]` at `t`.
fn interp(table: &[(f64, f64)], t: f64) -> f64 {
    if t <= table[0].0 {
        return table[0].1;
    }
    if t >= table[table.len() - 1].0 {
        return table[table.len() - 1].1;
    }
    for w in table.windows(2) {
        let (t0, f0) = w[0];
        let (t1, f1) = w[1];
        if t <= t1 {
            return f0 + (f1 - f0) * (t - t0) / (t1 - t0);
        }
    }
    table[table.len() - 1].1
}

/// Modulus retention of printed nylon vs temperature (fraction of the 23 °C value).
pub fn modulus_retention(temp_c: f64) -> f64 {
    interp(
        &[
            (23.0, 1.0),
            (50.0, 0.70),
            (80.0, 0.45),
            (120.0, 0.20),
            (160.0, 0.08),
        ],
        temp_c,
    )
}

/// Tensile-strength retention of printed nylon vs temperature (strength falls faster
/// than modulus near the glass transition).
pub fn strength_retention(temp_c: f64) -> f64 {
    interp(
        &[
            (23.0, 1.0),
            (50.0, 0.60),
            (80.0, 0.35),
            (120.0, 0.15),
            (160.0, 0.05),
        ],
        temp_c,
    )
}

/// The thermal-softening report.
#[derive(Clone, Debug)]
pub struct ThermalSofteningReport {
    /// Ambient air temperature assumed, °C.
    pub ambient_c: f64,
    /// Equilibrium blade temperature in sun, °C.
    pub blade_temp_c: f64,
    /// Modulus retention at the hot temperature.
    pub modulus_retention: f64,
    /// Strength retention at the hot temperature.
    pub strength_retention: f64,
    /// Blade centrifugal root stress, MPa.
    pub root_stress_mpa: f64,
    /// Hot tensile allowable, MPa.
    pub hot_allowable_mpa: f64,
    /// Margin of safety at the hot strength.
    pub margin_of_safety: f64,
    /// Passes hot?
    pub ok: bool,
}

/// Re-check the blade's centrifugal-tension margin at its hot (sun-baked) strength.
pub fn analyze_thermal_softening(c: &DesignCandidate, ambient_c: f64) -> ThermalSofteningReport {
    let blade_temp = equilibrium_blade_temp_c(ambient_c);
    let e_ret = modulus_retention(blade_temp);
    let s_ret = strength_retention(blade_temp);

    // Blade root centrifugal tension (same as the structural check).
    let omega = c.omega();
    let span = c.radius_m - c.root_cutout * c.radius_m;
    let m_blade = c.blade_areal_density_kg_m2 * c.chord_m * span;
    let r_cg = 0.5 * (c.radius_m + c.root_cutout * c.radius_m);
    let f_cf = omega * omega * m_blade * r_cg;
    let a_root = crate::naca_section::structural_area(c.chord_m);
    let sigma_root = f_cf / a_root;

    let hot_allow = STRENGTH_23C_PA * s_ret;
    let ms = if sigma_root > 0.0 {
        hot_allow / sigma_root - 1.0
    } else {
        f64::INFINITY
    };
    ThermalSofteningReport {
        ambient_c,
        blade_temp_c: blade_temp,
        modulus_retention: e_ret,
        strength_retention: s_ret,
        root_stress_mpa: sigma_root / 1e6,
        hot_allowable_mpa: hot_allow / 1e6,
        margin_of_safety: ms,
        ok: ms >= 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn equilibrium_temp_is_above_ambient_by_the_solar_balance() {
        // ΔT = α·G/h = 0.9·1000/30 = 30 °C.
        let t = equilibrium_blade_temp_c(35.0);
        assert!((t - 65.0).abs() < 1e-6);
        assert!(equilibrium_blade_temp_c(35.0) > equilibrium_blade_temp_c(20.0));
    }

    #[test]
    fn retention_is_unity_at_room_temp_and_falls_with_heat() {
        assert!((modulus_retention(23.0) - 1.0).abs() < 1e-9);
        assert!((strength_retention(23.0) - 1.0).abs() < 1e-9);
        assert!(modulus_retention(80.0) < modulus_retention(50.0));
        assert!(strength_retention(80.0) < modulus_retention(80.0)); // strength falls faster
        // Interpolation midpoint check (50→80: modulus 0.70→0.45 ⇒ 65 °C ≈ 0.575).
        assert!((modulus_retention(65.0) - 0.575).abs() < 1e-6);
    }

    #[test]
    fn hot_blade_margin_is_lower_than_room_temp() {
        let c = DesignCandidate::model();
        let cool = analyze_thermal_softening(&c, 15.0);
        let hot = analyze_thermal_softening(&c, 45.0);
        assert!(hot.blade_temp_c > cool.blade_temp_c);
        assert!(hot.strength_retention <= cool.strength_retention);
        assert!(hot.margin_of_safety <= cool.margin_of_safety);
    }
}
