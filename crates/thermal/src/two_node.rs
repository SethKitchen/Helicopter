//! Two-node (core + surface) lumped thermal model — the refinement that lets the
//! engine reason about a cell's **internal** temperature, not just its skin.
//!
//! ## Why this exists (tabless cells)
//! Heat is generated in the jelly roll (`I²R`) and must conduct *radially* out to
//! the can before convection can remove it. A single-node model
//! ([`crate::LumpedThermalCell`]) assumes the whole cell is one temperature, so it
//! is blind to the core running hotter than the surface — exactly the quantity
//! that decides how hard a cell can be pushed before the datasheet's surface-temp
//! cutoff (or a core safety limit) is reached.
//!
//! The core-to-surface conduction resistance `R_int` is set by the jelly roll's
//! **radial** thermal conductivity (`k_r ≈ 0.2–0.5 W/(m·K)`, highly anisotropic)
//! and the cell geometry — it is *largely independent of tab design* (tabs change
//! axial current paths, not the radial electrode stack). So the tabless advantage
//! is **NOT** a lower `R_int`; it is **lower heat generation** (lower `R` → less
//! `I²R`, plus no localized tab hotspots). This model represents the first half of
//! that (less generated heat → cooler core and surface, via the cell's lower `R`);
//! the localized-hotspot half is a 3-D/axial effect this radial model still cannot
//! resolve — named, not faked.
//!
//! Source for the geometry: for uniform volumetric generation in a cylinder of
//! radius `a`, length `L`, the centre-to-surface rise is `ΔT = Q/(4π k_r L)`
//! (Incropera & DeWitt, conduction with generation), so the effective core→surface
//! resistance is `R_int = 1/(4π k_r L)` — derived, not fitted.

use crate::cooling::Cooling;

/// Representative radial (cross-plane) thermal conductivity of a wound Li-ion
/// jelly roll, W/(m·K). Highly anisotropic vs the ~20–30 in-plane; the radial
/// value governs core→surface conduction. Central value of the literature
/// 0.2–0.5 band (Incropera-style conduction analyses of cylindrical cells).
pub const JELLY_ROLL_RADIAL_K: f64 = 0.3;

/// A cell as two isothermal lumps: a heat-generating **core** and a convecting
/// **surface**, linked by a radial conduction resistance `R_int`.
#[derive(Clone, Copy, Debug)]
pub struct TwoNodeThermalCell {
    /// Core heat capacity, J/K (most of the mass — the jelly roll).
    pub c_core_j_per_k: f64,
    /// Surface/can heat capacity, J/K (small).
    pub c_surface_j_per_k: f64,
    /// Core→surface conduction resistance, K/W.
    pub r_internal_k_per_w: f64,
    /// External convecting surface area, m².
    pub surface_area_m2: f64,
    /// Ambient temperature, °C.
    pub ambient_c: f64,
}

/// Effective core→surface thermal resistance of a cylindrical cell of `length_m`
/// with jelly-roll radial conductivity `k_radial`, from the uniform-generation
/// cylinder solution `R_int = 1/(4π k L)`. Tab design does not enter — this is the
/// radial conduction stack.
pub fn effective_r_internal(k_radial: f64, length_m: f64) -> f64 {
    1.0 / (4.0 * std::f64::consts::PI * k_radial * length_m)
}

impl TwoNodeThermalCell {
    /// Build from a lumped heat capacity split into core/surface by `core_fraction`
    /// (the jelly roll is most of the thermal mass; ~0.9 is typical), with an
    /// explicit `R_int`.
    pub fn new(
        heat_capacity_j_per_k: f64,
        core_fraction: f64,
        r_internal_k_per_w: f64,
        surface_area_m2: f64,
        ambient_c: f64,
    ) -> Self {
        TwoNodeThermalCell {
            c_core_j_per_k: heat_capacity_j_per_k * core_fraction,
            c_surface_j_per_k: heat_capacity_j_per_k * (1.0 - core_fraction),
            r_internal_k_per_w,
            surface_area_m2,
            ambient_c,
        }
    }

    /// Build with `R_int` derived from cell geometry (`length_m`) and the
    /// representative jelly-roll radial conductivity [`JELLY_ROLL_RADIAL_K`].
    pub fn from_geometry(
        heat_capacity_j_per_k: f64,
        core_fraction: f64,
        length_m: f64,
        surface_area_m2: f64,
        ambient_c: f64,
    ) -> Self {
        let r_int = effective_r_internal(JELLY_ROLL_RADIAL_K, length_m);
        Self::new(
            heat_capacity_j_per_k,
            core_fraction,
            r_int,
            surface_area_m2,
            ambient_c,
        )
    }

    /// Advance both nodes one explicit-Euler step. Heat is generated in the core;
    /// `q_internal = (T_core − T_surface)/R_int` conducts outward; the surface
    /// convects to ambient. Returns `(T_core, T_surface)` in °C.
    pub fn step(
        &self,
        t_core_c: f64,
        t_surface_c: f64,
        heat_gen_w: f64,
        cooling: &dyn Cooling,
        dt_s: f64,
    ) -> (f64, f64) {
        let q_internal = (t_core_c - t_surface_c) / self.r_internal_k_per_w;
        let q_cool = cooling.heat_removed(t_surface_c, self.ambient_c, self.surface_area_m2);
        let d_core = (heat_gen_w - q_internal) / self.c_core_j_per_k;
        let d_surf = (q_internal - q_cool) / self.c_surface_j_per_k;
        (t_core_c + d_core * dt_s, t_surface_c + d_surf * dt_s)
    }

    /// Steady-state `(T_core, T_surface)` for a constant core heat input. At steady
    /// state all generated heat leaves the surface, so the surface temperature
    /// equals the single-node result, and the core sits exactly `Q·R_int` above it.
    pub fn steady_state(&self, heat_gen_w: f64, cooling: &dyn Cooling) -> (f64, f64) {
        let per_degree =
            cooling.heat_removed(self.ambient_c + 1.0, self.ambient_c, self.surface_area_m2);
        let t_surface = if per_degree <= 0.0 {
            f64::INFINITY
        } else {
            self.ambient_c + heat_gen_w / per_degree
        };
        let t_core = t_surface + heat_gen_w * self.r_internal_k_per_w;
        (t_core, t_surface)
    }

    /// Core-above-surface temperature rise at steady state, `Q·R_int` (K). The
    /// quantity the single-node model cannot see.
    pub fn core_gradient(&self, heat_gen_w: f64) -> f64 {
        heat_gen_w * self.r_internal_k_per_w
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cooling::Convective;

    /// The derived R_int lands in the literature band (2–6 K/W) for a 21700 and
    /// matches the closed-form `1/(4πkL)`.
    #[test]
    fn derived_r_internal_is_physical() {
        let r = effective_r_internal(JELLY_ROLL_RADIAL_K, 0.070); // 21700 length
        assert!((2.0..=6.0).contains(&r), "r_int {r}");
        let expect = 1.0 / (4.0 * std::f64::consts::PI * 0.3 * 0.070);
        assert!((r - expect).abs() < 1e-12);
    }

    /// Steady-state gradient is exactly `Q·R_int` — the model's defining closed form.
    #[test]
    fn steady_state_gradient_is_q_rint() {
        let cell = TwoNodeThermalCell::from_geometry(70.0, 0.9, 0.070, 0.0049, 25.0);
        let cooling = Convective::forced_air();
        let q = 6.0;
        let (t_core, t_surf) = cell.steady_state(q, &cooling);
        assert!((t_core - t_surf - q * cell.r_internal_k_per_w).abs() < 1e-9);
        assert!((cell.core_gradient(q) - q * cell.r_internal_k_per_w).abs() < 1e-12);
        // Surface matches the single-node steady state (all heat leaves the skin).
        let one_node = crate::LumpedThermalCell::new(70.0, 0.0049, 25.0);
        assert!((t_surf - one_node.steady_state_temp(q, &cooling)).abs() < 1e-9);
    }

    /// `R_int → 0` collapses to the single-node model: the core equals the surface.
    #[test]
    fn collapses_to_single_node_when_rint_zero() {
        let cell = TwoNodeThermalCell::new(70.0, 0.9, 1e-9, 0.0049, 25.0);
        let cooling = Convective::natural_air();
        let (t_core, t_surf) = cell.steady_state(5.0, &cooling);
        assert!(
            (t_core - t_surf).abs() < 1e-6,
            "core {t_core} surf {t_surf}"
        );
    }

    /// Time-marching both nodes relaxes to the closed-form steady state.
    #[test]
    fn marches_to_steady_state() {
        let cell = TwoNodeThermalCell::from_geometry(70.0, 0.9, 0.070, 0.0049, 25.0);
        let cooling = Convective::forced_air();
        let q = 6.0;
        let (ss_core, ss_surf) = cell.steady_state(q, &cooling);
        let (mut tc, mut ts) = (25.0, 25.0);
        for _ in 0..2_000_000 {
            let (a, b) = cell.step(tc, ts, q, &cooling, 0.01);
            tc = a;
            ts = b;
        }
        assert!((tc - ss_core).abs() < 0.5, "core {tc} vs {ss_core}");
        assert!((ts - ss_surf).abs() < 0.5, "surf {ts} vs {ss_surf}");
        // The core really is hotter than the skin.
        assert!(ss_core > ss_surf + 5.0);
    }
}
