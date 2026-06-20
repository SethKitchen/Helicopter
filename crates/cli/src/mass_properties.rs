//! Mass properties → centre of gravity → trim/stability effect.
//!
//! Every studio component contributes its mass at its location, so the CG is a
//! real consequence of the layout (move the avionics aft and the CG moves aft).
//! The longitudinal CG offset feeds the trim model's `cg_offset` — the SAME
//! parameter validated in Milestone 6 (it shifts the trimmed pitch attitude and,
//! through it, the stability derivatives). So "where the avionics sit" changes
//! the trim attitude here, not by assertion but through the validated balance.
//!
//! Honest modelling: a component's mass is its mesh **volume × a representative
//! material density**, then the whole set is **normalised so the total equals the
//! design's gross mass** (the validated number). So the *total* is exact and the
//! *distribution* is geometry-derived; the densities are representative (flagged),
//! not sourced per-part. The CG is the mass-weighted volume-centroid of the parts.

use crate::studio_scene::{ScenePart, studio_scene};
use helisim_airfoil::LinearAirfoil;
use helisim_design::{DesignCandidate, DesignReport};
use helisim_dynamics::analyze_hover_longitudinal;
use helisim_manufacture::boom_for;
use helisim_manufacture::mesh::{Tri, Vec3};
use helisim_rotor::{Operating, Rotor};
use helisim_trim::{Aircraft, NewtonConfig, TailRotor, TrimCondition, trim};

/// Which mass category a component group falls in. The big movers (battery,
/// motor) get physics-based masses; the rotor group shares its sized estimate;
/// the structure absorbs the remainder of the gross mass.
fn category(group: &str) -> &'static str {
    match group {
        "battery" => "battery",
        "motor" => "motor",
        "esc" => "esc",
        "avionics" => "avionics",
        "blade" | "hub" | "mast" | "swashplate" | "tail_rotor" => "rotor",
        _ => "structure", // fuselage, boom, landing_gear
    }
}

/// Representative specific energy of the pack, Wh/kg (flagged representative).
const PACK_WH_PER_KG: f64 = 200.0;
/// Representative motor specific power, W/kg (flagged representative).
const MOTOR_W_PER_KG: f64 = 3000.0;

/// Signed mesh volume (mm³) and volume-centroid (mm) by the divergence/tetrahedron
/// method: each triangle (a,b,c) forms a tet with the origin.
fn volume_centroid(tris: &[Tri]) -> (f64, Vec3) {
    let mut v6 = 0.0;
    let (mut cx, mut cy, mut cz) = (0.0, 0.0, 0.0);
    for t in tris {
        let (a, b, c) = (t.0, t.1, t.2);
        let cross = (
            b.y * c.z - b.z * c.y,
            b.z * c.x - b.x * c.z,
            b.x * c.y - b.y * c.x,
        );
        let sv = a.x * cross.0 + a.y * cross.1 + a.z * cross.2; // 6× tet volume
        v6 += sv;
        cx += sv * (a.x + b.x + c.x);
        cy += sv * (a.y + b.y + c.y);
        cz += sv * (a.z + b.z + c.z);
    }
    if v6.abs() < 1e-9 {
        return (0.0, Vec3::new(0.0, 0.0, 0.0));
    }
    let vol = v6 / 6.0;
    let c = Vec3::new(cx / (4.0 * v6), cy / (4.0 * v6), cz / (4.0 * v6));
    (vol.abs(), c)
}

/// One component's mass contribution.
pub struct PartMass {
    pub id: String,
    pub name: String,
    pub group: &'static str,
    pub mass_kg: f64,
    pub centroid_mm: [f64; 3],
}

/// One hover eigenmode, flattened for the manifest.
pub struct ModeOut {
    pub re: f64,
    pub im: f64,
    pub stable: bool,
    pub oscillatory: bool,
    pub period_s: f64,
    pub t_half_double_s: f64,
}

/// Hover stability & control summary — the modes (eigenvalues) and key derivatives
/// of the candidate at its computed CG/inertia. The layout feeds this: the pitch
/// inertia `i_yy` and the CG offset both come from the component masses & positions.
pub struct StabilitySummary {
    pub modes: Vec<ModeOut>,
    pub unstable: bool,
    pub has_unstable_oscillation: bool,
    pub mu: f64,
    pub mq: f64,
    pub zw: f64,
    pub xu: f64,
}

/// The whole mass/balance result.
pub struct Balance {
    pub parts: Vec<PartMass>,
    pub total_mass_kg: f64,
    pub cg_mm: [f64; 3],
    /// Mass moments of inertia about the CG, kg·m² (from the component layout).
    pub i_xx: f64,
    pub i_yy: f64,
    pub i_zz: f64,
    /// Longitudinal CG offset aft of the shaft, m (positive aft) — the trim `cg_offset`.
    pub cg_offset_m: f64,
    /// Trimmed hover pitch at the computed CG, deg (nose-up positive).
    pub trim_pitch_deg: f64,
    /// Trimmed hover pitch with the CG centred under the shaft, deg.
    pub trim_pitch_centered_deg: f64,
    /// Sensitivity dΘ/d(cg_offset), deg per metre.
    pub dpitch_dcg_deg_per_m: f64,
    pub converged: bool,
    /// Hover stability modes + derivatives at this CG/inertia.
    pub stability: Option<StabilitySummary>,
}

/// Build a representative 6-DOF trim model for a design candidate. The rotor and
/// mass are the candidate's; the tail/flap/parasite are representative (this model
/// exists to show the CG→attitude balance, not to re-derive the rotor aero).
fn aircraft_from_candidate(c: &DesignCandidate, report: &DesignReport, cg_offset: f64) -> Aircraft {
    let omega = c.omega();
    let torque = if report.hover_shaft_power_w.is_finite() && omega > 0.0 {
        report.hover_shaft_power_w / omega
    } else {
        1.0
    };
    let boom = boom_for(torque, c.radius_m, omega);
    let hub_height = 0.20 * c.radius_m + 0.05;
    let tail = TailRotor {
        rotor: Rotor::rectangular(
            2,
            0.18 * c.radius_m,
            0.03 * c.radius_m,
            6f64.to_radians(),
            0.15,
        ),
        op: Operating::from_rpm(4000.0),
        airfoil: Box::new(LinearAirfoil::naca0012()),
        arm: boom.length_m,
        height: 0.10,
    };
    Aircraft {
        main: c.rotor(),
        main_op: c.operating(),
        main_airfoil: Box::new(LinearAirfoil::naca0012()),
        flap: helisim_flapping::FlapProperties::with_offset(8.0, 0.04),
        tail,
        mass: c.gross_mass_kg,
        rho: 1.225,
        hub_height,
        cg_offset,
        parasite_area: 0.05,
        shaft_tilt: 0.0,
    }
}

fn hover_pitch_deg(c: &DesignCandidate, report: &DesignReport, cg_offset: f64) -> Option<f64> {
    let ac = aircraft_from_candidate(c, report, cg_offset);
    let res = trim(&ac, &TrimCondition::hover(), &NewtonConfig::default());
    res.converged.then(|| res.pitch.to_degrees())
}

/// Compute the mass properties, CG, and the trim-attitude effect for a design.
pub fn balance(c: &DesignCandidate, report: &DesignReport) -> Balance {
    let scene = studio_scene(c, report);
    balance_from_scene(&scene, c, report)
}

fn balance_from_scene(scene: &[ScenePart], c: &DesignCandidate, report: &DesignReport) -> Balance {
    // 1. geometry: each part's volume-centroid + volume (for sharing within a category).
    struct Raw {
        meta: PartMass,
        vol_m3: f64,
        cat: &'static str,
    }
    let mut raws: Vec<Raw> = Vec::new();
    for p in scene {
        let (vol_mm3, c_mm) = volume_centroid(&p.tris);
        raws.push(Raw {
            meta: PartMass {
                id: p.id.clone(),
                name: p.name.clone(),
                group: p.group,
                mass_kg: 0.0,
                centroid_mm: [c_mm.x, c_mm.y, c_mm.z],
            },
            vol_m3: vol_mm3 * 1e-9,
            cat: category(p.group),
        });
    }

    // 2. category masses (physics-based for the big movers; structure = the rest).
    let battery_total = (c.pack_energy_wh / PACK_WH_PER_KG).max(0.0);
    let rotor_total = c.estimate_rotor_group_mass();
    let motor_total = (report.hover_shaft_power_w.max(0.0) / MOTOR_W_PER_KG).max(0.02);
    let esc_total = 0.03;
    let avionics_total = 0.04;
    let accounted = battery_total + rotor_total + motor_total + esc_total + avionics_total;
    let structure_total = (c.gross_mass_kg - accounted).max(0.10);
    let vol_sum = |cat: &str| -> f64 {
        raws.iter()
            .filter(|r| r.cat == cat)
            .map(|r| r.vol_m3)
            .sum::<f64>()
            .max(1e-12)
    };
    let (vsum_rotor, vsum_struct) = (vol_sum("rotor"), vol_sum("structure"));

    // 3. assign per-part mass, then scale so the total equals the gross mass exactly.
    let mut parts: Vec<PartMass> = Vec::with_capacity(raws.len());
    for r in &raws {
        let m = match r.cat {
            "battery" => battery_total,
            "motor" => motor_total,
            "esc" => esc_total,
            "avionics" => avionics_total,
            "rotor" => rotor_total * r.vol_m3 / vsum_rotor,
            _ => structure_total * r.vol_m3 / vsum_struct,
        };
        parts.push(PartMass {
            id: r.meta.id.clone(),
            name: r.meta.name.clone(),
            group: r.meta.group,
            mass_kg: m,
            centroid_mm: r.meta.centroid_mm,
        });
    }
    let pre_total: f64 = parts.iter().map(|p| p.mass_kg).sum();
    let scale = if pre_total > 1e-9 {
        c.gross_mass_kg / pre_total
    } else {
        0.0
    };
    let (mut mx, mut my, mut mz, mut m_tot) = (0.0, 0.0, 0.0, 0.0);
    for p in &mut parts {
        p.mass_kg *= scale;
        mx += p.mass_kg * p.centroid_mm[0];
        my += p.mass_kg * p.centroid_mm[1];
        mz += p.mass_kg * p.centroid_mm[2];
        m_tot += p.mass_kg;
    }
    let cg_mm = if m_tot > 1e-9 {
        [mx / m_tot, my / m_tot, mz / m_tot]
    } else {
        [0.0, 0.0, 0.0]
    };

    // 3. inertia about the mass-weighted CG (point-mass approximation). Component
    // placement, including avionics, changes the stability analysis through Iyy.
    let (mut i_xx, mut i_yy, mut i_zz) = (0.0, 0.0, 0.0);
    for p in &parts {
        let dx = (p.centroid_mm[0] - cg_mm[0]) / 1000.0;
        let dy = (p.centroid_mm[1] - cg_mm[1]) / 1000.0;
        let dz = (p.centroid_mm[2] - cg_mm[2]) / 1000.0;
        i_xx += p.mass_kg * (dy * dy + dz * dz);
        i_yy += p.mass_kg * (dx * dx + dz * dz);
        i_zz += p.mass_kg * (dx * dx + dy * dy);
    }
    i_xx = i_xx.max(1e-6);
    i_yy = i_yy.max(1e-6);
    i_zz = i_zz.max(1e-6);

    // 4. longitudinal CG offset aft of the shaft (shaft at x=0; body +x forward,
    //    cg_offset positive aft) → the validated trim parameter.
    let cg_offset_m = -cg_mm[0] / 1000.0;

    // 5. trim effect: pitch at the computed CG vs CG centred under the shaft.
    let p_cg = hover_pitch_deg(c, report, cg_offset_m);
    let p_0 = hover_pitch_deg(c, report, 0.0);
    let (trim_pitch_deg, trim_pitch_centered_deg, converged) = match (p_cg, p_0) {
        (Some(a), Some(b)) => (a, b, true),
        _ => (f64::NAN, f64::NAN, false),
    };
    let dpitch_dcg = if converged && cg_offset_m.abs() > 1e-4 {
        (trim_pitch_deg - trim_pitch_centered_deg) / cg_offset_m
    } else {
        f64::NAN
    };

    // 6. stability/control at the computed layout. This uses the same CG offset
    // and pitch inertia that the mass layout produced, so moving avionics alters
    // both the force/moment trim balance and the modal result.
    let stability = {
        let ac = aircraft_from_candidate(c, report, cg_offset_m);
        let modal = analyze_hover_longitudinal(&ac, i_yy);
        Some(StabilitySummary {
            modes: modal
                .modes
                .iter()
                .map(|m| ModeOut {
                    re: m.eigenvalue.re,
                    im: m.eigenvalue.im,
                    stable: m.stable,
                    oscillatory: m.oscillatory,
                    period_s: m.period,
                    t_half_double_s: m.time_to_half_or_double,
                })
                .collect(),
            unstable: modal.unstable,
            has_unstable_oscillation: modal.has_unstable_oscillation,
            mu: modal.derivatives.mu,
            mq: modal.derivatives.mq,
            zw: modal.derivatives.zw,
            xu: modal.derivatives.xu,
        })
    };

    Balance {
        parts,
        total_mass_kg: m_tot,
        cg_mm,
        i_xx,
        i_yy,
        i_zz,
        cg_offset_m,
        trim_pitch_deg,
        trim_pitch_centered_deg,
        dpitch_dcg_deg_per_m: dpitch_dcg,
        converged,
        stability,
    }
}
