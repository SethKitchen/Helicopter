//! Blade spanwise shape optimization — the inverse rotor-design problem.
//!
//! The BEMT answers "given a blade, what power?"; this answers "what blade twist and
//! taper give the LEAST hover power at a target thrust?" — using the
//! [`helisim_optimize`] simplex over the BEMT evaluator. It optimizes the **real,
//! buildable** knobs (a linear twist rate and a linear taper ratio), trimming
//! collective to hold thrust at each trial.
//!
//! The oracle is the classic minimum-induced-loss result: induced power is minimized
//! when the inflow is **uniform across the disk**, achieved by *ideal twist*
//! `θ(x)=θ_tip/x` ([`Rotor::ideal_twist`]). So the validated claims
//! (`tests/blade_opt_validation.rs`) are: (1) ideal twist drives the inflow's
//! coefficient of variation toward zero; (2) the optimizer's linear twist reduces
//! both power and inflow non-uniformity versus an untwisted blade; (3) it cannot
//! reach the hyperbolic ideal — the residual non-uniformity is the honest cost of a
//! buildable linear blade, not a solver failure.

use helisim_airfoil::Airfoil;
use helisim_bemt::{Config, HoverSolution};
use helisim_mission::trim_hover_collective;
use helisim_optimize::{FnObjective, NmOptions, minimize};
use helisim_rotor::{Operating, Rotor};

/// Fixed geometry + operating context for a blade-shape optimization. The optimizer
/// varies only the spanwise *distribution* (twist, taper); blade count, radius,
/// mean chord (≡ solidity), tip speed and target thrust are held.
pub struct BladeProblem<'a> {
    /// Number of blades.
    pub n_blades: usize,
    /// Rotor radius, m.
    pub radius_m: f64,
    /// Area-mean chord, m. Taper pivots about this so solidity is preserved.
    pub mean_chord_m: f64,
    /// Inboard lifting start, fraction of radius.
    pub root_cutout: f64,
    /// Operating point (RPM / air).
    pub op: Operating,
    /// Sectional aerodynamics.
    pub airfoil: &'a dyn Airfoil,
    /// Thrust to hold while minimizing power, N.
    pub target_thrust_n: f64,
    /// BEMT settings (use `tip_loss: false` to match the clean ideal-twist theory).
    pub cfg: Config,
}

/// A trimmed evaluation of one blade shape.
#[derive(Clone, Debug)]
pub struct BladeEval {
    /// Linear twist rate, rad over the full span (negative = washout).
    pub twist_rate: f64,
    /// Taper ratio `tip_chord / root_chord` (1 = rectangular).
    pub taper_ratio: f64,
    /// Collective (root pitch) the trim found, rad.
    pub collective: f64,
    /// Shaft power at the target thrust, W (induced + profile).
    pub power_w: f64,
    /// **Induced** power coefficient `C_{P,i} = ∫ λ·dC_T/dx dx` — the part the
    /// minimum-induced-loss theorem bounds (uniform inflow minimizes it). Profile
    /// power is the rest of `power_w`; ideal twist minimizes `induced_cp` but not
    /// necessarily total power (its singular root pitch inflates profile drag).
    pub induced_cp: f64,
    /// Figure of merit.
    pub figure_of_merit: f64,
    /// Coefficient of variation of the spanwise inflow `λ` (0 = perfectly uniform =
    /// the minimum-induced-power condition).
    pub inflow_cv: f64,
}

impl BladeProblem<'_> {
    /// Build a linearly twisted/tapered rotor with the given shape, holding the
    /// area-mean chord fixed (root/tip chord straddle `mean_chord_m`).
    pub fn build_rotor(&self, twist_rate: f64, taper_ratio: f64) -> Rotor {
        // (root + tip)/2 = mean, tip = taper·root ⇒ root = 2·mean/(1+taper).
        let root_chord = 2.0 * self.mean_chord_m / (1.0 + taper_ratio);
        let tip_chord = taper_ratio * root_chord;
        let mut r = Rotor::rectangular(
            self.n_blades,
            self.radius_m,
            root_chord,
            0.0,
            self.root_cutout,
        );
        r.tip_chord = tip_chord;
        r.twist_rate = twist_rate;
        r
    }

    /// Trim collective to the target thrust and report the shape's performance.
    /// `None` if the rotor cannot make the target thrust within the collective range.
    pub fn evaluate(&self, twist_rate: f64, taper_ratio: f64) -> Option<BladeEval> {
        let rotor = self.build_rotor(twist_rate, taper_ratio);
        let (collective, sol) = trim_hover_collective(
            &rotor,
            &self.op,
            self.airfoil,
            self.target_thrust_n,
            &self.cfg,
        )?;
        Some(BladeEval {
            twist_rate,
            taper_ratio,
            collective,
            power_w: sol.power,
            induced_cp: induced_cp(&sol),
            figure_of_merit: sol.figure_of_merit,
            inflow_cv: inflow_cv(&sol),
        })
    }

    /// The minimum-induced-loss **anchor**: an ideal-twist rotor (`θ=θ_tip/x`)
    /// trimmed to the same thrust. Its inflow is ~uniform — the optimum the buildable
    /// linear blade is measured against. `None` if it cannot be trimmed.
    pub fn ideal_twist_anchor(&self) -> Option<BladeEval> {
        let rotor = Rotor::ideal_twist(
            self.n_blades,
            self.radius_m,
            self.mean_chord_m,
            6f64.to_radians(), // initial tip pitch; trim retunes it
            self.root_cutout,
        );
        let (tip_pitch, sol) = trim_hover_collective(
            &rotor,
            &self.op,
            self.airfoil,
            self.target_thrust_n,
            &self.cfg,
        )?;
        Some(BladeEval {
            twist_rate: f64::NAN, // hyperbolic, not a linear rate
            taper_ratio: 1.0,
            collective: tip_pitch,
            power_w: sol.power,
            induced_cp: induced_cp(&sol),
            figure_of_merit: sol.figure_of_merit,
            inflow_cv: inflow_cv(&sol),
        })
    }

    /// Optimize `[twist_rate, taper_ratio]` for minimum power at the target thrust.
    /// Untrimmable trial shapes are penalized so the simplex stays in the feasible
    /// region. Returns the trimmed best shape.
    pub fn optimize(&self) -> BladeEval {
        // Bounds: washout-to-mild-wash-in, and tip-narrowing-to-mild-inverse taper.
        let bounds = vec![(-0.6_f64, 0.10_f64), (0.3_f64, 1.2_f64)];
        // Power of an untwisted rectangular blade — the penalty scale for infeasible
        // trials (a big multiple of a real, achievable power).
        let baseline = self.evaluate(0.0, 1.0).map(|e| e.power_w).unwrap_or(1.0e4);
        let obj = FnObjective::bounded(2, bounds, |v: &[f64]| match self.evaluate(v[0], v[1]) {
            Some(e) => e.power_w,
            None => 10.0 * baseline + 1.0e6,
        });
        let res = minimize(&obj, &[0.0, 1.0], &NmOptions::default());
        // Re-trim at the optimum so the returned eval is exact (not the penalized value).
        self.evaluate(res.x[0], res.x[1])
            .unwrap_or_else(|| self.evaluate(0.0, 1.0).expect("baseline trimmable"))
    }
}

/// Induced power coefficient `C_{P,i} = ∫₀¹ λ·(dC_T/dx) dx` (trapezoidal over the
/// converged stations). The local induced power is inflow × thrust; uniform inflow
/// minimizes it for a given total `C_T` (the minimum-induced-loss theorem).
pub fn induced_cp(sol: &HoverSolution) -> f64 {
    let mut cp_i = 0.0;
    for w in sol.stations.windows(2) {
        let dx = w[1].x - w[0].x;
        let f0 = w[0].lambda * w[0].dct_dx;
        let f1 = w[1].lambda * w[1].dct_dx;
        cp_i += 0.5 * (f0 + f1) * dx;
    }
    cp_i
}

/// Coefficient of variation (std/mean) of the inflow `λ` over the lifting stations.
/// Zero ⇔ uniform inflow ⇔ the minimum-induced-power condition. Stations with
/// non-positive `dC_T/dx` (no net lift, e.g. a stalled/negative inner section) are
/// excluded so the metric reflects the working part of the blade.
pub fn inflow_cv(sol: &HoverSolution) -> f64 {
    let lams: Vec<f64> = sol
        .stations
        .iter()
        .filter(|s| s.dct_dx > 0.0)
        .map(|s| s.lambda)
        .collect();
    if lams.len() < 2 {
        return 0.0;
    }
    let n = lams.len() as f64;
    let mean = lams.iter().sum::<f64>() / n;
    if mean.abs() < 1e-12 {
        return 0.0;
    }
    let var = lams.iter().map(|&l| (l - mean) * (l - mean)).sum::<f64>() / n;
    var.sqrt() / mean
}
