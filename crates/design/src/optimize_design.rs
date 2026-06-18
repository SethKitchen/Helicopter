//! Continuous design optimization — find the TRUE optimum, not a grid corner.
//!
//! The grid recommender ([`crate::recommend`]) lands the winner on a discrete node,
//! so it can only ever say "the optimum is near here (and may lie beyond the grid)".
//! This drives the real [`helisim_optimize`] Nelder–Mead simplex over **continuous**
//! geometry (radius, tip speed, solidity) per blade count, with the mission+life
//! weight closure ([`crate::SizingPolicy`]) and every hard constraint folded in as
//! penalties — so it returns the actual minimum, located between grid nodes.
//!
//! Objective: **minimize the closed gross mass** subject to hover feasibility, the
//! flare-margin safety floor, the tip-Mach ceiling, and the forward-flight envelope.
//! With the rotor-group structural mass now in the closure (grows with radius), gross
//! mass has an *interior* minimum in disk size — a small rotor needs a huge (heavy)
//! life-pack, a big rotor needs heavy blades/hub/boom — so the optimum is a real
//! point, not the largest rotor in range. `at_bound` flags any variable that still
//! rests on a physical bound (a genuine limit, not a grid artifact).

use crate::candidate::DesignCandidate;
use crate::metrics::evaluate;
use crate::recommend::{EnvelopeConstraint, candidate_envelope};
use crate::report::DesignReport;
use crate::sizing::{SizedCandidate, SizingPolicy};
use helisim_airfoil::Airfoil;
use helisim_bemt::Config;
use helisim_optimize::{FnObjective, NmOptions, minimize};
use std::f64::consts::PI;

/// Continuous, physical bounds for the optimizer (not grid nodes).
#[derive(Clone, Copy, Debug)]
pub struct DesignBounds {
    /// Rotor radius range, m.
    pub radius_m: (f64, f64),
    /// Tip speed range, m/s.
    pub tip_speed_ms: (f64, f64),
    /// Solidity range.
    pub solidity: (f64, f64),
}

/// Hard constraints the optimum must satisfy.
#[derive(Clone, Copy, Debug)]
pub struct DesignConstraints {
    /// Minimum flare margin (safety floor).
    pub min_flare_margin: f64,
    /// Maximum tip Mach.
    pub max_tip_mach: f64,
    /// Optional forward-flight envelope floor.
    pub envelope: Option<EnvelopeConstraint>,
}

/// The continuously-optimized design.
#[derive(Clone, Debug)]
pub struct OptimizedDesign {
    /// The optimum candidate (mission+life-sized).
    pub candidate: DesignCandidate,
    /// Its evaluated report.
    pub report: DesignReport,
    /// The weight/life closure detail.
    pub sized: SizedCandidate,
    /// Variables sitting on a physical bound at the optimum (a real limit).
    pub at_bound: Vec<&'static str>,
}

/// Find the minimum-gross-mass design over continuous geometry (looping the integer
/// blade count) that flies the policy's mission for its service life and meets every
/// constraint. `None` if nothing in the bounds closes.
pub fn optimize_design(
    base: &DesignCandidate,
    blade_counts: &[usize],
    bounds: &DesignBounds,
    constraints: &DesignConstraints,
    sizing: &SizingPolicy,
    airfoil: &dyn Airfoil,
    cfg: &Config,
) -> Option<OptimizedDesign> {
    let mut best: Option<OptimizedDesign> = None;

    for &nb in blade_counts {
        // Penalized minimize-gross objective over [radius, tip_speed, solidity].
        let obj = FnObjective::bounded(
            3,
            vec![bounds.radius_m, bounds.tip_speed_ms, bounds.solidity],
            |x: &[f64]| objective(x, nb, base, constraints, sizing, airfoil, cfg),
        );
        let seed = [
            base.radius_m.clamp(bounds.radius_m.0, bounds.radius_m.1),
            0.5 * (bounds.tip_speed_ms.0 + bounds.tip_speed_ms.1),
            base.solidity().clamp(bounds.solidity.0, bounds.solidity.1),
        ];
        // 3-D smooth problem — a few hundred simplex steps is ample; keep it bounded
        // because each evaluation runs a full mission+life weight closure.
        let res = minimize(
            &obj,
            &seed,
            &NmOptions {
                max_iter: 500,
                step: 0.08,
                ..NmOptions::default()
            },
        );

        // Rebuild and accept only if genuinely feasible.
        let (r, vt, sigma) = (res.x[0], res.x[1], res.x[2]);
        let chord = sigma * PI * r / nb as f64;
        let geom = base.with_geometry(nb, r, chord, vt);
        let Some((cand, s)) = sizing.sized_candidate(&geom, airfoil, cfg) else {
            continue;
        };
        let rep = evaluate(&cand, airfoil, cfg);
        if !feasible(&cand, &rep, constraints) {
            continue;
        }
        if best
            .as_ref()
            .map(|b| s.gross_kg < b.sized.gross_kg)
            .unwrap_or(true)
        {
            best = Some(OptimizedDesign {
                at_bound: bounds_hit(r, vt, sigma, bounds),
                candidate: cand,
                report: rep,
                sized: s,
            });
        }
    }
    best
}

/// The penalized objective: gross mass plus large penalties for any violation.
fn objective(
    x: &[f64],
    nb: usize,
    base: &DesignCandidate,
    constraints: &DesignConstraints,
    sizing: &SizingPolicy,
    airfoil: &dyn Airfoil,
    cfg: &Config,
) -> f64 {
    let (r, vt, sigma) = (x[0], x[1], x[2]);
    let chord = sigma * PI * r / nb as f64;
    let geom = base.with_geometry(nb, r, chord, vt);
    let Some((cand, s)) = sizing.sized_candidate(&geom, airfoil, cfg) else {
        return 1.0e7; // can't close / hover here
    };
    let rep = evaluate(&cand, airfoil, cfg);
    let mut penalty = 0.0;
    if !rep.hover_feasible {
        penalty += 1.0e6;
    }
    penalty += 1.0e4 * (constraints.min_flare_margin - rep.flare_margin).max(0.0);
    penalty += 1.0e5 * (rep.tip_mach - constraints.max_tip_mach).max(0.0);
    if let Some(ec) = &constraints.envelope {
        let env = candidate_envelope(&cand, &rep, &ec.limits);
        penalty += 1.0e3 * (ec.min_speed_limit_mps - env.speed_limit_mps).max(0.0);
    }
    s.gross_kg + penalty
}

fn feasible(cand: &DesignCandidate, rep: &DesignReport, c: &DesignConstraints) -> bool {
    if !rep.hover_feasible
        || rep.flare_margin < c.min_flare_margin - 1e-3
        || rep.tip_mach > c.max_tip_mach + 1e-3
    {
        return false;
    }
    if let Some(ec) = &c.envelope {
        let env = candidate_envelope(cand, rep, &ec.limits);
        if env.speed_limit_mps < ec.min_speed_limit_mps - 1e-3 {
            return false;
        }
    }
    true
}

fn bounds_hit(r: f64, vt: f64, sigma: f64, b: &DesignBounds) -> Vec<&'static str> {
    let mut hit = Vec::new();
    let near = |x: f64, (lo, hi): (f64, f64)| (x - lo).abs() < 1e-3 || (x - hi).abs() < 1e-3;
    if near(r, b.radius_m) {
        hit.push("radius");
    }
    if near(vt, b.tip_speed_ms) {
        hit.push("tip speed");
    }
    if near(sigma, b.solidity) {
        hit.push("solidity");
    }
    hit
}
