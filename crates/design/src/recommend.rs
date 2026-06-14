//! Suggest a design — search the space and recommend a concrete point.
//!
//! The point of this project is to *propose* targets, not consume them. This
//! module turns the evaluator into a recommender: it sweeps the rotor geometry,
//! throws out anything that cannot hover or fails the **safety** floor (a hard
//! constraint, priority #1), then ranks the survivors by the rest of the priority
//! order — vertical integration → cost → airtime → efficiency → noise — and
//! returns the best concrete [`DesignCandidate`] with a rationale.
//!
//! Safety is a *constraint*, not a weighted term: a design that cannot flare is
//! rejected outright, however good its airtime or noise. The remaining priorities
//! are combined with rank-decreasing weights on min–max-normalised metrics — a
//! transparent scalarisation, documented as a modelling choice, with the full
//! ranked list returned so the priority trade can be inspected rather than hidden
//! inside one number.

use crate::candidate::DesignCandidate;
use crate::metrics::evaluate;
use crate::report::DesignReport;
use helisim_airfoil::Airfoil;
use helisim_bemt::Config;
use std::f64::consts::PI;

/// Rank-decreasing weights for priorities 2..6 (vert-integration, cost, airtime,
/// efficiency, noise). Priority 1 (safety) is a hard constraint, not weighted.
const W_VII: f64 = 5.0;
const W_COST: f64 = 4.0;
const W_ENDURANCE: f64 = 3.0;
const W_FM: f64 = 2.0;
const W_NOISE: f64 = 1.0;

/// The search space and the hard constraints.
#[derive(Clone, Debug)]
pub struct DesignSpace {
    /// Blade counts to try.
    pub blade_counts: Vec<usize>,
    /// Rotor radii to try, m.
    pub radii_m: Vec<f64>,
    /// Tip speeds to try, m/s.
    pub tip_speeds_ms: Vec<f64>,
    /// Solidities to try (chord is derived: `c = σ π R / N_b`).
    pub solidities: Vec<f64>,
    /// Safety floor: minimum acceptable flare margin (hard constraint).
    pub min_flare_margin: f64,
    /// Minimum acceptable hover endurance, min (hard constraint; 0 to disable).
    pub min_endurance_min: f64,
    /// Maximum tip Mach (compressibility / noise ceiling).
    pub max_tip_mach: f64,
}

impl DesignSpace {
    /// A sensible model-scale search grid.
    pub fn model_default() -> Self {
        DesignSpace {
            blade_counts: vec![2, 3],
            radii_m: vec![0.4, 0.5, 0.6, 0.7, 0.8],
            tip_speeds_ms: vec![90.0, 110.0, 130.0, 150.0],
            solidities: vec![0.05, 0.07, 0.09],
            min_flare_margin: 1.5,
            min_endurance_min: 10.0,
            max_tip_mach: 0.55,
        }
    }
}

/// A candidate with its evaluation and priority score.
#[derive(Clone, Debug)]
pub struct ScoredCandidate {
    /// The design.
    pub candidate: DesignCandidate,
    /// Its evaluated consequences.
    pub report: DesignReport,
    /// Priority-weighted desirability score (higher is better).
    pub score: f64,
}

/// The recommendation: the winner, the full ranked list, and why.
#[derive(Clone, Debug)]
pub struct Recommendation {
    /// The recommended design.
    pub best: ScoredCandidate,
    /// All feasible+safe candidates, best first.
    pub ranked: Vec<ScoredCandidate>,
    /// Total candidates evaluated.
    pub n_evaluated: usize,
    /// How many passed the hard constraints.
    pub n_feasible: usize,
    /// Human-readable rationale lines.
    pub rationale: Vec<String>,
}

/// Search `space` around `base` and recommend a design. Returns `None` if nothing
/// satisfies the hard constraints (the rationale on the caller side should then
/// relax the floor or widen the grid).
pub fn recommend(
    space: &DesignSpace,
    base: &DesignCandidate,
    airfoil: &dyn Airfoil,
    cfg: &Config,
) -> Option<Recommendation> {
    // 1. Enumerate + evaluate the grid.
    let mut evaluated: Vec<(DesignCandidate, DesignReport)> = Vec::new();
    for &nb in &space.blade_counts {
        for &r in &space.radii_m {
            for &vt in &space.tip_speeds_ms {
                for &sigma in &space.solidities {
                    let chord = sigma * PI * r / nb as f64;
                    let cand = base.with_geometry(nb, r, chord, vt);
                    let rep = evaluate(&cand, airfoil, cfg);
                    evaluated.push((cand, rep));
                }
            }
        }
    }
    let n_evaluated = evaluated.len();

    // 2. Hard constraints: safety floor + feasibility + airtime + tip Mach.
    let feasible: Vec<(DesignCandidate, DesignReport)> = evaluated
        .into_iter()
        .filter(|(_, rep)| {
            rep.hover_feasible
                && rep.flare_margin >= space.min_flare_margin
                && rep.endurance_min >= space.min_endurance_min
                && rep.tip_mach <= space.max_tip_mach
        })
        .collect();
    let n_feasible = feasible.len();
    if feasible.is_empty() {
        return None;
    }

    // 3. Min–max normalise each priority metric across the survivors.
    let vii: Vec<f64> = feasible.iter().map(|(_, r)| r.vertical_integration_index).collect();
    let cost: Vec<f64> = feasible.iter().map(|(_, r)| r.total_cost).collect();
    let endur: Vec<f64> = feasible.iter().map(|(_, r)| r.endurance_min).collect();
    let fm: Vec<f64> = feasible.iter().map(|(_, r)| r.figure_of_merit).collect();
    let noise: Vec<f64> = feasible.iter().map(|(_, r)| r.oaspl_db).collect();

    let mut ranked: Vec<ScoredCandidate> = feasible
        .iter()
        .map(|(cand, rep)| {
            let score = W_VII * norm_up(rep.vertical_integration_index, &vii)
                + W_COST * norm_down(rep.total_cost, &cost)
                + W_ENDURANCE * norm_up(rep.endurance_min, &endur)
                + W_FM * norm_up(rep.figure_of_merit, &fm)
                + W_NOISE * norm_down(rep.oaspl_db, &noise);
            ScoredCandidate { candidate: *cand, report: *rep, score }
        })
        .collect();
    ranked.sort_by(|a, b| b.score.total_cmp(&a.score));

    let best = ranked[0].clone();
    let rationale = build_rationale(space, &best, n_evaluated, n_feasible);
    Some(Recommendation { best, ranked, n_evaluated, n_feasible, rationale })
}

/// Normalise `x` to [0,1] where higher is better.
fn norm_up(x: f64, all: &[f64]) -> f64 {
    let (lo, hi) = min_max(all);
    if hi > lo { (x - lo) / (hi - lo) } else { 1.0 }
}

/// Normalise `x` to [0,1] where lower is better.
fn norm_down(x: f64, all: &[f64]) -> f64 {
    let (lo, hi) = min_max(all);
    if hi > lo { (hi - x) / (hi - lo) } else { 1.0 }
}

fn min_max(all: &[f64]) -> (f64, f64) {
    let lo = all.iter().cloned().fold(f64::INFINITY, f64::min);
    let hi = all.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    (lo, hi)
}

/// True if `x` sits at the min or max of a swept axis (a grid-edge optimum, where
/// the true optimum may lie outside the searched range).
fn at_edge(x: f64, axis: &[f64]) -> bool {
    let (lo, hi) = min_max(axis);
    (x - lo).abs() < 1e-9 || (x - hi).abs() < 1e-9
}

fn build_rationale(
    space: &DesignSpace,
    best: &ScoredCandidate,
    n_eval: usize,
    n_feas: usize,
) -> Vec<String> {
    let c = &best.candidate;
    let r = &best.report;

    // Honesty: flag axes where the winner sits on a grid boundary.
    let mut edges = Vec::new();
    if at_edge(c.tip_speed_ms, &space.tip_speeds_ms) {
        edges.push("tip speed");
    }
    if at_edge(c.radius_m, &space.radii_m) {
        edges.push("radius");
    }
    if at_edge(c.solidity(), &space.solidities) {
        edges.push("solidity");
    }

    let mut lines = vec![
        format!(
            "Searched {n_eval} candidates; {n_feas} passed safety (flare margin ≥ {:.1}), \
             airtime (≥ {:.0} min) and tip-Mach (≤ {:.2}) constraints.",
            space.min_flare_margin, space.min_endurance_min, space.max_tip_mach
        ),
        format!(
            "Recommended: {} blades, R={:.2} m, chord={:.3} m, V_tip={:.0} m/s (σ={:.3}).",
            c.n_blades, c.radius_m, c.chord_m, c.tip_speed_ms, c.solidity()
        ),
        format!(
            "Safety (priority 1, constraint): flare margin {:.2}, rotor-decay reaction {:.2} s, \
             forward auto best-glide {:.1}°.",
            r.flare_margin, r.rotor_decay_time_s, r.best_glide_angle_deg
        ),
        format!(
            "Then by priority: vert-integ {:.0}%, cost ≈ ${:.0}, endurance {:.1} min, \
             FM {:.2}, OASPL {:.1} dB.",
            r.vertical_integration_index * 100.0,
            r.total_cost,
            r.endurance_min,
            r.figure_of_merit,
            r.oaspl_db
        ),
    ];

    if !edges.is_empty() {
        lines.push(format!(
            "⚠ Winner sits at a grid edge on: {}. The true optimum may lie beyond \
             the searched range — widen the grid on these axes to confirm.",
            edges.join(", ")
        ));
    }
    lines
}
