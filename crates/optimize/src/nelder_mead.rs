//! Nelder–Mead downhill-simplex minimization — derivative-free, std-only.
//!
//! A NEW solver shape for the project (the 7th): the others find an *equilibrium*
//! or a *response* (bisection, linear solve, Newton root, eigenvalue, RK4, outer-
//! state integration); this finds a *minimum*. It needs no gradient (our evaluators
//! — BEMT power, mission energy — are cheap but not analytically differentiable), so
//! a simplex method fits: reflect/expand/contract/shrink a set of `n+1` vertices
//! downhill. Box bounds are honoured by clamping trial points before evaluation.
//!
//! Validated (`tests/optimizer_validation.rs`) against problems with KNOWN optima:
//! the sphere (origin), Rosenbrock (the (1,1) banana valley), a box-bound-active
//! case, and a penalty-constrained case with an analytic KKT solution.

use crate::objective::{Objective, clamp_to_bounds};

/// Nelder–Mead tuning. Defaults are the textbook coefficients with tolerances
/// suited to smooth O(1)-scaled objectives.
#[derive(Clone, Copy, Debug)]
pub struct NmOptions {
    /// Maximum objective evaluations-worth of iterations.
    pub max_iter: usize,
    /// Converge when the simplex value spread `f_worst − f_best < ftol`.
    pub ftol: f64,
    /// …and the simplex point spread (max vertex distance) `< xtol`.
    pub xtol: f64,
    /// Initial simplex edge length (absolute) for each coordinate.
    pub step: f64,
}

impl Default for NmOptions {
    fn default() -> Self {
        NmOptions {
            max_iter: 5000,
            ftol: 1e-10,
            xtol: 1e-9,
            step: 0.1,
        }
    }
}

/// Outcome of a minimization.
#[derive(Clone, Debug)]
pub struct NmResult {
    /// Best point found.
    pub x: Vec<f64>,
    /// Objective value there.
    pub value: f64,
    /// Iterations executed.
    pub iters: usize,
    /// Whether the simplex contracted below the tolerances (vs hit `max_iter`).
    pub converged: bool,
}

// Textbook Nelder–Mead coefficients.
const ALPHA: f64 = 1.0; // reflection
const GAMMA: f64 = 2.0; // expansion
const RHO: f64 = 0.5; // contraction
const SIGMA: f64 = 0.5; // shrink

/// Minimize `obj` starting from `x0` (length must equal `obj.dim()`).
pub fn minimize(obj: &dyn Objective, x0: &[f64], opts: &NmOptions) -> NmResult {
    let n = obj.dim();
    assert_eq!(x0.len(), n, "x0 length must equal objective dim");
    let bounds = obj.bounds();

    let eval = |p: &[f64]| -> f64 {
        let mut q = p.to_vec();
        clamp_to_bounds(&mut q, bounds);
        obj.value(&q)
    };

    // Initial simplex: x0 plus one bumped vertex per coordinate.
    let mut simplex: Vec<Vec<f64>> = Vec::with_capacity(n + 1);
    simplex.push(x0.to_vec());
    for i in 0..n {
        let mut v = x0.to_vec();
        v[i] += opts.step;
        clamp_to_bounds(&mut v, bounds);
        // If clamping collapsed the bump (x0 sat on the bound), step the other way.
        if (v[i] - x0[i]).abs() < 1e-15 {
            v[i] = x0[i] - opts.step;
            clamp_to_bounds(&mut v, bounds);
        }
        simplex.push(v);
    }
    let mut fvals: Vec<f64> = simplex.iter().map(|v| eval(v)).collect();

    let mut iters = 0;
    let mut converged = false;
    while iters < opts.max_iter {
        iters += 1;
        // Order vertices best → worst.
        order_simplex(&mut simplex, &mut fvals);

        // Convergence: both value spread and point spread small.
        let fspread = fvals[n] - fvals[0];
        let xspread = max_vertex_spread(&simplex);
        if fspread < opts.ftol && xspread < opts.xtol {
            converged = true;
            break;
        }

        // Centroid of all but the worst vertex.
        let centroid = centroid_excluding_worst(&simplex, n);

        // Reflection.
        let xr = combine(&centroid, &simplex[n], ALPHA);
        let fr = eval(&xr);

        if fr < fvals[0] {
            // Better than best → try to expand further.
            let xe = combine(&centroid, &simplex[n], ALPHA * GAMMA);
            let fe = eval(&xe);
            if fe < fr {
                simplex[n] = xe;
                fvals[n] = fe;
            } else {
                simplex[n] = xr;
                fvals[n] = fr;
            }
        } else if fr < fvals[n - 1] {
            // Mid-pack: accept the reflection.
            simplex[n] = xr;
            fvals[n] = fr;
        } else {
            // Worst region: contract.
            if fr < fvals[n] {
                // Outside contraction.
                let xc = combine(&centroid, &simplex[n], ALPHA * RHO);
                let fc = eval(&xc);
                if fc <= fr {
                    simplex[n] = xc;
                    fvals[n] = fc;
                } else {
                    shrink(&mut simplex, &mut fvals, &eval);
                }
            } else {
                // Inside contraction.
                let xcc = combine(&centroid, &simplex[n], -RHO);
                let fcc = eval(&xcc);
                if fcc < fvals[n] {
                    simplex[n] = xcc;
                    fvals[n] = fcc;
                } else {
                    shrink(&mut simplex, &mut fvals, &eval);
                }
            }
        }
    }

    order_simplex(&mut simplex, &mut fvals);
    let mut best = simplex[0].clone();
    clamp_to_bounds(&mut best, bounds);
    NmResult {
        value: fvals[0],
        x: best,
        iters,
        converged,
    }
}

/// `centroid + coeff·(centroid − vertex)` — the reflect/expand/contract template.
fn combine(centroid: &[f64], vertex: &[f64], coeff: f64) -> Vec<f64> {
    centroid
        .iter()
        .zip(vertex.iter())
        .map(|(&c, &v)| c + coeff * (c - v))
        .collect()
}

fn centroid_excluding_worst(simplex: &[Vec<f64>], n: usize) -> Vec<f64> {
    let mut c = vec![0.0; n];
    for v in &simplex[..n] {
        for (ci, &vi) in c.iter_mut().zip(v.iter()) {
            *ci += vi;
        }
    }
    for ci in c.iter_mut() {
        *ci /= n as f64;
    }
    c
}

fn shrink(simplex: &mut [Vec<f64>], fvals: &mut [f64], eval: &dyn Fn(&[f64]) -> f64) {
    let best = simplex[0].clone();
    for i in 1..simplex.len() {
        for (s, &b) in simplex[i].iter_mut().zip(best.iter()) {
            *s = b + SIGMA * (*s - b);
        }
        fvals[i] = eval(&simplex[i]);
    }
}

fn order_simplex(simplex: &mut [Vec<f64>], fvals: &mut [f64]) {
    // Small simplex (n+1 vertices): selection sort, keeping vertices and values paired.
    let m = fvals.len();
    for i in 0..m {
        let mut k = i;
        for j in (i + 1)..m {
            if fvals[j] < fvals[k] {
                k = j;
            }
        }
        if k != i {
            fvals.swap(i, k);
            simplex.swap(i, k);
        }
    }
}

fn max_vertex_spread(simplex: &[Vec<f64>]) -> f64 {
    let best = &simplex[0];
    let mut m = 0.0_f64;
    for v in &simplex[1..] {
        let d: f64 = v
            .iter()
            .zip(best.iter())
            .map(|(&a, &b)| (a - b) * (a - b))
            .sum();
        m = m.max(d.sqrt());
    }
    m
}
