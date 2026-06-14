//! Beam-FEM validation against closed-form beam theory — the oracle.
//!
//! These are reductions to known cases (the project's preferred validation): the
//! cantilever and simply-supported deflections are textbook, and Euler-Bernoulli
//! cubic elements are exact for point loads, so the agreement is to machine
//! precision (not a tolerance fudge).

use helisim_fea::{Bc, NodalLoad, uniform_beam};

const EI: f64 = 1000.0; // N·m²
const Z: f64 = 1e-5; // m³ (section modulus, for stress)

#[test]
fn cantilever_point_load_is_exact() {
    // Clamp the root, load the tip with P. Tip deflection = P L³ / 3EI (exact for
    // a single cubic element).
    let l = 2.0;
    let p = -150.0; // N, downward
    let beam = uniform_beam(l, 1, EI, Z);
    let sol = beam
        .solve(
            &[NodalLoad {
                node: 1,
                force: p,
                moment: 0.0,
            }],
            None,
            &[Bc::Clamped(0)],
        )
        .unwrap();
    let exact = p * l * l * l / (3.0 * EI);
    assert!(
        (sol.deflection[1] - exact).abs() < 1e-9,
        "{} vs {exact}",
        sol.deflection[1]
    );
    // Root bending moment = |P|·L.
    assert!((sol.max_moment_nm - p.abs() * l).abs() < 1e-9);
    // And the peak stress is M/Z.
    assert!((sol.max_stress_pa - p.abs() * l / Z).abs() < 1e-3);
}

#[test]
fn cantilever_distributed_load_converges() {
    // Uniform load q over the span; tip deflection = q L⁴ / 8EI. Distributed loads
    // need refinement (the exact shape is quartic) — 16 elements is plenty.
    let l = 3.0;
    let q = -40.0; // N/m
    let beam = uniform_beam(l, 16, EI, Z);
    let dist = beam.uniform_load_vector(q);
    let sol = beam.solve(&[], Some(&dist), &[Bc::Clamped(0)]).unwrap();
    let exact = q * l.powi(4) / (8.0 * EI);
    let tip = *sol.deflection.last().unwrap();
    assert!((tip - exact).abs() / exact.abs() < 1e-3, "{tip} vs {exact}");
}

#[test]
fn simply_supported_center_load_is_exact() {
    // Pin both ends, point load P at the centre. Centre deflection = P L³ / 48EI.
    let l = 4.0;
    let p = -200.0;
    let n_el = 4; // even → a node at the centre
    let beam = uniform_beam(l, n_el, EI, Z);
    let mid = n_el / 2;
    let sol = beam
        .solve(
            &[NodalLoad {
                node: mid,
                force: p,
                moment: 0.0,
            }],
            None,
            &[Bc::Pinned(0), Bc::Pinned(n_el)],
        )
        .unwrap();
    let exact = p * l.powi(3) / (48.0 * EI);
    assert!(
        (sol.deflection[mid] - exact).abs() / exact.abs() < 1e-9,
        "{} vs {exact}",
        sol.deflection[mid]
    );
}

#[test]
fn geometric_stiffening_reduces_deflection_and_zero_tension_recovers_beam() {
    // A cantilever under a tip load: adding axial tension must stiffen it (smaller
    // deflection), and zero tension must reproduce the pure-beam answer exactly.
    let (l, p) = (2.0, -150.0);
    let base = uniform_beam(l, 8, EI, Z);
    let no_t = base
        .solve(
            &[NodalLoad {
                node: 8,
                force: p,
                moment: 0.0,
            }],
            None,
            &[Bc::Clamped(0)],
        )
        .unwrap();
    let mut tensioned = uniform_beam(l, 8, EI, Z);
    tensioned.tension = vec![5000.0; 8];
    let with_t = tensioned
        .solve(
            &[NodalLoad {
                node: 8,
                force: p,
                moment: 0.0,
            }],
            None,
            &[Bc::Clamped(0)],
        )
        .unwrap();
    assert!((no_t.max_deflection_m - (p * l * l * l / (3.0 * EI)).abs()).abs() < 1e-9);
    assert!(
        with_t.max_deflection_m < no_t.max_deflection_m,
        "tension must stiffen"
    );
}

#[test]
fn taut_string_limit_recovers_q_l2_over_8t() {
    // As EI→0 with axial tension T, a pinned-pinned beam under uniform load q
    // becomes a string: centre deflection = q L² / 8T (the string equation).
    let (l, q, t) = (4.0, -50.0, 1000.0);
    let n_el = 24;
    let tiny_ei = 1e-6;
    let mut beam = uniform_beam(l, n_el, tiny_ei, Z);
    beam.tension = vec![t; n_el];
    let dist = beam.uniform_load_vector(q);
    let sol = beam
        .solve(&[], Some(&dist), &[Bc::Pinned(0), Bc::Pinned(n_el)])
        .unwrap();
    let string = q * l * l / (8.0 * t); // centre deflection of a loaded string
    let mid = sol.deflection[n_el / 2];
    assert!(
        (mid - string).abs() / string.abs() < 0.02,
        "{mid} vs string {string}"
    );
}

#[test]
fn combined_loads_superpose_exactly() {
    // Assembly-level check: a linear FE model must superpose. A cantilever under a
    // tip point load P AND a uniform load q deflects exactly the sum of the two
    // solved separately — and equals the closed-form PL³/3EI + qL⁴/8EI.
    let (l, p, q) = (2.0, -100.0, -30.0);
    let n = 16;
    let tip = |dist, pt| -> f64 {
        let b = uniform_beam(l, n, EI, Z);
        let d: Option<Vec<f64>> = if dist {
            Some(b.uniform_load_vector(q))
        } else {
            None
        };
        let loads: Vec<NodalLoad> = if pt {
            vec![NodalLoad {
                node: n,
                force: p,
                moment: 0.0,
            }]
        } else {
            vec![]
        };
        *b.solve(&loads, d.as_deref(), &[Bc::Clamped(0)])
            .unwrap()
            .deflection
            .last()
            .unwrap()
    };
    let combined = tip(true, true);
    let separate = tip(false, true) + tip(true, false);
    assert!(
        (combined - separate).abs() < 1e-9,
        "superposition: {combined} vs {separate}"
    );
    let closed = p * l.powi(3) / (3.0 * EI) + q * l.powi(4) / (8.0 * EI);
    assert!((combined - closed).abs() / closed.abs() < 1e-3);
}

#[test]
fn a_soft_outboard_segment_adds_tip_compliance() {
    // Per-element EI: a multi-segment beam with a soft outer half deflects MORE at
    // the tip than a uniformly-stiff beam under the same tip load — the assembly
    // correctly carries variable stiffness along the span.
    let (l, p, n) = (2.0, -100.0, 8);
    let stiff = uniform_beam(l, n, EI, Z)
        .solve(
            &[NodalLoad {
                node: n,
                force: p,
                moment: 0.0,
            }],
            None,
            &[Bc::Clamped(0)],
        )
        .unwrap();
    let mut soft_tip = uniform_beam(l, n, EI, Z);
    for e in n / 2..n {
        soft_tip.ei[e] = EI / 4.0; // outer half is 4× more flexible
    }
    let soft = soft_tip
        .solve(
            &[NodalLoad {
                node: n,
                force: p,
                moment: 0.0,
            }],
            None,
            &[Bc::Clamped(0)],
        )
        .unwrap();
    assert!(soft.max_deflection_m > stiff.max_deflection_m);
    // Root bending moment is unchanged (statically determinate: M_root = |P|·L).
    assert!((soft.max_moment_nm - p.abs() * l).abs() / (p.abs() * l) < 0.05);
}

#[test]
fn refinement_does_not_change_the_exact_point_load_answer() {
    // The point-load cantilever is exact at any mesh density — 1 vs 8 elements agree.
    let (l, p) = (2.5, -120.0);
    let coarse = uniform_beam(l, 1, EI, Z)
        .solve(
            &[NodalLoad {
                node: 1,
                force: p,
                moment: 0.0,
            }],
            None,
            &[Bc::Clamped(0)],
        )
        .unwrap();
    let fine = uniform_beam(l, 8, EI, Z)
        .solve(
            &[NodalLoad {
                node: 8,
                force: p,
                moment: 0.0,
            }],
            None,
            &[Bc::Clamped(0)],
        )
        .unwrap();
    assert!((coarse.max_deflection_m - fine.max_deflection_m).abs() < 1e-9);
}
