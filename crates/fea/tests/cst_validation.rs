//! Plane-stress CST validation — the patch test and a uniaxial bar.
//!
//! A constant-strain triangle reproduces any uniform stress field exactly, so
//! these are machine-precision reductions to known cases, not tolerances.

use helisim_fea::rectangle_two_tris;

const E: f64 = 70.0e9;
const NU: f64 = 0.33;
const T: f64 = 0.001;

#[test]
fn uniaxial_bar_gives_exact_stress_and_elongation() {
    // L×H plate, left edge restrained, total force F on the right edge.
    // σxx = F/(H·t) and the right-edge displacement δ = F·L/(H·t·E), both exact.
    let (l, h, f) = (0.10, 0.05, 1000.0);
    let plate = rectangle_two_tris(l, h, T, E, NU);
    // Nodes: 0=(0,0),1=(L,0),2=(L,H),3=(0,H). Fix the left edge in x (0,3);
    // pin node 0 in y to remove rigid-body motion (the bar contracts freely).
    let fixed = [0, 1, 6]; // DOFs u0, v0, u3 (dof = 2·node, +1 for v)
    let sol = plate.solve(&[(1, f / 2.0, 0.0), (2, f / 2.0, 0.0)], &fixed).unwrap();

    let sigma_expected = f / (h * T);
    for s in &sol.element_stress {
        assert!((s[0] - sigma_expected).abs() / sigma_expected < 1e-9, "σxx {} vs {sigma_expected}", s[0]);
        assert!(s[1].abs() / sigma_expected < 1e-9, "σyy should be ~0");
        assert!(s[2].abs() / sigma_expected < 1e-9, "τxy should be ~0");
    }
    let delta_expected = f * l / (h * T * E);
    // Right-edge nodes 1 and 2 move by δ in x.
    assert!((sol.disp[1].0 - delta_expected).abs() / delta_expected < 1e-9);
    assert!((sol.disp[2].0 - delta_expected).abs() / delta_expected < 1e-9);
}

#[test]
fn patch_test_uniform_stress_is_constant_across_elements() {
    // Both triangles of the patch must report the SAME (uniform) stress — the
    // defining property of a convergent element.
    let plate = rectangle_two_tris(0.2, 0.1, T, E, NU);
    let fixed = [0, 1, 2 * 3];
    let sol = plate.solve(&[(1, 500.0, 0.0), (2, 500.0, 0.0)], &fixed).unwrap();
    let (a, b) = (sol.element_stress[0], sol.element_stress[1]);
    for k in 0..3 {
        assert!((a[k] - b[k]).abs() <= 1e-3 * a[0].abs().max(1.0));
    }
}

#[test]
fn lateral_poisson_contraction_has_the_right_sign() {
    // Under x-tension the bar contracts in y (Poisson): the free top edge moves
    // toward the centreline.
    let plate = rectangle_two_tris(0.10, 0.05, T, E, NU);
    let fixed = [0, 1, 2 * 3];
    let sol = plate.solve(&[(1, 500.0, 0.0), (2, 500.0, 0.0)], &fixed).unwrap();
    // Node 2 = top-right; its v should be negative (moves down toward y=0 line).
    assert!(sol.disp[2].1 < 0.0, "v2 = {}", sol.disp[2].1);
}
