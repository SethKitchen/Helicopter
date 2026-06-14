//! Validation for the QR eigenvalue solver ([`schur_eigenvalues`]) — the
//! replacement for the characteristic-polynomial route at the large matrix sizes
//! the augmented control systems reach (13–15 states), where Faddeev–LeVerrier +
//! Durand–Kerner is ill-conditioned.

use helisim_dynamics::{Complex, schur_eigenvalues};

#[test]
fn diagonal() {
    let a = vec![
        vec![2.0, 0.0, 0.0],
        vec![0.0, -3.0, 0.0],
        vec![0.0, 0.0, 5.0],
    ];
    let mut ev: Vec<f64> = schur_eigenvalues(&a).iter().map(|c| c.re).collect();
    ev.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert!((ev[0] + 3.0).abs() < 1e-9 && (ev[1] - 2.0).abs() < 1e-9 && (ev[2] - 5.0).abs() < 1e-9);
}

#[test]
fn complex_pair() {
    let a = vec![vec![0.0, 1.0], vec![-1.0, 0.0]];
    let ev = schur_eigenvalues(&a);
    for e in &ev {
        assert!(e.re.abs() < 1e-9 && (e.im.abs() - 1.0).abs() < 1e-9);
    }
}

#[test]
fn block_diagonal_15_known_spectrum() {
    // A 15×15 with a known spectrum spanning ~3 orders of magnitude (−73 … −0.5)
    // and four complex pairs (one unstable, 0.5±1.2i) — the regime where the
    // char-poly route fails but QR stays accurate.
    let reals = [-73.0, -20.0, -8.0, -0.5, -2.0, -15.0, -31.0];
    let pairs = [(-1.0, 2.0), (-0.9, 0.2), (0.5, 1.2), (-3.0, 0.7)];
    let n = reals.len() + 2 * pairs.len();
    let mut a = vec![vec![0.0; n]; n];
    let mut i = 0;
    for &re in &reals {
        a[i][i] = re;
        i += 1;
    }
    for &(re, im) in &pairs {
        a[i][i] = re;
        a[i][i + 1] = im;
        a[i + 1][i] = -im;
        a[i + 1][i + 1] = re;
        i += 2;
    }
    let ev = schur_eigenvalues(&a);
    let known: Vec<Complex> = reals
        .iter()
        .map(|&r| Complex::new(r, 0.0))
        .chain(
            pairs
                .iter()
                .flat_map(|&(re, im)| [Complex::new(re, im), Complex::new(re, -im)]),
        )
        .collect();
    for k in &known {
        let found = ev
            .iter()
            .any(|e| (e.re - k.re).abs() < 1e-6 && (e.im - k.im).abs() < 1e-6);
        assert!(found, "eigenvalue {:.3}{:+.3}i not found", k.re, k.im);
    }
    assert_eq!(ev.len(), n);
}
