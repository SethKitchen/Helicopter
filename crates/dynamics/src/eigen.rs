//! Eigenvalues of a modest real matrix, std-only.
//!
//! Two classic primitives composed:
//! * [`char_poly`] — the characteristic polynomial via the Faddeev–LeVerrier
//!   algorithm (no pivoting, exact for small matrices).
//! * [`roots`] — all complex roots of a real polynomial via Durand–Kerner
//!   simultaneous iteration.
//!
//! [`eigenvalues`] is their composition. This is the project's new solver shape;
//! it is validated against the analytically-rootable longitudinal characteristic
//! polynomial.

use crate::complex::Complex;

/// Monic characteristic polynomial coefficients `[1, c1, c2, …, cn]` of the
/// `n×n` matrix `a` (so `p(λ) = λ^n + c1 λ^{n-1} + … + cn`), via Faddeev–LeVerrier.
pub fn char_poly(a: &[Vec<f64>]) -> Vec<f64> {
    let n = a.len();
    let mut m = identity(n);
    let mut coeffs = vec![1.0];
    for k in 1..=n {
        let am = matmul(a, &m);
        let ck = -trace(&am) / k as f64;
        coeffs.push(ck);
        // M_k = A·M_{k-1} + ck·I
        m = am;
        for (i, row) in m.iter_mut().enumerate() {
            row[i] += ck;
        }
    }
    coeffs
}

/// All complex roots of the real polynomial with monic coefficients
/// `[1, c1, …, cn]`, via Durand–Kerner.
pub fn roots(coeffs: &[f64]) -> Vec<Complex> {
    let n = coeffs.len() - 1;
    if n == 0 {
        return Vec::new();
    }
    // Distinct starting guesses on a spiral (a standard robust seed).
    let seed = Complex::new(0.4, 0.9);
    let mut z: Vec<Complex> = (0..n).map(|i| seed.powi(i as u32)).collect();

    for _ in 0..500 {
        let mut max_delta = 0.0_f64;
        for i in 0..n {
            let p = horner(coeffs, z[i]);
            let mut denom = Complex::real(1.0);
            for j in 0..n {
                if j != i {
                    denom = denom * (z[i] - z[j]);
                }
            }
            let delta = p / denom;
            z[i] = z[i] - delta;
            max_delta = max_delta.max(delta.abs());
        }
        if max_delta < 1e-12 {
            break;
        }
    }
    z
}

/// Eigenvalues of a real matrix — via the QR algorithm ([`crate::schur`]), which
/// stays accurate for the large augmented control systems (the char-poly route
/// [`char_poly`] + [`roots`] is ill-conditioned past ~10×10 and is retained only
/// for the small analytic anchors that validate the primitives).
pub fn eigenvalues(a: &[Vec<f64>]) -> Vec<Complex> {
    crate::schur::schur_eigenvalues(a)
}

/// Eigenvalues via the characteristic-polynomial route (Faddeev–LeVerrier +
/// Durand–Kerner). Kept for the small analytic anchors; use [`eigenvalues`] in
/// general.
pub fn eigenvalues_via_char_poly(a: &[Vec<f64>]) -> Vec<Complex> {
    roots(&char_poly(a))
}

/// Evaluate a real-coefficient polynomial at a complex point (Horner).
fn horner(coeffs: &[f64], z: Complex) -> Complex {
    let mut acc = Complex::zero();
    for &c in coeffs {
        acc = acc * z + Complex::real(c);
    }
    acc
}

fn identity(n: usize) -> Vec<Vec<f64>> {
    let mut m = vec![vec![0.0; n]; n];
    for (i, row) in m.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    m
}

fn matmul(a: &[Vec<f64>], b: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let n = a.len();
    let mut c = vec![vec![0.0; n]; n];
    for i in 0..n {
        for k in 0..n {
            let aik = a[i][k];
            for j in 0..n {
                c[i][j] += aik * b[k][j];
            }
        }
    }
    c
}

fn trace(a: &[Vec<f64>]) -> f64 {
    (0..a.len()).map(|i| a[i][i]).sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn diagonal_eigenvalues() {
        let a = vec![
            vec![2.0, 0.0, 0.0],
            vec![0.0, -3.0, 0.0],
            vec![0.0, 0.0, 5.0],
        ];
        let mut ev: Vec<f64> = eigenvalues(&a).iter().map(|c| c.re).collect();
        ev.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!(
            (ev[0] + 3.0).abs() < 1e-6 && (ev[1] - 2.0).abs() < 1e-6 && (ev[2] - 5.0).abs() < 1e-6
        );
    }

    #[test]
    fn complex_pair() {
        // [[0,1],[-1,0]] has eigenvalues ±i.
        let a = vec![vec![0.0, 1.0], vec![-1.0, 0.0]];
        let ev = eigenvalues(&a);
        for e in &ev {
            assert!(e.re.abs() < 1e-6 && (e.im.abs() - 1.0).abs() < 1e-6);
        }
    }

    #[test]
    fn char_poly_of_companion() {
        // p(λ)=λ²-3λ+2 → roots 1,2. Companion matrix [[3,-2],[1,0]].
        let a = vec![vec![3.0, -2.0], vec![1.0, 0.0]];
        let c = char_poly(&a);
        assert!(
            (c[0] - 1.0).abs() < 1e-12 && (c[1] + 3.0).abs() < 1e-9 && (c[2] - 2.0).abs() < 1e-9
        );
    }
}
