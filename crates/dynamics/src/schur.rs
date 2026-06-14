//! Eigenvalues of a real matrix via the QR algorithm — Hessenberg reduction
//! followed by Francis double-shift iteration (the EISPACK `elmhes`/`hqr` pair).
//!
//! This replaces the characteristic-polynomial route ([`crate::eigen`]) for the
//! general eigenvalue call: forming the char poly (Faddeev–LeVerrier) and rooting
//! it (Durand–Kerner) is numerically ill-conditioned once the matrix exceeds
//! ~10×10 (the coefficients span too many orders of magnitude). The QR algorithm
//! works on the matrix directly and stays accurate for the 13–15-state augmented
//! control systems. Real arithmetic throughout; complex eigenvalues emerge as the
//! 2×2 diagonal blocks of the real Schur form.

use crate::complex::Complex;

/// Eigenvalues of a real `n×n` matrix.
pub fn schur_eigenvalues(a: &[Vec<f64>]) -> Vec<Complex> {
    let n = a.len();
    if n == 0 {
        return Vec::new();
    }
    let mut h = a.to_vec();
    elmhes(&mut h);
    hqr(&mut h)
}

/// Reduce to upper Hessenberg form by Gaussian elimination with pivoting
/// (EISPACK `elmhes`). Eigenvalues are preserved (similarity transform).
#[allow(clippy::needless_range_loop)] // index arithmetic is intrinsic to the matrix sweep
fn elmhes(a: &mut [Vec<f64>]) {
    let n = a.len();
    if n < 3 {
        return;
    }
    for m in 1..(n - 1) {
        // Pivot: largest subdiagonal entry in column m-1.
        let mut x = 0.0_f64;
        let mut i_piv = m;
        for i in m..n {
            if a[i][m - 1].abs() > x.abs() {
                x = a[i][m - 1];
                i_piv = i;
            }
        }
        if i_piv != m {
            for j in (m - 1)..n {
                a.swap_cols_row(i_piv, m, j);
            }
            for row in a.iter_mut() {
                row.swap(i_piv, m);
            }
        }
        if x != 0.0 {
            for i in (m + 1)..n {
                let mut y = a[i][m - 1];
                if y != 0.0 {
                    y /= x;
                    a[i][m - 1] = y;
                    for j in m..n {
                        a[i][j] -= y * a[m][j];
                    }
                    for k in 0..n {
                        a[k][m] += y * a[k][i];
                    }
                }
            }
        }
    }
    for i in 2..n {
        for j in 0..(i - 1) {
            a[i][j] = 0.0;
        }
    }
}

// Small helper trait to swap entries between two rows at a fixed column.
trait SwapRows {
    fn swap_cols_row(&mut self, r1: usize, r2: usize, col: usize);
}
impl SwapRows for [Vec<f64>] {
    fn swap_cols_row(&mut self, r1: usize, r2: usize, col: usize) {
        let tmp = self[r1][col];
        self[r1][col] = self[r2][col];
        self[r2][col] = tmp;
    }
}

/// Francis double-shift QR on an upper-Hessenberg matrix (EISPACK `hqr`),
/// returning all eigenvalues. Translated to 0-indexing.
#[allow(clippy::needless_range_loop)] // index arithmetic is intrinsic to the QR sweep
fn hqr(a: &mut [Vec<f64>]) -> Vec<Complex> {
    let n = a.len();
    let mut wr = vec![0.0; n];
    let mut wi = vec![0.0; n];

    let mut anorm = 0.0;
    for i in 0..n {
        for j in i.saturating_sub(1)..n {
            anorm += a[i][j].abs();
        }
    }
    let eps = f64::EPSILON;
    let mut nn: isize = n as isize - 1;
    let mut t = 0.0;

    while nn >= 0 {
        let mut its = 0;
        loop {
            // Locate a single small subdiagonal element.
            let mut l = nn;
            while l >= 1 {
                let li = l as usize;
                let mut s = a[li - 1][li - 1].abs() + a[li][li].abs();
                if s == 0.0 {
                    s = anorm;
                }
                if a[li][li - 1].abs() <= eps * s {
                    break;
                }
                l -= 1;
            }
            let nu = nn as usize;
            let mut x = a[nu][nu];
            if l == nn {
                // One real root.
                wr[nu] = x + t;
                wi[nu] = 0.0;
                nn -= 1;
                break;
            }
            let mut y = a[nu - 1][nu - 1];
            let mut w = a[nu][nu - 1] * a[nu - 1][nu];
            if l == nn - 1 {
                // Two roots (real pair or complex conjugate pair).
                let p = 0.5 * (y - x);
                let q = p * p + w;
                let z = q.abs().sqrt();
                x += t;
                if q >= 0.0 {
                    let z = p + if p >= 0.0 { z } else { -z };
                    wr[nu - 1] = x + z;
                    wr[nu] = if z != 0.0 { x - w / z } else { x + z };
                    wi[nu - 1] = 0.0;
                    wi[nu] = 0.0;
                } else {
                    wr[nu - 1] = x + p;
                    wr[nu] = x + p;
                    wi[nu - 1] = z;
                    wi[nu] = -z;
                }
                nn -= 2;
                break;
            }
            if its >= 60 {
                // Non-convergence safety: emit the diagonal entry and deflate.
                wr[nu] = x + t;
                wi[nu] = 0.0;
                nn -= 1;
                break;
            }
            // Exceptional shift every 10 iterations.
            if its == 10 || its == 20 || its == 30 || its == 40 || its == 50 {
                t += x;
                for i in 0..=nu {
                    a[i][i] -= x;
                }
                let s = a[nu][nu - 1].abs() + a[nu - 1][nu - 2].abs();
                x = 0.75 * s;
                y = x;
                w = -0.4375 * s * s;
            }
            its += 1;

            // Look for two consecutive small subdiagonal elements.
            let (mut p, mut q, mut r) = (0.0, 0.0, 0.0);
            let mut m = nn - 2;
            while m >= l {
                let mi = m as usize;
                let z = a[mi][mi];
                let rr = x - z;
                let ss = y - z;
                p = (rr * ss - w) / a[mi + 1][mi] + a[mi][mi + 1];
                q = a[mi + 1][mi + 1] - z - rr - ss;
                r = a[mi + 2][mi + 1];
                let s = p.abs() + q.abs() + r.abs();
                p /= s;
                q /= s;
                r /= s;
                if m == l {
                    break;
                }
                let u = a[mi][mi - 1].abs() * (q.abs() + r.abs());
                let v = p.abs() * (a[mi - 1][mi - 1].abs() + z.abs() + a[mi + 1][mi + 1].abs());
                if u <= eps * v {
                    break;
                }
                m -= 1;
            }
            let mu = m as usize;
            for i in (mu + 2)..=nu {
                a[i][i - 2] = 0.0;
                if i != mu + 2 {
                    a[i][i - 3] = 0.0;
                }
            }

            // Double QR sweep over rows/cols m..=nn.
            for k in mu..nu {
                if k != mu {
                    p = a[k][k - 1];
                    q = a[k + 1][k - 1];
                    r = if k + 2 <= nu { a[k + 2][k - 1] } else { 0.0 };
                    x = p.abs() + q.abs() + r.abs();
                    if x != 0.0 {
                        p /= x;
                        q /= x;
                        r /= x;
                    }
                }
                let s_mag = (p * p + q * q + r * r).sqrt();
                let s = if p >= 0.0 { s_mag } else { -s_mag };
                if s == 0.0 {
                    continue;
                }
                if k == mu {
                    if l != m {
                        a[k][k - 1] = -a[k][k - 1];
                    }
                } else {
                    a[k][k - 1] = -s * x;
                }
                p += s;
                let px = p / s;
                let qx = q / s;
                let rx = r / s;
                let qq = q / p;
                let rr = r / p;
                // Row modifications.
                for j in k..n {
                    let mut pp = a[k][j] + qq * a[k + 1][j];
                    if k + 2 <= nu {
                        pp += rr * a[k + 2][j];
                        a[k + 2][j] -= pp * rx;
                    }
                    a[k + 1][j] -= pp * qx;
                    a[k][j] -= pp * px;
                }
                // Column modifications.
                let imax = if nu < k + 3 { nu } else { k + 3 };
                for i in 0..=imax {
                    let mut pp = px * a[i][k] + qx * a[i][k + 1];
                    if k + 2 <= nu {
                        pp += rx * a[i][k + 2];
                        a[i][k + 2] -= pp * rr;
                    }
                    a[i][k + 1] -= pp * qq;
                    a[i][k] -= pp;
                }
            }
        }
    }

    (0..n).map(|i| Complex::new(wr[i], wi[i])).collect()
}
