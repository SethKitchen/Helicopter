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
fn elmhes(a: &mut [Vec<f64>]) {
    let n = a.len();
    if n < 3 {
        return;
    }
    for m in 1..(n - 1) {
        // Pivot: largest subdiagonal entry in column m-1.
        let mut x = 0.0_f64;
        let mut i_piv = m;
        for (i, row) in a.iter().enumerate().skip(m) {
            if row[m - 1].abs() > x.abs() {
                x = row[m - 1];
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
                    // Row op: row_i[j] -= y·row_m[j] for j ≥ m (i > m → disjoint rows).
                    {
                        let (left, right) = a.split_at_mut(i);
                        let row_i = &mut right[0];
                        let row_m = &left[m];
                        for (ri, rm) in row_i.iter_mut().zip(row_m.iter()).skip(m) {
                            *ri -= y * rm;
                        }
                    }
                    // Column op: col_m += y·col_i across every row.
                    for row in a.iter_mut() {
                        let col_i = row[i];
                        row[m] += y * col_i;
                    }
                }
            }
        }
    }
    for (i, row) in a.iter_mut().enumerate().skip(2) {
        for v in row.iter_mut().take(i - 1) {
            *v = 0.0;
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
fn hqr(a: &mut [Vec<f64>]) -> Vec<Complex> {
    let n = a.len();
    let mut wr = vec![0.0; n];
    let mut wi = vec![0.0; n];

    let mut anorm = 0.0;
    for (i, row) in a.iter().enumerate() {
        for v in row.iter().skip(i.saturating_sub(1)) {
            anorm += v.abs();
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
                for (i, row) in a.iter_mut().enumerate().take(nu + 1) {
                    row[i] -= x;
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
                // Row modifications: rows k, k+1 (and k+2 if present), column-wise.
                // Split into disjoint row borrows so the sweep touches three rows.
                let (left, right) = a.split_at_mut(k + 1);
                let row_k = &mut left[k];
                let (mid, tail) = right.split_at_mut(1);
                let row_k1 = &mut mid[0];
                let has_k2 = k + 2 <= nu;
                for j in k..n {
                    let mut pp = row_k[j] + qq * row_k1[j];
                    if has_k2 {
                        let row_k2 = &mut tail[0];
                        pp += rr * row_k2[j];
                        row_k2[j] -= pp * rx;
                    }
                    row_k1[j] -= pp * qx;
                    row_k[j] -= pp * px;
                }
                // Column modifications (each row touches only columns k..=k+2).
                let imax = if nu < k + 3 { nu } else { k + 3 };
                for row in a.iter_mut().take(imax + 1) {
                    let mut pp = px * row[k] + qx * row[k + 1];
                    if k + 2 <= nu {
                        pp += rx * row[k + 2];
                        row[k + 2] -= pp * rr;
                    }
                    row[k + 1] -= pp * qq;
                    row[k] -= pp;
                }
            }
        }
    }

    (0..n).map(|i| Complex::new(wr[i], wi[i])).collect()
}
