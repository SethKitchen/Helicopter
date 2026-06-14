//! Dense linear solve `A x = b` by Gaussian elimination with partial pivoting.
//!
//! The FE assembly produces a small dense symmetric system after the constrained
//! DOFs are removed; this solves it. Derivative-free, std-only, and validated on
//! systems with a known solution (a new solver shape for the project — the
//! existing `solve3`/Newton machinery is for fixed sizes / nonlinear roots).

/// Solve `A x = b` for a dense `n×n` system. `a` is row-major, length `n*n`;
/// returns `None` if the matrix is singular to working precision.
pub fn solve(a: &[f64], b: &[f64], n: usize) -> Option<Vec<f64>> {
    let mut m = a.to_vec();
    let mut x = b.to_vec();
    for col in 0..n {
        // Partial pivot: largest |entry| in this column at/below the diagonal.
        let mut piv = col;
        let mut best = m[col * n + col].abs();
        for r in (col + 1)..n {
            let v = m[r * n + col].abs();
            if v > best {
                best = v;
                piv = r;
            }
        }
        if best < 1e-14 {
            return None;
        }
        if piv != col {
            for c in 0..n {
                m.swap(col * n + c, piv * n + c);
            }
            x.swap(col, piv);
        }
        // Eliminate below.
        let d = m[col * n + col];
        for r in (col + 1)..n {
            let f = m[r * n + col] / d;
            if f != 0.0 {
                for c in col..n {
                    m[r * n + c] -= f * m[col * n + c];
                }
                x[r] -= f * x[col];
            }
        }
    }
    // Back-substitution.
    let mut sol = vec![0.0; n];
    for r in (0..n).rev() {
        let mut s = x[r];
        for c in (r + 1)..n {
            s -= m[r * n + c] * sol[c];
        }
        sol[r] = s / m[r * n + r];
    }
    Some(sol)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solves_a_known_2x2() {
        // [2 1; 1 3] x = [3; 5] → x = [4/5, 7/5].
        let x = solve(&[2.0, 1.0, 1.0, 3.0], &[3.0, 5.0], 2).unwrap();
        assert!((x[0] - 0.8).abs() < 1e-12 && (x[1] - 1.4).abs() < 1e-12);
    }

    #[test]
    fn solves_a_3x3_requiring_pivoting() {
        // Zero leading pivot forces a row swap.
        let a = [0.0, 2.0, 1.0, 1.0, 1.0, 1.0, 2.0, 1.0, 0.0];
        let b = [3.0, 3.0, 3.0];
        let x = solve(&a, &b, 3).unwrap();
        // Verify A x = b.
        for r in 0..3 {
            let s: f64 = (0..3).map(|c| a[r * 3 + c] * x[c]).sum();
            assert!((s - b[r]).abs() < 1e-10);
        }
    }

    #[test]
    fn singular_returns_none() {
        assert!(solve(&[1.0, 2.0, 2.0, 4.0], &[1.0, 2.0], 2).is_none());
    }
}
