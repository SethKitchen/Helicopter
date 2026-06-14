//! A tiny 3×3 linear solver (Gaussian elimination with partial pivoting).
//!
//! This is the project's first non-bisection solver: harmonic balance produces a
//! 3-coefficient linear system rather than a 1-D root find.

/// Solve `A x = b` for a 3×3 system. Returns the solution vector.
pub fn solve3(mut a: [[f64; 3]; 3], mut b: [f64; 3]) -> [f64; 3] {
    for col in 0..3 {
        // Partial pivot: largest magnitude in this column.
        let mut piv = col;
        for r in (col + 1)..3 {
            if a[r][col].abs() > a[piv][col].abs() {
                piv = r;
            }
        }
        a.swap(col, piv);
        b.swap(col, piv);

        let d = a[col][col];
        for r in 0..3 {
            if r != col {
                let f = a[r][col] / d;
                let pivot_row = a[col]; // [f64; 3] is Copy — disjoint from a[r]
                for (c, v) in a[r].iter_mut().enumerate() {
                    *v -= f * pivot_row[c];
                }
                b[r] -= f * b[col];
            }
        }
    }
    [b[0] / a[0][0], b[1] / a[1][1], b[2] / a[2][2]]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solves_identity() {
        let i = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let x = solve3(i, [3.0, -2.0, 5.0]);
        assert!(
            (x[0] - 3.0).abs() < 1e-12 && (x[1] + 2.0).abs() < 1e-12 && (x[2] - 5.0).abs() < 1e-12
        );
    }

    #[test]
    fn solves_general() {
        // 2x+y−z=8 ; −3x−y+2z=−11 ; −2x+y+2z=−3  → (2,3,−1)
        let a = [[2.0, 1.0, -1.0], [-3.0, -1.0, 2.0], [-2.0, 1.0, 2.0]];
        let x = solve3(a, [8.0, -11.0, -3.0]);
        assert!((x[0] - 2.0).abs() < 1e-9);
        assert!((x[1] - 3.0).abs() < 1e-9);
        assert!((x[2] + 1.0).abs() < 1e-9);
    }
}
