//! Multidimensional Newton solver with a numerically-estimated Jacobian and a
//! simple damped (backtracking) step. The project's third solver shape.

/// Newton solver settings.
#[derive(Clone, Copy, Debug)]
pub struct NewtonConfig {
    /// Convergence tolerance on the residual L2 norm.
    pub tol: f64,
    /// Maximum Newton iterations.
    pub max_iter: usize,
    /// Relative finite-difference step for the Jacobian.
    pub fd_step: f64,
}

impl Default for NewtonConfig {
    fn default() -> Self {
        NewtonConfig {
            tol: 1e-8,
            max_iter: 80,
            fd_step: 1e-5,
        }
    }
}

fn l2(v: &[f64]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

/// Solve `f(x) = 0` starting from `x0`. Returns `(solution, residual_norm,
/// converged)`. The Jacobian is estimated by forward differences and the step is
/// backtracked until the residual norm decreases.
pub fn solve_newton(
    mut f: impl FnMut(&[f64]) -> Vec<f64>,
    x0: &[f64],
    cfg: &NewtonConfig,
) -> (Vec<f64>, f64, bool) {
    let n = x0.len();
    let mut x = x0.to_vec();
    let mut fx = f(&x);
    let mut norm = l2(&fx);

    for _ in 0..cfg.max_iter {
        if norm < cfg.tol {
            return (x, norm, true);
        }

        // Numerical Jacobian J[i][j] = ∂f_i/∂x_j.
        let mut j = vec![vec![0.0; n]; n];
        for col in 0..n {
            let h = cfg.fd_step * (1.0 + x[col].abs());
            let mut xp = x.clone();
            xp[col] += h;
            let fp = f(&xp);
            for (row, fp_row) in fp.iter().enumerate() {
                j[row][col] = (fp_row - fx[row]) / h;
            }
        }

        // Solve J dx = -f.
        let rhs: Vec<f64> = fx.iter().map(|v| -v).collect();
        let dx = match linear_solve(j, rhs) {
            Some(d) => d,
            None => return (x, norm, false), // singular Jacobian
        };

        // Backtracking line search: accept the largest step that reduces ‖f‖.
        let mut lambda = 1.0;
        let mut improved = false;
        for _ in 0..25 {
            let xn: Vec<f64> = x.iter().zip(&dx).map(|(a, b)| a + lambda * b).collect();
            let fn_ = f(&xn);
            let nn = l2(&fn_);
            if nn < norm {
                x = xn;
                fx = fn_;
                norm = nn;
                improved = true;
                break;
            }
            lambda *= 0.5;
        }
        if !improved {
            return (x, norm, false);
        }
    }
    (x, norm, norm < cfg.tol)
}

/// Gaussian elimination with partial pivoting for a dense `n×n` system.
/// Returns `None` if the matrix is singular.
// Index-based row reduction is clearer than iterator gymnastics here.
#[allow(clippy::needless_range_loop)]
fn linear_solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    for col in 0..n {
        let mut piv = col;
        for r in (col + 1)..n {
            if a[r][col].abs() > a[piv][col].abs() {
                piv = r;
            }
        }
        if a[piv][col].abs() < 1e-14 {
            return None;
        }
        a.swap(col, piv);
        b.swap(col, piv);
        for r in (col + 1)..n {
            let f = a[r][col] / a[col][col];
            for c in col..n {
                a[r][c] -= f * a[col][c];
            }
            b[r] -= f * b[col];
        }
    }
    // Back-substitution.
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut s = b[i];
        for c in (i + 1)..n {
            s -= a[i][c] * x[c];
        }
        x[i] = s / a[i][i];
    }
    Some(x)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solves_linear_system() {
        // f(x) = A x - b, root is A^-1 b.
        let f = |x: &[f64]| vec![2.0 * x[0] + x[1] - 5.0, x[0] - 3.0 * x[1] + 1.0];
        let (x, norm, ok) = solve_newton(f, &[0.0, 0.0], &NewtonConfig::default());
        assert!(ok && norm < 1e-7);
        assert!((x[0] - 2.0).abs() < 1e-5 && (x[1] - 1.0).abs() < 1e-5);
    }

    #[test]
    fn solves_nonlinear_system() {
        // x²+y²=2, x−y=0 → (1,1)
        let f = |x: &[f64]| vec![x[0] * x[0] + x[1] * x[1] - 2.0, x[0] - x[1]];
        let (x, _, ok) = solve_newton(f, &[2.0, 0.5], &NewtonConfig::default());
        assert!(ok);
        assert!((x[0] - 1.0).abs() < 1e-4 && (x[1] - 1.0).abs() < 1e-4);
    }
}
