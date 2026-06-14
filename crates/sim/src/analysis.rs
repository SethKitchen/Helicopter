//! Fit the period and growth rate of a growing oscillation from a trajectory —
//! the quantities compared against the 5c eigenvalues.

/// Fitted parameters of a (growing or decaying) oscillation.
#[derive(Clone, Copy, Debug)]
pub struct ModeFit {
    /// Oscillation period from successive same-sign peaks, s.
    pub period: f64,
    /// Growth rate `σ` (1/s): positive = growing. From the peak-amplitude ratio.
    pub growth_rate: f64,
    /// Time to double (growing) or half (decaying) amplitude, s.
    pub time_to_double: f64,
    /// Number of peaks used in the fit.
    pub peaks_used: usize,
}

/// Find interior local maxima of `y` (with their times), within the *linear*
/// window where `|y|` stays below `limit` (so a fast-growing mode is fitted
/// before nonlinearity bends it). A tiny absolute floor keeps the early,
/// small-amplitude peaks that a relative floor would cull.
fn peaks(t: &[f64], y: &[f64], limit: f64) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    for i in 1..y.len() - 1 {
        if y[i].abs() > limit {
            break; // left the linear regime
        }
        if y[i] > y[i - 1] && y[i] >= y[i + 1] && y[i] > 1e-12 {
            out.push((t[i], y[i]));
        }
    }
    out
}

/// Fit a growing oscillation to series `y(t)`: period from peak spacing, growth
/// rate from the geometric mean of successive peak ratios, over the linear-regime
/// peaks (those below `linear_limit`). Needs at least two peaks.
pub fn fit_growing_oscillation(t: &[f64], y: &[f64], linear_limit: f64) -> Option<ModeFit> {
    let pk = peaks(t, y, linear_limit);
    if pk.len() < 2 {
        return None;
    }
    let use_pk = &pk[..];

    // Period = mean spacing between successive peaks.
    let mut dt_sum = 0.0;
    for w in use_pk.windows(2) {
        dt_sum += w[1].0 - w[0].0;
    }
    let period = dt_sum / (use_pk.len() - 1) as f64;

    // Growth rate from successive peak-amplitude ratios: amp ~ e^{σ t}.
    let mut sigma_sum = 0.0;
    for w in use_pk.windows(2) {
        let dt = w[1].0 - w[0].0;
        sigma_sum += (w[1].1 / w[0].1).ln() / dt;
    }
    let growth_rate = sigma_sum / (use_pk.len() - 1) as f64;
    let time_to_double = if growth_rate.abs() > 1e-9 {
        (2.0_f64).ln() / growth_rate.abs()
    } else {
        f64::INFINITY
    };

    Some(ModeFit {
        period,
        growth_rate,
        time_to_double,
        peaks_used: use_pk.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fits_a_known_growing_oscillation() {
        // y(t) = e^{0.3 t} sin(2π t / 4)  → period 4 s, σ = 0.3.
        let dt = 0.01;
        let n = (24.0 / dt) as usize;
        let t: Vec<f64> = (0..n).map(|i| i as f64 * dt).collect();
        let y: Vec<f64> = t
            .iter()
            .map(|&ti| (0.3 * ti).exp() * (2.0 * std::f64::consts::PI * ti / 4.0).sin())
            .collect();
        let fit = fit_growing_oscillation(&t, &y, 1e9).unwrap();
        assert!((fit.period - 4.0).abs() < 0.05, "period {}", fit.period);
        assert!(
            (fit.growth_rate - 0.3).abs() < 0.02,
            "sigma {}",
            fit.growth_rate
        );
    }
}
