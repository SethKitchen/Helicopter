//! Cell balancing. The [`Pack`](helisim_pack::Pack) model assumes a perfectly
//! balanced pack (one shared SoC). Real series strings drift apart — manufacturing
//! spread, temperature gradients, self-discharge — and a series string is only as
//! usable as its **weakest cell**: discharge must stop when the *lowest* cell hits
//! cutoff, charge when the *highest* hits full. The imbalance is dead capacity
//! until a balancer removes it.
//!
//! This models the spread and **passive** balancing (bleed the high cells down
//! toward the pack minimum through a resistor). Active balancing (shuttle charge
//! between cells) is a named extension, not yet modelled.

/// Per-(series-)cell state of charge in a string. `socs[i] ∈ [0, 1]`.
#[derive(Clone, Debug)]
pub struct CellSpread {
    pub socs: Vec<f64>,
}

impl CellSpread {
    /// A string of `n` identical cells all at `soc`.
    pub fn uniform(n: usize, soc: f64) -> Self {
        CellSpread { socs: vec![soc; n] }
    }

    /// From an explicit per-cell SoC list.
    pub fn new(socs: Vec<f64>) -> Self {
        assert!(!socs.is_empty(), "spread needs at least one cell");
        CellSpread { socs }
    }

    pub fn min(&self) -> f64 {
        self.socs.iter().copied().fold(f64::INFINITY, f64::min)
    }

    pub fn max(&self) -> f64 {
        self.socs.iter().copied().fold(f64::NEG_INFINITY, f64::max)
    }

    pub fn mean(&self) -> f64 {
        self.socs.iter().sum::<f64>() / self.socs.len() as f64
    }

    /// Imbalance = `max − min` SoC across the string.
    pub fn spread(&self) -> f64 {
        self.max() - self.min()
    }

    /// Fraction of cell capacity still dischargeable right now: discharge ends
    /// when the lowest cell reaches empty, so it is the **minimum** SoC.
    pub fn dischargeable_fraction(&self) -> f64 {
        self.min()
    }

    /// Fraction of cell capacity still chargeable: charge ends when the highest
    /// cell reaches full, so it is `1 − max`.
    pub fn chargeable_fraction(&self) -> f64 {
        1.0 - self.max()
    }

    /// Capacity stranded by imbalance, as a fraction of cell capacity. After a
    /// full charge (limited by the highest cell) the string holds `spread` less
    /// usable charge than a balanced one — that is the cost a balancer recovers.
    pub fn stranded_fraction(&self) -> f64 {
        self.spread()
    }

    /// One step of passive balancing: every cell above the string minimum bleeds
    /// `bleed` SoC toward it (clamped so it never crosses the minimum). Repeated
    /// application drives the spread to zero — the balancer's job.
    pub fn passive_balance_step(&mut self, bleed: f64) {
        let target = self.min();
        for s in self.socs.iter_mut() {
            if *s > target {
                *s = (*s - bleed).max(target);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn balanced_string_strands_nothing() {
        let s = CellSpread::uniform(96, 0.7);
        assert!(s.spread() < 1e-12);
        assert!(s.stranded_fraction() < 1e-12);
        assert!((s.dischargeable_fraction() - 0.7).abs() < 1e-12);
        assert!((s.chargeable_fraction() - 0.3).abs() < 1e-12);
    }

    #[test]
    fn weakest_cell_limits_the_string() {
        // One sagging cell in an otherwise-full string.
        let s = CellSpread::new(vec![0.95, 0.95, 0.60, 0.95]);
        assert!((s.dischargeable_fraction() - 0.60).abs() < 1e-12); // weakest sets discharge
        assert!((s.chargeable_fraction() - 0.05).abs() < 1e-12); // strongest sets charge
        assert!((s.stranded_fraction() - 0.35).abs() < 1e-12);
    }

    #[test]
    fn passive_balancing_converges_to_balanced() {
        let mut s = CellSpread::new(vec![0.95, 0.90, 0.60, 0.80]);
        let start = s.spread();
        assert!(start > 0.3);
        for _ in 0..1000 {
            s.passive_balance_step(0.001);
        }
        assert!(s.spread() < 1e-6, "spread {}", s.spread());
        // Balancing bleeds DOWN to the minimum, never below it.
        assert!((s.min() - 0.60).abs() < 1e-6);
    }
}
