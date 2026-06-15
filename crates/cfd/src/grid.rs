//! Uniform structured grid on the unit square `[0,1]²`.
//!
//! `n` nodes per side, spacing `h = 1/(n−1)`, node `(i,j)` at `(i·h, j·h)`.
//! Scalar fields are flat `Vec<f64>` of length `n²`, row-major: `idx = j·nx + i`
//! (`i` the x-index, `j` the y-index).

/// A uniform node-centred grid on the unit square.
#[derive(Clone, Copy, Debug)]
pub struct Grid {
    /// Nodes in x.
    pub nx: usize,
    /// Nodes in y.
    pub ny: usize,
    /// Node spacing, m (= 1/(n−1) for the unit square).
    pub h: f64,
}

impl Grid {
    /// A square `n × n` grid covering the unit square.
    pub fn square(n: usize) -> Self {
        assert!(n >= 3, "need at least a 3×3 grid");
        Grid { nx: n, ny: n, h: 1.0 / (n as f64 - 1.0) }
    }

    /// Flat index of node `(i, j)`.
    #[inline]
    pub fn idx(&self, i: usize, j: usize) -> usize {
        j * self.nx + i
    }

    /// Number of nodes.
    pub fn len(&self) -> usize {
        self.nx * self.ny
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// x-coordinate of column `i`.
    pub fn x(&self, i: usize) -> f64 {
        i as f64 * self.h
    }

    /// y-coordinate of row `j`.
    pub fn y(&self, j: usize) -> f64 {
        j as f64 * self.h
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unit_square_spacing_and_indexing() {
        let g = Grid::square(5); // h = 1/4
        assert!((g.h - 0.25).abs() < 1e-12);
        assert_eq!(g.len(), 25);
        assert_eq!(g.idx(2, 3), 3 * 5 + 2);
        assert!((g.x(4) - 1.0).abs() < 1e-12 && (g.y(0) - 0.0).abs() < 1e-12);
    }
}
