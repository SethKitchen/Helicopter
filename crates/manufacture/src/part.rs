//! The [`BuildPart`] trait — the polymorphism boundary for buildable components.
//!
//! Every physical part (blade, hub, mast, swashplate, boom, mount) is sized from
//! the design and can report its material, key dimensions, and the steps to make
//! it. A [`crate::assembly::BuildPackage`] holds them as `Box<dyn BuildPart>` and
//! adds the assembly sequence, so a complete build is just a list of parts plus an
//! order to join them.

/// How a part is sourced — mirrors the cost crate's buildability idea, kept local
/// to avoid a dependency cycle: a part is either made from stock or bought.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    /// Cut/shaped from raw stock.
    RawStock,
    /// Machined / printed / laid-up with tooling.
    Fabricated,
    /// Assembled from purchased sub-parts (bearings etc.).
    Assembled,
    /// Bought outright.
    Purchased,
}

impl Source {
    pub fn label(&self) -> &'static str {
        match self {
            Source::RawStock => "raw-stock",
            Source::Fabricated => "fabricated",
            Source::Assembled => "assembled",
            Source::Purchased => "purchased",
        }
    }
}

/// A buildable part: anything that can be sized, dimensioned, and built.
pub trait BuildPart {
    /// Part name.
    fn name(&self) -> &str;
    /// Suggested material / construction.
    fn material(&self) -> &str;
    /// How it is sourced.
    fn source(&self) -> Source;
    /// Key dimensions as `(label, millimetres)` pairs.
    fn key_dimensions_mm(&self) -> Vec<(&'static str, f64)>;
    /// Step-by-step build (or sourcing) instructions.
    fn build_steps(&self) -> Vec<String>;

    /// The part's bounding box `(L, W, H)` in mm, largest first — the envelope
    /// that must fit a printer's build volume. The default takes the three
    /// largest key dimensions (conservative: an over-estimate only triggers a
    /// split sooner); parts with non-extent key dims can override.
    fn bounding_box_mm(&self) -> (f64, f64, f64) {
        let mut v: Vec<f64> = self
            .key_dimensions_mm()
            .iter()
            .map(|(_, d)| d.abs())
            .collect();
        v.sort_by(|a, b| b.total_cmp(a));
        let l = v.first().copied().unwrap_or(0.0);
        let w = v.get(1).copied().unwrap_or(l);
        let h = v.get(2).copied().unwrap_or(w);
        (l, w, h)
    }
}
