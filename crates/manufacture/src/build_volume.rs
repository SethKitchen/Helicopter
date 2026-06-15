//! Printer **build volumes** — the box a part must fit, or be split to fit.
//!
//! Each fabrication route has a maximum build envelope. A part whose bounding box
//! exceeds it must be printed on a bigger machine (a service bed) or **split** into
//! pieces that fit ([`crate::split`]). Sizes are the published build volumes
//! (sourced per entry), sorted by capability so the planner can pick the smallest
//! adequate one.

/// A printable build envelope (mm).
#[derive(Clone, Copy, Debug)]
pub struct BuildVolume {
    pub name: &'static str,
    pub x_mm: f64,
    pub y_mm: f64,
    pub z_mm: f64,
}

impl BuildVolume {
    /// Axes sorted descending (largest first) — for orientation-independent fit.
    pub fn axes_desc(&self) -> [f64; 3] {
        let mut a = [self.x_mm, self.y_mm, self.z_mm];
        a.sort_by(|p, q| q.total_cmp(p));
        a
    }

    /// Longest build axis, mm.
    pub fn longest_mm(&self) -> f64 {
        self.axes_desc()[0]
    }

    /// Can `bbox` (any orientation) fit in one piece?
    pub fn fits(&self, bbox: (f64, f64, f64)) -> bool {
        let b = sort_desc(bbox);
        let v = self.axes_desc();
        b[0] <= v[0] + 1e-6 && b[1] <= v[1] + 1e-6 && b[2] <= v[2] + 1e-6
    }

    /// How many pieces the part must be cut into to fit (best orientation): the
    /// product of the per-axis split counts. 1 if it already fits.
    pub fn pieces_needed(&self, bbox: (f64, f64, f64)) -> usize {
        let b = sort_desc(bbox);
        let v = self.axes_desc();
        let n = |dim: f64, lim: f64| (dim / lim).ceil().max(1.0) as usize;
        n(b[0], v[0]) * n(b[1], v[1]) * n(b[2], v[2])
    }
}

fn sort_desc(t: (f64, f64, f64)) -> [f64; 3] {
    let mut a = [t.0, t.1, t.2];
    a.sort_by(|p, q| q.total_cmp(p));
    a
}

/// Markforged desktop (Onyx One / **Onyx Pro** / Mark Two): 320 × 132 × 154 mm.
/// Source: Markforged Onyx Pro datasheet.
pub fn onyx_pro() -> BuildVolume {
    BuildVolume {
        name: "Markforged Onyx Pro (in-house)",
        x_mm: 320.0,
        y_mm: 132.0,
        z_mm: 154.0,
    }
}

/// Markforged industrial X-series (X3/X5/X7): 330 × 270 × 200 mm.
pub fn markforged_x7() -> BuildVolume {
    BuildVolume {
        name: "Markforged X7",
        x_mm: 330.0,
        y_mm: 270.0,
        z_mm: 200.0,
    }
}

/// HP Multi Jet Fusion 4200: 380 × 284 × 380 mm (service MJF). Source: HP.
pub fn hp_mjf_4200() -> BuildVolume {
    BuildVolume {
        name: "HP MJF 4200 (service)",
        x_mm: 380.0,
        y_mm: 284.0,
        z_mm: 380.0,
    }
}

/// EOS SLS (P396-class, PA12): 340 × 340 × 600 mm — the tallest common bed.
pub fn eos_sls_pa12() -> BuildVolume {
    BuildVolume {
        name: "EOS SLS PA12 (service)",
        x_mm: 340.0,
        y_mm: 340.0,
        z_mm: 600.0,
    }
}

/// CNC machining — for these small parts the machine travel is effectively
/// unbounded; represented as a generous billet envelope (subtractive, not a print).
pub fn cnc_envelope() -> BuildVolume {
    BuildVolume {
        name: "CNC (service)",
        x_mm: 600.0,
        y_mm: 400.0,
        z_mm: 400.0,
    }
}

/// All build volumes, smallest-capability first (by longest axis then total).
pub fn build_volumes() -> Vec<BuildVolume> {
    let mut v = vec![
        onyx_pro(),
        markforged_x7(),
        hp_mjf_4200(),
        eos_sls_pa12(),
        cnc_envelope(),
    ];
    v.sort_by(|a, b| {
        a.longest_mm()
            .total_cmp(&b.longest_mm())
            .then((a.x_mm * a.y_mm * a.z_mm).total_cmp(&(b.x_mm * b.y_mm * b.z_mm)))
    });
    v
}

/// The smallest build volume that fits `bbox` in one piece (avoids a split), or
/// `None` if nothing in the catalogue does.
pub fn smallest_fitting(bbox: (f64, f64, f64)) -> Option<BuildVolume> {
    build_volumes().into_iter().find(|v| v.fits(bbox))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// DOCUMENTED — the Onyx Pro envelope and a fit check a reader can verify: a
    /// 595 mm blade (recommended design) does NOT fit the 320 mm Onyx Pro, nor the
    /// 380 mm MJF, but DOES fit the 600 mm SLS bed in one piece.
    #[test]
    fn build_volumes_and_blade_fit() {
        assert_eq!(onyx_pro().longest_mm(), 320.0);
        let blade = (595.0, 37.0, 5.0);
        assert!(!onyx_pro().fits(blade));
        assert!(!hp_mjf_4200().fits(blade));
        assert!(eos_sls_pa12().fits(blade)); // 600 mm Z takes it whole
        assert_eq!(
            smallest_fitting(blade).unwrap().name,
            "EOS SLS PA12 (service)"
        );
    }

    #[test]
    fn split_count_matches_overflow() {
        // 595 mm along a 320 mm axis → 2 pieces; small cross-section → no more.
        assert_eq!(onyx_pro().pieces_needed((595.0, 37.0, 5.0)), 2);
        // A part that fits → 1 piece.
        assert_eq!(onyx_pro().pieces_needed((300.0, 100.0, 100.0)), 1);
        // 700 mm → ceil(700/320)=3 pieces.
        assert_eq!(onyx_pro().pieces_needed((700.0, 50.0, 50.0)), 3);
    }
}
