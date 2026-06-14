//! Coupled constant-power pack-current solve.
//!
//! A hovering rotor (with RPM held by the controller) is a roughly *constant
//! power* load. The pack must supply `p_elec = V * I`, but the terminal voltage
//! `V = OCV(soc) - I * R(soc)` itself depends on the current — so `I` and `V` are
//! coupled. This is the electrical twin of the BEMT inflow problem, and is
//! solved the same way: bisection on a monotone residual.
//!
//! Substituting gives `R*I^2 - OCV*I + p_elec = 0`, a downward parabola in the
//! delivered power `V*I` that peaks at the matched-load current `OCV/(2R)`. For
//! `p_elec` below the peak there are two roots; the physical one is the
//! lower-current / higher-voltage root, found by bisecting on `[0, OCV/(2R)]`
//! where the power-balance residual is monotone increasing.

use helisim_pack::Pack;

/// The solved electrical operating point of the pack.
#[derive(Clone, Copy, Debug)]
pub struct ElectricalState {
    /// Pack current, amps.
    pub pack_current: f64,
    /// Pack terminal voltage under load, volts.
    pub terminal_voltage: f64,
}

/// Solve for the pack current that delivers `p_elec` watts at state of charge
/// `soc`. Returns `None` if the demand exceeds the pack's matched-load maximum
/// power `OCV^2/(4R)` (no real solution — the pack cannot source that power).
pub fn solve_pack_current(pack: &Pack, soc: f64, p_elec: f64) -> Option<ElectricalState> {
    if p_elec <= 0.0 {
        return Some(ElectricalState {
            pack_current: 0.0,
            terminal_voltage: pack.ocv(soc),
        });
    }
    let ocv = pack.ocv(soc);
    let r = pack.internal_resistance(soc);

    // Feasibility: delivered power peaks at OCV^2/(4R).
    if p_elec > ocv * ocv / (4.0 * r) {
        return None;
    }

    // Bisection on the physical (low-current) branch, I in [0, OCV/(2R)].
    // residual(I) = V(I)*I - p_elec, V(I) = OCV - I*R; monotone increasing here.
    let mut lo = 0.0;
    let mut hi = ocv / (2.0 * r);
    let residual = |i: f64| (ocv - i * r) * i - p_elec;
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        let f = residual(mid);
        if f.abs() < 1e-9 || (hi - lo) < 1e-12 {
            lo = mid;
            break;
        }
        if f < 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let pack_current = 0.5 * (lo + hi);
    Some(ElectricalState {
        pack_current,
        terminal_voltage: ocv - pack_current * r,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use helisim_cell::TheveninCell;

    fn pack() -> Pack {
        Pack::new(Box::new(TheveninCell::samsung_25r()), 6, 2)
    }

    #[test]
    fn power_balance_holds() {
        let p = pack();
        let st = solve_pack_current(&p, 0.8, 500.0).unwrap();
        assert!((st.terminal_voltage * st.pack_current - 500.0).abs() < 1e-3);
        // Terminal voltage must equal OCV - I*R.
        let v = p.ocv(0.8) - st.pack_current * p.internal_resistance(0.8);
        assert!((v - st.terminal_voltage).abs() < 1e-9);
    }

    #[test]
    fn infeasible_above_max_power() {
        let p = pack();
        let pmax = p.max_power(0.8);
        assert!(solve_pack_current(&p, 0.8, pmax * 1.01).is_none());
        assert!(solve_pack_current(&p, 0.8, pmax * 0.5).is_some());
    }

    #[test]
    fn higher_power_draws_more_current() {
        let p = pack();
        let a = solve_pack_current(&p, 0.8, 200.0).unwrap();
        let b = solve_pack_current(&p, 0.8, 400.0).unwrap();
        assert!(b.pack_current > a.pack_current);
        assert!(b.terminal_voltage < a.terminal_voltage); // more sag
    }
}
