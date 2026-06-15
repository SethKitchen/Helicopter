//! helisim — command-line driver (a thin shell over [`helisim_cli::dispatch`]).
//!
//! Usage:
//!   helisim                 run the full validation report (default)
//!   helisim spanwise        also print the C&T θ=8° spanwise distribution
//!   helisim harrington      run the Harrington FM sweep
//!   helisim study           C_T sensitivity diagnostic
//!   helisim forward         forward-flight sweep: power bucket + rolling moment
//!   helisim flapping        blade flapping: moment→TPP-tilt + 90° phase lag
//!   helisim trim            steady-flight trim (Newton) + hover cross-check
//!   helisim dynamics        hover stability derivatives + modes (instability)
//!   helisim sim             nonlinear time-march vs the linear eigenvalue gate
//!   helisim lateral         lateral-directional oracle + coupled 8-state gate
//!   helisim coupled         nonlinear 8-state march vs the coupled linear gate
//!   helisim inflow          Pitt-Peters dynamic inflow: τ→0 gate + off-axis sign flip
//!   helisim fly             control-input time histories: effectiveness + open-loop divergence
//!   helisim sas             stability augmentation: off-seam design, hover damping, nonlinear hold
//!   helisim attitude        attitude hold: phugoid→LHP, off-seam regulation, hover seam-residual
//!   helisim hover           velocity/position hold: timescale separation + hands-off hover capstone
//!   helisim mission         end-to-end electric hover: power → C-rate → endurance
//!   helisim bms             battery + BMS benchmark: 4-cell trade + protection/SoC/balancing
//!   helisim battery-build   exact pack+BMS shopping list (qty, prices, links) + build steps
//!   helisim charging        charge the pack: 120 V mains + solar, CC/CV time/energy/limits
//!   helisim charge-build    10-yr-life pack (mass-propagated) + 1:1 charging equipment BOM
//!   helisim design          model-scale sizing study: priority vector + disk-loading trade
//!   helisim build           recommend a design → full part list, assembly, STL/DXF export
//!   (safety track) autorotation power-off + acoustics noise feed the design study

fn main() {
    let mode = std::env::args().nth(1).unwrap_or_default();
    helisim_cli::dispatch(&mode);
}
