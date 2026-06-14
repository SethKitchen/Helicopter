//! `mission` subcommand: the end-to-end electric-hover answer, plus a
//! disk-loading sweep that surfaces the power → C-rate → endurance → *thermal*
//! design tension. The thermal column reveals where the temperature limit bites
//! before the C-rate limit in a sustained climb.

use helisim_airfoil::LinearAirfoil;
use helisim_bemt::Config;
use helisim_cell::TheveninCell;
use helisim_mission::{HoverReport, MissionConfig, analyze_climb, analyze_hover};
use helisim_pack::Pack;
use helisim_powertrain::{ConstantEfficiency, Powertrain};
use helisim_rotor::{Operating, Rotor};
use helisim_thermal::{Convective, Cooling, ThermalLimits, ThermalStatus};

fn demo_pack() -> Pack {
    // Realistically sized for this rotor (a 6S3P pack is so oversized the cells
    // never get warm); 6S2P puts the climb near the C-rate rating where the
    // thermal question actually matters.
    Pack::new(Box::new(TheveninCell::samsung_25r()), 6, 2)
}

fn status_str(s: ThermalStatus) -> &'static str {
    match s {
        ThermalStatus::Safe => "safe",
        ThermalStatus::Warning => "WARN",
        ThermalStatus::OverTemp => "OVER-TEMP",
    }
}

pub fn run() {
    let rotor = Rotor::rectangular(2, 0.5, 0.045, 0.0, 0.15);
    let op = Operating::from_rpm(2200.0);
    let af = LinearAirfoil::naca0012();
    let pt = ConstantEfficiency::typical_electric_heli();
    let cooling = Convective::natural_air();
    let limits = ThermalLimits::default();
    let bemt = Config::default();
    // Hot-day ambient to expose the thermal margin.
    let mission = MissionConfig {
        ambient_c: 30.0,
        ..MissionConfig::default()
    };
    let climb_rate = 6.0; // m/s
    let climb_duration = 360.0; // s (sustained 6-minute climb)

    let pack = demo_pack();
    println!("helisim — electric hover mission with thermal (end-to-end, zero deps)\n");
    println!(
        "Rotor: 2 blades, R=0.50 m, c=0.045 m, {:.0} RPM (M_tip={:.2}), NACA0012",
        op.rpm(),
        op.tip_mach(rotor.radius)
    );
    println!(
        "Pack:  {}S{}P Samsung 25R = {} cells, {:.1} V nom, {:.1} Ah, {:.0} Wh, {:.2} kg",
        pack.series,
        pack.parallel,
        pack.cell_count(),
        pack.nominal_voltage(),
        pack.capacity_ah(),
        pack.energy_wh(),
        pack.mass_kg(),
    );
    println!(
        "Driveline η={:.0}%  |  rating {:.0}C  |  cooling {}  |  T_amb {:.0}°C, limits {:.0}/{:.0}°C\n",
        pt.efficiency(0.0) * 100.0,
        pack.continuous_c_rating(),
        cooling.label(),
        mission.ambient_c,
        limits.warn_c,
        limits.max_c,
    );

    let rep = analyze_hover(
        &rotor, &op, &af, &pack, &pt, 3.0, &cooling, limits, &bemt, &mission,
    );
    print_report(&rep);

    println!(
        "\n=== Design-tension sweep: hover + {:.0}-min, {:.0} m/s climb (T_amb {:.0}°C, natural convection) ===",
        climb_duration / 60.0,
        climb_rate,
        mission.ambient_c
    );
    println!(
        "{:>6} {:>7} {:>7} {:>7} {:>8} {:>9} | {:>7} {:>7} {:>9}",
        "mass", "P_mech", "C-rate", "endur", "hovT", "hovStat", "clmbC", "clmbT", "climbStat"
    );
    for &m in &[2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0] {
        let pack = demo_pack();
        let h = analyze_hover(
            &rotor, &op, &af, &pack, &pt, m, &cooling, limits, &bemt, &mission,
        );
        if !h.hover_feasible {
            println!("{m:>5.1}kg   cannot hover");
            continue;
        }
        let pack = demo_pack();
        let c = analyze_climb(
            &rotor,
            &op,
            &af,
            &pack,
            &pt,
            m,
            climb_rate,
            climb_duration,
            &cooling,
            limits,
            &bemt,
            &mission,
        );
        let crating = if c.within_c_rating {
            format!("{:.1}C", c.cell_c_rate)
        } else {
            format!("{:.1}C!", c.cell_c_rate)
        };
        println!(
            "{:>5.1}kg {:>6.0}W {:>6.2}C {:>6.1}m {:>7.0}°C {:>9} | {:>7} {:>6.0}°C {:>9}",
            m,
            h.mech_power_w,
            h.hover_cell_c_rate,
            h.endurance.endurance_min,
            h.hover_peak_temp_c,
            status_str(h.hover_thermal_status),
            crating,
            c.peak_temp_c,
            status_str(c.thermal_status),
        );
    }
    println!(
        "\nInsight: hover stays cool and within rating across the range; the sustained\n\
         climb is where the cell temperature limit bites — often while still inside the\n\
         C-rate rating. That is the safety answer the tool could not give before."
    );
}

fn print_report(r: &HoverReport) {
    println!("=== Hover analysis @ {:.1} kg gross ===", r.gross_mass_kg);
    if !r.hover_feasible {
        println!("  INFEASIBLE: rotor cannot be trimmed to hover this mass at this RPM.");
        return;
    }
    println!(
        "  thrust {:.1} N   collective {:.2}°   FM {:.3}",
        r.required_thrust_n, r.collective_deg, r.figure_of_merit
    );
    println!(
        "  power  {:.0} W mech -> {:.0} W elec   pack {:.1} V, {:.1} A",
        r.mech_power_w, r.elec_power_w, r.hover_pack_voltage, r.hover_pack_current
    );
    println!(
        "  C-rate {:.2}C / {:.0}C rated -> {}   |   endurance {:.1} min",
        r.hover_cell_c_rate,
        r.continuous_c_rating,
        if r.within_continuous_rating {
            "within"
        } else {
            "OVER"
        },
        r.endurance.endurance_min,
    );
    println!(
        "  hover peak cell temp {:.0}°C -> {}",
        r.hover_peak_temp_c,
        status_str(r.hover_thermal_status)
    );
}
