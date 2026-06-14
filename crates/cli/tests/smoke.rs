//! CLI smoke test — every subcommand runs end-to-end without panicking.
//!
//! The CLI is a presentation layer over the validated cores; it has no oracle
//! numbers of its own (the values it prints are the confirmed results already
//! checked in each core crate's tests). So the meaningful test here is that every
//! `helisim <mode>` dispatches and completes — exercising the formatting and the
//! full physics→report path of each command.

use helisim_cli::dispatch;

#[test]
fn every_subcommand_runs() {
    // All documented modes plus the default (empty → validation report).
    let modes = [
        "", "spanwise", "harrington", "study", "forward", "flapping", "trim", "dynamics", "sim",
        "lateral", "coupled", "inflow", "fly", "sas", "attitude", "hover", "mission", "design",
        "build", "unknown-mode-falls-through-to-default",
    ];
    for m in modes {
        dispatch(m);
    }
}
