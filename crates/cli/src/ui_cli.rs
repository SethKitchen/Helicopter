//! `ui` subcommand: build the design-studio data bundle from the recommended
//! design, write it next to the static frontend, and serve the studio locally.
//!
//! The studio (HTML/CSS/JS in `ui/`, Three.js vendored) renders the *validated*
//! geometry the rest of the project produces — it adds no physics, it visualises
//! it. Run from the repo root (like `build`, which writes `build_output/`).

use crate::{ui_export, ui_server};
use std::fs;
use std::path::PathBuf;

pub fn run() {
    println!("helisim — design studio (recommend → export geometry+data → serve)\n");

    // Locate the static frontend. Default to ./ui (repo root); allow override via
    // the 2nd CLI arg so it works from a build dir too.
    let ui_dir = std::env::args()
        .nth(2)
        .filter(|a| a.parse::<u16>().is_err()) // a bare number is a port, not a dir
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("ui"));
    let port: u16 = std::env::args()
        .skip(2)
        .find_map(|a| a.parse::<u16>().ok())
        .unwrap_or(8080);

    if !ui_dir.join("index.html").exists() {
        eprintln!(
            "  could not find the studio frontend at {}/index.html.\n  \
             Run from the repo root, or pass the ui directory: `helisim ui <dir> [port]`.",
            ui_dir.display()
        );
        return;
    }

    println!("  building data bundle from the recommended design…");
    let Some(bundle) = ui_export::build_bundle() else {
        eprintln!("  no design met the constraints — cannot build the studio bundle.");
        return;
    };

    let data_dir = ui_dir.join("data");
    if let Err(e) = fs::create_dir_all(&data_dir) {
        eprintln!("  could not create {}: {e}", data_dir.display());
        return;
    }
    for (name, content) in [
        ("geometry.json", &bundle.geometry),
        ("manifest.json", &bundle.manifest),
    ] {
        let path = data_dir.join(name);
        match fs::write(&path, content) {
            Ok(()) => println!("  wrote {} ({} bytes)", path.display(), content.len()),
            Err(e) => {
                eprintln!("  could not write {}: {e}", path.display());
                return;
            }
        }
    }

    if let Err(e) = ui_server::serve(ui_dir, port) {
        eprintln!("  server error: {e}");
    }
}
