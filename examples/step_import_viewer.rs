//! Reads STEP files and sends each one straight to the debug viewer.
//!
//! ```text
//! uv run python tests/exchange/foreign/generate/generate_ttt.py
//! cargo run --example step_import_viewer
//! cargo run --example step_import_viewer -- tests/exchange/foreign/files/torus.step
//! ```
//!
//! With no arguments it reads every `.step` file in
//! `tests/exchange/foreign/files/` — both the generated primitives and the Too
//! Tall Toby challenge models, which are real parts rather than shapes ngk
//! could have built itself. With arguments it reads exactly those files.
//!
//! Import then show, and nothing in between: what reaches the viewer is the map
//! as the importer built it, so anything wrong with a face is wrong on screen.
//! Start the viewer first — this talks to it on `127.0.0.1:3941`, or whatever
//! `NGK_DEBUG_VIEWER_PORT` names.

use std::error::Error;
use std::path::PathBuf;

use ngk::exchange::step::{StepReadOptions, read_step_file};
use ngk::viz::debug_viewer::{DebugViewerOptions, show_gmap_with_options};

const DEFAULT_DIR: &str = "tests/exchange/foreign/files";

fn main() -> Result<(), Box<dyn Error>> {
    let files = files_to_show()?;
    if files.is_empty() {
        println!("no .step files found — run the generator, or name a file");
        return Ok(());
    }

    for path in files {
        let name = path.file_stem().map_or_else(
            || path.display().to_string(),
            |stem| stem.to_string_lossy().into_owned(),
        );

        let import = match read_step_file(&path, &StepReadOptions::default()) {
            Ok(import) => import,
            Err(error) => {
                println!("{name:26} refused: {error}");
                continue;
            }
        };

        let Some(shape) = import.shapes.first() else {
            println!(
                "{name:26} no solid survived, {} skipped",
                import.report.skipped.len()
            );
            continue;
        };

        let solid = shape.solid();
        println!(
            "{name:26} {} faces, {} edges, {} vertices, {} skipped",
            solid.faces().len(),
            solid.edges().len(),
            solid.vertices().len(),
            import.report.skipped.len(),
        );

        show_gmap_with_options(
            shape.map(),
            &DebugViewerOptions {
                name,
                ..Default::default()
            },
        )?;
    }
    Ok(())
}

/// The files named on the command line, or every `.step` in the default
/// directory.
///
/// Sorted, so a run is comparable with the one before it.
fn files_to_show() -> Result<Vec<PathBuf>, Box<dyn Error>> {
    let named: Vec<PathBuf> = std::env::args().skip(1).map(PathBuf::from).collect();
    if !named.is_empty() {
        return Ok(named);
    }

    let Ok(entries) = std::fs::read_dir(DEFAULT_DIR) else {
        return Ok(Vec::new());
    };
    let mut files: Vec<PathBuf> = entries
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| {
            path.extension()
                .is_some_and(|extension| extension == "step")
        })
        .collect();
    files.sort();
    Ok(files)
}
