//! The crate's only filesystem access.
//!
//! Everything else in the STEP stack works on `&str` and `impl Write`, so the
//! whole feature builds for wasm where there is no filesystem. These wrappers
//! are the one exception, and they are compiled out for that target rather
//! than stubbed — a `cdylib` that cannot open a file should not pretend to.
//!
//! They add nothing but the path: each is its own function purely so that an
//! I/O failure can name the file it happened to.

use std::fs;
use std::path::Path;

use crate::topology::payload::Payload;
use crate::topology::shape::{Shape, SolidTag};

use super::error::StepError;
use super::options::{StepReadOptions, StepWriteOptions};
use super::part21::{StepExchange, parse_exchange};
use super::{StepImport, read_step, step_to_string};

/// Writes one solid to a STEP file.
///
/// ```no_run
/// use ngk::exchange::step::{StepWriteOptions, write_step_file};
///
/// let block = ngk::modeling::solids::block(10.0, 20.0, 30.0)?;
/// write_step_file("block.step", &block, &StepWriteOptions::named("BLOCK"))?;
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub fn write_step_file<P: Payload>(
    path: impl AsRef<Path>,
    shape: &Shape<SolidTag, P>,
    options: &StepWriteOptions,
) -> Result<(), StepError> {
    let path = path.as_ref();
    let text = step_to_string(shape, options)?;
    fs::write(path, text).map_err(|source| StepError::Io {
        path: path.to_path_buf(),
        source,
    })
}

/// Reads a STEP file into solids.
///
/// ```no_run
/// use ngk::exchange::step::{StepReadOptions, read_step_file};
///
/// let import = read_step_file("part.step", &StepReadOptions::default())?;
/// println!("{} solid(s), {} skipped", import.shapes.len(), import.report.skipped.len());
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub fn read_step_file(
    path: impl AsRef<Path>,
    options: &StepReadOptions,
) -> Result<StepImport, StepError> {
    let text = read_to_string(path)?;
    read_step(&text, options)
}

/// Reads a STEP file into its Part 21 instance table, without interpreting it.
///
/// This one *does* work today: L1 is complete, so a file can be opened, parsed
/// and inspected entity by entity — which is enough to answer what a vendor
/// file actually contains before anything can import it.
///
/// ```no_run
/// use ngk::exchange::step::read_exchange_file;
///
/// let exchange = read_exchange_file("part.step")?;
/// for solid in exchange.instances_of("MANIFOLD_SOLID_BREP") {
///     println!("{} on line {}", solid.id, solid.line);
/// }
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub fn read_exchange_file(path: impl AsRef<Path>) -> Result<StepExchange, StepError> {
    let text = read_to_string(path)?;
    Ok(parse_exchange(&text)?)
}

fn read_to_string(path: impl AsRef<Path>) -> Result<String, StepError> {
    let path = path.as_ref();
    fs::read_to_string(path).map_err(|source| StepError::Io {
        path: path.to_path_buf(),
        source,
    })
}
