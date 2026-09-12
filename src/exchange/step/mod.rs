//! STEP (ISO 10303) interchange.
//!
//! The feature is split into four layers with three seams, so that no single
//! module lexes, resolves references, interprets units, builds geometry and
//! sews topology at once:
//!
//! | Layer | Knows about | Does **not** know about |
//! |---|---|---|
//! | [`part21`] | ISO 10303-21 syntax only | any AP, any entity name, any NGK type |
//! | [`schema`] | entity names, reference resolution, units | NGK types |
//! | [`convert`] | geometry mapping and parameter maps | topology, the GMap, darts |
//! | [`topology`] | shells, faces, loops, stitching, seams | Part 21 text |
//!
//! Writing works on `impl Write` and reading on `&str`: nothing here touches a
//! filesystem, so the whole stack builds for wasm.
//!
//! ## What is written so far
//!
//! Planar solids: `PLANE` supports, `LINE` edges, and the AP214 product
//! structure. A curved support, a periodic face or a cavity is refused by name
//! rather than approximated — see [`error::TopologyError`] and
//! [`error::GeometryError`], whose variants each say which stage closes them.

pub mod builder;
pub mod convert;
pub mod error;
#[cfg(not(target_arch = "wasm32"))]
pub mod fs;
pub mod options;
pub mod part21;
pub mod report;
pub mod schema;
pub mod topology;

use std::io::Write;

use crate::topology::StandardPayload;
use crate::topology::gmap::GMap;
use crate::topology::payload::Payload;
use crate::topology::shape::{Shape, SolidTag};

use builder::InstanceBuilder;
use part21::{StepExchange, exchange_to_string, parse_exchange, write_exchange};

pub use error::{GeometryError, StepError, TopologyError};
pub use options::{StepReadOptions, StepWriteOptions};
pub use report::{ImportReport, ImportSkip, ImportSkipReason};

#[cfg(not(target_arch = "wasm32"))]
pub use fs::{read_exchange_file, read_step_file, write_step_file};

/// Writes one solid as a STEP exchange structure.
///
/// Returns the assembled instance table rather than text, so a caller can
/// inspect what was produced without reparsing it. [`write_step`] renders it.
pub fn solid_to_exchange<P: Payload>(
    shape: &Shape<SolidTag, P>,
    options: &StepWriteOptions,
) -> Result<StepExchange, StepError> {
    map_to_exchange(shape.map(), &[shape.key()], options)
}

/// Writes several solids from one map into a single exchange structure.
///
/// They share one product and one document context, which is as much assembly
/// structure as a file can carry before the neutral document type (D13) lands.
pub fn map_to_exchange<P: Payload>(
    gmap: &GMap<P>,
    solids: &[crate::topology::shape_keys::SolidKey],
    options: &StepWriteOptions,
) -> Result<StepExchange, StepError> {
    let mut builder = InstanceBuilder::new();

    // The context is written first so that the geometry it governs cannot be
    // emitted without it, and the product last so it can name every B-Rep.
    let context = schema::product::write_context(&mut builder, options);

    let mut breps = Vec::with_capacity(solids.len());
    for &solid in solids {
        breps.push(topology::export::write_solid(&mut builder, gmap, solid)?);
    }

    schema::product::write_product(&mut builder, options, context, &breps);

    let header = schema::product::write_header(options);
    Ok(builder.finish(header))
}

/// Writes one solid as STEP text.
pub fn write_step<P: Payload>(
    sink: &mut impl Write,
    shape: &Shape<SolidTag, P>,
    options: &StepWriteOptions,
) -> Result<(), StepError> {
    let exchange = solid_to_exchange(shape, options)?;
    write_exchange(sink, &exchange)?;
    Ok(())
}

/// Writes one solid as STEP text into a string.
pub fn step_to_string<P: Payload>(
    shape: &Shape<SolidTag, P>,
    options: &StepWriteOptions,
) -> Result<String, StepError> {
    Ok(exchange_to_string(&solid_to_exchange(shape, options)?)?)
}

/// What one STEP read produced.
///
/// The solids and what had to be given up on travel together, because neither
/// answers for the file alone: a read that returns six solids is not a success
/// if it also had to drop a face, and the caller has to be able to see both
/// without asking twice.
#[derive(Default)]
pub struct StepImport<P: Payload = StandardPayload> {
    /// The solids the file yielded, in the order it named them.
    pub shapes: Vec<Shape<SolidTag, P>>,
    /// What could not be carried across faithfully.
    pub report: ImportReport,
}

impl<P: Payload> std::fmt::Debug for StepImport<P> {
    /// Reports what came back rather than what is in it.
    ///
    /// A `Shape` owns a whole `GMap`, so printing the shapes themselves would
    /// bury the two numbers a reader of this actually wants.
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("StepImport")
            .field("shapes", &self.shapes.len())
            .field("skipped", &self.report.skipped.len())
            .finish()
    }
}

/// Reads STEP text into solids.
///
/// **Not implemented yet.** The text is parsed as Part 21 first, so a
/// malformed file is reported properly today — with a line number — but a
/// well-formed one then returns [`StepError::NotImplemented`] until stage 3 of
/// `plan/step_interop.md` lands.
///
/// The signature is settled, so code written against it now keeps compiling.
/// It is deliberately *not* generic over the payload: the importer can only
/// produce `P::F: Default`, and an imported shape has to stay compatible with
/// `modeling::fuse`, which is [`StandardPayload`] (D12).
///
/// To inspect a file today, reach for [`part21::parse_exchange`] — or
/// [`read_exchange_file`] — which is complete.
pub fn read_step(text: &str, options: &StepReadOptions) -> Result<StepImport, StepError> {
    // Parsed rather than skipped, so that the errors this *can* answer today
    // are answered rather than hidden behind the one it cannot.
    let exchange = parse_exchange(text)?;
    read_exchange(&exchange, options)
}

/// Reads an already-parsed exchange structure into solids.
///
/// **Not implemented yet** — see [`read_step`].
pub fn read_exchange(
    exchange: &StepExchange,
    options: &StepReadOptions,
) -> Result<StepImport, StepError> {
    let _ = (exchange, options);
    Err(StepError::NotImplemented {
        what: "STEP import",
        stage: 3,
    })
}
