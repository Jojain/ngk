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
//! | [`convert`] | geometry mapping and parameter maps | topology, the map, darts |
//! | [`topology`] | shells, faces, loops, stitching, seams | Part 21 text |
//!
//! Writing works on `impl Write` and reading on `&str`: nothing here touches a
//! filesystem, so the whole stack builds for wasm.
//!
//! ## What is carried so far
//!
//! Solids, both ways, on every support NGK has: `PLANE`,
//! `CYLINDRICAL_SURFACE`, `SPHERICAL_SURFACE`, `CONICAL_SURFACE`,
//! `TOROIDAL_SURFACE`, `SURFACE_OF_REVOLUTION` and `B_SPLINE_SURFACE`; `LINE`,
//! `CIRCLE`, `ELLIPSE` and `B_SPLINE_CURVE` edges; and the AP214 product
//! structure. A face whose parameterization closes on itself is written along
//! a cut synthesized from its domain, and read back by letting healing remove
//! that cut again — a whole sphere or torus included, which NGK stores as one
//! face with no boundary at all and STEP has to be handed cut open. Solids
//! with cavities go both ways as `BREP_WITH_VOIDS`.
//!
//! A B-spline is written in whichever of Part 21's two spellings its weights
//! call for: one record when polynomial, and a complex instance when rational,
//! since a rational B-spline has no keyword of its own.
//!
//! What a file holds beyond one solid's geometry is read for the solids and
//! dropped: an assembly's names, nesting and placements do not survive, and
//! NGK cannot write one.
//!
//! The two directions are not symmetric in what they *have* to do. Writing
//! walks topology that is already sewn; reading is handed loose faces that
//! name shared edges by number, so it has to stitch them and rebuild
//! the parameter curves the file need not carry. It is also best-effort by
//! default — a face it cannot assemble is reported rather than thrown — since
//! real files contain faces that do not close.

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

use crate::model::Model;
use crate::topology::StandardPayload;
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
    map_to_exchange(shape.model(), &[shape.key()], options)
}

/// Writes several solids from one map into a single exchange structure.
///
/// They share one product and one document context, which is as much assembly
/// structure as one product can express. A file holding an assembly of placed
/// parts needs a hierarchy of named solids with transforms, which NGK has
/// nowhere to hold.
pub fn map_to_exchange<P: Payload>(
    gmap: &Model<P>,
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
    /// A `Shape` owns a whole `Model`, so printing the shapes themselves would
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
/// Every `MANIFOLD_SOLID_BREP` and `BREP_WITH_VOIDS` in the file becomes one
/// [`Shape`], found by sweeping for it rather than by walking down from
/// `SHAPE_DEFINITION_REPRESENTATION`: product structure is where vendor files
/// diverge most and none of it is needed to recover the geometry.
///
/// Reading is best-effort by default. A face that cannot be assembled is
/// dropped and recorded in [`StepImport::report`] rather than costing the
/// file; [`StepReadOptions::strict`] turns each of those into an error
/// instead.
///
/// It is deliberately *not* generic over the payload: the importer can only
/// produce `P::F: Default`, and an imported shape has to stay compatible with
/// `modeling::fuse`, which is [`StandardPayload`].
///
/// ```
/// use ngk::exchange::step::{StepReadOptions, StepWriteOptions, read_step, step_to_string};
///
/// let block = ngk::modeling::solids::block(10.0, 20.0, 30.0)?;
/// let text = step_to_string(&block, &StepWriteOptions::named("BLOCK"))?;
///
/// let import = read_step(&text, &StepReadOptions::default())?;
/// assert_eq!(import.shapes.len(), 1);
/// assert!(import.report.is_clean());
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub fn read_step(text: &str, options: &StepReadOptions) -> Result<StepImport, StepError> {
    let exchange = parse_exchange(text)?;
    read_exchange(&exchange, options)
}

/// Reads an already-parsed exchange structure into solids.
///
/// The split exists because L1 is complete on its own: a vendor file can be
/// parsed once, inspected with [`part21::StepExchange`]'s own queries, and
/// only then interpreted.
pub fn read_exchange(
    exchange: &StepExchange,
    options: &StepReadOptions,
) -> Result<StepImport, StepError> {
    topology::import::read_solids(exchange, options)
}
