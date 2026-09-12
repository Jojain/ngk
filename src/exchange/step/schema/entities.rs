//! One Rust type per STEP entity, read and written through one field order.
//!
//! A Part 21 file carries no field names: `#21 = EDGE_CURVE('',#22,#24,#26,.T.);`
//! says nothing about which reference is the start vertex. The ISO 10303
//! schema declares the order, the file relies on the reader knowing it, and
//! so *something* has to turn a position into a meaning.
//!
//! This module is that something, and it is the only place in the STEP stack
//! that does it. Each entity's [`Entity::read`] and [`Entity::record`] walk
//! their attributes in the same order, sequentially, a few lines apart — so
//! the two directions state the schema once between them rather than twice
//! independently, which is what keeps a reader and a writer from drifting
//! into a pair of mappings that disagree. `tests/exchange/step_entities.rs`
//! holds the property that proves it: `read(x.record())` is `x`.
//!
//! ## Reading conventions
//!
//! - **Attributes are consumed in order, never indexed.** The struct's field
//!   order *is* the schema, so restating it as integers would be a second
//!   copy that can disagree with the first.
//! - **Bind with `let` before building `Self`.** Rust evaluates struct-literal
//!   fields in the order they are written, so filling them inline would make
//!   the literal's source order silently load-bearing.
//! - **Skips are named, never counted.** [`Attributes::name`] consumes the
//!   decorative name every entity carries and [`Attributes::derived`] consumes
//!   a `*`. A counted skip shifts everything after it when it is wrong.
//! - **The decorative name is not a field.** Every geometric and topological
//!   entity has a `name` that NGK writes as `''` and reads nothing from, so
//!   modelling it would be noise. It appears as a field only where it
//!   identifies the entity, as `CONVERSION_BASED_UNIT`'s `'INCH'` does.
//! - **Numbers are as the file spells them.** Converting to millimetres and
//!   radians is the resolver's job, so nothing here consults a unit.

use super::super::part21::{EntityId, Record, Value};
use super::resolver::{Attributes, SchemaError};

/// One STEP entity, in both directions.
pub trait Entity: Sized {
    /// The keywords this entity may be spelled with.
    ///
    /// More than one where the schema gives subtypes the same attributes, as
    /// `FACE_BOUND` and `FACE_OUTER_BOUND` have. The entity then records which
    /// spelling it was, so writing it back produces the same one.
    const KEYWORDS: &'static [&'static str];

    /// Decodes the entity from a record already known to carry one of
    /// [`Self::KEYWORDS`].
    fn read(attributes: Attributes<'_>) -> Result<Self, SchemaError>;

    /// Encodes it back into a record.
    fn record(&self) -> Record;
}

/// The decorative name every geometric and topological entity carries.
fn unnamed() -> Value {
    Value::Text(String::new())
}

fn boolean(value: bool) -> Value {
    Value::Enum(if value { "T" } else { "F" }.to_string())
}

fn reals<const N: usize>(values: &[f64; N]) -> Value {
    Value::List(values.iter().copied().map(Value::Real).collect())
}

fn references(ids: &[EntityId]) -> Value {
    Value::List(ids.iter().copied().map(Value::Ref).collect())
}

/// A number and the measure type it was written as, such as
/// `LENGTH_MEASURE(25.4)`.
///
/// STEP states a quantity as a typed parameter rather than a bare real, and
/// the type is what says whether a unit block is talking about a length or an
/// angle — so the two travel together.
#[derive(Debug, Clone, PartialEq)]
pub struct Measure {
    /// The measure keyword, such as `LENGTH_MEASURE`.
    pub kind: String,
    /// The quantity, in whatever unit the surrounding entity names.
    pub value: f64,
}

impl Measure {
    /// Returns a length measure of `value`.
    pub fn length(value: f64) -> Self {
        Self {
            kind: "LENGTH_MEASURE".to_string(),
            value,
        }
    }

    /// Returns this measure as a typed parameter.
    pub fn value(&self) -> Value {
        Value::Typed(Box::new(Record::new(
            self.kind.clone(),
            vec![Value::Real(self.value)],
        )))
    }
}

// ---------------------------------------------------------------- geometry

/// `CARTESIAN_POINT(name, coordinates)`.
///
/// Generic over the dimension so that a 2D point reaching a 3D reader fails
/// on arity rather than on whatever it is mistaken for.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CartesianPoint<const N: usize> {
    /// The position, in the document's own length unit.
    pub coordinates: [f64; N],
}

impl<const N: usize> Entity for CartesianPoint<N> {
    const KEYWORDS: &'static [&'static str] = &["CARTESIAN_POINT"];

    fn read(mut attributes: Attributes<'_>) -> Result<Self, SchemaError> {
        attributes.name()?;
        let coordinates = attributes.reals::<N>()?;
        Ok(Self { coordinates })
    }

    fn record(&self) -> Record {
        Record::new("CARTESIAN_POINT", vec![unnamed(), reals(&self.coordinates)])
    }
}

/// `DIRECTION(name, direction_ratios)`.
///
/// The ratios are not required to be normalized, and carry no unit: a
/// direction is a ratio, so the document's length scale has nothing to say
/// about it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Direction<const N: usize> {
    /// The direction ratios, unnormalized as written.
    pub direction_ratios: [f64; N],
}

impl<const N: usize> Entity for Direction<N> {
    const KEYWORDS: &'static [&'static str] = &["DIRECTION"];

    fn read(mut attributes: Attributes<'_>) -> Result<Self, SchemaError> {
        attributes.name()?;
        let direction_ratios = attributes.reals::<N>()?;
        Ok(Self { direction_ratios })
    }

    fn record(&self) -> Record {
        Record::new("DIRECTION", vec![unnamed(), reals(&self.direction_ratios)])
    }
}

/// `VECTOR(name, orientation, magnitude)`: a direction carrying a length.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector {
    /// The `DIRECTION` it points along.
    pub orientation: EntityId,
    /// Its length, in the document's own length unit.
    pub magnitude: f64,
}

impl Entity for Vector {
    const KEYWORDS: &'static [&'static str] = &["VECTOR"];

    fn read(mut attributes: Attributes<'_>) -> Result<Self, SchemaError> {
        attributes.name()?;
        let orientation = attributes.reference()?;
        let magnitude = attributes.real()?;
        Ok(Self {
            orientation,
            magnitude,
        })
    }

    fn record(&self) -> Record {
        Record::new(
            "VECTOR",
            vec![
                unnamed(),
                Value::Ref(self.orientation),
                Value::Real(self.magnitude),
            ],
        )
    }
}

/// `AXIS2_PLACEMENT_3D(name, location, axis, ref_direction)`.
///
/// Both directions are optional. An unset `axis` means the global z, and an
/// unset `ref_direction` means any direction perpendicular to the axis — the
/// schema constrains the placement to be well formed and leaves the choice to
/// the reader.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Axis2Placement3d {
    /// The origin.
    pub location: EntityId,
    /// The local z, which STEP calls the axis.
    pub axis: Option<EntityId>,
    /// An in-plane reference the local x is taken from.
    pub ref_direction: Option<EntityId>,
}

impl Entity for Axis2Placement3d {
    const KEYWORDS: &'static [&'static str] = &["AXIS2_PLACEMENT_3D"];

    fn read(mut attributes: Attributes<'_>) -> Result<Self, SchemaError> {
        attributes.name()?;
        let location = attributes.reference()?;
        let axis = attributes.optional_reference()?;
        let ref_direction = attributes.optional_reference()?;
        Ok(Self {
            location,
            axis,
            ref_direction,
        })
    }

    fn record(&self) -> Record {
        Record::new(
            "AXIS2_PLACEMENT_3D",
            vec![
                unnamed(),
                Value::Ref(self.location),
                self.axis.map_or(Value::Null, Value::Ref),
                self.ref_direction.map_or(Value::Null, Value::Ref),
            ],
        )
    }
}

/// `LINE(name, pnt, dir)`, parameterized as `pnt + magnitude · dir · t`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Line {
    /// The point at `t = 0`.
    pub pnt: EntityId,
    /// The `VECTOR` whose magnitude scales the parameter.
    pub dir: EntityId,
}

impl Entity for Line {
    const KEYWORDS: &'static [&'static str] = &["LINE"];

    fn read(mut attributes: Attributes<'_>) -> Result<Self, SchemaError> {
        attributes.name()?;
        let pnt = attributes.reference()?;
        let dir = attributes.reference()?;
        Ok(Self { pnt, dir })
    }

    fn record(&self) -> Record {
        Record::new(
            "LINE",
            vec![unnamed(), Value::Ref(self.pnt), Value::Ref(self.dir)],
        )
    }
}

/// `PLANE(name, position)`, whose `(u, v)` are distances along the
/// placement's x and y.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Plane {
    /// The `AXIS2_PLACEMENT_3D` the surface is built on.
    pub position: EntityId,
}

impl Entity for Plane {
    const KEYWORDS: &'static [&'static str] = &["PLANE"];

    fn read(mut attributes: Attributes<'_>) -> Result<Self, SchemaError> {
        attributes.name()?;
        let position = attributes.reference()?;
        Ok(Self { position })
    }

    fn record(&self) -> Record {
        Record::new("PLANE", vec![unnamed(), Value::Ref(self.position)])
    }
}

/// Which subtype of `SURFACE_CURVE` a curve-on-surface was spelled as.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfaceCurveKind {
    /// A curve lying on one or two surfaces.
    Surface,
    /// The cut a periodic face's parameterization was opened along.
    Seam,
    /// A curve defined as the intersection of two surfaces.
    Intersection,
}

impl SurfaceCurveKind {
    fn keyword(self) -> &'static str {
        match self {
            Self::Surface => "SURFACE_CURVE",
            Self::Seam => "SEAM_CURVE",
            Self::Intersection => "INTERSECTION_CURVE",
        }
    }
}

/// `SURFACE_CURVE(name, curve_3d, associated_geometry, master_representation)`,
/// and its `SEAM_CURVE` and `INTERSECTION_CURVE` subtypes.
///
/// A 3D curve with the parameter curves of the surfaces it lies on hung off
/// it. The associated geometry is optional in the schema and OpenCascade
/// writes it for every edge; NGK writes none and rebuilds its own.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceCurve {
    /// Which of the three spellings this was.
    pub kind: SurfaceCurveKind,
    /// The curve in model space.
    pub curve_3d: EntityId,
    /// The `PCURVE`s and surfaces it is associated with.
    pub associated_geometry: Vec<EntityId>,
    /// Which representation the file calls definitive.
    pub master_representation: String,
}

impl Entity for SurfaceCurve {
    const KEYWORDS: &'static [&'static str] =
        &["SURFACE_CURVE", "SEAM_CURVE", "INTERSECTION_CURVE"];

    fn read(mut attributes: Attributes<'_>) -> Result<Self, SchemaError> {
        let kind = if attributes.is("SEAM_CURVE") {
            SurfaceCurveKind::Seam
        } else if attributes.is("INTERSECTION_CURVE") {
            SurfaceCurveKind::Intersection
        } else {
            SurfaceCurveKind::Surface
        };
        attributes.name()?;
        let curve_3d = attributes.reference()?;
        let associated_geometry = attributes.references()?;
        let master_representation = attributes.enumeration()?.to_string();
        Ok(Self {
            kind,
            curve_3d,
            associated_geometry,
            master_representation,
        })
    }

    fn record(&self) -> Record {
        Record::new(
            self.kind.keyword(),
            vec![
                unnamed(),
                Value::Ref(self.curve_3d),
                references(&self.associated_geometry),
                Value::Enum(self.master_representation.clone()),
            ],
        )
    }
}

// ---------------------------------------------------------------- topology

/// `VERTEX_POINT(name, vertex_geometry)`: a corner of a shell.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VertexPoint {
    /// The `CARTESIAN_POINT` it sits at.
    pub vertex_geometry: EntityId,
}

impl Entity for VertexPoint {
    const KEYWORDS: &'static [&'static str] = &["VERTEX_POINT"];

    fn read(mut attributes: Attributes<'_>) -> Result<Self, SchemaError> {
        attributes.name()?;
        let vertex_geometry = attributes.reference()?;
        Ok(Self { vertex_geometry })
    }

    fn record(&self) -> Record {
        Record::new(
            "VERTEX_POINT",
            vec![unnamed(), Value::Ref(self.vertex_geometry)],
        )
    }
}

/// `EDGE_CURVE(name, edge_start, edge_end, edge_geometry, same_sense)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EdgeCurve {
    /// The corner the edge runs from.
    pub edge_start: EntityId,
    /// The corner it runs to.
    pub edge_end: EntityId,
    /// The curve it lies on, which may be a `SURFACE_CURVE` wrapping one.
    pub edge_geometry: EntityId,
    /// Whether the curve's own direction agrees with start → end.
    pub same_sense: bool,
}

impl Entity for EdgeCurve {
    const KEYWORDS: &'static [&'static str] = &["EDGE_CURVE"];

    fn read(mut attributes: Attributes<'_>) -> Result<Self, SchemaError> {
        attributes.name()?;
        let edge_start = attributes.reference()?;
        let edge_end = attributes.reference()?;
        let edge_geometry = attributes.reference()?;
        let same_sense = attributes.boolean()?;
        Ok(Self {
            edge_start,
            edge_end,
            edge_geometry,
            same_sense,
        })
    }

    fn record(&self) -> Record {
        Record::new(
            "EDGE_CURVE",
            vec![
                unnamed(),
                Value::Ref(self.edge_start),
                Value::Ref(self.edge_end),
                Value::Ref(self.edge_geometry),
                boolean(self.same_sense),
            ],
        )
    }
}

/// `ORIENTED_EDGE(name, edge_start, edge_end, edge_element, orientation)`.
///
/// The two vertex attributes are redeclared and derived by the subtype, so
/// the schema requires them to be written as `*` and nothing stores them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OrientedEdge {
    /// The `EDGE_CURVE` this use refers to.
    pub edge_element: EntityId,
    /// Whether this use walks the edge in the edge's own direction.
    pub orientation: bool,
}

impl Entity for OrientedEdge {
    const KEYWORDS: &'static [&'static str] = &["ORIENTED_EDGE"];

    fn read(mut attributes: Attributes<'_>) -> Result<Self, SchemaError> {
        attributes.name()?;
        attributes.derived()?;
        attributes.derived()?;
        let edge_element = attributes.reference()?;
        let orientation = attributes.boolean()?;
        Ok(Self {
            edge_element,
            orientation,
        })
    }

    fn record(&self) -> Record {
        Record::new(
            "ORIENTED_EDGE",
            vec![
                unnamed(),
                Value::Derived,
                Value::Derived,
                Value::Ref(self.edge_element),
                boolean(self.orientation),
            ],
        )
    }
}

/// `EDGE_LOOP(name, edge_list)`: one closed boundary, in traversal order.
#[derive(Debug, Clone, PartialEq)]
pub struct EdgeLoop {
    /// The `ORIENTED_EDGE`s it walks, in order.
    pub edge_list: Vec<EntityId>,
}

impl Entity for EdgeLoop {
    const KEYWORDS: &'static [&'static str] = &["EDGE_LOOP"];

    fn read(mut attributes: Attributes<'_>) -> Result<Self, SchemaError> {
        attributes.name()?;
        let edge_list = attributes.references()?;
        Ok(Self { edge_list })
    }

    fn record(&self) -> Record {
        Record::new("EDGE_LOOP", vec![unnamed(), references(&self.edge_list)])
    }
}

/// `FACE_BOUND(name, bound, orientation)` and its `FACE_OUTER_BOUND` subtype.
///
/// The two carry identical attributes and differ only in saying whether the
/// boundary encloses the face or a hole in it. `FACE_OUTER_BOUND` is optional
/// and some writers use only the base type, so the spelling is recorded
/// rather than relied upon.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FaceBound {
    /// The `EDGE_LOOP` forming the boundary.
    pub bound: EntityId,
    /// Whether the loop is traversed in its own direction.
    pub orientation: bool,
    /// Whether the file spelled this `FACE_OUTER_BOUND`.
    pub outer: bool,
}

impl Entity for FaceBound {
    const KEYWORDS: &'static [&'static str] = &["FACE_OUTER_BOUND", "FACE_BOUND"];

    fn read(mut attributes: Attributes<'_>) -> Result<Self, SchemaError> {
        let outer = attributes.is("FACE_OUTER_BOUND");
        attributes.name()?;
        let bound = attributes.reference()?;
        let orientation = attributes.boolean()?;
        Ok(Self {
            bound,
            orientation,
            outer,
        })
    }

    fn record(&self) -> Record {
        let keyword = if self.outer {
            "FACE_OUTER_BOUND"
        } else {
            "FACE_BOUND"
        };
        Record::new(
            keyword,
            vec![unnamed(), Value::Ref(self.bound), boolean(self.orientation)],
        )
    }
}

/// `ADVANCED_FACE(name, bounds, face_geometry, same_sense)`.
#[derive(Debug, Clone, PartialEq)]
pub struct AdvancedFace {
    /// The `FACE_BOUND`s enclosing the face and its holes.
    pub bounds: Vec<EntityId>,
    /// The support surface.
    pub face_geometry: EntityId,
    /// Whether the face's normal agrees with the surface's.
    pub same_sense: bool,
}

impl Entity for AdvancedFace {
    const KEYWORDS: &'static [&'static str] = &["ADVANCED_FACE"];

    fn read(mut attributes: Attributes<'_>) -> Result<Self, SchemaError> {
        attributes.name()?;
        let bounds = attributes.references()?;
        let face_geometry = attributes.reference()?;
        let same_sense = attributes.boolean()?;
        Ok(Self {
            bounds,
            face_geometry,
            same_sense,
        })
    }

    fn record(&self) -> Record {
        Record::new(
            "ADVANCED_FACE",
            vec![
                unnamed(),
                references(&self.bounds),
                Value::Ref(self.face_geometry),
                boolean(self.same_sense),
            ],
        )
    }
}

/// `CLOSED_SHELL(name, cfs_faces)`: a shell with no boundary.
#[derive(Debug, Clone, PartialEq)]
pub struct ClosedShell {
    /// The faces it is made of.
    pub cfs_faces: Vec<EntityId>,
}

impl Entity for ClosedShell {
    const KEYWORDS: &'static [&'static str] = &["CLOSED_SHELL"];

    fn read(mut attributes: Attributes<'_>) -> Result<Self, SchemaError> {
        attributes.name()?;
        let cfs_faces = attributes.references()?;
        Ok(Self { cfs_faces })
    }

    fn record(&self) -> Record {
        Record::new("CLOSED_SHELL", vec![unnamed(), references(&self.cfs_faces)])
    }
}

/// `MANIFOLD_SOLID_BREP(name, outer)`: a solid bounded by one closed shell.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ManifoldSolidBrep {
    /// The `CLOSED_SHELL` bounding it.
    pub outer: EntityId,
}

impl Entity for ManifoldSolidBrep {
    const KEYWORDS: &'static [&'static str] = &["MANIFOLD_SOLID_BREP"];

    fn read(mut attributes: Attributes<'_>) -> Result<Self, SchemaError> {
        attributes.name()?;
        let outer = attributes.reference()?;
        Ok(Self { outer })
    }

    fn record(&self) -> Record {
        Record::new(
            "MANIFOLD_SOLID_BREP",
            vec![unnamed(), Value::Ref(self.outer)],
        )
    }
}

// ------------------------------------------------------------------- units

/// `SI_UNIT(prefix, name)`.
///
/// The first attribute is the prefix, not a decorative name: this entity is
/// one of the few with no `name` in front of its data.
#[derive(Debug, Clone, PartialEq)]
pub struct SiUnit {
    /// The SI prefix, such as `MILLI`, or `None` for the base unit.
    pub prefix: Option<String>,
    /// The unit name, such as `METRE` or `RADIAN`.
    pub name: String,
}

impl Entity for SiUnit {
    const KEYWORDS: &'static [&'static str] = &["SI_UNIT"];

    fn read(mut attributes: Attributes<'_>) -> Result<Self, SchemaError> {
        let prefix = attributes.optional_enumeration()?.map(str::to_string);
        let name = attributes.enumeration()?.to_string();
        Ok(Self { prefix, name })
    }

    fn record(&self) -> Record {
        Record::new(
            "SI_UNIT",
            vec![
                self.prefix
                    .as_ref()
                    .map_or(Value::Null, |prefix| Value::Enum(prefix.clone())),
                Value::Enum(self.name.clone()),
            ],
        )
    }
}

/// `CONVERSION_BASED_UNIT(name, conversion_factor)`: an inch, or a degree.
///
/// Its name identifies the unit rather than decorating it, so unlike every
/// geometric entity here it is carried.
#[derive(Debug, Clone, PartialEq)]
pub struct ConversionBasedUnit {
    /// What the unit is called, such as `INCH`.
    pub name: String,
    /// The `MEASURE_WITH_UNIT` saying how much of the base unit it is.
    pub conversion_factor: EntityId,
}

impl Entity for ConversionBasedUnit {
    const KEYWORDS: &'static [&'static str] = &["CONVERSION_BASED_UNIT"];

    fn read(mut attributes: Attributes<'_>) -> Result<Self, SchemaError> {
        let name = attributes.text()?;
        let conversion_factor = attributes.reference()?;
        Ok(Self {
            name,
            conversion_factor,
        })
    }

    fn record(&self) -> Record {
        Record::new(
            "CONVERSION_BASED_UNIT",
            vec![
                Value::Text(self.name.clone()),
                Value::Ref(self.conversion_factor),
            ],
        )
    }
}

/// `MEASURE_WITH_UNIT(value_component, unit_component)`: a quantity in a unit.
#[derive(Debug, Clone, PartialEq)]
pub struct MeasureWithUnit {
    /// The quantity.
    pub value_component: Measure,
    /// The unit it is measured in.
    pub unit_component: EntityId,
}

impl Entity for MeasureWithUnit {
    const KEYWORDS: &'static [&'static str] = &["MEASURE_WITH_UNIT"];

    fn read(mut attributes: Attributes<'_>) -> Result<Self, SchemaError> {
        let value_component = attributes.measure()?;
        let unit_component = attributes.reference()?;
        Ok(Self {
            value_component,
            unit_component,
        })
    }

    fn record(&self) -> Record {
        Record::new(
            "MEASURE_WITH_UNIT",
            vec![
                self.value_component.value(),
                Value::Ref(self.unit_component),
            ],
        )
    }
}

/// `UNCERTAINTY_MEASURE_WITH_UNIT(value_component, unit_component, name,
/// description)`: how close two positions must be to count as one.
#[derive(Debug, Clone, PartialEq)]
pub struct UncertaintyMeasureWithUnit {
    /// The tolerance.
    pub value_component: Measure,
    /// The unit it is measured in.
    pub unit_component: EntityId,
    /// What the measure is for, such as `distance_accuracy_value`.
    pub name: String,
    /// A human-readable gloss on it.
    pub description: String,
}

impl Entity for UncertaintyMeasureWithUnit {
    const KEYWORDS: &'static [&'static str] = &["UNCERTAINTY_MEASURE_WITH_UNIT"];

    fn read(mut attributes: Attributes<'_>) -> Result<Self, SchemaError> {
        let value_component = attributes.measure()?;
        let unit_component = attributes.reference()?;
        let name = attributes.text()?;
        let description = attributes.text()?;
        Ok(Self {
            value_component,
            unit_component,
            name,
            description,
        })
    }

    fn record(&self) -> Record {
        Record::new(
            "UNCERTAINTY_MEASURE_WITH_UNIT",
            vec![
                self.value_component.value(),
                Value::Ref(self.unit_component),
                Value::Text(self.name.clone()),
                Value::Text(self.description.clone()),
            ],
        )
    }
}
