//! The one entity Part 21 spells two ways.
//!
//! ISO 10303 gives a rational B-spline no keyword of its own. It is the
//! intersection of `B_SPLINE_CURVE_WITH_KNOTS` and `RATIONAL_B_SPLINE_CURVE`,
//! and Part 21 writes an intersection as a **complex instance**: several
//! records under one name, each carrying only the attributes declared on its
//! own supertype. The polynomial case is a leaf type and writes one record
//! carrying every inherited attribute as well.
//!
//! ```text
//! #7 = B_SPLINE_CURVE_WITH_KNOTS('',3,(#8,#9,#10,#11),.UNSPECIFIED.,.F.,.F.,
//!        (4,4),(0.,1.),.UNSPECIFIED.);
//!
//! #7 = ( BOUNDED_CURVE() B_SPLINE_CURVE(3,(#8,#9,#10,#11),.UNSPECIFIED.,.F.,.F.)
//!        B_SPLINE_CURVE_WITH_KNOTS((4,4),(0.,1.),.UNSPECIFIED.) CURVE()
//!        GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE((1.,.8,.8,1.))
//!        REPRESENTATION_ITEM('') );
//! ```
//!
//! So the keyword `B_SPLINE_CURVE_WITH_KNOTS` heads a nine-attribute record in
//! one spelling and a three-attribute record in the other, and which it is is a
//! property of the *instance*, not of the record. That is why neither public
//! type here implements [`Entity`](super::entities::Entity): that trait reads
//! one record and writes one record, and cannot tell which spelling it is in.
//!
//! **What replaces it is [`Fragment`], and D15 survives intact.** Look at the
//! two spellings again and the shared structure is the point: both are the same
//! attribute slices in the same order, once written end to end after a name and
//! once each under its own supertype's keyword. So a slice — not a record — is
//! the unit that states an attribute order, and each spelling is a composition
//! of the same slices. Every keyword, every attribute order and every default
//! is written **once**, and the two spellings cannot disagree because neither
//! spells anything out on its own.
//!
//! The records of a complex instance go in alphabetical order of keyword, which
//! Part 21 requires; the supertypes that declare no attributes contribute an
//! empty record each and are the only keywords named as bare strings, because
//! there is nothing for them to be the keyword *of*.
//!
//! Nothing here converts anything: the values are as the file spells them,
//! knots still run-length coded and control points still unresolved references.
//! [`super::super::convert::nurbs`] is where they become geometry.

use super::super::part21::{EntityId, Instance, Record, Value};
use super::resolver::{Attributes, Origin, SchemaError};

/// One supertype's own attributes, which Part 21 writes in two places.
///
/// Not an [`Entity`](super::entities::Entity): an entity is a whole record, and
/// a fragment is a *slice* of one. The difference is in the signatures and it
/// is the whole point — [`read`](Self::read) continues a cursor rather than
/// consuming a record, and [`params`](Self::params) yields attributes rather
/// than a record — because that is what lets the same slice be read inline in a
/// leaf type's record and as a record of its own in a complex instance.
trait Fragment: Sized {
    /// The supertype whose record carries these attributes in a complex
    /// instance.
    const KEYWORD: &'static str;

    /// Reads the attributes from wherever the cursor has reached.
    fn read(attributes: &mut Attributes<'_>) -> Result<Self, SchemaError>;

    /// The attributes, in the order [`read`](Self::read) expects them.
    fn params(&self) -> Vec<Value>;
}

/// Reads a fragment from the record a complex instance carries it in.
fn fragment<F: Fragment>(origin: Origin, instance: &Instance) -> Result<F, SchemaError> {
    let record = instance
        .record(F::KEYWORD)
        .ok_or_else(|| SchemaError::UnreadableUnit {
            origin,
            detail: format!("a complex instance with no {} record", F::KEYWORD),
        })?;
    F::read(&mut Attributes::new(origin, record))
}

/// The record a fragment is written as inside a complex instance.
fn record<F: Fragment>(fragment: &F) -> Record {
    Record::new(F::KEYWORD, fragment.params())
}

/// The empty record a supertype declaring no attributes contributes.
fn marker(keyword: &str) -> Record {
    Record::new(keyword, Vec::new())
}

/// The `REPRESENTATION_ITEM` record, which is where a complex instance's name
/// lives.
fn representation_item() -> Record {
    Record::new("REPRESENTATION_ITEM", vec![name()])
}

// -------------------------------------------------------------------- curves

/// `B_SPLINE_CURVE`'s own attributes.
#[derive(Debug, Clone, PartialEq)]
pub struct CurveSpline {
    /// The degree, which with the control-point count fixes the knot count.
    pub degree: usize,
    /// The control points, in order.
    pub control_points: Vec<EntityId>,
    /// Whether the writer declared the curve closed.
    pub closed: bool,
}

impl Fragment for CurveSpline {
    const KEYWORD: &'static str = "B_SPLINE_CURVE";

    fn read(attributes: &mut Attributes<'_>) -> Result<Self, SchemaError> {
        let degree = degree(attributes)?;
        let control_points = attributes.references()?;
        attributes.enumeration()?;
        let closed = attributes.boolean()?;
        attributes.boolean()?;
        Ok(Self {
            degree,
            control_points,
            closed,
        })
    }

    fn params(&self) -> Vec<Value> {
        vec![
            Value::Integer(self.degree as i64),
            references(&self.control_points),
            unspecified(),
            boolean(self.closed),
            boolean(false),
        ]
    }
}

/// `B_SPLINE_CURVE_WITH_KNOTS`'s own attributes.
///
/// This is also the leaf type, so its keyword heads the simple spelling.
#[derive(Debug, Clone, PartialEq)]
pub struct CurveKnots {
    /// How many times each distinct knot repeats.
    pub multiplicities: Vec<i64>,
    /// The distinct knot values, increasing.
    pub knots: Vec<f64>,
}

impl Fragment for CurveKnots {
    const KEYWORD: &'static str = "B_SPLINE_CURVE_WITH_KNOTS";

    fn read(attributes: &mut Attributes<'_>) -> Result<Self, SchemaError> {
        let multiplicities = attributes.integers()?;
        let knots = attributes.real_list()?;
        attributes.enumeration()?;
        Ok(Self {
            multiplicities,
            knots,
        })
    }

    fn params(&self) -> Vec<Value> {
        vec![
            integers(&self.multiplicities),
            reals(&self.knots),
            unspecified(),
        ]
    }
}

/// `RATIONAL_B_SPLINE_CURVE`'s own attributes: one weight per control point.
#[derive(Debug, Clone, PartialEq)]
pub struct CurveWeights {
    /// The weights, in control-point order.
    pub weights: Vec<f64>,
}

impl Fragment for CurveWeights {
    const KEYWORD: &'static str = "RATIONAL_B_SPLINE_CURVE";

    fn read(attributes: &mut Attributes<'_>) -> Result<Self, SchemaError> {
        Ok(Self {
            weights: attributes.real_list()?,
        })
    }

    fn params(&self) -> Vec<Value> {
        vec![reals(&self.weights)]
    }
}

/// A B-spline curve as the file states it.
///
/// Run-length coded knots and unresolved control-point references: this is the
/// record, not the curve.
#[derive(Debug, Clone, PartialEq)]
pub struct BSplineCurve {
    /// Degree, control points and the closed flag.
    pub spline: CurveSpline,
    /// The knot vector, run-length coded.
    pub knots: CurveKnots,
    /// The weights, or `None` where the curve is polynomial.
    ///
    /// The `Option` *is* the spelling: `Some` is the complex instance and
    /// `None` the simple record, so a caller cannot write weights into a form
    /// that has nowhere to put them.
    pub weights: Option<CurveWeights>,
}

impl BSplineCurve {
    /// Reads either spelling, or declines an instance that is neither.
    pub fn read(origin: Origin, instance: &Instance) -> Option<Result<Self, SchemaError>> {
        match instance.simple() {
            Some(leaf) => leaf
                .is(CurveKnots::KEYWORD)
                .then(|| Self::read_simple(Attributes::new(origin, leaf))),
            None => (instance.is(CurveSpline::KEYWORD) && instance.is(CurveKnots::KEYWORD))
                .then(|| Self::read_complex(origin, instance)),
        }
    }

    /// The leaf type's record: a name, then every slice end to end.
    fn read_simple(mut attributes: Attributes<'_>) -> Result<Self, SchemaError> {
        attributes.name()?;
        Ok(Self {
            spline: CurveSpline::read(&mut attributes)?,
            knots: CurveKnots::read(&mut attributes)?,
            weights: None,
        })
    }

    /// The same slices, each read from the record of the supertype declaring it.
    fn read_complex(origin: Origin, instance: &Instance) -> Result<Self, SchemaError> {
        Ok(Self {
            spline: fragment(origin, instance)?,
            knots: fragment(origin, instance)?,
            weights: Some(fragment(origin, instance)?),
        })
    }

    /// The records this curve is written as: one when polynomial, seven when
    /// rational.
    pub fn records(&self) -> Vec<Record> {
        let Some(weights) = &self.weights else {
            return vec![leaf::<CurveKnots>(&[
                self.spline.params(),
                self.knots.params(),
            ])];
        };
        vec![
            marker("BOUNDED_CURVE"),
            record(&self.spline),
            record(&self.knots),
            marker("CURVE"),
            marker("GEOMETRIC_REPRESENTATION_ITEM"),
            record(weights),
            representation_item(),
        ]
    }
}

// ------------------------------------------------------------------ surfaces

/// `B_SPLINE_SURFACE`'s own attributes.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceSpline {
    /// The degree along `u`.
    pub degree_u: usize,
    /// The degree along `v`.
    pub degree_v: usize,
    /// The control points, `[u][v]` — the transposition NGK's flat row-major
    /// control net has to be read into.
    pub control_points: Vec<Vec<EntityId>>,
    /// Whether the writer declared the surface closed along `u`.
    pub closed_u: bool,
    /// Whether the writer declared it closed along `v`.
    pub closed_v: bool,
}

impl Fragment for SurfaceSpline {
    const KEYWORD: &'static str = "B_SPLINE_SURFACE";

    fn read(attributes: &mut Attributes<'_>) -> Result<Self, SchemaError> {
        let degree_u = degree(attributes)?;
        let degree_v = degree(attributes)?;
        let control_points = reference_grid(attributes)?;
        attributes.enumeration()?;
        let closed_u = attributes.boolean()?;
        let closed_v = attributes.boolean()?;
        attributes.boolean()?;
        Ok(Self {
            degree_u,
            degree_v,
            control_points,
            closed_u,
            closed_v,
        })
    }

    fn params(&self) -> Vec<Value> {
        vec![
            Value::Integer(self.degree_u as i64),
            Value::Integer(self.degree_v as i64),
            Value::List(
                self.control_points
                    .iter()
                    .map(|row| references(row))
                    .collect(),
            ),
            unspecified(),
            boolean(self.closed_u),
            boolean(self.closed_v),
            boolean(false),
        ]
    }
}

/// `B_SPLINE_SURFACE_WITH_KNOTS`'s own attributes, and the leaf type's keyword.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceKnots {
    /// How many times each distinct `u` knot repeats.
    pub multiplicities_u: Vec<i64>,
    /// How many times each distinct `v` knot repeats.
    pub multiplicities_v: Vec<i64>,
    /// The distinct `u` knot values, increasing.
    pub knots_u: Vec<f64>,
    /// The distinct `v` knot values, increasing.
    pub knots_v: Vec<f64>,
}

impl Fragment for SurfaceKnots {
    const KEYWORD: &'static str = "B_SPLINE_SURFACE_WITH_KNOTS";

    fn read(attributes: &mut Attributes<'_>) -> Result<Self, SchemaError> {
        let multiplicities_u = attributes.integers()?;
        let multiplicities_v = attributes.integers()?;
        let knots_u = attributes.real_list()?;
        let knots_v = attributes.real_list()?;
        attributes.enumeration()?;
        Ok(Self {
            multiplicities_u,
            multiplicities_v,
            knots_u,
            knots_v,
        })
    }

    fn params(&self) -> Vec<Value> {
        vec![
            integers(&self.multiplicities_u),
            integers(&self.multiplicities_v),
            reals(&self.knots_u),
            reals(&self.knots_v),
            unspecified(),
        ]
    }
}

/// `RATIONAL_B_SPLINE_SURFACE`'s own attributes: one weight per control point.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceWeights {
    /// The weights, `[u][v]`, in the control net's own order.
    pub weights: Vec<Vec<f64>>,
}

impl Fragment for SurfaceWeights {
    const KEYWORD: &'static str = "RATIONAL_B_SPLINE_SURFACE";

    fn read(attributes: &mut Attributes<'_>) -> Result<Self, SchemaError> {
        Ok(Self {
            weights: real_grid(attributes)?,
        })
    }

    fn params(&self) -> Vec<Value> {
        vec![Value::List(
            self.weights.iter().map(|row| reals(row)).collect(),
        )]
    }
}

/// A B-spline surface as the file states it.
#[derive(Debug, Clone, PartialEq)]
pub struct BSplineSurface {
    /// Degrees, control points and the two closed flags.
    pub spline: SurfaceSpline,
    /// The two knot vectors, run-length coded.
    pub knots: SurfaceKnots,
    /// The weights, or `None` where the surface is polynomial.
    pub weights: Option<SurfaceWeights>,
}

impl BSplineSurface {
    /// Reads either spelling, or declines an instance that is neither.
    pub fn read(origin: Origin, instance: &Instance) -> Option<Result<Self, SchemaError>> {
        match instance.simple() {
            Some(leaf) => leaf
                .is(SurfaceKnots::KEYWORD)
                .then(|| Self::read_simple(Attributes::new(origin, leaf))),
            None => (instance.is(SurfaceSpline::KEYWORD) && instance.is(SurfaceKnots::KEYWORD))
                .then(|| Self::read_complex(origin, instance)),
        }
    }

    /// The leaf type's record: a name, then every slice end to end.
    fn read_simple(mut attributes: Attributes<'_>) -> Result<Self, SchemaError> {
        attributes.name()?;
        Ok(Self {
            spline: SurfaceSpline::read(&mut attributes)?,
            knots: SurfaceKnots::read(&mut attributes)?,
            weights: None,
        })
    }

    /// The same slices, each read from the record of the supertype declaring it.
    fn read_complex(origin: Origin, instance: &Instance) -> Result<Self, SchemaError> {
        Ok(Self {
            spline: fragment(origin, instance)?,
            knots: fragment(origin, instance)?,
            weights: Some(fragment(origin, instance)?),
        })
    }

    /// The records this surface is written as: one when polynomial, seven when
    /// rational.
    pub fn records(&self) -> Vec<Record> {
        let Some(weights) = &self.weights else {
            return vec![leaf::<SurfaceKnots>(&[
                self.spline.params(),
                self.knots.params(),
            ])];
        };
        vec![
            marker("BOUNDED_SURFACE"),
            record(&self.spline),
            record(&self.knots),
            marker("GEOMETRIC_REPRESENTATION_ITEM"),
            record(weights),
            representation_item(),
            marker("SURFACE"),
        ]
    }
}

// ------------------------------------------------------------------- helpers

/// The leaf type's single record: its name, then every slice end to end.
///
/// Its keyword is the deepest fragment's, because the leaf type *is* that
/// supertype — `b_spline_curve_with_knots` names both the record a complex
/// instance carries its knots in and the whole entity when written simply.
fn leaf<F: Fragment>(slices: &[Vec<Value>]) -> Record {
    let mut params = vec![name()];
    for slice in slices {
        params.extend(slice.iter().cloned());
    }
    Record::new(F::KEYWORD, params)
}

/// Reads a degree, which the schema declares as an integer and NGK as a count.
fn degree(attributes: &mut Attributes<'_>) -> Result<usize, SchemaError> {
    let degree = attributes.integer()?;
    usize::try_from(degree).map_err(|_| SchemaError::UnreadableUnit {
        origin: attributes.origin,
        detail: format!("a degree of {degree}"),
    })
}

/// Reads a list of lists of references, the control net's own shape.
fn reference_grid(attributes: &mut Attributes<'_>) -> Result<Vec<Vec<EntityId>>, SchemaError> {
    let origin = attributes.origin;
    let rows = attributes.list()?;
    rows.iter()
        .map(|row| {
            row.as_list()
                .and_then(|row| row.iter().map(Value::as_reference).collect())
                .ok_or_else(|| SchemaError::UnreadableUnit {
                    origin,
                    detail: "a control-point list of lists of references".to_string(),
                })
        })
        .collect()
}

/// Reads a list of lists of reals, the weight grid's own shape.
fn real_grid(attributes: &mut Attributes<'_>) -> Result<Vec<Vec<f64>>, SchemaError> {
    let origin = attributes.origin;
    let rows = attributes.list()?;
    rows.iter()
        .map(|row| {
            row.as_list()
                .and_then(|row| row.iter().map(Value::as_real).collect())
                .ok_or_else(|| SchemaError::UnreadableUnit {
                    origin,
                    detail: "a weight list of lists of reals".to_string(),
                })
        })
        .collect()
}

/// The decorative name, written `''` and read for nothing.
fn name() -> Value {
    Value::Text(String::new())
}

/// The `curve_form` / `surface_form` and `knot_spec` every B-spline NGK writes
/// declares.
///
/// A writer may name a form it recognized — `POLYLINE_FORM`, `CIRCULAR_ARC` —
/// and a reader gains nothing from it, since the knots and control points say
/// the same thing exactly. So it is consumed and discarded on the way in, and
/// declared unspecified on the way out.
fn unspecified() -> Value {
    Value::Enum("UNSPECIFIED".to_string())
}

fn boolean(value: bool) -> Value {
    Value::Enum(if value { "T" } else { "F" }.to_string())
}

fn references(ids: &[EntityId]) -> Value {
    Value::List(ids.iter().copied().map(Value::Ref).collect())
}

fn reals(values: &[f64]) -> Value {
    Value::List(values.iter().copied().map(Value::Real).collect())
}

fn integers(values: &[i64]) -> Value {
    Value::List(values.iter().copied().map(Value::Integer).collect())
}
