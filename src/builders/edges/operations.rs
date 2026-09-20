//! Edge operations and their edit-scoped implementations.

use std::collections::{HashMap, HashSet};

use crate::builders::errors::{EdgeCreationError, ModelEditFailure};
use crate::geometry::parameter::NativeParam;
use crate::geometry::{
    Curve, Fraction, Interval, LINEAR_TOLERANCE, NurbsError, Plane, Point3, PointCoincidence,
};
use crate::model::{Cell0, Cell2, Model};
use crate::topology::ModelEdit;
use crate::topology::attributes::{EdgeAttr, VertexAttr};
use crate::topology::edge::Edge;
use crate::topology::edit::{EditKey, ModelEditError};
use crate::topology::embedding::EntityOwner;
use crate::topology::gmap::{Dart, Dim};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, VertexKey};
use radians::Rad64;
use thiserror::Error;

/// What cutting an edge left behind.
///
/// A cut always adds a corner. Whether the edge *separates* depends on whether
/// it already had one: cutting an unmarked edge marks it and creates nothing,
/// while every other cut leaves two edges meeting at the new corner.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EdgeSplit {
    /// Two edges where there was one, meeting at `vertex`.
    ///
    /// `first` keeps the key the edge already had.
    Separated {
        /// The piece before the cut, under the original key.
        first: EdgeKey,
        /// The piece beyond the cut, under a key derived from the original.
        second: EdgeKey,
        /// The corner the two pieces meet at.
        vertex: VertexKey,
    },
    /// One closed edge, now carrying the corner the cut asked for.
    ///
    /// Nothing was created: an unmarked edge already holds the orbit its corner
    /// sits on, so marking it costs no darts and leaves no second edge.
    Marked {
        /// The edge, still under the key it had.
        edge: EdgeKey,
        /// The corner it now carries.
        vertex: VertexKey,
    },
}

impl EdgeSplit {
    /// Returns the corner the cut added.
    pub fn vertex(&self) -> VertexKey {
        match *self {
            Self::Separated { vertex, .. } | Self::Marked { vertex, .. } => vertex,
        }
    }

    /// Returns the edge the cut created, or `None` where it created none.
    pub fn created(&self) -> Option<EdgeKey> {
        match *self {
            Self::Separated { second, .. } => Some(second),
            Self::Marked { .. } => None,
        }
    }

    /// Returns the edge carrying what lies beyond the cut.
    ///
    /// A caller walking an edge and cutting as it goes continues here: the
    /// created edge where the cut separated one, and the same edge where
    /// marking left only one.
    pub fn continuation(&self) -> EdgeKey {
        match *self {
            Self::Separated { second, .. } => second,
            Self::Marked { edge, .. } => edge,
        }
    }

    /// Returns every edge now covering what was cut.
    pub fn edges(&self) -> impl Iterator<Item = EdgeKey> {
        match *self {
            Self::Separated { first, second, .. } => [Some(first), Some(second)],
            Self::Marked { edge, .. } => [Some(edge), None],
        }
        .into_iter()
        .flatten()
    }
}

#[derive(Debug, Error, Clone, PartialEq)]
pub enum EdgeSplitError {
    #[error("edge {edge:?} does not exist")]
    MissingEdge { edge: EdgeKey },
    #[error("edge {edge:?} is connected outside a profile")]
    EdgeNotProfileOnly { edge: EdgeKey },
    #[error("edge {edge:?} belongs to a face; use a face-boundary split builder")]
    EdgeBelongsToFace { edge: EdgeKey },
    #[error("split parameter must be finite, got {parameter}")]
    NonFiniteParameter { parameter: Fraction },
    #[error("split parameter {parameter} is outside edge domain {domain:?}")]
    ParameterOutOfRange {
        parameter: Fraction,
        domain: Interval,
    },
    #[error("split parameter {parameter} is too close to an edge boundary")]
    DegenerateSplit { parameter: Fraction },
    #[error("failed to trim edge {edge:?} at split parameter {parameter}")]
    CurveTrimFailed {
        edge: EdgeKey,
        parameter: NativeParam,
        #[source]
        source: NurbsError,
    },
    #[error("edge split model edit failed")]
    ModelEditFailed(#[source] ModelEditFailure),
}

impl From<ModelEditError> for EdgeSplitError {
    fn from(error: ModelEditError) -> Self {
        Self::ModelEditFailed(ModelEditFailure::new(error))
    }
}

/// Where a cut lands on an edge, resolved in every space that names it.
///
/// A split reads geometry off an edge and then rewires the map, and the view
/// that answers the first borrows the model the second mutates. So the reading
/// happens once, while the view is still in hand, and what it derives travels
/// as one value: the span the edge covers of its support, the cut expressed in
/// that support's own parameters, and the point the new corner takes. A native
/// parameter apart from the span it was resolved against would be the loose
/// pair this exists to avoid.
#[derive(Debug, Clone, Copy)]
struct EdgeCut {
    /// The span the edge covers, in its stored dart's direction.
    interval: Interval,
    /// Where the cut falls in the support's own parameterization.
    native: NativeParam,
    /// The point the corner the cut adds is placed at.
    point: Point3,
}

impl EdgeCut {
    /// Resolves `parameter` -- a fraction of `edge`'s span -- against that span.
    fn new<P: Payload>(edge: Edge<'_, P>, parameter: Fraction) -> Self {
        let interval = edge.parameter_interval();
        let native = interval.at(parameter);
        Self {
            interval,
            native,
            point: edge.curve().point_at(native),
        }
    }
}

/// A cut on a free profile edge, read off the model before it is touched.
struct PreparedFreeEdgeSplit {
    first_dart: Dart,
    second_dart: Dart,
    curve: Curve,
    cut: EdgeCut,
}

/// A cut on an edge a face hangs off, read off the model before it is touched.
struct PreparedAttachedEdgeSplit {
    first_dart: Dart,
    second_dart: Dart,
    curve: Curve,
    cut: EdgeCut,
    edge_darts: Vec<Dart>,
}

/// Adds an isolated edge with the supplied endpoints and curve geometry.
///
/// The new edge contains two vertices joined by an alpha-0 link and remains
/// free in higher dimensions. The curve is stored as provided; this function
/// does not verify that it interpolates `start` and `end`.
///
/// Returns an error when the endpoints coincide within [`LINEAR_TOLERANCE`].
pub fn add_edge<P: Payload>(
    g: &mut Model<P>,
    start: Point3,
    end: Point3,
    curve: Curve,
) -> Result<EdgeKey, EdgeCreationError> {
    g.transaction(|edit| add_edge_edit(edit, start, end, curve))
}

/// Builds an open edge without introducing an independent transaction boundary.
pub(crate) fn add_edge_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    start: Point3,
    end: Point3,
    curve: Curve,
) -> Result<EdgeKey, EdgeCreationError> {
    check_non_coincident_points(start, end)?;
    let d1 = edit.add_dart();
    let d2 = edit.add_dart();
    edit.link(Dim::Zero, d1, d2)?;
    edit.add_vertex(VertexAttr::new(d1, start));
    edit.add_vertex(VertexAttr::new(d2, end));
    Ok(edit.add_edge(EdgeAttr::new(d1, curve)))
}

/// Adds an isolated straight edge between `start` and `end`.
///
/// Returns an error when the endpoints coincide within [`LINEAR_TOLERANCE`].
pub fn add_line<P: Payload>(
    g: &mut Model<P>,
    start: Point3,
    end: Point3,
) -> Result<EdgeKey, EdgeCreationError> {
    g.transaction(|edit| add_edge_edit(edit, start, end, Curve::line(start, end)))
}

/// Splits a profile-only edge at a parameter of its stored curve.
///
/// The original edge key is retained for the first segment. The returned
/// [`EdgeSplit`] also identifies the newly created second segment and the
/// inserted vertex. Existing alpha-1 profile links are preserved on both sides
/// of the split.
///
/// This operation rejects edges attached to faces; use
/// [`crate::builders::faces::split_face_edge`] for face-boundary edges.
pub fn split_edge<P: Payload>(
    g: &mut Model<P>,
    edge: EdgeKey,
    parameter: Fraction,
) -> Result<EdgeSplit, EdgeSplitError> {
    g.transaction(|edit| split_edge_edit(edit, edge, parameter))
}

/// Splits a profile-only edge inside an existing builder transaction.
pub(crate) fn split_edge_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    edge: EdgeKey,
    parameter: Fraction,
) -> Result<EdgeSplit, EdgeSplitError> {
    let split = prepare_profile_edge_split(edit.model(), edge, parameter)?;
    if edit.attribute::<Cell0>(split.first_dart).is_none() {
        return Ok(mark_closed_edge(edit, edge, split.first_dart, split.cut));
    }
    split_edge_with_profile_links(edit, edge, split)
}

pub(crate) fn split_face_boundary_edge<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    edge: EdgeKey,
    parameter: Fraction,
    reversed: bool,
) -> Result<EdgeSplit, EdgeSplitError> {
    let split = prepare_attached_edge_split(edit.model(), edge, parameter)?;
    if edit.attribute::<Cell0>(split.first_dart).is_none() {
        return Ok(mark_closed_edge(edit, edge, split.first_dart, split.cut));
    }
    split_attached_edge_with_profile_links(edit, edge, split, reversed)
}

/// Materializes an unmarked edge's own 0-cell as the corner a cut asked for.
///
/// An unmarked edge already holds the orbit the corner sits on -- its two ends
/// meet there -- so this adds no darts and links nothing. One cut cannot
/// separate such an edge, because there is no second corner to separate it
/// from; it marks it instead, and a later cut on the marked edge is the one
/// that leaves two arcs.
///
/// The cut is honoured by *where the corner is placed*, not by moving anything
/// in the map: the curve is left alone, and the edge's span is derived from the
/// corner afterwards rather than from the support's own domain.
fn mark_closed_edge<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    edge: EdgeKey,
    dart: Dart,
    cut: EdgeCut,
) -> EdgeSplit {
    edit.disown_cell(Dim::Zero, dart);
    let vertex =
        edit.add_vertex_derived_from(vec![EditKey::Edge(edge)], VertexAttr::new(dart, cut.point));
    EdgeSplit::Marked { edge, vertex }
}

fn split_edge_with_profile_links<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    edge: EdgeKey,
    split: PreparedFreeEdgeSplit,
) -> Result<EdgeSplit, EdgeSplitError> {
    let (first_curve, second_curve) = split_curve_at_parameter(edge, &split.curve, split.cut)?;
    let first_mid = edit.add_dart();
    let second_mid = edit.add_dart();

    edit.unlink(Dim::Zero, split.first_dart)?;
    edit.link(Dim::Zero, split.first_dart, first_mid)?;
    edit.link(Dim::Zero, second_mid, split.second_dart)?;
    edit.link(Dim::One, first_mid, second_mid)?;

    let vertex = edit.add_vertex_derived_from(
        vec![EditKey::Edge(edge)],
        VertexAttr::new(first_mid, split.cut.point),
    );
    edit.edge_attr_mut_unchecked(edge).curve = first_curve;
    let second = edit.add_edge_split_from(edge, EdgeAttr::new(second_mid, second_curve));

    Ok(EdgeSplit::Separated {
        first: edge,
        second,
        vertex,
    })
}

fn split_attached_edge_with_profile_links<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    edge: EdgeKey,
    split: PreparedAttachedEdgeSplit,
    reversed: bool,
) -> Result<EdgeSplit, EdgeSplitError> {
    let (mut first_curve, mut second_curve) =
        split_curve_at_parameter(edge, &split.curve, split.cut)?;
    if reversed {
        (first_curve, second_curve) = (second_curve.reversed(), first_curve.reversed());
    }
    let alpha0_pairs = alpha_pairs(edit, &split.edge_darts, Dim::Zero);
    let alpha2_pairs = alpha_pairs(edit, &split.edge_darts, Dim::Two);
    let mid_darts = split
        .edge_darts
        .iter()
        .map(|dart| (*dart, edit.add_dart()))
        .collect::<HashMap<_, _>>();

    for (first, second) in alpha0_pairs {
        let first_mid = mid_darts[&first];
        let second_mid = mid_darts[&second];
        edit.unlink(Dim::Zero, first)?;
        edit.link(Dim::Zero, first, first_mid)?;
        edit.link(Dim::Zero, second, second_mid)?;
        edit.link(Dim::One, first_mid, second_mid)?;
    }

    for (first, second) in alpha2_pairs {
        edit.link(Dim::Two, mid_darts[&first], mid_darts[&second])?;
    }

    let vertex = edit.add_vertex_derived_from(
        vec![EditKey::Edge(edge)],
        VertexAttr::new(mid_darts[&split.first_dart], split.cut.point),
    );
    edit.edge_attr_mut_unchecked(edge).curve = first_curve;
    let second = edit.add_edge_split_from(
        edge,
        EdgeAttr::new(mid_darts[&split.second_dart], second_curve),
    );

    Ok(EdgeSplit::Separated {
        first: edge,
        second,
        vertex,
    })
}

fn prepare_profile_edge_split<P: Payload>(
    g: &Model<P>,
    edge: EdgeKey,
    parameter: Fraction,
) -> Result<PreparedFreeEdgeSplit, EdgeSplitError> {
    if !parameter.is_finite() {
        return Err(EdgeSplitError::NonFiniteParameter { parameter });
    }

    let view = g.edge(edge).ok_or(EdgeSplitError::MissingEdge { edge })?;
    let attr = view.attr();
    let first_dart = attr.dart;
    let second_dart = g.alpha(Dim::Zero, first_dart);
    let cut = EdgeCut::new(view, parameter);
    let split = PreparedFreeEdgeSplit {
        first_dart,
        second_dart,
        curve: attr.curve.clone(),
        cut,
    };

    check_profile_edge(g, edge, first_dart, second_dart)?;
    check_split_parameter(g, parameter, first_dart, cut)?;
    Ok(split)
}

fn prepare_attached_edge_split<P: Payload>(
    g: &Model<P>,
    edge: EdgeKey,
    parameter: Fraction,
) -> Result<PreparedAttachedEdgeSplit, EdgeSplitError> {
    if !parameter.is_finite() {
        return Err(EdgeSplitError::NonFiniteParameter { parameter });
    }

    let view = g.edge(edge).ok_or(EdgeSplitError::MissingEdge { edge })?;
    let attr = view.attr();
    let first_dart = attr.dart;
    let second_dart = g.alpha(Dim::Zero, first_dart);
    let cut = EdgeCut::new(view, parameter);
    let edge_darts = g
        .orbit(first_dart, g.orbit_indices(Dim::One))
        .collect::<Vec<_>>();
    let split = PreparedAttachedEdgeSplit {
        first_dart,
        second_dart,
        curve: attr.curve.clone(),
        cut,
        edge_darts,
    };

    check_attached_edge(g, edge, first_dart, second_dart, &split.edge_darts)?;
    check_split_parameter(g, parameter, first_dart, cut)?;
    Ok(split)
}

fn check_profile_edge<P: Payload>(
    g: &Model<P>,
    edge: EdgeKey,
    first_dart: Dart,
    second_dart: Dart,
) -> Result<(), EdgeSplitError> {
    let edge_darts = g
        .orbit(first_dart, g.orbit_indices(Dim::One))
        .collect::<Vec<_>>();

    if edge_darts.len() != 2 || second_dart == first_dart {
        return Err(EdgeSplitError::EdgeNotProfileOnly { edge });
    }

    if edge_darts
        .iter()
        .any(|dart| g.attribute::<Cell2>(*dart).is_some())
    {
        return Err(EdgeSplitError::EdgeBelongsToFace { edge });
    }

    if [Dim::Two, Dim::Three]
        .into_iter()
        .any(|dim| edge_darts.iter().any(|dart| !g.is_free(*dart, dim)))
    {
        return Err(EdgeSplitError::EdgeNotProfileOnly { edge });
    }

    Ok(())
}

fn check_attached_edge<P: Payload>(
    g: &Model<P>,
    edge: EdgeKey,
    first_dart: Dart,
    second_dart: Dart,
    edge_darts: &[Dart],
) -> Result<(), EdgeSplitError> {
    let edge_dart_set = edge_darts.iter().copied().collect::<HashSet<_>>();
    if edge_darts.len() < 2 || second_dart == first_dart {
        return Err(EdgeSplitError::EdgeNotProfileOnly { edge });
    }

    if edge_darts.iter().any(|dart| {
        !edge_dart_set.contains(&g.alpha(Dim::Zero, *dart)) || !g.is_free(*dart, Dim::Three)
    }) {
        return Err(EdgeSplitError::EdgeNotProfileOnly { edge });
    }

    Ok(())
}
/// Cuts `curve` into the part of the edge's span before the cut and the part
/// beyond it.
///
/// Both ends of the span and the cut itself are native on `curve`, so each
/// piece is named directly. Going through a fraction would have to say a
/// fraction of what, and the edge's span is not the support's own extent: the
/// two agree only on an edge that covers the whole support.
fn split_curve_at_parameter(
    edge: EdgeKey,
    curve: &Curve,
    cut: EdgeCut,
) -> Result<(Curve, Curve), EdgeSplitError> {
    let trim = |interval| {
        curve
            .trimmed_native(interval)
            .map_err(|source| EdgeSplitError::CurveTrimFailed {
                edge,
                parameter: cut.native,
                source,
            })
    };
    Ok((
        trim(Interval::new(cut.interval.start, cut.native))?,
        trim(Interval::new(cut.native, cut.interval.end))?,
    ))
}

fn alpha_pairs<P: Payload>(g: &Model<P>, darts: &[Dart], dim: Dim) -> Vec<(Dart, Dart)> {
    let dart_set = darts.iter().copied().collect::<HashSet<_>>();
    darts
        .iter()
        .filter_map(|dart| {
            let linked = g.alpha(dim, *dart);
            (linked != *dart && dart.id() < linked.id() && dart_set.contains(&linked))
                .then_some((*dart, linked))
        })
        .collect()
}

fn check_split_parameter<P: Payload>(
    g: &Model<P>,
    parameter: Fraction,
    first_dart: Dart,
    cut: EdgeCut,
) -> Result<(), EdgeSplitError> {
    let domain = cut.interval.ordered();

    if !domain.contains(cut.native, LINEAR_TOLERANCE) {
        return Err(EdgeSplitError::ParameterOutOfRange { parameter, domain });
    }

    // An unmarked edge has no corner for a cut to land on, so no parameter of it
    // is degenerate. Its span is the whole support, and the ends of that span
    // are where the curve closes -- which is a place to put a corner like any
    // other, not one already taken. Only an edge that has corners can be cut at
    // one.
    if g.attribute::<Cell0>(first_dart).is_none() {
        return Ok(());
    }

    // Asked in the support's own space rather than in fractions: the tolerance
    // is a distance, and a fraction becomes one only once the span it measures
    // has been applied to it.
    if (cut.native - domain.start).abs() <= LINEAR_TOLERANCE
        || (cut.native - domain.end).abs() <= LINEAR_TOLERANCE
    {
        return Err(EdgeSplitError::DegenerateSplit { parameter });
    }

    Ok(())
}

/// Adds an isolated circular arc edge on `plane`.
///
/// The endpoint positions are sampled from the circle at `start_angle` and
/// `end_angle`. The radius must be positive and finite, both angles must be
/// finite, and the resulting endpoints must not coincide.
pub fn add_arc<P: Payload>(
    g: &mut Model<P>,
    plane: Plane,
    radius: f64,
    start_angle: Rad64,
    end_angle: Rad64,
) -> Result<EdgeKey, EdgeCreationError> {
    g.transaction(|edit| add_arc_edit(edit, plane, radius, start_angle, end_angle))
}

/// Validates and builds an arc inside the caller's active transaction.
pub(crate) fn add_arc_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    plane: Plane,
    radius: f64,
    start_angle: Rad64,
    end_angle: Rad64,
) -> Result<EdgeKey, EdgeCreationError> {
    check_valid_radius(radius)?;

    let circle = Curve::circle(plane, radius);
    let start = circle.point_at(NativeParam::new(start_angle.val()));
    let end = circle.point_at(NativeParam::new(end_angle.val()));
    let curve = if end_angle < start_angle {
        circle.reversed()
    } else {
        circle
    };
    add_edge_edit(edit, start, end, curve)
}

/// Adds a closed, single-edge circle on `plane`.
///
/// The edge has one topological vertex at the plane's positive x-axis and its
/// two darts are alpha-0- and alpha-1-linked to form a closed profile. `radius`
/// must be positive and finite.
pub fn add_circle<P: Payload>(
    g: &mut Model<P>,
    plane: Plane,
    radius: f64,
) -> Result<EdgeKey, EdgeCreationError> {
    g.transaction(|edit| add_circle_edit(edit, plane, radius))
}

/// Builds a closed circular edge inside the caller's active transaction.
pub(crate) fn add_circle_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    plane: Plane,
    radius: f64,
) -> Result<EdgeKey, EdgeCreationError> {
    check_valid_radius(radius)?;
    let d1 = edit.add_dart();
    let d2 = edit.add_dart();
    let curve = Curve::circle(plane, radius);
    edit.link(Dim::Zero, d1, d2)?;
    edit.link(Dim::One, d1, d2)?;
    let key = edit.add_edge(EdgeAttr::new(d1, curve));
    // The point where the circle closes is a fact about how the map is drawn,
    // not a feature of the shape: nothing meets there. It is classified inside
    // the edge, which is what makes an unmarked circle a vertex-free logical
    // edge. Marking it deliberately is a separate operation that promotes it.
    edit.own_cell(Dim::Zero, d1, EntityOwner::Edge(key));
    Ok(key)
}

fn check_non_coincident_points(start: Point3, end: Point3) -> Result<(), EdgeCreationError> {
    if start.coincides(end, LINEAR_TOLERANCE) {
        Err(EdgeCreationError::CoincidentPoints { start, end })
    } else {
        Ok(())
    }
}

fn check_valid_radius(radius: f64) -> Result<(), EdgeCreationError> {
    if radius.is_finite() && radius > 0.0 {
        Ok(())
    } else {
        Err(EdgeCreationError::InvalidRadius { radius })
    }
}

#[cfg(test)]
mod tests {
    use super::{add_line, split_edge_edit};
    use crate::builders::test_support::LineageRecorder;
    use crate::geometry::{Fraction, Point3};
    use crate::model::Model;
    use crate::topology::edit::{EditKey, Origin};
    use crate::topology::payload::StandardPayload;

    #[test]
    fn splitting_an_edge_declares_vertex_and_edge_lineage() {
        let mut model = Model::<StandardPayload>::new();
        let edge = add_line(
            &mut model,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        )
        .expect("source edge should build");
        let mut recorder = LineageRecorder::default();

        let split = model
            .transaction_with_policy(&mut recorder, |edit| {
                split_edge_edit(edit, edge, Fraction::new(0.5))
            })
            .expect("edge split should commit");

        recorder.assert_exact(&[
            (
                EditKey::Vertex(split.vertex()),
                Origin::derived(EditKey::Edge(edge)),
            ),
            (
                EditKey::Edge(split.created().expect("bounded split creates an edge")),
                Origin::Split(EditKey::Edge(edge)),
            ),
        ]);
    }
}
