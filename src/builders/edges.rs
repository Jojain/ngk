use std::collections::{HashMap, HashSet};

use crate::builders::errors::EdgeCreationError;
use crate::geometry::axis::Axis3;
use crate::geometry::{Circle, Curve, Interval, LINEAR_TOLERANCE, Plane, Point3, PointCoincidence};
use crate::topology::attributes::{EdgeAttr, VertexAttr};
use crate::topology::gmap::{Cell0, Cell2, Dart, Dim, GMap};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, VertexKey};
use thiserror::Error;

#[derive(Debug, Clone, PartialEq)]
pub struct EdgeSplit {
    pub first: EdgeKey,
    pub second: EdgeKey,
    pub vertex: VertexKey,
}

#[derive(Debug, Error, Clone, PartialEq)]
pub enum EdgeSplitError {
    #[error("edge {edge:?} does not exist")]
    MissingEdge { edge: EdgeKey },
    #[error("edge {edge:?} is connected outside a profile")]
    EdgeNotProfileOnly { edge: EdgeKey },
    #[error("edge {edge:?} belongs to a face; use a face-boundary split builder")]
    EdgeBelongsToFace { edge: EdgeKey },
    #[error("edge {edge:?} has missing endpoint geometry")]
    MissingEndpointGeometry { edge: EdgeKey },
    #[error("split parameter must be finite, got {parameter}")]
    NonFiniteParameter { parameter: f64 },
    #[error("split parameter {parameter} is outside edge domain {domain:?}")]
    ParameterOutOfRange { parameter: f64, domain: Interval },
    #[error("split parameter {parameter} is too close to an edge boundary")]
    DegenerateSplit { parameter: f64 },
}

struct PreparedFreeEdgeSplit {
    first_dart: Dart,
    second_dart: Dart,
    curve: Curve,
}

struct PreparedAttachedEdgeSplit {
    first_dart: Dart,
    second_dart: Dart,
    curve: Curve,
    edge_darts: Vec<Dart>,
}

pub fn add_edge<P: Payload>(
    g: &mut GMap<P>,
    start: Point3,
    end: Point3,
    curve: Curve,
) -> Result<(Dart, EdgeKey), EdgeCreationError> {
    check_non_coincident_points(start, end)?;
    let d1 = g.add_dart();
    let d2 = g.add_dart();
    g.sew_unchecked(Dim::Zero, d1, d2);
    g.add_vertex(VertexAttr::new(d1, start, P::V::default()));
    g.add_vertex(VertexAttr::new(d2, end, P::V::default()));
    let e = g.add_edge(EdgeAttr::new(d1, curve, P::E::default()));
    Ok((d1, e))
}

pub fn add_line<P: Payload>(
    g: &mut GMap<P>,
    start: Point3,
    end: Point3,
) -> Result<(Dart, EdgeKey), EdgeCreationError> {
    let curve = Curve::line(Axis3::from_points(start, end));
    add_edge(g, start, end, curve)
}

pub fn split_edge<P: Payload>(
    g: &mut GMap<P>,
    edge: EdgeKey,
    parameter: f64,
) -> Result<EdgeSplit, EdgeSplitError> {
    let split = prepare_profile_edge_split(g, edge, parameter)?;
    split_edge_with_profile_links(g, edge, parameter, split)
}

pub(crate) fn split_face_boundary_edge<P: Payload>(
    g: &mut GMap<P>,
    edge: EdgeKey,
    parameter: f64,
) -> Result<EdgeSplit, EdgeSplitError> {
    let split = prepare_attached_edge_split(g, edge, parameter)?;
    split_attached_edge_with_profile_links(g, edge, parameter, split)
}

fn split_edge_with_profile_links<P: Payload>(
    g: &mut GMap<P>,
    edge: EdgeKey,
    parameter: f64,
    split: PreparedFreeEdgeSplit,
) -> Result<EdgeSplit, EdgeSplitError> {
    let midpoint = split.curve.point_at(parameter);
    let first_mid = g.add_dart();
    let second_mid = g.add_dart();

    g.sew_unchecked(Dim::Zero, split.first_dart, first_mid);
    g.sew_unchecked(Dim::Zero, second_mid, split.second_dart);
    g.sew_unchecked(Dim::One, first_mid, second_mid);

    let vertex = g.add_vertex(VertexAttr::new(first_mid, midpoint, P::V::default()));
    let second = g.add_edge(EdgeAttr::new(
        second_mid,
        split.curve.clone(),
        P::E::default(),
    ));

    Ok(EdgeSplit {
        first: edge,
        second,
        vertex,
    })
}

fn split_attached_edge_with_profile_links<P: Payload>(
    g: &mut GMap<P>,
    edge: EdgeKey,
    parameter: f64,
    split: PreparedAttachedEdgeSplit,
) -> Result<EdgeSplit, EdgeSplitError> {
    let midpoint = split.curve.point_at(parameter);
    let alpha0_pairs = alpha_pairs(g, &split.edge_darts, Dim::Zero);
    let alpha2_pairs = alpha_pairs(g, &split.edge_darts, Dim::Two);
    let mid_darts = split
        .edge_darts
        .iter()
        .map(|dart| (*dart, g.add_dart()))
        .collect::<HashMap<_, _>>();

    for (first, second) in alpha0_pairs {
        let first_mid = mid_darts[&first];
        let second_mid = mid_darts[&second];
        g.sew_unchecked(Dim::Zero, first, first_mid);
        g.sew_unchecked(Dim::Zero, second, second_mid);
        g.sew_unchecked(Dim::One, first_mid, second_mid);
    }

    for (first, second) in alpha2_pairs {
        g.sew_unchecked(Dim::Two, mid_darts[&first], mid_darts[&second]);
    }

    let vertex = g.add_vertex(VertexAttr::new(
        mid_darts[&split.first_dart],
        midpoint,
        P::V::default(),
    ));
    let second = g.add_edge(EdgeAttr::new(
        mid_darts[&split.second_dart],
        split.curve.clone(),
        P::E::default(),
    ));

    Ok(EdgeSplit {
        first: edge,
        second,
        vertex,
    })
}

fn prepare_profile_edge_split<P: Payload>(
    g: &GMap<P>,
    edge: EdgeKey,
    parameter: f64,
) -> Result<PreparedFreeEdgeSplit, EdgeSplitError> {
    if !parameter.is_finite() {
        return Err(EdgeSplitError::NonFiniteParameter { parameter });
    }

    let attr = g.edge(edge).ok_or(EdgeSplitError::MissingEdge { edge })?;
    let first_dart = attr.dart;
    let second_dart = g.alpha(Dim::Zero, first_dart);
    let split = PreparedFreeEdgeSplit {
        first_dart,
        second_dart,
        curve: attr.curve.clone(),
    };

    check_profile_edge(g, edge, first_dart, second_dart)?;
    check_split_parameter(g, edge, parameter, first_dart, second_dart, &split.curve)?;
    Ok(split)
}

fn prepare_attached_edge_split<P: Payload>(
    g: &GMap<P>,
    edge: EdgeKey,
    parameter: f64,
) -> Result<PreparedAttachedEdgeSplit, EdgeSplitError> {
    if !parameter.is_finite() {
        return Err(EdgeSplitError::NonFiniteParameter { parameter });
    }

    let attr = g.edge(edge).ok_or(EdgeSplitError::MissingEdge { edge })?;
    let first_dart = attr.dart;
    let second_dart = g.alpha(Dim::Zero, first_dart);
    let edge_darts = g
        .orbit(first_dart, g.orbit_indices(Dim::One))
        .collect::<Vec<_>>();
    let split = PreparedAttachedEdgeSplit {
        first_dart,
        second_dart,
        curve: attr.curve.clone(),
        edge_darts,
    };

    check_attached_edge(g, edge, first_dart, second_dart, &split.edge_darts)?;
    check_split_parameter(g, edge, parameter, first_dart, second_dart, &split.curve)?;
    Ok(split)
}

fn check_profile_edge<P: Payload>(
    g: &GMap<P>,
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
    g: &GMap<P>,
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

fn alpha_pairs<P: Payload>(g: &GMap<P>, darts: &[Dart], dim: Dim) -> Vec<(Dart, Dart)> {
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
    g: &GMap<P>,
    edge: EdgeKey,
    parameter: f64,
    first_dart: Dart,
    second_dart: Dart,
    curve: &Curve,
) -> Result<(), EdgeSplitError> {
    let start = g
        .attribute::<Cell0>(first_dart)
        .map(|vertex| vertex.point)
        .ok_or(EdgeSplitError::MissingEndpointGeometry { edge })?;
    let end = g
        .attribute::<Cell0>(second_dart)
        .map(|vertex| vertex.point)
        .ok_or(EdgeSplitError::MissingEndpointGeometry { edge })?;
    let domain = curve.parameters_between(start, end).ordered();

    if !domain.contains(parameter, LINEAR_TOLERANCE) {
        return Err(EdgeSplitError::ParameterOutOfRange { parameter, domain });
    }

    if (parameter - domain.start).abs() <= LINEAR_TOLERANCE
        || (parameter - domain.end).abs() <= LINEAR_TOLERANCE
    {
        return Err(EdgeSplitError::DegenerateSplit { parameter });
    }

    Ok(())
}

pub fn add_arc<P: Payload>(
    g: &mut GMap<P>,
    plane: Plane,
    radius: f64,
    start_angle: f64,
    end_angle: f64,
) -> Result<(Dart, EdgeKey), EdgeCreationError> {
    check_valid_radius(radius)?;
    check_valid_angle("start", start_angle)?;
    check_valid_angle("end", end_angle)?;

    let curve = Curve::Circle(Circle::new(plane, radius));
    let start = curve.point_at(start_angle);
    let end = curve.point_at(end_angle);
    add_edge(g, start, end, curve)
}

pub fn add_circle<P: Payload>(
    g: &mut GMap<P>,
    plane: Plane,
    radius: f64,
) -> Result<(Dart, EdgeKey), EdgeCreationError> {
    check_valid_radius(radius)?;
    let d1 = g.add_dart();
    let d2 = g.add_dart();
    let start = plane.point_at(radius, 0.0);
    g.add_vertex(VertexAttr::new(d1, start, P::V::default()));
    let curve = Curve::circle(plane, radius);
    g.sew_unchecked(Dim::Zero, d1, d2);
    g.sew_unchecked(Dim::One, d1, d2);
    let e = g.add_edge(EdgeAttr::new(d1, curve, P::E::default()));
    Ok((d1, e))
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

fn check_valid_angle(name: &'static str, angle: f64) -> Result<(), EdgeCreationError> {
    if angle.is_finite() {
        Ok(())
    } else {
        Err(EdgeCreationError::InvalidAngle { name, angle })
    }
}
