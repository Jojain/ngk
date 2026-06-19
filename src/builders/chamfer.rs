use crate::builders::edges::add_line;
use crate::builders::errors::ChamferError;
use crate::geometry::{Curve, LINEAR_TOLERANCE, Line, Point3};
use crate::topology::gmap::{Cell0, Cell1, Dart, Dim, GMap};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::EdgeKey;

#[derive(Clone, Copy, PartialEq, Eq)]
enum CornerRole {
    IncomingEnd,
    OutgoingStart,
}

pub fn chamfer_profile_vertex<P: Payload>(
    g: &mut GMap<P>,
    vertex_dart: Dart,
    distance: f64,
) -> Result<EdgeKey, ChamferError> {
    validate_distance(distance)?;

    let (incoming_end, outgoing_start) = profile_corner_darts(g, vertex_dart)?;
    let vertex = vertex_point(g, incoming_end)?;
    let previous = vertex_point(g, g.alpha(Dim::Zero, incoming_end))?;
    let next = vertex_point(g, g.alpha(Dim::Zero, outgoing_start))?;

    let incoming_edge = line_edge_dart(g, incoming_end)?;
    let outgoing_edge = line_edge_dart(g, outgoing_start)?;
    let incoming_offset = offset_point(incoming_edge, vertex, previous, distance)?;
    let outgoing_offset = offset_point(outgoing_edge, vertex, next, distance)?;

    g.unsew(incoming_end, Dim::One);
    g.set_vertex_point(incoming_end, incoming_offset);
    g.set_vertex_point(outgoing_start, outgoing_offset);
    reset_line_edge(g, incoming_end)?;
    reset_line_edge(g, outgoing_start)?;

    let chamfer_edge = add_line(g, incoming_offset, outgoing_offset)
        .map_err(|_| ChamferError::ZeroLengthEdge { dart: incoming_end })?;
    let chamfer_start = g
        .edge_attr(chamfer_edge)
        .expect("newly added chamfer edge must exist")
        .dart;
    let chamfer_end = g.alpha(Dim::Zero, chamfer_start);
    sew(g, incoming_end, chamfer_start)?;
    sew(g, chamfer_end, outgoing_start)?;

    Ok(chamfer_edge)
}

fn validate_distance(distance: f64) -> Result<(), ChamferError> {
    if distance.is_finite() && distance > 0.0 {
        Ok(())
    } else {
        Err(ChamferError::InvalidDistance { distance })
    }
}

fn profile_corner_darts<P: Payload>(
    g: &GMap<P>,
    vertex_dart: Dart,
) -> Result<(Dart, Dart), ChamferError> {
    let linked = g.alpha(Dim::One, vertex_dart);
    if linked == vertex_dart {
        return Err(ChamferError::EndpointVertex { dart: vertex_dart });
    }

    let role = corner_role(g, vertex_dart)?;
    let linked_role = corner_role(g, linked)?;
    match (role, linked_role) {
        (CornerRole::IncomingEnd, CornerRole::OutgoingStart) => Ok((vertex_dart, linked)),
        (CornerRole::OutgoingStart, CornerRole::IncomingEnd) => Ok((linked, vertex_dart)),
        _ => Err(ChamferError::AmbiguousProfileVertex { dart: vertex_dart }),
    }
}

fn corner_role<P: Payload>(g: &GMap<P>, dart: Dart) -> Result<CornerRole, ChamferError> {
    let edge_dart = line_edge_dart(g, dart)?;
    if edge_dart == dart {
        Ok(CornerRole::OutgoingStart)
    } else if g.alpha(Dim::Zero, edge_dart) == dart {
        Ok(CornerRole::IncomingEnd)
    } else {
        Err(ChamferError::AmbiguousProfileVertex { dart })
    }
}

fn line_edge_dart<P: Payload>(g: &GMap<P>, dart: Dart) -> Result<Dart, ChamferError> {
    let attr = g
        .attribute::<Cell1>(dart)
        .ok_or(ChamferError::MissingEdgeCurve { dart })?;
    match attr.curve {
        Curve::Line(_) => Ok(attr.dart),
        _ => Err(ChamferError::UnsupportedEdgeCurve { dart: attr.dart }),
    }
}

fn offset_point(
    edge_dart: Dart,
    vertex: Point3,
    neighbor: Point3,
    distance: f64,
) -> Result<Point3, ChamferError> {
    let direction = neighbor - vertex;
    let edge_length = direction.norm();
    if edge_length <= LINEAR_TOLERANCE {
        return Err(ChamferError::ZeroLengthEdge { dart: edge_dart });
    }
    if distance >= edge_length - LINEAR_TOLERANCE {
        return Err(ChamferError::DistanceTooLarge {
            dart: edge_dart,
            distance,
            edge_length,
        });
    }

    Ok(vertex + direction / edge_length * distance)
}

fn reset_line_edge<P: Payload>(g: &mut GMap<P>, dart: Dart) -> Result<(), ChamferError> {
    let edge_dart = line_edge_dart(g, dart)?;
    let start = vertex_point(g, edge_dart)?;
    let end = vertex_point(g, g.alpha(Dim::Zero, edge_dart))?;
    let attr = g
        .attribute_mut::<Cell1>(edge_dart)
        .ok_or(ChamferError::MissingEdgeCurve { dart: edge_dart })?;
    attr.curve = Curve::line(start, end);
    Ok(())
}


fn vertex_point<P: Payload>(g: &GMap<P>, dart: Dart) -> Result<Point3, ChamferError> {
    g.attribute::<Cell0>(dart)
        .map(|attr| attr.point)
        .ok_or(ChamferError::MissingVertexPoint { dart })
}

fn sew<P: Payload>(g: &mut GMap<P>, first: Dart, second: Dart) -> Result<(), ChamferError> {
    g.sew(Dim::One, first, second)
        .map_err(|_| ChamferError::SewFailed {
            dim: Dim::One,
            first,
            second,
        })
}
