use crate::builders::errors::EdgeCreationError;
use crate::geometry::{Circle, Curve, LINEAR_TOLERANCE, Line, Plane, Point3, PointCoincidence};
use crate::topology::attributes::{EdgeAttr, VertexAttr};
use crate::topology::gmap::{Dart, Dim, GMap};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::EdgeKey;

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
    let curve = Curve::Line(Line::new(start, end));
    add_edge(g, start, end, curve)
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
    g.add_vertex(VertexAttr::new(d2, start, P::V::default()));
    let curve = Curve::Circle(Circle::new(plane, radius));
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
