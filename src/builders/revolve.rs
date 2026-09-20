use crate::geometry::TrimmedCurve2;
use std::collections::HashMap;

use nalgebra::{Vector3, distance};
use radians::{Angle, Rad64};
use thiserror::Error;

use crate::builders::errors::ClosedFaceCellError;
use crate::builders::faces::reverse_face_winding;
use crate::builders::scaffold::{add_closed_face_cell, cut_between_loops};
use crate::geometry::axis::Axis3;
use crate::geometry::parameter::{Fraction, NativeParam};
use crate::geometry::transform::Rigid;
use crate::geometry::{
    ANGULAR_TOLERANCE, Axis2, Circle, Cone, Curve, Cylinder, DomainSide, Frame, LINEAR_TOLERANCE,
    Plane, Point2, Point3, Surface, SurfaceOfRevolution, SurfacePeriodicity,
};
use crate::model::{Cell2, MergeTopology, Model};
use crate::topology::IsolatedDart;
use crate::topology::attributes::{
    EdgeAttr, FaceAttr, LoopDefinition, LoopKind, ProfileAttr, SheetAttr, SolidAttr, VertexAttr,
};
use crate::topology::closed::Closeable;
use crate::topology::edge::Edge;
use crate::topology::edit::{ModelEdit, ModelEditError};
use crate::topology::face::Face;
use crate::topology::gmap::{Dart, Dim};
use crate::topology::payload::Payload;
use crate::topology::planar::{Planar, PlanarityError};
use crate::topology::profile::Profile;
use crate::topology::shape::{FaceTag, Shape};
use crate::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey, SheetKey, SolidKey, VertexKey};
use crate::topology::vertex::Vertex;

#[derive(Debug, Error)]
pub enum RevolveError {
    #[error("The provided profile is not planar")]
    PlanarError(PlanarityError),

    #[error("Missing edge for key {key:?}")]
    MissingEdge { key: EdgeKey },

    #[error("Missing vertex point for dart {dart:?}")]
    MissingVertexPoint { dart: Dart },

    #[error("Missing edge curve for dart {dart:?}")]
    MissingEdgeCurve { dart: Dart },

    #[error("Missing face for key {key:?}")]
    MissingFace { key: FaceKey },

    #[error("Edge {key:?} must be isolated before it can be consumed by a full revolution")]
    SourceEdgeNotIsolated { key: EdgeKey },

    #[error("Edge {key:?} lies on the revolution axis and sweeps no area")]
    EdgeOnRevolutionAxis { key: EdgeKey },

    /// A marked closed profile swept a whole turn.
    ///
    /// An unmarked circle sweeps a torus with no boundary anywhere, and the
    /// source loop is consumed. A marked one carries a corner, and a corner
    /// sweeps a circle: the result is a torus bounded by that swept edge, used
    /// twice. Refused rather than consumed, because consuming it would delete a
    /// corner the caller put there deliberately and hand back the boundaryless
    /// torus as if nothing had been asked for.
    #[error("edge {key:?} carries a corner, which a whole turn sweeps into a boundary")]
    MarkedProfileRevolve { key: EdgeKey },

    #[error("Edge {key:?} touches the revolution axis; apex faces are not supported yet")]
    ApexRevolveUnsupported { key: EdgeKey },

    #[error("failed to build the 2-cell of a boundaryless revolved face")]
    ClosedFaceCell(#[from] ClosedFaceCellError),

    #[error("Darts {first:?} and {second:?} are not sewable in dimension {dim:?}")]
    SewFailed { dim: Dim, first: Dart, second: Dart },

    #[error("failed to create revolved face topology")]
    ModelEditFailed(#[from] ModelEditError),
}

/// Where one end of a source edge is, and which logical vertex is there.
///
/// A whole circle with nothing marked on it has a position but no vertex: the
/// point where it closes is inside the edge. Revolving such an edge a full turn
/// consumes it outright, so there is nothing to remove and nothing to carry
/// onto the result's boundary -- which is why the key is optional and the point
/// is not.
#[derive(Clone)]
struct RevolvedSourceVertex {
    key: Option<VertexKey>,
    point: Point3,
}

#[derive(Clone)]
struct RevolvedSourceEdge {
    key: EdgeKey,
    dart: Dart,
    start: RevolvedSourceVertex,
    end: RevolvedSourceVertex,
    curve: Curve,
}

impl RevolvedSourceVertex {
    fn at_dart<P: Payload>(g: &Model<P>, dart: Dart) -> Result<Self, RevolveError> {
        let point = g
            .point_at_dart(dart)
            .ok_or(RevolveError::MissingVertexPoint { dart })?;
        Ok(Self {
            key: Vertex::from_dart(g, dart).map(|vertex| vertex.key()),
            point,
        })
    }
}

/// The vertices at the two ends of one edge occurrence.
///
/// A dart-level question rather than an endpoint one: a closed circle answers
/// with the same vertex twice, which is exactly what an alpha2 merge of two such
/// circles has to reconcile. Callers deciding what *section* an edge is want
/// [`Edge::kind`] instead.
fn edge_end_vertices<P: Payload>(
    g: &Model<P>,
    dart: Dart,
) -> Result<(VertexKey, VertexKey), RevolveError> {
    let at = |dart: Dart| {
        Vertex::from_dart(g, dart)
            .map(|vertex| vertex.key())
            .ok_or(RevolveError::MissingVertexPoint { dart })
    };
    Ok((at(dart)?, at(g.alpha(Dim::Zero, dart))?))
}

impl RevolvedSourceEdge {
    fn from_key<P: Payload>(g: &Model<P>, key: EdgeKey) -> Result<Self, RevolveError> {
        let edge = g.edge(key).ok_or(RevolveError::MissingEdge { key })?;
        Self::from_edge(g, edge)
    }

    fn from_edge<P: Payload>(g: &Model<P>, edge: Edge<'_, P>) -> Result<Self, RevolveError> {
        let key = edge.key();
        let dart = edge.dart();
        // A closed source edge is legitimate — revolving a circle sweeps a torus
        // — and both its ends are the one place it closes. The callers that
        // cannot take a closed edge test whether the two ends are the same and
        // refuse it by name, so the pair is kept rather than collapsed here.
        let (start, end) = match edge {
            Edge::Bounded(_) => (
                RevolvedSourceVertex::at_dart(g, dart)?,
                RevolvedSourceVertex::at_dart(g, g.alpha(Dim::Zero, dart))?,
            ),
            Edge::Marked(_) | Edge::Unmarked(_) => {
                let single = RevolvedSourceVertex::at_dart(g, dart)?;
                (single.clone(), single)
            }
        };
        let curve = edge.curve().clone();

        Ok(Self {
            key,
            dart,
            start,
            end,
            curve,
        })
    }
}

/// Adds a face generated by revolving `edge` around `axis`.
///
/// Angles are clamped to the inclusive range from zero to one full turn. A
/// partial turn reuses the source edge and its vertices as one face boundary.
/// A full turn consumes the isolated source topology and creates only the
/// non-degenerate circular loops swept by endpoints away from the axis.
/// A whole turn of a *closed* source edge is a torus: closed in the sweep and
/// closed in the profile, so the source loop is consumed and the face that comes
/// back has no boundary at all.
pub fn add_revolved_edge<P: Payload>(
    g: &mut Model<P>,
    edge: EdgeKey,
    axis: Axis3,
    angle: Rad64,
) -> Result<FaceKey, RevolveError> {
    g.transaction(|edit| add_revolved_edge_staged(edit, edge, axis, angle))
}

/// Revolves an edge inside the caller's active transaction.
pub(crate) fn add_revolved_edge_staged<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    edge: EdgeKey,
    axis: Axis3,
    angle: Rad64,
) -> Result<FaceKey, RevolveError> {
    let source = RevolvedSourceEdge::from_key(edit, edge)?;
    let angle = angle.clamp(Angle::ZERO, Angle::FULL_TURN);
    if is_full_turn(angle) {
        return add_full_revolved_edge_face(edit, &source, axis, angle);
    }

    add_partial_revolved_edge_face(edit, &source, axis, angle)
}

/// The support a revolution really has, with the map onto that support's own
/// parameters.
///
/// The revolve builders parameterize a face by `(profile parameter, angle)`.
/// Where the swept surface has a closed form, that form has a parameterization
/// of its own -- a cylinder's is `(angle, height)`, transposed and rescaled --
/// so adopting the closed form means rewriting every pcurve corner through
/// [`Self::map_pcurve_point`].
struct RevolvedSupport {
    surface: Surface,
    map_pcurve_point: Box<dyn Fn(Point2) -> Point2>,
    sweep: SweptParameters,
}

/// What the sweep angle looks like in a support's own parameters.
///
/// This decides the shape of every pcurve a revolution writes for a swept
/// circle, and nothing else about the support does.
enum SweptParameters {
    /// The sweep is one of the support's own parameter directions, so a swept
    /// circle runs straight along it and the whole reparameterization is affine.
    Linear,
    /// The support's parameters are a plane's own Cartesian ones, where a swept
    /// circle is still a circle about `center`.
    Circular { center: Point2 },
}

impl RevolvedSupport {
    /// Maps one revolution-space corner into the support's parameters.
    fn corner(&self, profile: f64, angle: f64) -> Point2 {
        (self.map_pcurve_point)(Point2::new(profile, angle))
    }

    /// The pcurve swept by the profile point at `profile`, from angle `from` to
    /// angle `to`.
    ///
    /// This is the one place a plane support differs from every other. A
    /// cylinder and a cone each put the sweep on a parameter axis of their own,
    /// so the swept circle runs straight there and a segment is its exact image;
    /// a plane's parameters are its own Cartesian ones, where the same circle is
    /// still a circle and a segment would quietly replace it with a chord.
    fn swept(&self, profile: f64, from: f64, to: f64) -> TrimmedCurve2 {
        let start = self.corner(profile, from);
        let SweptParameters::Circular { center } = self.sweep else {
            return TrimmedCurve2::segment(start, self.corner(profile, to));
        };
        let radial = start - center;
        // A profile point on the axis sweeps nothing, and names no circle.
        if radial.norm() <= LINEAR_TOLERANCE {
            return TrimmedCurve2::segment(start, self.corner(profile, to));
        }
        TrimmedCurve2::arc(center, radial, radial.norm(), to - from)
    }

    /// The pcurve of the profile itself at one angle, from `from` to `to`.
    ///
    /// A meridian is straight in every support recognized here, a plane's
    /// radial section included, so this is always a segment.
    fn meridian(&self, angle: f64, from: f64, to: f64) -> TrimmedCurve2 {
        TrimmedCurve2::segment(self.corner(from, angle), self.corner(to, angle))
    }

    /// The generic support, whose parameters are the revolution's own.
    fn generic(curve: &Curve, axis: Axis3) -> Self {
        Self {
            surface: Surface::Revolution(SurfaceOfRevolution::new(curve.clone(), axis)),
            map_pcurve_point: Box::new(|point| point),
            sweep: SweptParameters::Linear,
        }
    }
}

/// Recognizes the closed-form support a revolved profile sweeps out.
///
/// - a straight profile perpendicular to the axis keeps every point at its own
///   height, so the whole sweep stays in one plane: a disk, or an annulus when
///   the profile does not reach the axis;
/// - one parallel to the axis sweeps a cylinder;
/// - one meeting the axis at an angle sweeps a cone.
///
/// Everything else, a circular profile included, keeps the generic support:
/// [`crate::builders::solids::add_sphere`] passes its sphere explicitly
/// because it also knows the arc's normalized parameterization, which is not
/// recoverable here.
///
/// Each recognized support carries how its parameters see the sweep, because a
/// plane's see it as a circle where the others see a straight run — see
/// [`SweptParameters`].
fn revolved_support(curve: &Curve, axis: Axis3) -> RevolvedSupport {
    let Some((profile_origin, profile_direction)) = linear_profile(curve) else {
        return RevolvedSupport::generic(curve, axis);
    };
    let radial = profile_origin - axis.project(profile_origin);
    let start_radius = radial.norm();
    // The profile is straight, so both its distance from the axis and its
    // height along it are affine in the profile parameter; these are the two
    // rates.
    let height_rate = profile_direction.dot(&axis.direction);
    let start_height = (profile_origin - axis.origin).dot(&axis.direction);

    if height_rate.abs() <= LINEAR_TOLERANCE
        && let Some(plane) = planar_revolved_support(
            profile_origin,
            profile_direction,
            axis,
            radial,
            start_radius,
            start_height,
        )
    {
        return plane;
    }

    if start_radius <= LINEAR_TOLERANCE {
        return RevolvedSupport::generic(curve, axis);
    }
    let x_dir = radial / start_radius;
    let radius_rate = profile_direction.dot(&x_dir);

    if radius_rate.abs() <= LINEAR_TOLERANCE {
        let cylinder = Cylinder::new(axis.origin, x_dir, axis.direction, start_radius);
        return RevolvedSupport {
            surface: Surface::Cylinder(cylinder),
            map_pcurve_point: Box::new(move |point| {
                Point2::new(point.y, start_height + point.x * height_rate)
            }),
            sweep: SweptParameters::Linear,
        };
    }

    // A generatrix advances `radius_rate` outward and `height_rate` along the
    // axis per unit of profile parameter, so the cone's half angle is the angle
    // between those two and `v` is profile parameter times generatrix speed.
    let half_angle = radius_rate.atan2(height_rate);
    let generatrix_rate = radius_rate.hypot(height_rate);
    let frame = Frame::from_xz(
        axis.origin + *axis.direction * start_height,
        x_dir,
        axis.direction,
    );
    RevolvedSupport {
        surface: Surface::Cone(Cone::new(frame, start_radius, half_angle)),
        map_pcurve_point: Box::new(move |point| Point2::new(point.y, point.x * generatrix_rate)),
        sweep: SweptParameters::Linear,
    }
}

/// The plane a profile perpendicular to the axis sweeps, when it sweeps one.
///
/// Perpendicular means every point keeps its height, so the sweep never leaves
/// the plane at that height. The profile must also be *radial*: a perpendicular
/// chord that misses the axis sweeps the same annulus, but its distance from the
/// axis is not affine in its parameter, and every pcurve here is written from
/// rates that assume it is. Such a profile keeps the generic support, which is
/// exact for any profile at all.
fn planar_revolved_support(
    profile_origin: Point3,
    profile_direction: Vector3<f64>,
    axis: Axis3,
    radial: Vector3<f64>,
    start_radius: f64,
    start_height: f64,
) -> Option<RevolvedSupport> {
    // On the axis the profile is radial by construction, and `radial` is too
    // small to take a direction from; off it, the two must be parallel.
    let x_dir = if start_radius > LINEAR_TOLERANCE {
        let x_dir = radial / start_radius;
        let off_meridian = profile_direction - x_dir * profile_direction.dot(&x_dir);
        if off_meridian.norm() > LINEAR_TOLERANCE {
            return None;
        }
        x_dir
    } else {
        let length = profile_direction.norm();
        if length <= LINEAR_TOLERANCE {
            return None;
        }
        profile_direction / length
    };

    // The plane is centred on the axis, so a swept circle is centred on its
    // parameter origin and the radius is just the first coordinate.
    let center = axis.origin + *axis.direction * start_height;
    let y_dir = axis.direction.cross(&x_dir);
    let start_offset = (profile_origin - center).dot(&x_dir);
    let radius_rate = profile_direction.dot(&x_dir);

    Some(RevolvedSupport {
        surface: Surface::Plane(Plane::from_xy(center, x_dir, y_dir)),
        map_pcurve_point: Box::new(move |point| {
            let radius = start_offset + point.x * radius_rate;
            Point2::new(radius * point.y.cos(), radius * point.y.sin())
        }),
        sweep: SweptParameters::Circular {
            center: Point2::origin(),
        },
    })
}

/// Returns a straight profile's point and direction per unit of its own parameter.
///
/// `None` for anything whose `point_at` is not affine in its parameter.
fn linear_profile(curve: &Curve) -> Option<(Point3, Vector3<f64>)> {
    match curve {
        Curve::Line(_) => {
            let origin = curve.point_at(NativeParam::new(0.0));
            Some((origin, curve.point_at(NativeParam::new(1.0)) - origin))
        }
        _ => None,
    }
}

fn add_partial_revolved_edge_face<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    source: &RevolvedSourceEdge,
    axis: Axis3,
    angle: Rad64,
) -> Result<FaceKey, RevolveError> {
    validate_revolvable_radii(axis, source)?;

    let start = source.start.point;
    let end = source.end.point;
    let rotated_start = rotate_point(axis, start, angle);
    let rotated_end = rotate_point(axis, end, angle);
    let rotated_curve = source.curve.moved(&Rigid::rotation(axis, angle));
    let interval = source.curve.interval_between(start, end);
    let surface = Surface::Revolution(SurfaceOfRevolution::new(source.curve.clone(), axis));

    let bottom_start = source.dart;
    let bottom_end = edit.alpha(Dim::Zero, bottom_start);
    let end_arc_start = edit.add_dart();
    let end_arc_end = edit.add_dart();
    let rotated_start_dart = edit.add_dart();
    let rotated_end_dart = edit.add_dart();
    let start_arc_start = edit.add_dart();
    let start_arc_end = edit.add_dart();

    edit.link(Dim::Zero, end_arc_start, end_arc_end)?;
    edit.link(Dim::Zero, rotated_start_dart, rotated_end_dart)?;
    edit.link(Dim::Zero, start_arc_start, start_arc_end)?;

    edit.link(Dim::One, bottom_end, end_arc_start)?;
    edit.link(Dim::One, end_arc_end, rotated_start_dart)?;
    edit.link(Dim::One, rotated_end_dart, start_arc_start)?;
    edit.link(Dim::One, start_arc_end, bottom_start)?;

    edit.add_vertex(VertexAttr::new(end_arc_end, rotated_end));
    edit.add_vertex(VertexAttr::new(rotated_end_dart, rotated_start));

    edit.add_edge(EdgeAttr::new(
        end_arc_start,
        revolve_circle_curve(axis, end, angle),
    ));
    edit.add_edge(EdgeAttr::new(rotated_end_dart, rotated_curve));
    edit.add_edge(EdgeAttr::new(
        start_arc_end,
        revolve_circle_curve(axis, start, angle),
    ));

    let mut pcurves = HashMap::with_capacity(4);
    pcurves.insert(
        bottom_start,
        TrimmedCurve2::segment(
            Point2::new(interval.start.value(), 0.0),
            Point2::new(interval.end.value(), 0.0),
        ),
    );
    pcurves.insert(
        end_arc_start,
        TrimmedCurve2::segment(
            Point2::new(interval.end.value(), 0.0),
            Point2::new(interval.end.value(), angle.val()),
        ),
    );
    pcurves.insert(
        rotated_start_dart,
        TrimmedCurve2::segment(
            Point2::new(interval.end.value(), angle.val()),
            Point2::new(interval.start.value(), angle.val()),
        ),
    );
    pcurves.insert(
        start_arc_start,
        TrimmedCurve2::segment(
            Point2::new(interval.start.value(), angle.val()),
            Point2::new(interval.start.value(), 0.0),
        ),
    );

    edit.add_profile(ProfileAttr::new(bottom_start));
    Ok(edit.add_face(FaceAttr::with_pcurves(
        surface,
        bottom_start,
        Vec::new(),
        pcurves,
    )))
}

fn add_full_revolved_edge_face<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    source: &RevolvedSourceEdge,
    axis: Axis3,
    angle: Rad64,
) -> Result<FaceKey, RevolveError> {
    if source.start.key == source.end.key {
        return add_full_revolved_closed_edge_face(edit, source, axis);
    }

    let start_on_axis = revolve_radius(axis, source.start.point) <= LINEAR_TOLERANCE;
    let end_on_axis = revolve_radius(axis, source.end.point) <= LINEAR_TOLERANCE;
    if start_on_axis && end_on_axis {
        let surface = Surface::Revolution(SurfaceOfRevolution::new(source.curve.clone(), axis));
        return add_full_revolved_apex_to_apex_face(edit, source, axis, angle, surface, &|point| {
            point
        });
    }

    add_full_revolved_open_edge_face(edit, source, axis, angle)
}

/// Revolves an unmarked closed profile a whole turn into one boundaryless face.
///
/// An unmarked circle swept a whole turn is a torus: closed in the sweep and
/// closed in the profile too, so it has no boundary anywhere and carries no
/// edge and no vertex. The source loop is therefore consumed outright rather
/// than reused as a boundary, and the face is laid on the polygon schema of its
/// own support — the same 2-cell [`crate::builders::solids::add_torus`] builds,
/// the difference being that here the support is swept rather than named.
///
/// A *marked* profile is a different shape and is refused: its corner sweeps a
/// circle, which bounds the result.
fn add_full_revolved_closed_edge_face<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    source: &RevolvedSourceEdge,
    axis: Axis3,
) -> Result<FaceKey, RevolveError> {
    let surface = Surface::Revolution(SurfaceOfRevolution::new(source.curve.clone(), axis));
    // A profile crossing the axis pinches the sweep to a point there, which is
    // not a torus: it leaves a degenerate row that no loop bounds and no
    // `LoopKind` describes. The support is what knows where its own rows
    // collapse, so it is asked rather than the axis re-intersected here.
    if !surface.degenerate_rows(Axis2::U).is_empty() {
        return Err(RevolveError::EdgeOnRevolutionAxis { key: source.key });
    }

    // A corner on the profile is swept into an edge of the result, so this face
    // is not the boundaryless one below.
    if source.start.key.is_some() {
        return Err(RevolveError::MarkedProfileRevolve { key: source.key });
    }

    validate_consumable_closed_source_edge(edit, source)?;
    consume_closed_source_edge(edit, source)?;
    let cell = add_closed_face_cell(edit, &surface)?;
    let face = edit.add_face(FaceAttr::closed(surface, cell.anchor(), HashMap::new()));
    cell.own(edit, face);
    Ok(face)
}

/// Closes an apex-to-apex revolution by identifying its two meridian seams.
///
/// The face starts as a two-edge loop containing the source meridian and its
/// full-turn copy. Sewing those occurrences in dimension two leaves one edge
/// incident twice to one face while preserving the two pole vertices.
fn add_full_revolved_apex_to_apex_face<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    source: &RevolvedSourceEdge,
    axis: Axis3,
    angle: Rad64,
    surface: Surface,
    map_pcurve_point: &impl Fn(Point2) -> Point2,
) -> Result<FaceKey, RevolveError> {
    validate_consumable_source_edge(edit, source)?;
    let interval = source
        .curve
        .interval_between(source.start.point, source.end.point);
    let middle = source.curve.point_at(NativeParam::new(
        0.5 * (interval.start.value() + interval.end.value()),
    ));
    if revolve_radius(axis, middle) <= LINEAR_TOLERANCE {
        return Err(RevolveError::EdgeOnRevolutionAxis { key: source.key });
    }

    let seam_start = source.dart;
    let seam_end = edit.alpha(Dim::Zero, seam_start);
    let opposite_end = edit.add_dart();
    let opposite_start = edit.add_dart();
    edit.link(Dim::Zero, opposite_start, opposite_end)?;
    edit.link(Dim::One, seam_end, opposite_end)?;
    edit.link(Dim::One, opposite_start, seam_start)?;

    edit.add_edge(EdgeAttr::new(opposite_start, source.curve.clone()));
    if Profile::from_dart(edit, seam_start).is_none() {
        edit.add_profile(ProfileAttr::new(seam_start));
    }

    let mut pcurves = HashMap::with_capacity(2);
    pcurves.insert(
        seam_start,
        TrimmedCurve2::segment(
            map_pcurve_point(Point2::new(interval.start.value(), 0.0)),
            map_pcurve_point(Point2::new(interval.end.value(), 0.0)),
        ),
    );
    pcurves.insert(
        opposite_end,
        TrimmedCurve2::segment(
            map_pcurve_point(Point2::new(interval.end.value(), angle.val())),
            map_pcurve_point(Point2::new(interval.start.value(), angle.val())),
        ),
    );
    let face = edit.add_face(FaceAttr::with_pcurves(
        surface,
        seam_start,
        Vec::new(),
        pcurves,
    ));

    sew_revolved_alpha2_edges(edit, seam_start, opposite_start, seam_start, opposite_start)?;
    Ok(face)
}

fn add_full_revolved_open_edge_face<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    source: &RevolvedSourceEdge,
    axis: Axis3,
    angle: Rad64,
) -> Result<FaceKey, RevolveError> {
    let interval = source
        .curve
        .interval_between(source.start.point, source.end.point);
    let support = revolved_support(&source.curve, axis);
    let surface = support.surface.clone();
    let start_radius = revolve_radius(axis, source.start.point);
    let end_radius = revolve_radius(axis, source.end.point);

    validate_consumable_source_edge(edit, source)?;

    let reuse_start = start_radius > LINEAR_TOLERANCE;
    let reused_loop_point = if reuse_start {
        source.start.point
    } else if end_radius > LINEAR_TOLERANCE {
        source.end.point
    } else {
        return Err(RevolveError::EdgeOnRevolutionAxis { key: source.key });
    };

    // Two circles bound the sweep on opposite sides, so they have to travel in
    // opposite directions for the face to lie to the left of both — the wider one
    // forward, the narrower one back. That is what the reversed pcurve below
    // says, whether the pair turns out to be a ring's two rims or an annulus and
    // its hole, and the circle under that pcurve has to be swept backwards to
    // match: `FaceAttr`'s contract is that a pcurve runs in its dart's direction,
    // and `add_annulus` keeps it the same way, by reversing the hole's own 3D
    // circle rather than its pcurve. Which circle is the narrower one is known
    // from the radii alone, so it is settled before either is built.
    let start_leads = start_radius >= end_radius;
    let two_loops = start_radius > LINEAR_TOLERANCE && end_radius > LINEAR_TOLERANCE;
    let sweep_of = |at_start: bool| match two_loops && at_start != start_leads {
        true => Rad64::new(-angle.val()),
        false => angle,
    };

    let reused_loop = consume_source_edge_as_closed_loop(
        edit,
        source,
        reused_loop_point,
        revolve_circle_curve(axis, reused_loop_point, sweep_of(reuse_start)),
    )?;

    let start_loop = (start_radius > LINEAR_TOLERANCE)
        .then(|| {
            if reuse_start {
                Ok(reused_loop)
            } else {
                add_closed_revolve_boundary_loop(
                    edit,
                    source.start.point,
                    revolve_circle_curve(axis, source.start.point, sweep_of(true)),
                )
            }
        })
        .transpose()?
        .map(|dart| (dart, interval.start.value(), start_radius));
    let end_loop = (end_radius > LINEAR_TOLERANCE)
        .then(|| {
            if !reuse_start {
                Ok(reused_loop)
            } else {
                add_closed_revolve_boundary_loop(
                    edit,
                    source.end.point,
                    revolve_circle_curve(axis, source.end.point, sweep_of(false)),
                )
            }
        })
        .transpose()?
        .map(|dart| (dart, interval.end.value(), end_radius));

    let (outer_loop, outer_u, inner_loop) = match (start_loop, end_loop) {
        (Some(start), Some(end)) if start.2 >= end.2 => (start.0, start.1, Some(end)),
        (Some(start), Some(end)) => (end.0, end.1, Some(start)),
        (Some(boundary), None) | (None, Some(boundary)) => (boundary.0, boundary.1, None),
        (None, None) => {
            return Err(RevolveError::EdgeOnRevolutionAxis { key: source.key });
        }
    };

    let mut pcurves = HashMap::with_capacity(1 + usize::from(inner_loop.is_some()));
    pcurves.insert(outer_loop, support.swept(outer_u, 0.0, angle.val()));
    let inner_loops = inner_loop
        .map(|(dart, u, _)| {
            pcurves.insert(dart, support.swept(u, angle.val(), 0.0));
            vec![dart]
        })
        .unwrap_or_default();

    let loops = match inner_loops.first() {
        // `outer_loop` is the wider of the pair by construction, so the kinds
        // come back in that order.
        Some(&inner) => {
            let kinds = pcurves
                .get(&outer_loop)
                .zip(pcurves.get(&inner))
                .map(|(outer, inner)| revolved_band_loop_kinds(&surface, [outer, inner], true))
                .expect("both loops were just given a pcurve");
            vec![
                LoopDefinition::from_kind(outer_loop, kinds[0]),
                LoopDefinition::from_kind(inner, kinds[1]),
            ]
        }
        // One circle and nothing else: the other end of the profile sits on the
        // axis and sweeps no circle at all, so that side of the band is closed
        // by the degeneracy there. The loop still runs a whole turn, so it is no
        // more an outer loop than a ring's are — it is a cap.
        None => {
            let degenerate_profile = if outer_u == interval.start.value() {
                interval.end.value()
            } else {
                interval.start.value()
            };
            match pcurves
                .get(&outer_loop)
                .and_then(|pcurve| swept_period_axis(&surface, [pcurve, pcurve]))
                .and_then(|axis| {
                    // The loop and the collapse are both read in the support's
                    // own parameters, which is where the pcurves already live.
                    let transverse = axis.transverse();
                    let loop_at = transverse.of(support.corner(outer_u, 0.0));
                    let collapse_at = transverse.of(support.corner(degenerate_profile, 0.0));
                    let side = DomainSide::of(loop_at, collapse_at);
                    // The support has to agree that it collapses there, since it
                    // is the support the chart will later ask for the row.
                    side.nearest(loop_at, surface.degenerate_rows(transverse))
                        .filter(|row| (row - collapse_at).abs() <= LINEAR_TOLERANCE)
                        .map(|_| LoopDefinition::capping(outer_loop, axis, side))
                }) {
                Some(capping) => vec![capping],
                None => vec![LoopDefinition::outer(outer_loop)],
            }
        }
    };

    let key = edit.add_face(FaceAttr::with_loops(surface, loops, pcurves));
    // Two circles bound this band, and until something joins them they sit in
    // two 2-cells that nothing connects. The cut is where the source edge's two
    // swept copies would have met, so the band stays seamless as a shape while
    // the map gains the connectivity one face needs.
    if let Some((inner, _, _)) = inner_loop {
        cut_between_loops(edit, key, outer_loop, inner)?;
    }
    Ok(key)
}

/// The axis every given loop spans a whole period of, if they all span one.
///
/// Which parameter the sweep angle becomes depends on the support recognized for
/// the profile: a cylinder's pcurves put it on `u`, a generic surface of
/// revolution leaves it on `v`. Reading the travel back off the pcurves answers
/// for either without the caller having to know which was chosen.
fn swept_period_axis(surface: &Surface, pcurves: [&TrimmedCurve2; 2]) -> Option<Axis2> {
    let periods = match surface.periodicity() {
        SurfacePeriodicity::None => return None,
        SurfacePeriodicity::UPeriodic(u) => [Some(u), None],
        SurfacePeriodicity::VPeriodic(v) => [None, Some(v)],
        SurfacePeriodicity::UVPeriodic(u, v) => [Some(u), Some(v)],
    };
    Axis2::ALL.into_iter().find(|axis| {
        periods[axis.index()].is_some_and(|period| {
            pcurves.iter().all(|pcurve| {
                let span = axis.of(pcurve.point_at(Fraction::new(1.0)))
                    - axis.of(pcurve.point_at(Fraction::new(0.0)));
                (span.abs() - period).abs() <= ANGULAR_TOLERANCE
            })
        })
    })
}

fn validate_consumable_source_edge<P: Payload>(
    edit: &ModelEdit<'_, P>,
    source: &RevolvedSourceEdge,
) -> Result<(), RevolveError> {
    let start = source.dart;
    let end = edit.alpha(Dim::Zero, start);
    let edge = Edge::from_dart(edit, start).ok_or(RevolveError::MissingEdge { key: source.key })?;
    let attached_to_face = !edge.faces().is_empty();
    let has_other_links = [Dim::One, Dim::Two, Dim::Three]
        .into_iter()
        .any(|dim| !edit.is_free(start, dim) || !edit.is_free(end, dim));
    if start == end || attached_to_face || has_other_links {
        return Err(RevolveError::SourceEdgeNotIsolated { key: source.key });
    }

    Ok(())
}

/// Checks that a closed source loop can be consumed by a full revolution.
///
/// A closed edge is a one-edge loop: its two darts are alpha0- *and* alpha1-linked
/// to each other, and free above. That is a different isolation from
/// [`validate_consumable_source_edge`]'s, which requires alpha1 free precisely
/// because the loop it closes has yet to be built, so the two cannot share a
/// check.
fn validate_consumable_closed_source_edge<P: Payload>(
    edit: &ModelEdit<'_, P>,
    source: &RevolvedSourceEdge,
) -> Result<(), RevolveError> {
    let start = source.dart;
    let end = edit.alpha(Dim::Zero, start);
    let edge = Edge::from_dart(edit, start).ok_or(RevolveError::MissingEdge { key: source.key })?;
    let closes_on_itself = edit.alpha(Dim::One, start) == end;
    let attached_to_face = !edge.faces().is_empty();
    let has_other_links = [Dim::Two, Dim::Three]
        .into_iter()
        .any(|dim| !edit.is_free(start, dim) || !edit.is_free(end, dim));
    if start == end || !closes_on_itself || attached_to_face || has_other_links {
        return Err(RevolveError::SourceEdgeNotIsolated { key: source.key });
    }

    Ok(())
}

/// Deletes a closed source loop outright, leaving no topology behind.
///
/// Its two darts are the whole loop, so unlinking the two involutions that hold
/// them together isolates both, and the result of the revolution has no
/// boundary for either to become.
fn consume_closed_source_edge<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    source: &RevolvedSourceEdge,
) -> Result<(), RevolveError> {
    let start = source.dart;
    let end = edit.alpha(Dim::Zero, start);

    if let Some(profile) = Profile::from_dart(edit, start).map(|profile| profile.key()) {
        edit.remove_profile(profile);
    }
    edit.remove_edge(source.key)
        .expect("validated source edge should remain registered");
    // Both ends of a closed edge are its one vertex, so it is removed once --
    // and a circle with nothing marked on it has none to remove at all.
    if let Some(vertex) = source.start.key {
        edit.remove_vertex(vertex)
            .expect("validated source vertex should remain registered");
    }

    for dim in [Dim::Zero, Dim::One] {
        edit.unlink(dim, start)?;
    }
    edit.remove_isolated_darts(vec![IsolatedDart::new(start), IsolatedDart::new(end)]);
    Ok(())
}

fn consume_source_edge_as_closed_loop<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    source: &RevolvedSourceEdge,
    point: Point3,
    curve: Curve,
) -> Result<Dart, RevolveError> {
    let start = source.dart;
    let end = edit.alpha(Dim::Zero, start);

    let profile = Profile::from_dart(edit, start).map(|profile| profile.key());
    if let Some(profile) = profile {
        edit.remove_profile(profile);
    }
    edit.remove_edge(source.key)
        .expect("validated source edge should remain registered");
    for end in [source.start.key, source.end.key].into_iter().flatten() {
        edit.remove_vertex(end)
            .expect("validated source vertex should remain registered");
    }

    edit.link(Dim::One, start, end)?;
    edit.add_vertex(VertexAttr::new(start, point));
    edit.add_edge(EdgeAttr::new(start, curve));
    edit.add_profile(ProfileAttr::new(start));
    Ok(start)
}

fn add_closed_revolve_boundary_loop<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    point: Point3,
    curve: Curve,
) -> Result<Dart, ModelEditError> {
    let first = edit.add_dart();
    let second = edit.add_dart();
    edit.link(Dim::Zero, first, second)?;
    edit.link(Dim::One, first, second)?;
    edit.add_vertex(VertexAttr::new(first, point));
    edit.add_edge(EdgeAttr::new(first, curve));
    edit.add_profile(ProfileAttr::new(first));
    Ok(first)
}

fn rotate_point(axis: Axis3, point: Point3, angle: Rad64) -> Point3 {
    Rigid::rotation(axis, angle).apply(point)
}

fn is_full_turn(angle: Rad64) -> bool {
    (angle.val() - Rad64::FULL_TURN.val()).abs() <= ANGULAR_TOLERANCE
}

/// Revolves every edge of a planar profile into a sheet of lateral faces.
///
/// Adjacent generated faces are sewn along their swept vertex trajectories; a
/// closed source profile also closes the ring between its last and first faces.
/// The source profile remains in the map and is not used as sheet boundary
/// topology. `angle` is clamped between zero and one full turn.
///
/// Returns an error if the profile is not planar, required edge or vertex
/// geometry is missing, a source curve is unsupported, or generated faces
/// cannot be sewn.
///
/// # Panics
///
/// Panics if `profile` does not identify a registered profile.
pub fn add_revolved_profile<P: Payload>(
    g: &mut Model<P>,
    profile: ProfileKey,
    axis: Axis3,
    angle: Rad64,
) -> Result<SheetKey, RevolveError> {
    let profile_dart = g.profile_unchecked(profile).dart;
    add_revolved_profile_from_dart(g, profile_dart, axis, angle)
}

/// Revolves a profile from an oriented traversal dart for internal callers.
pub(crate) fn add_revolved_profile_from_dart<P: Payload>(
    g: &mut Model<P>,
    profile_dart: Dart,
    axis: Axis3,
    angle: Rad64,
) -> Result<SheetKey, RevolveError> {
    g.transaction(|edit| add_revolved_profile_from_dart_staged(edit, profile_dart, axis, angle))
}

fn add_revolved_profile_from_dart_staged<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    profile_dart: Dart,
    axis: Axis3,
    angle: Rad64,
) -> Result<SheetKey, RevolveError> {
    let profile = Profile::from_dart(edit, profile_dart)
        .expect("profile dart must belong to a registered profile");
    let planar = Planar::new(profile).map_err(RevolveError::PlanarError)?;
    let close_ring = planar.inner().is_closed();
    let dart = add_revolved_profile_faces(edit, profile_dart, axis, angle, close_ring)?.swept_dart;
    Ok(edit.add_sheet(SheetAttr::new(dart)))
}

struct RevolvedProfile {
    swept_dart: Dart,
    bottom_edges: Vec<Dart>,
    top_edges: Vec<Dart>,
    faces: Vec<FaceKey>,
}

struct RevolvedFace {
    /// Dart of the un-rotated source edge, sitting at the source `start` vertex.
    ///
    /// `None` for a band swept through a whole turn, which has no copy of the
    /// source edge on its boundary at all: the sweep closes on itself, so the
    /// band is bounded by its two swept circles and nothing else.
    bottom_edge: Option<Dart>,
    /// Dart of the rotated edge, sitting at the rotated `start` vertex.
    ///
    /// The quad loop traverses the rotated edge backwards (`rotated_end` to
    /// `rotated_start`), so this is the *second* dart of that edge rather than
    /// the loop-order one. Keeping it at `rotated_start` makes it line up with
    /// the matching cap edge dart, which also sits at `rotated_start`. `None`
    /// for a whole turn, for the same reason as `bottom_edge`.
    top_edge: Option<Dart>,
    start_side: Dart,
    end_side: Dart,
    outer_loop: Dart,
    key: FaceKey,
}

fn add_revolved_profile_faces<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    profile_dart: Dart,
    axis: Axis3,
    angle: Rad64,
    close_ring: bool,
) -> Result<RevolvedProfile, RevolveError> {
    let angle = angle.clamp(Angle::ZERO, Angle::FULL_TURN);
    let model: &Model<P> = edit;
    let profile = Profile::from_dart(model, profile_dart)
        .expect("profile dart must belong to a registered profile");
    let source_edges = profile
        .edges()
        .into_iter()
        .map(|edge| RevolvedSourceEdge::from_edge(model, edge))
        .collect::<Result<Vec<_>, _>>()?;

    let mut faces = Vec::with_capacity(source_edges.len());
    for source in &source_edges {
        faces.push(add_revolved_edge_face(edit, source, axis, angle)?);
    }

    sew_revolved_faces(edit, &faces, close_ring)?;
    let swept_dart = faces.first().map_or(profile_dart, |face| face.outer_loop);
    Ok(RevolvedProfile {
        swept_dart,
        bottom_edges: faces.iter().filter_map(|face| face.bottom_edge).collect(),
        top_edges: faces.iter().filter_map(|face| face.top_edge).collect(),
        faces: faces.iter().map(|face| face.key).collect(),
    })
}

fn add_revolved_edge_face<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    source: &RevolvedSourceEdge,
    axis: Axis3,
    angle: Rad64,
) -> Result<RevolvedFace, RevolveError> {
    validate_revolvable_radii(axis, source)?;

    let start = source.start.point;
    let end = source.end.point;
    let curve = source.curve.clone();
    let rotated_start = rotate_point(axis, start, angle);
    let rotated_end = rotate_point(axis, end, angle);
    let rotated_curve = curve.moved(&Rigid::rotation(axis, angle));
    let start_arc = revolve_circle_curve(axis, start, angle);
    let end_arc = revolve_circle_curve(axis, end, angle);
    let interval = curve.interval_between(start, end);
    let support = revolved_support(&curve, axis);
    let surface = support.surface.clone();
    // The quad boundary in order: the profile at angle zero, the arc its far end
    // sweeps, the profile back at the far angle, and the arc its near end sweeps
    // in reverse.
    let pcurves = [
        support.meridian(0.0, interval.start.value(), interval.end.value()),
        support.swept(interval.end.value(), 0.0, angle.val()),
        support.meridian(angle.val(), interval.end.value(), interval.start.value()),
        support.swept(interval.start.value(), angle.val(), 0.0),
    ];

    // A whole turn brings the swept copy back onto its source, so the band
    // closes on itself and is bounded by the two circles its endpoints swept and
    // by nothing else: build it that way rather than building the copy and
    // sewing it back on. What the two circles bound — a ring or an annulus —
    // is the support's answer, not a reason to build the band differently.
    //
    // `validate_revolvable_radii` has already refused any edge touching the
    // axis, so both radii are non-zero here today. The check is kept because it
    // is the real precondition: a band with an end on the axis sweeps no circle
    // there, and is closed by that degeneracy rather than by a second loop.
    //
    // The two swept circles are each traversed as the quad loop traverses its
    // arc: the end arc with the sweep, the start arc against it, so that the
    // band lies to the left of both. Neighbouring bands share a circle as one's
    // end and the next one's start, so tying those directions to traversal order
    // rather than to the radii is what makes the two agree across the sew that
    // joins them. `FaceAttr`'s contract is that a pcurve runs in its dart's
    // direction, so the backwards one gets its own 3D circle swept backwards to
    // match, the way `add_annulus` reverses a hole's circle rather than its
    // pcurve.
    let band_pcurves = [pcurves[3].clone(), pcurves[1].clone()];
    if is_full_turn(angle)
        && [start, end]
            .iter()
            .all(|point| revolve_radius(axis, *point) > LINEAR_TOLERANCE)
    {
        let kinds = revolved_band_loop_kinds(
            &surface,
            [&band_pcurves[0], &band_pcurves[1]],
            revolve_radius(axis, start) >= revolve_radius(axis, end),
        );
        let start_circle = revolve_circle_curve(axis, start, Rad64::new(-angle.val()));
        return add_full_revolved_band_face(
            edit,
            [start, end],
            [start_circle, end_arc],
            surface,
            band_pcurves,
            kinds,
        );
    }

    add_revolved_quad_face(
        edit,
        [start, end, rotated_end, rotated_start],
        [curve, end_arc, rotated_curve, start_arc],
        surface,
        pcurves,
    )
}

fn add_revolved_quad_face<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    corners: [Point3; 4],
    boundary_curves: [Curve; 4],
    surface: Surface,
    pcurves: [TrimmedCurve2; 4],
) -> Result<RevolvedFace, RevolveError> {
    let darts: Vec<Dart> = (0..8).map(|_| edit.add_dart()).collect();

    for i in 0..4 {
        edit.link(Dim::Zero, darts[2 * i], darts[2 * i + 1])?;
    }
    for i in 0..4 {
        edit.link(Dim::One, darts[2 * i + 1], darts[(2 * i + 2) % darts.len()])?;
    }

    for i in 0..4 {
        let dart = edit.cell_representative(darts[2 * i], Dim::Zero);
        edit.add_vertex(VertexAttr::new(dart, corners[i]));
    }

    for i in 0..4 {
        let edge_dart = darts[2 * i];
        edit.add_edge(EdgeAttr::new(edge_dart, boundary_curves[i].clone()));
    }

    edit.add_profile(ProfileAttr::new(darts[0]));
    let key = edit.add_face(FaceAttr::with_pcurves(
        surface,
        darts[0],
        Vec::new(),
        quad_pcurves(&pcurves, &darts),
    ));

    Ok(RevolvedFace {
        bottom_edge: Some(darts[0]),
        // darts[4] is at `rotated_end`; darts[5] is its alpha0 partner at
        // `rotated_start`, which is the vertex the cap edge dart carries.
        top_edge: Some(darts[5]),
        start_side: darts[7],
        end_side: darts[2],
        outer_loop: darts[0],
        key,
    })
}

/// Classifies the two whole-turn circles bounding a band, in the order given.
///
/// Two loops that each run a whole period of the support bound a ring, not a
/// disk with a hole: in the support's own parameters the band is a rectangle
/// spanning the sweep entirely, and neither loop closes there. Which circle is
/// the wider one is then a fact about the shape in space, not about the domain,
/// so it does not make one of them an outer loop.
///
/// A plane has no period for either loop to wrap, and both circles close in its
/// Cartesian parameters, so there the wider circle really does bound the band
/// from outside and the narrower one really is a hole. `first_is_wider` says
/// which of the two that is.
fn revolved_band_loop_kinds(
    surface: &Surface,
    pcurves: [&TrimmedCurve2; 2],
    first_is_wider: bool,
) -> [LoopKind; 2] {
    match swept_period_axis(surface, pcurves) {
        Some(axis) => [LoopKind::Wrapping { axis }; 2],
        None if first_is_wider => [LoopKind::Outer, LoopKind::Inner],
        None => [LoopKind::Inner, LoopKind::Outer],
    }
}

/// Builds one band of a whole turn from two circles joined by a scaffold cut.
///
/// A quad band carries a copy of the source edge at each end of the sweep, and a
/// whole turn then sews those two copies together — the seam. They are the same
/// curve in the same place, so the honest answer is not to make them: the band is
/// bounded by the two circles its endpoints sweep, each running the whole turn,
/// and the source edge appears on it nowhere. On a periodic support the circles
/// close only on the quotient and the band is a ring; on a plane they close in
/// the support's own parameters and it is an annulus. `kinds` carries which,
/// decided by [`revolved_band_loop_kinds`].
///
/// Each circle starts as a closed one-edge loop. The cut joins their 2-cells
/// while keeping them separate profiles, and carries no logical edge. The
/// neighbouring band sews to each circle through its boundary darts.
fn add_full_revolved_band_face<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    ends: [Point3; 2],
    circles: [Curve; 2],
    surface: Surface,
    pcurves: [TrimmedCurve2; 2],
    kinds: [LoopKind; 2],
) -> Result<RevolvedFace, RevolveError> {
    let [start_first, start_second, end_first, end_second]: [Dart; 4] =
        std::array::from_fn(|_| edit.add_dart());
    for (first, second) in [(start_first, start_second), (end_first, end_second)] {
        edit.link(Dim::Zero, first, second)?;
        edit.link(Dim::One, first, second)?;
    }

    for (seed, point, curve) in [
        (start_first, ends[0], &circles[0]),
        (end_first, ends[1], &circles[1]),
    ] {
        edit.add_vertex(VertexAttr::new(seed, point));
        edit.add_edge(EdgeAttr::new(seed, curve.clone()));
        edit.add_profile(ProfileAttr::new(seed));
    }

    let [start_pcurve, end_pcurve] = pcurves;
    let key = edit.add_face(FaceAttr::with_loops(
        surface,
        vec![
            LoopDefinition::from_kind(start_first, kinds[0]),
            LoopDefinition::from_kind(end_first, kinds[1]),
        ],
        HashMap::from([(start_first, start_pcurve), (end_first, end_pcurve)]),
    ));
    cut_between_loops(edit, key, start_first, end_first)?;

    Ok(RevolvedFace {
        bottom_edge: None,
        top_edge: None,
        // A neighbouring band must traverse the shared circle the other way
        // round, so the sew has to land on the far dart of this loop rather than
        // on its seed: `alpha2` of one seed then reaches the other seed through
        // `alpha0`, which is what a consistently oriented shell means. The two
        // ends are therefore offered asymmetrically, as a swept wrapping face
        // offers its bottom and top.
        start_side: start_second,
        end_side: end_first,
        outer_loop: start_first,
        key,
    })
}

fn sew_revolved_faces<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    faces: &[RevolvedFace],
    close_ring: bool,
) -> Result<(), RevolveError> {
    for i in 0..faces.len().saturating_sub(1) {
        sew_revolved_side_edges(edit, faces[i].end_side, faces[i + 1].start_side)?;
    }

    if close_ring && !faces.is_empty() {
        sew_revolved_side_edges(edit, faces[0].start_side, faces[faces.len() - 1].end_side)?;
    }

    Ok(())
}

struct Alpha2RevolveMerge {
    survivor_edge: EdgeKey,
    removed_edge: EdgeKey,
    survivor_start: VertexKey,
    removed_start: VertexKey,
    survivor_end: VertexKey,
    removed_end: VertexKey,
}

fn sew_revolved_side_edges<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    survivor: Dart,
    removed: Dart,
) -> Result<(), RevolveError> {
    sew_revolved_alpha2_edges(edit, survivor, removed, survivor, removed)
}

pub(crate) fn sew_revolved_alpha2_edges<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    first: Dart,
    second: Dart,
    survivor: Dart,
    removed: Dart,
) -> Result<(), RevolveError> {
    let merge = alpha2_revolve_merge(edit, first, second, survivor, removed)?;
    edit.sew(Dim::Two, first, second)
        .map_err(|_| RevolveError::SewFailed {
            dim: Dim::Two,
            first,
            second,
        })?;
    if merge.survivor_edge != merge.removed_edge {
        edit.merge_edges_into(merge.survivor_edge, merge.removed_edge);
    }
    let mut removed_vertices = Vec::new();
    if merge.survivor_start != merge.removed_start {
        edit.merge_vertices_into(merge.survivor_start, merge.removed_start);
        removed_vertices.push(merge.removed_start);
    }
    if merge.survivor_end != merge.removed_end && !removed_vertices.contains(&merge.removed_end) {
        edit.merge_vertices_into(merge.survivor_end, merge.removed_end);
    }
    Ok(())
}

fn alpha2_revolve_merge<P: Payload>(
    g: &Model<P>,
    first: Dart,
    second: Dart,
    survivor: Dart,
    removed: Dart,
) -> Result<Alpha2RevolveMerge, RevolveError> {
    let first_edge =
        Edge::from_dart(g, first).ok_or(RevolveError::MissingEdgeCurve { dart: first })?;
    let second_edge =
        Edge::from_dart(g, second).ok_or(RevolveError::MissingEdgeCurve { dart: second })?;

    let (first_start, first_end) = edge_end_vertices(g, first_edge.dart())?;
    let (second_start, second_end) = edge_end_vertices(g, second_edge.dart())?;

    if survivor == first && removed == second {
        return Ok(Alpha2RevolveMerge {
            survivor_edge: first_edge.key(),
            removed_edge: second_edge.key(),
            survivor_start: first_start,
            removed_start: second_start,
            survivor_end: first_end,
            removed_end: second_end,
        });
    }

    if survivor == second && removed == first {
        return Ok(Alpha2RevolveMerge {
            survivor_edge: second_edge.key(),
            removed_edge: first_edge.key(),
            survivor_start: second_start,
            removed_start: first_start,
            survivor_end: second_end,
            removed_end: first_end,
        });
    }

    Err(RevolveError::MissingEdgeCurve { dart: survivor })
}

/// Returns the trajectory swept by `point` turning `angle` around `axis`.
///
/// The circle parameterization follows the requested turn direction. Its edge
/// endpoints select the swept section of that support.
fn revolve_circle_curve(axis: Axis3, point: Point3, angle: Rad64) -> Curve {
    let projected = axis.project(point);
    let radius = distance(&projected, &point);
    let direction = if angle.val() < 0.0 {
        -axis.direction
    } else {
        axis.direction
    };
    let plane = if radius <= LINEAR_TOLERANCE {
        Circle::from_axis(Axis3::new(projected, direction), radius)
            .plane()
            .clone()
    } else {
        Plane::new(projected, point - projected, direction)
    };
    Curve::circle(plane, radius)
}

/// Rejects source edges that cannot sweep a well-formed face.
fn validate_revolvable_radii(axis: Axis3, source: &RevolvedSourceEdge) -> Result<(), RevolveError> {
    let start_on_axis = revolve_radius(axis, source.start.point) <= LINEAR_TOLERANCE;
    let end_on_axis = revolve_radius(axis, source.end.point) <= LINEAR_TOLERANCE;
    match (start_on_axis, end_on_axis) {
        (true, true) => Err(RevolveError::EdgeOnRevolutionAxis { key: source.key }),
        (true, false) | (false, true) => {
            Err(RevolveError::ApexRevolveUnsupported { key: source.key })
        }
        (false, false) => Ok(()),
    }
}

fn revolve_radius(axis: Axis3, point: Point3) -> f64 {
    distance(&axis.project(point), &point)
}

fn quad_pcurves(uv: &[TrimmedCurve2; 4], darts: &[Dart]) -> HashMap<Dart, TrimmedCurve2> {
    let mut pcurves = HashMap::with_capacity(4);
    for i in 0..4 {
        pcurves.insert(darts[2 * i], uv[i].clone());
    }
    pcurves
}

/// Revolves an existing face into a solid around `axis`.
///
/// For a partial turn, the source face and a rotated copy form the caps and the
/// revolved boundary loops form the lateral shell. A full turn omits coincident
/// caps and builds the shell directly from the face's outer and inner loops.
/// `angle` is clamped between zero and one full turn.
///
/// Returns an error if the face is missing, its loops cannot be revolved, a
/// partial turn uses an unsupported cap surface, or generated topology cannot
/// be sewn.
pub fn add_revolved_face<P: Payload>(
    g: &mut Model<P>,
    face_key: FaceKey,
    axis: Axis3,
    angle: Rad64,
) -> Result<SolidKey, RevolveError> {
    g.transaction(|edit| add_revolved_face_staged(edit, face_key, axis, angle))
}

/// Builds caps and lateral sheets, then registers the resulting staged solid.
fn add_revolved_face_staged<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face_key: FaceKey,
    axis: Axis3,
    angle: Rad64,
) -> Result<SolidKey, RevolveError> {
    let angle = angle.clamp(Angle::ZERO, Angle::FULL_TURN);
    let face = edit
        .face_attr(face_key)
        .map(|attr| attr.face(edit))
        .ok_or(RevolveError::MissingFace { key: face_key })?;
    let loops = face
        .loops()
        .into_iter()
        .map(|loop_| loop_.dart())
        .collect::<Vec<_>>();

    let sweep = revolve_sweep_direction(axis, &face);
    let cap_faces_sweep = face.normal_at(0.0, 0.0).dot(&sweep);

    if is_full_turn(angle) {
        return add_full_revolved_face(edit, face_key, loops, axis, angle, cap_faces_sweep);
    }

    let rotated_face = rotate_face(&face, axis, angle)?;
    let top_face_dart = edit.merge(rotated_face.face());
    let top_face_key = *edit.attribute_unchecked::<Cell2>(top_face_dart);
    let top_face_attr = edit.face_attr_unchecked(top_face_key);
    let mut top_loops = Vec::with_capacity(1 + top_face_attr.inner().count());
    top_loops.push(top_face_attr.outer_unchecked());
    top_loops.extend(top_face_attr.inner());

    let mut lateral_faces = Vec::new();
    for (bottom_loop, top_loop) in loops.into_iter().zip(top_loops) {
        let revolved = sew_revolved_loop_to_caps(edit, bottom_loop, top_loop, axis, angle)?;
        lateral_faces.extend(revolved.faces);
    }

    orient_revolved_shell(
        edit,
        face_key,
        top_face_key,
        &lateral_faces,
        cap_faces_sweep,
    );

    // Contextual, like the extruded shell: it must keep the outward
    // orientation just established for the source cap.
    let shell = edit.face_attr_unchecked(face_key).outer_unchecked();
    if edit.sheet_key(shell).is_none() {
        edit.add_sheet(SheetAttr::new(shell));
    }
    Ok(edit.add_solid(SolidAttr::new(shell, None)))
}

/// Returns the direction the revolution sweeps a point of `face`, `axis x r`.
fn revolve_sweep_direction<P: Payload>(axis: Axis3, face: &Face<'_, P>) -> Vector3<f64> {
    let point = face
        .edges()
        .first()
        .map(|edge| edge.trimmed_curve().point_at(Fraction::new(0.0)))
        .unwrap_or(axis.origin);
    axis.direction.cross(&(point - axis.project(point)))
}

/// Makes every face of a revolved shell point away from the material.
///
/// The cap at angle zero must face against the sweep, so exactly one of the two
/// caps is reversed. The lateral faces carry `dS/du x dS/dv`, which points at
/// the inside of the swept region precisely when the source cap already faces
/// against the sweep, so they flip together with the far cap.
fn orient_revolved_shell<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    bottom_face: FaceKey,
    top_face: FaceKey,
    lateral_faces: &[FaceKey],
    bottom_normal_dot_sweep: f64,
) {
    if bottom_normal_dot_sweep > LINEAR_TOLERANCE {
        reverse_face_winding(edit, bottom_face);
        return;
    }

    reverse_face_winding(edit, top_face);
    for &face in lateral_faces {
        reverse_face_winding(edit, face);
    }
}

/// Builds the shell of a full turn, which has no caps.
///
/// Every lateral face closes onto itself along the seam at angle zero, so the
/// swept edge at the end of the turn is sewn back to the one at its start. The
/// source face is then dropped: it is interior to the solid, and its boundary
/// wire survives only as the loops the lateral faces were built from.
fn add_full_revolved_face<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face_key: FaceKey,
    loops: Vec<Dart>,
    axis: Axis3,
    angle: Rad64,
    source_normal_dot_sweep: f64,
) -> Result<SolidKey, RevolveError> {
    let mut shell = None;
    let mut lateral_faces = Vec::new();
    for &loop_dart in &loops {
        let revolved = add_revolved_profile_faces(edit, loop_dart, axis, angle, true)?;
        lateral_faces.extend(revolved.faces);
        shell.get_or_insert(revolved.swept_dart);
    }

    if source_normal_dot_sweep <= LINEAR_TOLERANCE {
        for &face in &lateral_faces {
            reverse_face_winding(edit, face);
        }
    }

    let shell = shell.expect("a face should have at least one boundary loop");
    let shell = consume_revolved_source_face(edit, face_key, &loops, shell)?;
    if edit.sheet_key(shell).is_none() {
        edit.add_sheet(SheetAttr::new(shell));
    }
    Ok(edit.add_solid(SolidAttr::new(shell, None)))
}

/// Deletes the source face and its boundary wire after a full turn.
///
/// A full turn leaves the source geometry strictly inside the solid, where a
/// leftover face and dangling wire would be counted by every cell traversal.
/// Removing darts compacts the map, so `shell` is returned remapped.
fn consume_revolved_source_face<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face_key: FaceKey,
    loops: &[Dart],
    shell: Dart,
) -> Result<Dart, RevolveError> {
    edit.remove_face(face_key);

    let mut darts = Vec::new();
    for &loop_dart in loops {
        let profile = Profile::from_dart(edit, loop_dart)
            .expect("source loop must have a registered profile");
        let profile_key = profile.key();
        let loop_darts = profile.darts().collect::<Vec<_>>();
        for dart in loop_darts.iter().copied().step_by(2) {
            if let Some(edge) = Edge::from_dart(edit, dart) {
                // Only the vertex this dart sits on: the loop removes vertices as
                // it walks, so by now the far end of the edge may already be gone.
                let vertex_key = Vertex::from_dart(edit, dart)
                    .map(|vertex| vertex.key())
                    .ok_or(RevolveError::MissingVertexPoint { dart })?;
                let edge_key = edge.key();
                edit.remove_edge(edge_key);
                edit.remove_vertex(vertex_key);
            }
        }
        edit.remove_profile(profile_key);
        darts.extend(loop_darts);
    }

    for &dart in &darts {
        for dim in [Dim::Zero, Dim::One, Dim::Two, Dim::Three] {
            if !edit.is_free(dart, dim) {
                edit.unlink(dim, dart)?;
            }
        }
    }

    let isolated = darts.into_iter().map(IsolatedDart::new).collect();
    let remapped = edit.remove_isolated_darts(isolated);
    Ok(remapped.get(&shell).copied().unwrap_or(shell))
}

fn sew_revolved_loop_to_caps<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    bottom_loop: Dart,
    top_loop: Dart,
    axis: Axis3,
    angle: Rad64,
) -> Result<RevolvedProfile, RevolveError> {
    let bottom_edges = Profile::from_dart(edit, bottom_loop)
        .expect("bottom loop must have a registered profile")
        .edges()
        .into_iter()
        .map(|edge| edge.dart())
        .collect::<Vec<_>>();
    let top_edges = Profile::from_dart(edit, top_loop)
        .expect("top loop must have a registered profile")
        .edges()
        .into_iter()
        .map(|edge| edge.dart())
        .collect::<Vec<_>>();
    let revolved = add_revolved_profile_faces(edit, bottom_loop, axis, angle, true)?;

    // `Profile::edges` yields the dart at each edge's traversal start, and
    // `Edge::start` is the vertex at that dart, so an alpha2 sew is only
    // vertex-correct when both darts sit on the same 3D point. `bottom_edge`
    // and `top_edge` are chosen to satisfy that against the cap loop darts.
    for (&cap_edge, &side_edge) in bottom_edges.iter().zip(revolved.bottom_edges.iter()) {
        sew_revolved_alpha2_edges(edit, side_edge, cap_edge, cap_edge, side_edge)?;
    }
    for (&cap_edge, &side_edge) in top_edges.iter().zip(revolved.top_edges.iter()) {
        sew_revolved_alpha2_edges(edit, side_edge, cap_edge, cap_edge, side_edge)?;
    }

    Ok(revolved)
}

fn rotate_face<P: Payload>(
    face: &Face<'_, P>,
    axis: Axis3,
    angle: Rad64,
) -> Result<Shape<FaceTag, P>, RevolveError> {
    let (mut rotated, rotated_dart) = face.isolate();

    let vertex_keys = rotated
        .iter_vertices()
        .map(|(key, _)| key)
        .collect::<Vec<_>>();
    let edge_keys = rotated.iter_edges().map(|(key, _)| key).collect::<Vec<_>>();
    let rotated_face_key = *rotated.attribute_unchecked::<Cell2>(rotated_dart);
    // A rigid motion preserves every parameterisation, so the copy keeps the
    // source face's pcurves unchanged and no interval has to be recomputed.
    let motion = Rigid::rotation(axis, angle);
    rotated.transaction(|edit| {
        for key in vertex_keys {
            let vertex = edit.vertex_attr_mut_unchecked(key);
            vertex.point = motion.apply(vertex.point);
        }

        for key in edge_keys {
            let edge = edit.edge_attr_mut_unchecked(key);
            edge.curve = edge.curve.moved(&motion);
        }

        edit.face_attr_mut_unchecked(rotated_face_key).surface = edit
            .face_attr_mut_unchecked(rotated_face_key)
            .surface
            .moved(&motion);
        Ok::<_, RevolveError>(())
    })?;

    Ok(Shape::new(rotated, rotated_face_key))
}

/// Returns the key of the face incident to `dart`, if the dart belongs to one.
pub fn face_key_for_dart<P: Payload>(g: &Model<P>, dart: Dart) -> Option<FaceKey> {
    g.attribute::<Cell2>(dart).copied()
}
