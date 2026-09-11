use crate::geometry::TrimmedCurve2;
use std::collections::HashMap;
use std::f64::consts::FRAC_PI_2;

use nalgebra::Vector3;
use thiserror::Error;

use crate::{
    Payload,
    builders::faces::reverse_face_winding,
    builders::{
        edges::add_arc_staged,
        errors::{EdgeCreationError, ExtrudeError},
        revolve::{RevolveError, add_full_revolved_edge_staged_with_surface},
    },
    geometry::{
        ANGULAR_TOLERANCE, Axis2, Curve, Cylinder, Frame, LINEAR_TOLERANCE, Plane, Point2, Point3,
        RuledSurface, Sphere, Surface, SurfacePeriodicity,
    },
    topology::{
        Dart, SheetAttr, SolidAttr, TopologyEdit,
        attributes::{EdgeAttr, FaceAttr, LoopDefinition, ProfileAttr},
        edge::Edge,
        face::Face,
        gmap::{Cell2, Dim, GMap, MergeTopology, TopologyEditError},
        profile::Profile,
        shape::{FaceTag, Shape},
        shape_keys::{FaceKey, SolidKey},
    },
};

#[derive(Debug, Error)]
pub enum SphereBuildError {
    #[error("failed to create a sphere profile arc")]
    Arc(#[from] EdgeCreationError),
    #[error("failed to revolve a sphere profile arc")]
    Revolve(#[from] RevolveError),
    #[error("failed to commit sphere topology")]
    TopologyEdit(#[from] TopologyEditError),
}

/// Adds a sphere by revolving a semicircular meridian around the frame z-axis.
///
/// The two coincident meridian boundary occurrences are sewn together, leaving
/// one face with one pole-to-pole seam edge and two pole vertices.
pub fn add_sphere<P: Payload>(
    g: &mut GMap<P>,
    frame: Frame,
    radius: f64,
) -> Result<SolidKey, SphereBuildError> {
    g.transaction(|edit| {
        let meridian = Plane::from_xy(frame.origin, frame.x_dir, frame.z_dir);
        let axis = frame.z_axis();
        let arc = add_arc_staged(edit, meridian, radius, FRAC_PI_2, -FRAC_PI_2)?;
        let face = add_full_revolved_edge_staged_with_surface(
            edit,
            arc,
            axis,
            Surface::Sphere(Sphere::new(frame, radius)),
            |point| Point2::new(point.y, -point.x),
        )?;
        let seam = edit.face_unchecked(face).dart();

        edit.add_sheet(SheetAttr::new(seam, P::Sheet::default()));
        Ok(edit.add_solid(SolidAttr::new(P::S::default(), seam, None)))
    })
}

/// Returns an isolated copy of `face` translated by `direction`.
///
/// Vertex positions, edge curves, and the supporting surface are translated;
/// face pcurves remain unchanged because they use the face's local parameter
/// space. The source map is not modified.
///
/// Returns an error for a zero direction or when curve or surface geometry
/// cannot be translated.
pub fn translate_face<P: Payload>(
    face: &Face<'_, P>,
    direction: Vector3<f64>,
) -> Result<Shape<FaceTag, P>, ExtrudeError> {
    if direction.norm_squared() <= LINEAR_TOLERANCE * LINEAR_TOLERANCE {
        return Err(ExtrudeError::ZeroDirection);
    }

    let (mut translated, translated_dart) = face.isolate();

    let vertex_keys = translated
        .iter_vertices()
        .map(|(key, _)| key)
        .collect::<Vec<_>>();
    let edge_keys = translated
        .iter_edges()
        .map(|(key, _)| key)
        .collect::<Vec<_>>();
    let translated_face_key = *translated.attribute_unchecked::<Cell2>(translated_dart);
    translated.transaction(|edit| {
        for key in vertex_keys {
            edit.vertex_attr_mut_unchecked(key).point += direction;
        }

        for key in edge_keys {
            let edge = edit.edge_attr_mut_unchecked(key);
            edge.curve = edge.curve.translated(direction).map_err(|source| {
                ExtrudeError::CurveTranslationFailed {
                    dart: edge.dart,
                    source,
                }
            })?;
        }

        let translated_face = edit.face_attr_mut_unchecked(translated_face_key);
        translated_face.surface =
            translated_face
                .surface
                .translated(direction)
                .map_err(|source| ExtrudeError::SurfaceTranslationFailed {
                    dart: translated_face.outer_unchecked(),
                    source,
                })?;
        Ok::<_, ExtrudeError>(())
    })?;

    Ok(Shape::new(translated, translated_face_key))
}

/// Extrudes an existing face into a solid along `direction`.
///
/// The source face becomes one cap, a translated copy becomes the opposite cap,
/// and one lateral face is added for every edge of the outer and inner boundary
/// loops. Cap winding is adjusted so the resulting shell faces outward.
///
/// Returns an error when the face is missing, the direction is zero, required
/// boundary geometry is absent, or a lateral face is degenerate or cannot be
/// sewn into the shell.
pub fn add_extruded_face<P: Payload>(
    g: &mut GMap<P>,
    face_key: FaceKey,
    direction: Vector3<f64>,
) -> Result<SolidKey, ExtrudeError> {
    g.transaction(|edit| add_extruded_face_staged(edit, face_key, direction))
}

/// Builds translated caps and lateral faces, then registers the staged solid.
fn add_extruded_face_staged<P: Payload>(
    edit: &mut TopologyEdit<'_, P>,
    face_key: FaceKey,
    direction: Vector3<f64>,
) -> Result<SolidKey, ExtrudeError> {
    let bot_face = edit
        .face_attr(face_key)
        .map(|attr| attr.face(edit))
        .ok_or(ExtrudeError::MissingFace { dart: face_key })?;
    let top_face = translate_face(&bot_face, direction)?;
    let bottom_loop_darts = bot_face
        .loops()
        .into_iter()
        .map(|loop_| loop_.dart)
        .collect::<Vec<_>>();

    let top_face_dart = edit.merge(top_face.face());
    let top_face_key = *edit.attribute_unchecked::<Cell2>(top_face_dart);
    let top_face_attr = edit.face_attr_unchecked(top_face_key);
    let mut top_loop_darts = Vec::with_capacity(1 + top_face_attr.inner().count());
    top_loop_darts.push(top_face_attr.outer_unchecked());
    top_loop_darts.extend(top_face_attr.inner());

    orient_extruded_caps(edit, face_key, top_face_key, direction);

    for (bottom_loop_dart, top_loop_dart) in bottom_loop_darts.into_iter().zip(top_loop_darts) {
        sew_extruded_loop(edit, bottom_loop_dart, top_loop_dart, direction)?;
    }

    // The shell dart is contextual: unlike a cell representative, it must retain
    // the outward orientation established for the bottom cap.
    let outer_shell = edit.face_attr_unchecked(face_key).outer_unchecked();
    if edit.sheet_key(outer_shell).is_none() {
        edit.add_sheet(SheetAttr::new(outer_shell, P::Sheet::default()));
    }
    let solid = edit.add_solid(SolidAttr::new(P::S::default(), outer_shell, None));
    Ok(solid)
}

fn orient_extruded_caps<P: Payload>(
    edit: &mut TopologyEdit<'_, P>,
    bottom_face: FaceKey,
    top_face: FaceKey,
    direction: Vector3<f64>,
) {
    let Some(bottom_normal_dot_direction) = edit
        .face_attr(bottom_face)
        .map(|attr| face_normal_dot_direction(edit, attr, direction))
    else {
        return;
    };

    if bottom_normal_dot_direction > LINEAR_TOLERANCE {
        reverse_face_winding(edit, bottom_face);
    } else if bottom_normal_dot_direction < -LINEAR_TOLERANCE {
        reverse_face_winding(edit, top_face);
    }
}

fn face_normal_dot_direction<P: Payload>(
    g: &GMap<P>,
    face: &FaceAttr<P::F>,
    direction: Vector3<f64>,
) -> f64 {
    face.face(g).normal_at(0.0, 0.0).dot(&direction)
}

fn sew_extruded_loop<P: Payload>(
    edit: &mut TopologyEdit<'_, P>,
    bottom_loop_dart: Dart,
    top_loop_dart: Dart,
    direction: Vector3<f64>,
) -> Result<Dart, ExtrudeError> {
    let bottom_edges = Profile::from_dart(edit, bottom_loop_dart)
        .expect("bottom loop must have a registered profile")
        .edges()
        .into_iter()
        .map(|edge| edge.dart())
        .collect::<Vec<_>>();
    let top_edges = Profile::from_dart(edit, top_loop_dart)
        .expect("top loop must have a registered profile")
        .edges()
        .into_iter()
        .map(|edge| edge.dart())
        .collect::<Vec<_>>();
    if let Some(representative) =
        sew_wrapping_lateral_face(edit, &bottom_edges, &top_edges, direction)?
    {
        return Ok(edit.cell_representative(representative, Dim::Three));
    }

    let laterals = bottom_edges
        .iter()
        .copied()
        .zip(top_edges.iter().copied())
        .map(|(bottom_edge, top_edge)| {
            let prepared = prepare_lateral_face(edit, bottom_edge, top_edge, direction)?;
            let topology = add_lateral_face_topology(edit)?;
            add_lateral_face_attributes(edit, &topology, &prepared);
            Ok(ExtrudedFaceLateral {
                topology,
                vertical_start: prepared.end,
                vertical_end: prepared.end + direction,
            })
        })
        .collect::<Result<Vec<_>, ExtrudeError>>()?;

    for pair in laterals.windows(2) {
        sew(
            edit,
            Dim::Two,
            pair[0].topology.end_vertical,
            pair[1].topology.start_vertical,
        )?;
    }
    if let (Some(first), Some(last)) = (laterals.first(), laterals.last()) {
        sew(
            edit,
            Dim::Two,
            last.topology.end_vertical,
            first.topology.start_vertical,
        )?;
    }

    for ((bottom_edge, top_edge), lateral) in bottom_edges.iter().zip(top_edges).zip(&laterals) {
        sew(edit, Dim::Two, lateral.topology.bottom_edge, *bottom_edge)?;
        sew(edit, Dim::Two, lateral.topology.top_edge, top_edge)?;
    }

    for lateral in &laterals {
        edit.add_edge(EdgeAttr::new(
            lateral.topology.end_vertical,
            Curve::line(lateral.vertical_start, lateral.vertical_end),
            P::E::default(),
        ));
    }

    let representative = laterals
        .first()
        .map(|lateral| lateral.topology.bottom_edge)
        .expect("a loop should have at least one lateral face");
    Ok(edit.cell_representative(representative, Dim::Three))
}

/// Builds the lateral face of a sweep that closes on itself, if this one does.
///
/// Sweeping a single closed edge produces a surface closed in the swept
/// direction, so the face wraps: the swept edge bounds it at each end of the
/// sweep, and nothing bounds it across the sweep. Building it as a quad
/// instead needs two coincident vertical edges sewn to each other — the seam —
/// which records where the parameter domain was cut open, not what the shape
/// is.
///
/// Returns the new face's boundary dart, or `None` when the sweep does not
/// close and the caller should build quads.
fn sew_wrapping_lateral_face<P: Payload>(
    edit: &mut TopologyEdit<'_, P>,
    bottom_edges: &[Dart],
    top_edges: &[Dart],
    direction: Vector3<f64>,
) -> Result<Option<Dart>, ExtrudeError> {
    let ([bottom_edge], [top_edge]) = (bottom_edges, top_edges) else {
        return Ok(None);
    };
    let prepared = prepare_lateral_face(edit, *bottom_edge, *top_edge, direction)?;
    let Some(axis) = swept_period_axis(&prepared) else {
        return Ok(None);
    };

    // Two closed one-edge loops: the swept edge at the start of the sweep and
    // its image at the end. `alpha1` closes each onto itself, exactly as a
    // circular edge's own profile does.
    let [bottom_start, bottom_end, top_start, top_end]: [Dart; 4] =
        std::array::from_fn(|_| edit.add_dart());
    for (start, end) in [(bottom_start, bottom_end), (top_start, top_end)] {
        edit.link(Dim::Zero, start, end)?;
        edit.link(Dim::One, start, end)?;
    }

    edit.add_profile(ProfileAttr::new(bottom_start, P::Profile::default()));
    edit.add_profile(ProfileAttr::new(top_start, P::Profile::default()));
    let uv = prepared.uv;
    edit.add_face(FaceAttr::with_loops(
        prepared.surface.clone(),
        P::F::default(),
        vec![
            LoopDefinition::wrapping(bottom_start, axis),
            LoopDefinition::wrapping(top_start, axis),
        ],
        HashMap::from([
            (bottom_start, TrimmedCurve2::segment(uv[0], uv[1])),
            (top_start, TrimmedCurve2::segment(uv[2], uv[3])),
        ]),
    ));

    // The swept loop runs with the sweep at the bottom and against it at the
    // top, so the top loop meets its cap through `alpha0`, as the quad path's
    // top edge does.
    sew(edit, Dim::Two, bottom_start, *bottom_edge)?;
    sew(edit, Dim::Two, top_end, *top_edge)?;
    Ok(Some(bottom_start))
}

/// The axis a prepared lateral face spans a whole period of, if it spans one.
///
/// The sweep closes exactly when the swept edge covers one full period of the
/// surface it sweeps out — which is also the only case where the edge's two
/// ends are the same vertex, since a shorter span would leave them apart.
fn swept_period_axis(prepared: &PreparedLateralFace) -> Option<Axis2> {
    let span = prepared.uv[1] - prepared.uv[0];
    let periods = match prepared.surface.periodicity() {
        SurfacePeriodicity::None => return None,
        SurfacePeriodicity::UPeriodic(u) => [Some(u), None],
        SurfacePeriodicity::VPeriodic(v) => [None, Some(v)],
        SurfacePeriodicity::UVPeriodic(u, v) => [Some(u), Some(v)],
    };
    Axis2::ALL.into_iter().find(|axis| {
        periods[axis.index()]
            .is_some_and(|period| (span[axis.index()].abs() - period).abs() <= ANGULAR_TOLERANCE)
    })
}

fn sew<P: Payload>(
    edit: &mut TopologyEdit<'_, P>,
    dim: Dim,
    first: Dart,
    second: Dart,
) -> Result<(), ExtrudeError> {
    edit.sew(dim, first, second)
        .map_err(|_| ExtrudeError::SewFailed { dim, first, second })
}

struct PreparedLateralFace {
    end: Point3,
    surface: Surface,
    uv: [Point2; 4],
}

struct ExtrudedFaceLateral {
    topology: LateralFaceTopology,
    vertical_start: Point3,
    vertical_end: Point3,
}

struct LateralFaceTopology {
    loop_dart: Dart,
    bottom_edge: Dart,
    top_edge: Dart,
    start_vertical: Dart,
    end_vertical: Dart,
    darts: [Dart; 8],
}

fn prepare_lateral_face<P: Payload>(
    g: &GMap<P>,
    bottom_edge: Dart,
    _top_edge: Dart,
    direction: Vector3<f64>,
) -> Result<PreparedLateralFace, ExtrudeError> {
    let bottom_edge_view = Edge::from_dart(g, bottom_edge)
        .ok_or(ExtrudeError::MissingEdgeCurve { dart: bottom_edge })?;
    let edge_dart = bottom_edge_view.dart();
    // The ends of the section being swept: for a closed edge they are the same
    // point, which is what makes the swept wall wrap.
    let section = bottom_edge_view
        .trimmed_curve()
        .ok_or(ExtrudeError::MissingVertexPoint { dart: edge_dart })?;
    let (start, end) = (section.point_at(0.0), section.point_at(1.0));
    let curve = bottom_edge_view
        .curve()
        .ok_or(ExtrudeError::MissingEdgeCurve { dart: edge_dart })?;
    let surface = lateral_face_surface(edge_dart, curve, start, end, direction)?;
    let uv = lateral_face_uv(&surface, curve, start, end, direction);

    Ok(PreparedLateralFace { end, uv, surface })
}

fn add_lateral_face_topology<P: Payload>(
    edit: &mut TopologyEdit<'_, P>,
) -> Result<LateralFaceTopology, ExtrudeError> {
    let darts = std::array::from_fn(|_| edit.add_dart());

    for i in 0..4 {
        sew(edit, Dim::Zero, darts[2 * i], darts[2 * i + 1])?;
    }
    for i in 0..4 {
        sew(
            edit,
            Dim::One,
            darts[2 * i + 1],
            darts[(2 * i + 2) % darts.len()],
        )?;
    }

    Ok(LateralFaceTopology {
        loop_dart: darts[0],
        bottom_edge: darts[0],
        top_edge: darts[5],
        start_vertical: darts[7],
        end_vertical: darts[2],
        darts,
    })
}

fn add_lateral_face_attributes<P: Payload>(
    edit: &mut TopologyEdit<'_, P>,
    topology: &LateralFaceTopology,
    prepared: &PreparedLateralFace,
) {
    edit.add_profile(ProfileAttr::new(topology.loop_dart, P::Profile::default()));
    edit.add_face(FaceAttr::with_pcurves(
        prepared.surface.clone(),
        P::F::default(),
        topology.loop_dart,
        Vec::new(),
        quad_pcurves(&prepared.uv, &topology.darts),
    ));
}

fn lateral_face_surface(
    dart: Dart,
    curve: &Curve,
    start: Point3,
    end: Point3,
    direction: Vector3<f64>,
) -> Result<Surface, ExtrudeError> {
    match curve {
        Curve::Line(_) => Ok(Surface::Plane(lateral_plane(dart, start, end, direction)?)),
        _ => match extruded_cylinder(curve, direction) {
            // Sweeping a circle along its own normal is a cylinder, and saying
            // so here is what lets every later stage recognize the kernel's own
            // cylinders instead of an opaque ruled surface over a circle.
            Some(cylinder) => Ok(Surface::Cylinder(cylinder)),
            None => Ok(Surface::Ruled(RuledSurface::new(curve.clone(), direction))),
        },
    }
}

/// Returns the cylinder a circular `curve` sweeps along its own axis, if it does.
///
/// The cylinder shares the circle's origin, reference direction and normal, so
/// the circle's own parameter is the cylinder's `u` with no correction: the
/// sweep is the identity in `u` and a translation in `v`.
fn extruded_cylinder(curve: &Curve, direction: Vector3<f64>) -> Option<Cylinder> {
    let Curve::Circle(circle) = curve else {
        return None;
    };
    let normal = circle.plane().normal();
    let length = direction.norm();
    if length <= LINEAR_TOLERANCE || direction.cross(&normal).norm() > LINEAR_TOLERANCE * length {
        return None;
    }
    Some(Cylinder::new(
        circle.plane().origin(),
        circle.plane().x_dir(),
        normal,
        circle.radius(),
    ))
}

fn lateral_face_uv(
    surface: &Surface,
    curve: &Curve,
    start: Point3,
    end: Point3,
    direction: Vector3<f64>,
) -> [Point2; 4] {
    match surface {
        Surface::Plane(plane) => [
            plane_uv(plane, start),
            plane_uv(plane, end),
            plane_uv(plane, end + direction),
            plane_uv(plane, start + direction),
        ],
        Surface::Ruled(_) => {
            let interval = curve.interval_between(start, end);
            [
                Point2::new(interval.start, 0.0),
                Point2::new(interval.end, 0.0),
                Point2::new(interval.end, 1.0),
                Point2::new(interval.start, 1.0),
            ]
        }
        // The cylinder was built on the circle's own frame, so `u` is the
        // circle's parameter unchanged; `v` is a signed height along the axis
        // rather than the ruled surface's normalized sweep fraction.
        Surface::Cylinder(cylinder) => {
            let interval = curve.interval_between(start, end);
            let height = direction.dot(&cylinder.axis());
            [
                Point2::new(interval.start, 0.0),
                Point2::new(interval.end, 0.0),
                Point2::new(interval.end, height),
                Point2::new(interval.start, height),
            ]
        }
        _ => unreachable!("lateral_face_surface only creates plane, cylinder or ruled surfaces"),
    }
}

fn lateral_plane(
    dart: Dart,
    start: Point3,
    end: Point3,
    direction: Vector3<f64>,
) -> Result<Plane, ExtrudeError> {
    let edge = end - start;
    if edge.norm_squared() <= LINEAR_TOLERANCE * LINEAR_TOLERANCE {
        return Err(ExtrudeError::ZeroLengthEdge { dart });
    }
    if edge.cross(&direction).norm_squared() <= LINEAR_TOLERANCE * LINEAR_TOLERANCE {
        return Err(ExtrudeError::DegenerateSweep { dart });
    }
    Ok(Plane::from_xy(start, edge, direction))
}

fn plane_uv(surface: &Plane, point: Point3) -> Point2 {
    let v = point - surface.origin();
    Point2::new(v.dot(&surface.x_dir()), v.dot(&surface.y_dir()))
}

fn quad_pcurves(
    uv: &[Point2; 4],
    darts: &[Dart; 8],
) -> std::collections::HashMap<Dart, TrimmedCurve2> {
    let mut pcurves = std::collections::HashMap::with_capacity(4);
    for i in 0..4 {
        pcurves.insert(
            darts[2 * i],
            TrimmedCurve2::segment(uv[i], uv[(i + 1) % uv.len()]),
        );
    }
    pcurves
}
