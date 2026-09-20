use std::collections::HashSet;

use super::*;
use crate::builders::edges::EdgeSplitError;
use crate::builders::errors::FaceCreationError;
use crate::builders::profiles::profile_pcurves;
use crate::geometry::parameter::Fraction;
use crate::geometry::{
    Curve, Interval, LINEAR_TOLERANCE, Plane, Point2, Point3, Surface, SurfacePeriodicity,
    TrimmedCurve2,
};
use crate::model::Model;
use crate::topology::ModelEdit;
use crate::topology::attributes::{EdgeAttr, FaceAttr, ProfileAttr};
use crate::topology::edge::Edge;
use crate::topology::embedding::EntityOwner;
use crate::topology::gmap::{Dart, Dim};
use crate::topology::orientation::Orientation;
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;
use crate::topology::shape_keys::{EdgeKey, FaceKey};

pub fn add_annulus<P: Payload>(
    g: &mut Model<P>,
    plane: Plane,
    outer_radius: f64,
    inner_radius: f64,
) -> Result<FaceKey, FaceCreationError> {
    g.transaction(|edit| add_annulus_edit(edit, plane, outer_radius, inner_radius))
}

/// Builds both annulus boundaries, bridges them, and registers the face.
///
/// The two rims are joined by a **bridge**: one 1-cell the face's boundary
/// walk uses twice, with the two uses `alpha2`-linked to each other. Without it
/// the rims would sit in two disconnected 2-cells and only the face attribute
/// would say they belong to the same face; with it the face is one raw 2-cell
/// and the involutions alone carry that fact.
///
/// The bridge is scaffold, not shape: it is owned by the face, so it is never
/// emitted as a boundary and carries no logical edge of its own. Each of its
/// feet is the 0-cell where a rim closes, owned by that rim's edge — which is
/// what keeps both circles unmarked.
pub(crate) fn add_annulus_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    plane: Plane,
    outer_radius: f64,
    inner_radius: f64,
) -> Result<FaceKey, FaceCreationError> {
    if inner_radius >= outer_radius {
        return Err(FaceCreationError::InvalidAnnulusRadii {
            outer_radius,
            inner_radius,
        });
    }

    // The boundary word, in cyclic order: out along the bridge, once round the
    // hole, back along the bridge, once round the outer rim. Reading the bridge
    // before and after the hole is what makes the walk turn out of the hole and
    // back onto the outer rim rather than round the hole for ever.
    let slots: Vec<[Dart; 2]> = (0..4).map(|_| [edit.add_dart(), edit.add_dart()]).collect();
    for slot in &slots {
        edit.link(Dim::Zero, slot[0], slot[1])?;
    }
    for i in 0..slots.len() {
        edit.link(Dim::One, slots[i][1], slots[(i + 1) % slots.len()][0])?;
    }
    let [bridge_out, inner_loop_slot, bridge_back, outer_loop_slot] =
        [0, 1, 2, 3].map(|i| slots[i]);
    edit.link(Dim::Two, bridge_out[0], bridge_back[1])?;
    edit.link(Dim::Two, bridge_out[1], bridge_back[0])?;

    let inner_plane = Plane::new(plane.origin(), plane.x_dir(), -plane.normal());
    let outer_edge = edit.add_edge(EdgeAttr::new(
        outer_loop_slot[0],
        Curve::circle(plane.clone(), outer_radius),
    ));
    let inner_edge = edit.add_edge(EdgeAttr::new(
        inner_loop_slot[0],
        Curve::circle(inner_plane, inner_radius),
    ));
    // Each rim closes where the bridge meets it. That 0-cell is interior to the
    // rim, not a corner: nothing else meets there, and the rim stays unmarked.
    edit.own_cell(Dim::Zero, outer_loop_slot[0], EntityOwner::Edge(outer_edge));
    edit.own_cell(Dim::Zero, inner_loop_slot[0], EntityOwner::Edge(inner_edge));

    let outer_loop = outer_loop_slot[0];
    let inner_loop = inner_loop_slot[0];
    edit.add_profile(ProfileAttr::new(outer_loop));
    edit.add_profile(ProfileAttr::new(inner_loop));

    let outer_profile =
        Profile::from_dart(edit, outer_loop).expect("outer loop must have a registered profile");
    let inner_profile =
        Profile::from_dart(edit, inner_loop).expect("inner loop must have a registered profile");
    let mut pcurves = profile_pcurves(&outer_profile, &plane)?;
    pcurves.extend(profile_pcurves(&inner_profile, &plane)?);

    let face_key = edit.add_face(FaceAttr::with_pcurves(
        Surface::Plane(plane),
        outer_loop,
        vec![inner_loop],
        pcurves,
    ));
    // The bridge belongs to the face's interior, which is what stops the
    // boundary walk emitting it and lets the walk cross it into the hole.
    edit.own_cell(Dim::One, bridge_out[0], EntityOwner::Face(face_key));
    Ok(face_key)
}

pub(crate) fn face_edge_dart<P: Payload>(
    g: &Model<P>,
    face: FaceKey,
    edge: EdgeKey,
) -> Result<Dart, FaceEdgeSplitError> {
    let face_attr = g
        .face_attr(face)
        .ok_or(FaceEdgeSplitError::MissingFace { face })?;
    let edge_attr = g
        .edge_attr(edge)
        .ok_or(FaceEdgeSplitError::EdgeSplitFailed(
            EdgeSplitError::MissingEdge { edge },
        ))?;
    let edge_dart = g.cell_representative(edge_attr.dart, Dim::One);
    let profile_darts: Vec<Dart> = face_attr
        .darts()
        .flat_map(|loop_dart| {
            Profile::from_dart(g, loop_dart)
                .expect("face loop must have a registered profile")
                .darts()
                .step_by(2)
                .collect::<Vec<_>>()
        })
        .collect();
    profile_darts
        .into_iter()
        .find(|profile_dart| g.cell_representative(*profile_dart, Dim::One) == edge_dart)
        .ok_or(FaceEdgeSplitError::EdgeNotOnFace { face, edge })
}

pub(crate) fn closed_boundary_curve_reversed<P: Payload>(
    g: &Model<P>,
    face: FaceKey,
    edge: EdgeKey,
    dart: Dart,
) -> Result<bool, FaceEdgeSplitError> {
    let edge_view =
        Edge::from_dart(g, dart).ok_or(FaceEdgeSplitError::EdgeNotOnFace { face, edge })?;
    // Only a closed edge can have its pcurve reversed relative to its curve
    // without that showing up in its endpoints — asked of the map rather than by
    // measuring whether two points happen to land within a tolerance.
    let (Edge::Marked(_) | Edge::Unmarked(_)) = edge_view else {
        return Ok(false);
    };

    let face_view = g
        .face(face)
        .ok_or(FaceEdgeSplitError::MissingFace { face })?;
    let pcurve = face_view
        .pcurve(dart)
        .ok_or(FaceEdgeSplitError::MissingPcurve { face, dart })?;
    // Against the edge's own span, not the support's extent: a marked closed
    // edge begins at its corner, which is not in general where the support's
    // domain starts. Reading the support would compare the sample with two
    // points it is nowhere near, and a closed span's ends are the one place
    // the two directions are told apart.
    let reference = g
        .edge(edge)
        .ok_or(FaceEdgeSplitError::EdgeSplitFailed(
            EdgeSplitError::MissingEdge { edge },
        ))?
        .parameter_interval();
    let curve = edge_view.curve();
    let fraction = 1.0e-4;
    let sample_uv = pcurve.point_at(Fraction::new(fraction));
    let sample = face_view.point_at(sample_uv.x, sample_uv.y);
    let forward = curve.point_at(reference.at(Fraction::new(fraction)));
    let reverse = curve.point_at(reference.at(Fraction::new(1.0 - fraction)));
    let against = (sample - reverse).norm_squared() < (sample - forward).norm_squared();

    // A closed boundary starts and ends at one point, so only its direction
    // says which half is which. The sample above follows the face from `dart`,
    // while the split hands its first half to the edge's own reference dart. On
    // an open edge the two endpoints settle that; here, when those two darts
    // run opposite ways, the halves have to be read the other way round.
    Ok(match g.edge_orientation_at_dart(edge, dart) {
        Orientation::Same => against,
        Orientation::Reversed => !against,
    })
}

/// Where a fraction of an edge's own span falls in model space.
pub(crate) fn edge_point_at<P: Payload>(
    g: &Model<P>,
    edge: EdgeKey,
    parameter: Fraction,
) -> Result<Point3, FaceEdgeSplitError> {
    let edge_view = g.edge(edge).ok_or(FaceEdgeSplitError::EdgeSplitFailed(
        EdgeSplitError::MissingEdge { edge },
    ))?;
    Ok(edge_view
        .curve()
        .point_at(edge_view.parameter_interval().at(parameter)))
}

/// Every boundary dart a face uses `edge` at.
///
/// A seam has two boundary occurrences on one face, each with its own UV curve.
pub(crate) fn edge_face_occurrences<P: Payload>(
    g: &Model<P>,
    edge: EdgeKey,
) -> Result<HashSet<(FaceKey, Dart)>, FaceEdgeSplitError> {
    let edge_view = g.edge(edge).ok_or(FaceEdgeSplitError::EdgeSplitFailed(
        EdgeSplitError::MissingEdge { edge },
    ))?;
    let mut occurrences = HashSet::new();
    for face in edge_view.faces() {
        for boundary in g.face_unchecked(face.key()).edges() {
            if boundary.key() == edge {
                occurrences.insert((face.key(), boundary.dart()));
            }
        }
    }
    Ok(occurrences)
}

pub(crate) fn incident_face_pcurves<P: Payload>(
    g: &Model<P>,
    edge: EdgeKey,
    parameter: Fraction,
) -> Result<Vec<IncidentFacePcurve>, FaceEdgeSplitError> {
    let split_point = edge_point_at(g, edge, parameter)?;
    edge_face_occurrences(g, edge)?
        .into_iter()
        .map(|(face, dart)| {
            let face_view = g
                .face(face)
                .ok_or(FaceEdgeSplitError::MissingFace { face })?;
            let pcurve = face_view
                .pcurve(dart)
                .ok_or(FaceEdgeSplitError::MissingPcurve { face, dart })?;
            let surface = face_view.surface();
            let uv = periodic_image_near_pcurve(surface, &pcurve, surface.param_at(split_point)?);
            let fraction = pcurve
                .try_parameter_at(uv, LINEAR_TOLERANCE)
                .ok_or(FaceEdgeSplitError::SplitPointNotOnPcurve { face, dart })?;
            Ok(IncidentFacePcurve {
                face,
                dart,
                pcurve,
                fraction,
            })
        })
        .collect()
}

/// Turns each face pcurve of a closed edge to begin where a mark lands.
///
/// A marked closed edge derives its span from its corner, so the span begins at
/// the mark rather than wherever the support's own domain does. A pcurve has no
/// corner to derive from and carries its own span, so it has to be turned with
/// it: left where it was it would run the same closed curve from a different
/// starting point, and every later cut would read the wrong arc of it.
pub(crate) fn rebased_face_pcurves<P: Payload>(
    g: &Model<P>,
    edge: EdgeKey,
    parameter: Fraction,
) -> Result<Vec<RebasedFacePcurve>, FaceEdgeSplitError> {
    let mark = edge_point_at(g, edge, parameter)?;
    edge_face_occurrences(g, edge)?
        .into_iter()
        .filter_map(|(face, dart)| rebased_face_pcurve(g, face, dart, mark).transpose())
        .collect()
}

/// One face's pcurve for a closed edge, turned to begin at `mark`.
///
/// `None` where there is nothing to turn: a pcurve that does not close in
/// parameter space runs between two distinct ends — a seam crossing, where the
/// face's own domain is what pins them — and only a closed one is free to say
/// where it starts.
pub(crate) fn rebased_face_pcurve<P: Payload>(
    g: &Model<P>,
    face: FaceKey,
    dart: Dart,
    mark: Point3,
) -> Result<Option<RebasedFacePcurve>, FaceEdgeSplitError> {
    let face_view = g
        .face(face)
        .ok_or(FaceEdgeSplitError::MissingFace { face })?;
    let pcurve = face_view
        .pcurve(dart)
        .ok_or(FaceEdgeSplitError::MissingPcurve { face, dart })?;
    if !pcurve.is_closed() {
        return Ok(None);
    }
    let surface = face_view.surface();
    let uv = periodic_image_near_pcurve(surface, &pcurve, surface.param_at(mark)?);
    if !pcurve.contains(uv, LINEAR_TOLERANCE) {
        return Err(FaceEdgeSplitError::SplitPointNotOnPcurve { face, dart });
    }
    // The span keeps its length and direction and only moves its ends, so the
    // turned pcurve still runs the whole closed curve once.
    let start = pcurve.native_parameter_at(uv);
    let span = Interval::new(start, start + pcurve.interval().delta());
    Ok(Some(RebasedFacePcurve {
        face,
        dart: stored_pcurve_dart(g, face, dart).unwrap_or(dart),
        pcurve: TrimmedCurve2::new(pcurve.into_curve(), span),
    }))
}

/// The dart a face keeps a boundary occurrence's pcurve under.
///
/// A pcurve is stored once per boundary occurrence but looked up from any of
/// the darts that occurrence covers, so replacing one means writing back to the
/// key it already lives at rather than adding a second entry beside it.
pub(crate) fn stored_pcurve_dart<P: Payload>(
    g: &Model<P>,
    face: FaceKey,
    dart: Dart,
) -> Option<Dart> {
    let attr = g.face_attr(face)?;
    [dart, g.alpha(Dim::Zero, dart), g.alpha(Dim::Two, dart)]
        .into_iter()
        .find(|candidate| attr.pcurves.contains_key(candidate))
}

/// Stores a turned pcurve back on the face it belongs to.
pub(crate) fn assign_rebased_pcurve<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    rebased: RebasedFacePcurve,
) -> Result<(), FaceEdgeSplitError> {
    let face_attr = edit
        .face_attr_mut(rebased.face)
        .ok_or(FaceEdgeSplitError::MissingFace { face: rebased.face })?;
    face_attr.pcurves.insert(rebased.dart, rebased.pcurve);
    Ok(())
}

pub(crate) fn periodic_image_near_pcurve(
    surface: &Surface,
    pcurve: &TrimmedCurve2,
    mut uv: Point2,
) -> Point2 {
    let start = pcurve.point_at(Fraction::new(0.0));
    let end = pcurve.point_at(Fraction::new(1.0));
    let center = Point2::from((start.coords + end.coords) * 0.5);
    match surface.periodicity() {
        SurfacePeriodicity::UPeriodic(period) => {
            uv.x += ((center.x - uv.x) / period).round() * period;
        }
        SurfacePeriodicity::VPeriodic(period) => {
            uv.y += ((center.y - uv.y) / period).round() * period;
        }
        SurfacePeriodicity::UVPeriodic(u_period, v_period) => {
            uv.x += ((center.x - uv.x) / u_period).round() * u_period;
            uv.y += ((center.y - uv.y) / v_period).round() * v_period;
        }
        SurfacePeriodicity::None => {}
    }
    uv
}

pub(crate) fn assign_split_pcurves<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    pcurve: IncidentFacePcurve,
) -> Result<(), FaceEdgeSplitError> {
    let second_dart = edit.alpha(Dim::One, edit.alpha(Dim::Zero, pcurve.dart));
    let (first_pcurve, second_pcurve) = pcurve.pcurve.split_at(pcurve.fraction);
    let face_attr = edit
        .face_attr_mut(pcurve.face)
        .ok_or(FaceEdgeSplitError::MissingFace { face: pcurve.face })?;
    face_attr.pcurves.remove(&pcurve.dart);
    face_attr.pcurves.insert(pcurve.dart, first_pcurve);
    face_attr.pcurves.insert(second_dart, second_pcurve);
    Ok(())
}
