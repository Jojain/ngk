use crate::builders::errors::FaceCreationError;
use crate::builders::profiles::profile_pcurves;
use crate::builders::scaffold::cut_between_loops;
use crate::geometry::{Curve, Plane, Point3, Surface};
use crate::model::Model;
use crate::topology::attributes::{EdgeAttr, FaceAttr, VertexAttr};
use crate::topology::gmap::{Dart, Dim};
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;
use crate::topology::shape_keys::FaceKey;
use crate::topology::{EditKey, ModelEdit, ModelEditError};

pub fn add_polygon_with_holes<P: Payload>(
    g: &mut Model<P>,
    plane: Plane,
    outer: &[Point3],
    holes: &[&[Point3]],
) -> Result<FaceKey, FaceCreationError> {
    g.transaction(|edit| add_polygon_with_holes_edit(edit, plane, outer, holes))
}

/// Builds the outer polygon and all hole loops before registering the face.
pub(crate) fn add_polygon_with_holes_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    plane: Plane,
    outer: &[Point3],
    holes: &[&[Point3]],
) -> Result<FaceKey, FaceCreationError> {
    validate_polygon(outer)?;
    for hole in holes {
        validate_polygon(hole)?;
    }

    let outer_profile = add_polygon_edit(edit, outer);
    let outer_loop = edit.profile_attr_unchecked(outer_profile).dart;
    let mut inner_loops = Vec::with_capacity(holes.len());
    let outer_profile =
        Profile::from_dart(edit, outer_loop).expect("outer loop must have a registered profile");
    let mut pcurves = profile_pcurves(&outer_profile, &plane)?;

    for hole in holes {
        let inner_profile = add_polygon_edit(edit, hole);
        let inner_loop = edit.profile_attr_unchecked(inner_profile).dart;
        let inner_profile = Profile::from_dart(edit, inner_loop)
            .expect("inner loop must have a registered profile");
        pcurves.extend(profile_pcurves(&inner_profile, &plane)?);
        inner_loops.push(inner_loop);
    }

    let sources = std::iter::once(outer_loop)
        .chain(inner_loops.iter().copied())
        .filter_map(|dart| edit.profile_key(dart).map(EditKey::Profile))
        .collect();
    let face_key = edit.add_face_derived_from(
        sources,
        FaceAttr::with_pcurves(
            Surface::Plane(plane),
            outer_loop,
            inner_loops.clone(),
            pcurves,
        ),
    );
    // Each hole is reached from the outer boundary along a cut the face owns,
    // so the face is one 2-cell rather than one boundary per hole with nothing
    // joining them.
    for inner_loop in inner_loops {
        cut_between_loops(edit, face_key, outer_loop, inner_loop)?;
    }
    Ok(face_key)
}

pub(crate) fn validate_polygon(points: &[Point3]) -> Result<(), FaceCreationError> {
    if points.len() >= 3 {
        Ok(())
    } else {
        Err(FaceCreationError::InvalidPolygon {
            point_count: points.len(),
        })
    }
}

/// Adds a single polygon face to `g` with the given corner points (in order).
///
/// Sews alpha0 and alpha1 to form a closed `n`-gon, stamps the vertex positions on
/// every dart of each corner's vertex orbit, and attaches a straight
/// [`Curve::Line`] on every 1-cell so downstream consumers (edge tessellation,
/// dart geometry) have a curve to follow. Does not touch alpha2; the face is
/// returned with free boundary, ready to be stitched to neighbors.
///
/// Returns the profile key whose stored dart defines the polygon's orientation.
pub fn add_polygon<P: Payload>(
    g: &mut Model<P>,
    corners: &[Point3],
) -> crate::topology::shape_keys::ProfileKey {
    g.transaction(|edit| Ok::<_, ModelEditError>(add_polygon_edit(edit, corners)))
        .expect("fresh polygon operation must commit")
}

/// Creates and links polygon segments without opening another transaction scope.
pub(crate) fn add_polygon_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    corners: &[Point3],
) -> crate::topology::shape_keys::ProfileKey {
    assert!(
        corners.len() >= 3,
        "add_polygon requires at least 3 corners, got {}",
        corners.len()
    );
    let n = corners.len();
    let darts: Vec<Dart> = (0..2 * n).map(|_| edit.add_dart()).collect();

    for i in 0..n {
        edit.sew(Dim::Zero, darts[2 * i], darts[2 * i + 1])
            .expect("fresh polygon edge darts must be alpha0-free");
    }
    for i in 0..n {
        let a = darts[2 * i + 1];
        let b = darts[(2 * i + 2) % (2 * n)];
        edit.sew(Dim::One, a, b)
            .expect("fresh polygon boundary darts must be alpha1-free");
    }

    for i in 0..n {
        let dart = edit.cell_representative(darts[2 * i], Dim::Zero);
        edit.add_vertex(VertexAttr::new(dart, corners[i]));
    }

    for i in 0..n {
        let edge_dart = darts[2 * i];
        let curve = Curve::line(corners[i], corners[(i + 1) % n]);
        edit.add_edge(EdgeAttr::new(edge_dart, curve));
    }
    edit.add_profile(crate::topology::attributes::ProfileAttr::new(darts[0]))
}

/// Flips a face's orientation in place.
///
/// Every boundary loop is re-rooted on its `alpha0` partner and every pcurve is
/// re-keyed to that partner and reversed, so the loops are traversed the other
/// way round and [`Face::normal_at`](crate::topology::face::Face::normal_at)
/// returns the opposite normal. The map's topology is untouched — only the
/// face attribute changes — so darts captured for sewing stay valid.
///
/// Does nothing when `face` is not a registered face.
pub(crate) fn reverse_face_winding_edit<P: Payload>(edit: &mut ModelEdit<'_, P>, face: FaceKey) {
    let Some(face_attr) = edit.face_attr(face).cloned() else {
        return;
    };

    // Reversing is atomic over the whole boundary: every loop seed becomes its
    // `alpha0`, whatever that loop bounds, and every pcurve is reversed onto
    // the dart that now carries it.
    let mut loops = face_attr.loops().to_vec();
    for loop_ in &mut loops {
        loop_.set_seed(edit.alpha(Dim::Zero, loop_.seed()));
    }
    let pcurves = face_attr
        .face(edit)
        .edges()
        .into_iter()
        .filter_map(|edge| {
            face_attr
                .pcurves
                .get(&edge.dart())
                .map(|pcurve| (edit.alpha(Dim::Zero, edge.dart()), pcurve.reversed()))
        })
        .collect();

    if let Some(face) = edit.face_attr_mut(face) {
        let anchor = face.seed();
        face.set_boundary(loops, anchor);
        face.pcurves = pcurves;
    }
}
