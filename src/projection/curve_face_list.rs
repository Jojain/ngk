//! A curve carried onto planar faces, as chains of segments each on one face.
//!
//! On every planar face the source's image is exact and trimmed to the face;
//! where several faces cover a parameter one wins, as for a point (see
//! [`winners`]). The winning parts are then chained: two parts continue each
//! other when they meet on an edge the two faces share in the model, and
//! otherwise the chain stops on a [`Jump`](super::ChainEnd::Jump).

use super::curve_face_clip::PlanarFace;
use super::curve_face_rank::{Part, winners};
use super::curve_result::{CurveProjection, Exactness, ProjectedSegment, SourceSpan};
use super::curve_walk::{Item, Walk, chain_items};
use super::error::ProjectionError;
use super::mode::Projection;
use super::options::ProjectOptions;
use super::plane_carrier::{PARAMETER_EPSILON, PlaneCarrier};
use super::point_face_list::FaceList;
use super::project::Project;
use crate::geometry::{Curve, Interval, Periodicity, Point3, PointCoincidence, TrimmedCurve};
use crate::model::Model;
use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;
use crate::topology::shape_keys::FaceKey;

/// A source curve that can be walked span by span.
pub(super) trait CurveSource {
    /// The spans of the source in the order they are walked.
    fn walk(&self) -> Result<Walk, ProjectionError>;
}

impl CurveSource for TrimmedCurve {
    fn walk(&self) -> Result<Walk, ProjectionError> {
        Ok(Walk::of_span(self.clone()))
    }
}

impl CurveSource for Curve {
    fn walk(&self) -> Result<Walk, ProjectionError> {
        Walk::of_curve(self)
    }
}

impl<P: Payload> CurveSource for Edge<'_, P> {
    fn walk(&self) -> Result<Walk, ProjectionError> {
        Ok(Walk::of_edge(self))
    }
}

impl<P: Payload> CurveSource for Profile<'_, P> {
    fn walk(&self) -> Result<Walk, ProjectionError> {
        Ok(Walk::of_profile(self))
    }
}

/// A curve, an edge or a profile projects onto each of the faces of the list
/// that has a plane for support, and is kept where it is inside a face's trim.
///
/// Where several faces take the same part of the source, the first wall of a
/// directional projection or the nearest point of a perpendicular one wins; a
/// perpendicular projection over a stretch where nearest points at different
/// places are equally near is [`Ambiguous`](ProjectionError::Ambiguous). The result is
/// chains of segments, each on one face and naming it. A chain runs on from a
/// face to another where they meet on an edge the two share in the model; where
/// the faces are not neighbours, or meet elsewhere than on that edge, it stops
/// on a [`Jump`](super::ChainEnd::Jump), and where no face takes the source, on
/// a [`Miss`](super::ChainEnd::Miss). Profile sources keep their vertex rules.
///
/// A list with a face that is not planar is refused with
/// [`UnsupportedSurface`](ProjectionError::UnsupportedSurface), and one that
/// takes no part of the source is a [`Miss`](ProjectionError::Miss).
impl<S: CurveSource, P: Payload> Project<S> for FaceList<'_, P> {
    type Mode = Projection;
    type Hit = FaceKey;
    type Output = CurveProjection<FaceKey>;

    fn project_with(
        &self,
        source: &S,
        mode: Projection,
        options: &ProjectOptions,
    ) -> Result<CurveProjection<FaceKey>, ProjectionError> {
        onto_faces(self.model(), self.faces(), &source.walk()?, mode, options)
    }
}

/// A one-face list.
impl<S: CurveSource, P: Payload> Project<S> for Face<'_, P> {
    type Mode = Projection;
    type Hit = FaceKey;
    type Output = CurveProjection<FaceKey>;

    fn project_with(
        &self,
        source: &S,
        mode: Projection,
        options: &ProjectOptions,
    ) -> Result<CurveProjection<FaceKey>, ProjectionError> {
        let keys = [self.key()];
        onto_faces(self.model(), &keys, &source.walk()?, mode, options)
    }
}

fn onto_faces<P: Payload>(
    model: &Model<P>,
    keys: &[FaceKey],
    walk: &Walk,
    mode: Projection,
    options: &ProjectOptions,
) -> Result<CurveProjection<FaceKey>, ProjectionError> {
    walk.ensure_bounded()?;
    let Some(first) = walk.edges.first() else {
        return Err(ProjectionError::Miss);
    };
    let faces = PlanarFace::all(model, keys, options)?;
    // A direction parallel to a face's plane reaches nothing on it.
    let mut carriers = Vec::with_capacity(faces.len());
    for face in &faces {
        carriers.push(
            match PlaneCarrier::new(&face.plane, mode, options, first.span.start()) {
                Ok(carrier) => Some(carrier),
                Err(ProjectionError::Miss) => None,
                Err(error) => return Err(error),
            },
        );
    }

    let mut items = Vec::new();
    for (index, edge) in walk.edges.iter().enumerate() {
        let mut parts = winners(&faces, &carriers, &edge.span, mode, options)?;
        in_walk_order(&mut parts, &edge.span);
        join_across_seam(&mut parts, walk);
        let count = parts.len();
        for (position, part) in parts.into_iter().enumerate() {
            let segment = match part.face {
                Some(face) => {
                    let carrier = carriers[face]
                        .as_ref()
                        .expect("a face that wins a part has a carrier");
                    let (curve, pcurve) =
                        carrier.project_piece(edge.span.curve(), part.interval)?;
                    Some(ProjectedSegment {
                        curve,
                        pcurve,
                        hit: faces[face].key,
                        source: SourceSpan {
                            source: edge.source,
                            interval: part.interval,
                        },
                        fit: Exactness::Exact,
                    })
                }
                None => None,
            };
            items.push(Item {
                edge: index,
                last_in_edge: position + 1 == count,
                segment,
            });
        }
    }
    if items.iter().all(|item| item.segment.is_none()) {
        return Err(ProjectionError::Miss);
    }
    let chains = chain_items(items, walk, |before, after| {
        before.hit == after.hit
            || (before
                .curve
                .end()
                .coincides(after.curve.start(), options.tolerance)
                && meet_on_shared_edge(model, before.hit, after.hit, before.curve.end(), options))
    });
    Ok(CurveProjection { chains })
}

/// Turns parts in increasing parameter into the order `span` runs.
fn in_walk_order(parts: &mut [Part], span: &TrimmedCurve) {
    if span.interval().delta() < 0.0 {
        parts.reverse();
        for part in parts {
            part.interval = part.interval.reversed();
        }
    }
}

/// Joins the last part of a closed periodic span to its first when the same
/// face wins both, so a face the span passes through at its seam takes one
/// segment, not two.
fn join_across_seam(parts: &mut Vec<Part>, walk: &Walk) {
    let [edge] = walk.edges.as_slice() else {
        return;
    };
    let Periodicity::Periodic(period) = edge.span.curve().periodicity() else {
        return;
    };
    let delta = edge.span.interval().delta();
    if !walk.cyclic || (delta.abs() - period).abs() > PARAMETER_EPSILON || parts.len() < 2 {
        return;
    }
    let (first, last) = (parts[0], parts[parts.len() - 1]);
    if first.face.is_none() || first.face != last.face {
        return;
    }
    parts.remove(0);
    let end = parts.len() - 1;
    parts[end].interval = Interval::new(
        last.interval.start.value(),
        first.interval.end.value() + delta,
    );
}

/// Whether `point` is on an edge that faces `first` and `second` share.
fn meet_on_shared_edge<P: Payload>(
    model: &Model<P>,
    first: FaceKey,
    second: FaceKey,
    point: Point3,
    options: &ProjectOptions,
) -> bool {
    model
        .face_unchecked(first)
        .edges()
        .into_iter()
        .filter(|edge| edge.faces().iter().any(|face| face.key() == second))
        .any(|edge| {
            let curve = edge.trimmed_curve();
            curve
                .point_at(curve.parameter_at(point))
                .coincides(point, options.tolerance)
        })
}

#[cfg(test)]
mod tests {
    use std::f64::consts::TAU;

    use nalgebra::Vector3;

    use super::*;
    use crate::builders::edges::add_circle;
    use crate::builders::faces::add_polygon_with_holes;
    use crate::builders::profiles::add_polyline;
    use crate::builders::sheets::add_sheet_from_faces;
    use crate::builders::solids::add_cylinder;
    use crate::geometry::parameter::Fraction;
    use crate::geometry::{Frame, Plane};
    use crate::projection::{ChainEnd, ProjectedChain, SegmentSource, project};
    use crate::topology::StandardPayload;

    type TestModel = Model<StandardPayload>;

    fn down() -> Projection {
        Projection::along(-Vector3::z())
    }

    fn both_modes() -> [Projection; 2] {
        [Projection::Normal, down()]
    }

    fn polygon(model: &mut TestModel, plane: Plane, points: &[Point3]) -> FaceKey {
        add_polygon_with_holes(model, plane, points, &[]).unwrap()
    }

    /// The square `[0, 2] x [0, 2]` in the plane `z = 0`.
    fn flat_square(model: &mut TestModel) -> FaceKey {
        polygon(
            model,
            Plane::xy(),
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
                Point3::new(2.0, 2.0, 0.0),
                Point3::new(0.0, 2.0, 0.0),
            ],
        )
    }

    /// A face 3 long and 2 wide, hinged on the edge of the flat square at
    /// `x = 2` and tilted by 20 degrees, rising when `rising` and falling
    /// otherwise.
    fn hinged_face(model: &mut TestModel, rising: bool) -> FaceKey {
        let tilt: f64 = 20.0_f64.to_radians();
        let rise = if rising { tilt.sin() } else { -tilt.sin() };
        let (across, height) = (3.0 * tilt.cos(), 3.0 * rise);
        let plane = Plane::from_xy(
            Point3::new(2.0, 0.0, 0.0),
            Vector3::new(tilt.cos(), 0.0, rise),
            Vector3::y(),
        );
        polygon(
            model,
            plane,
            &[
                Point3::new(2.0, 2.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
                Point3::new(2.0 + across, 0.0, height),
                Point3::new(2.0 + across, 2.0, height),
            ],
        )
    }

    /// A circle of radius `0.8` in a horizontal plane at `height`, centred on
    /// `(x, 1)`, from the seam at its largest `x`.
    fn circle_over(x: f64, height: f64) -> TrimmedCurve {
        let plane = Plane::new(Point3::new(x, 1.0, height), Vector3::x(), Vector3::z());
        TrimmedCurve::arc(plane, 0.8, TAU)
    }

    /// A rectangle `[0, width] x [0, 2]` at `height`.
    fn plate(model: &mut TestModel, height: f64, width: f64) -> FaceKey {
        let at = |x: f64, y: f64| Point3::new(x, y, height);
        polygon(
            model,
            Plane::from_frame(Frame::at(at(0.0, 0.0))),
            &[at(0.0, 0.0), at(width, 0.0), at(width, 2.0), at(0.0, 2.0)],
        )
    }

    fn assert_continuous(chain: &ProjectedChain<FaceKey>) {
        let count = chain.segments.len();
        let wrap = if chain.closed() { count } else { count - 1 };
        for index in 0..wrap {
            let here = chain.segments[index].curve.end();
            let next = chain.segments[(index + 1) % count].curve.start();
            assert!((here - next).norm() < 1e-7, "{here:?} vs {next:?}");
        }
    }

    /// Checks each segment's curve is its pcurve lifted through its face.
    fn assert_pcurves_on_faces(model: &TestModel, projection: &CurveProjection<FaceKey>) {
        for segment in projection.segments() {
            let face = model.face_unchecked(segment.hit);
            for step in 0..=4 {
                let fraction = Fraction::new(f64::from(step) / 4.0);
                let uv = segment.pcurve.point_at(fraction);
                let lifted = face.point_at(uv.x, uv.y);
                assert!((lifted - segment.curve.point_at(fraction)).norm() < 1e-7);
            }
            assert_eq!(segment.fit, Exactness::Exact);
        }
    }

    #[test]
    fn a_circle_over_a_sheet_fold_is_one_closed_chain_of_one_segment_per_face() {
        for rising in [false, true] {
            let mut model = TestModel::new();
            let flat = flat_square(&mut model);
            let hinged = hinged_face(&mut model, rising);
            add_sheet_from_faces(&mut model, &[flat, hinged]).unwrap();
            let faces = [flat, hinged];
            let list = FaceList::new(&model, &faces);
            let result = project(&circle_over(2.0, 5.0), &list, down()).unwrap();
            assert_eq!(result.chains.len(), 1);
            let chain = &result.chains[0];
            assert!(chain.closed());
            assert_eq!(chain.segments.len(), 2);
            let mut hits: Vec<_> = chain.segments.iter().map(|s| s.hit).collect();
            hits.sort();
            let mut expected = faces;
            expected.sort();
            assert_eq!(hits, expected);
            assert_continuous(chain);
            assert_pcurves_on_faces(&model, &result);
            // The segments meet on the edge the faces share: x = 2, z = 0.
            for segment in &chain.segments {
                let end = segment.curve.end();
                assert!((end.x - 2.0).abs() < 1e-7 && end.z.abs() < 1e-7);
                assert!(end.y > 0.0 && end.y < 2.0);
            }
        }
    }

    #[test]
    fn a_perpendicular_projection_over_a_rising_fold_jumps_where_the_nearer_face_changes() {
        // Above a valley the nearest point changes on the bisector plane of the
        // two faces, which is not where they meet.
        let mut model = TestModel::new();
        let flat = flat_square(&mut model);
        let hinged = hinged_face(&mut model, true);
        add_sheet_from_faces(&mut model, &[flat, hinged]).unwrap();
        let faces = [flat, hinged];
        let list = FaceList::new(&model, &faces);
        let source = circle_over(1.1, 5.0);
        let result = project(&source, &list, Projection::Normal).unwrap();
        assert_eq!(result.chains.len(), 2);
        for chain in &result.chains {
            assert_eq!((chain.start, chain.end), (ChainEnd::Jump, ChainEnd::Jump));
            assert_eq!(chain.segments.len(), 1);
        }
        assert_pcurves_on_faces(&model, &result);
    }

    #[test]
    fn a_perpendicular_projection_over_a_falling_fold_has_no_nearest_point_beside_the_edge() {
        // Above a ridge the perpendicular to the lower face lands back on the
        // other side of the edge, so the wedge over the edge has no nearest
        // point on either face.
        let mut model = TestModel::new();
        let flat = flat_square(&mut model);
        let hinged = hinged_face(&mut model, false);
        add_sheet_from_faces(&mut model, &[flat, hinged]).unwrap();
        let faces = [flat, hinged];
        let list = FaceList::new(&model, &faces);
        let result = project(&circle_over(2.0, 5.0), &list, Projection::Normal).unwrap();
        assert_eq!(result.chains.len(), 1);
        let chain = &result.chains[0];
        assert_eq!((chain.start, chain.end), (ChainEnd::Miss, ChainEnd::Miss));
        assert_eq!(chain.segments.len(), 1);
        assert_eq!(chain.segments[0].hit, flat);
    }

    #[test]
    fn the_same_faces_left_loose_split_the_chain_on_a_jump() {
        let cases = [
            (false, down(), 2.0),
            (true, down(), 2.0),
            (true, Projection::Normal, 1.1),
        ];
        for (rising, mode, x) in cases {
            let mut model = TestModel::new();
            let flat = flat_square(&mut model);
            let hinged = hinged_face(&mut model, rising);
            let faces = [flat, hinged];
            let list = FaceList::new(&model, &faces);
            let result = project(&circle_over(x, 5.0), &list, mode).unwrap();
            assert_eq!(result.chains.len(), 2);
            for chain in &result.chains {
                assert_eq!((chain.start, chain.end), (ChainEnd::Jump, ChainEnd::Jump));
                assert_eq!(chain.segments.len(), 1);
            }
            assert_pcurves_on_faces(&model, &result);
        }
    }

    #[test]
    fn a_circle_partly_off_the_face_ends_its_chain_on_a_miss() {
        let mut model = TestModel::new();
        let flat = flat_square(&mut model);
        let face = model.face_unchecked(flat);
        for mode in both_modes() {
            let result = project(&circle_over(0.0, 5.0), &face, mode).unwrap();
            assert_eq!(result.chains.len(), 1);
            let chain = &result.chains[0];
            assert_eq!((chain.start, chain.end), (ChainEnd::Miss, ChainEnd::Miss));
            assert_eq!(chain.segments.len(), 1);
            assert_pcurves_on_faces(&model, &result);
        }
    }

    #[test]
    fn a_circle_wholly_over_a_face_is_one_closed_segment() {
        let mut model = TestModel::new();
        let flat = flat_square(&mut model);
        let face = model.face_unchecked(flat);
        for mode in both_modes() {
            let result = project(&circle_over(1.0, 5.0), &face, mode).unwrap();
            assert_eq!(result.chains.len(), 1);
            assert!(result.chains[0].closed());
            assert_eq!(result.chains[0].segments.len(), 1);
            assert_eq!(result.chains[0].segments[0].hit, flat);
        }
    }

    #[test]
    fn a_source_wholly_off_the_faces_misses() {
        let mut model = TestModel::new();
        let flat = flat_square(&mut model);
        let faces = [flat];
        let list = FaceList::new(&model, &faces);
        for mode in both_modes() {
            let result = project(&circle_over(9.0, 5.0), &list, mode);
            assert!(matches!(result, Err(ProjectionError::Miss)));
        }
    }

    #[test]
    fn a_hole_in_a_face_is_a_miss_across_it() {
        let mut model = TestModel::new();
        let at = |x: f64, y: f64| Point3::new(x, y, 0.0);
        let outer = [at(0.0, 0.0), at(4.0, 0.0), at(4.0, 4.0), at(0.0, 4.0)];
        let inner = [at(1.0, 1.0), at(3.0, 1.0), at(3.0, 3.0), at(1.0, 3.0)];
        let key = add_polygon_with_holes(&mut model, Plane::xy(), &outer, &[&inner]).unwrap();
        let face = model.face_unchecked(key);
        let span = TrimmedCurve::segment(Point3::new(-1.0, 2.0, 3.0), Point3::new(5.0, 2.0, 3.0));
        for mode in both_modes() {
            let result = project(&span, &face, mode).unwrap();
            assert_eq!(result.chains.len(), 2);
            for chain in &result.chains {
                assert_eq!((chain.start, chain.end), (ChainEnd::Miss, ChainEnd::Miss));
                assert_eq!(chain.segments.len(), 1);
            }
            let first = &result.chains[0].segments[0].curve;
            let second = &result.chains[1].segments[0].curve;
            assert!((first.start() - Point3::new(0.0, 2.0, 0.0)).norm() < 1e-7);
            assert!((first.end() - Point3::new(1.0, 2.0, 0.0)).norm() < 1e-7);
            assert!((second.start() - Point3::new(3.0, 2.0, 0.0)).norm() < 1e-7);
            assert!((second.end() - Point3::new(4.0, 2.0, 0.0)).norm() < 1e-7);
        }
    }

    #[test]
    fn the_upper_of_two_stacked_faces_hides_the_lower_and_the_chain_jumps() {
        let mut model = TestModel::new();
        let lower = plate(&mut model, 0.0, 4.0);
        let upper = plate(&mut model, 1.0, 2.0);
        let faces = [lower, upper];
        let list = FaceList::new(&model, &faces);
        let result = project(&circle_over(2.0, 3.0), &list, down()).unwrap();
        assert_eq!(result.chains.len(), 2);
        for chain in &result.chains {
            assert_eq!((chain.start, chain.end), (ChainEnd::Jump, ChainEnd::Jump));
            assert_eq!(chain.segments.len(), 1);
        }
        let hits: Vec<_> = result.segments().map(|s| s.hit).collect();
        assert_eq!(hits.iter().filter(|hit| **hit == upper).count(), 1);
        assert_eq!(hits.iter().filter(|hit| **hit == lower).count(), 1);
        for segment in result.segments() {
            let z = segment.curve.start().z;
            assert!((z - if segment.hit == upper { 1.0 } else { 0.0 }).abs() < 1e-7);
        }
        assert_pcurves_on_faces(&model, &result);
    }

    #[test]
    fn a_directional_projection_changes_face_where_two_crossing_planes_meet_the_ray() {
        let mut model = TestModel::new();
        let flat = plate(&mut model, 0.0, 4.0);
        let slope = polygon(
            &mut model,
            Plane::from_xy(
                Point3::new(0.0, 0.0, -1.0),
                Vector3::new(4.0, 0.0, 2.0),
                Vector3::y(),
            ),
            &[
                Point3::new(0.0, 0.0, -1.0),
                Point3::new(4.0, 0.0, 1.0),
                Point3::new(4.0, 2.0, 1.0),
                Point3::new(0.0, 2.0, -1.0),
            ],
        );
        let faces = [flat, slope];
        let list = FaceList::new(&model, &faces);
        let span = TrimmedCurve::segment(Point3::new(0.5, 1.0, 5.0), Point3::new(3.5, 1.0, 5.0));
        let result = project(&span, &list, down()).unwrap();
        // The planes cross on the line x = 2, z = 0; the flat one is above to
        // its left, the sloping one to its right.
        assert_eq!(result.chains.len(), 2);
        assert_eq!(result.chains[0].segments[0].hit, flat);
        assert_eq!(result.chains[1].segments[0].hit, slope);
        assert_eq!(result.chains[0].end, ChainEnd::Jump);
        let kept = result.chains[0].segments[0].source.interval;
        assert!((kept.end.value() - 0.5).abs() < 1e-7);
        assert_pcurves_on_faces(&model, &result);
    }

    #[test]
    fn two_stacked_faces_are_ambiguous_to_a_perpendicular_projection_midway() {
        let mut model = TestModel::new();
        let lower = plate(&mut model, 0.0, 4.0);
        let upper = plate(&mut model, 2.0, 4.0);
        let faces = [lower, upper];
        let list = FaceList::new(&model, &faces);
        let result = project(&circle_over(2.0, 1.0), &list, Projection::Normal);
        assert!(matches!(result, Err(ProjectionError::Ambiguous)));
    }

    #[test]
    fn a_perpendicular_projection_changes_face_where_the_two_are_equally_near() {
        let mut model = TestModel::new();
        let lower = plate(&mut model, 0.0, 4.0);
        let upper = plate(&mut model, 2.0, 4.0);
        let faces = [lower, upper];
        let list = FaceList::new(&model, &faces);
        // A segment running along the direction of projection has a point for
        // image.
        let upright = TrimmedCurve::segment(Point3::new(1.0, 1.0, 0.5), Point3::new(1.0, 1.0, 1.5));
        let result = project(&upright, &list, Projection::Normal);
        assert!(matches!(result, Err(ProjectionError::DegenerateImage)));
        // A slanted one climbs through the mid-plane z = 1 half way along.
        let slant = TrimmedCurve::segment(Point3::new(1.0, 1.0, 0.5), Point3::new(3.0, 1.0, 1.5));
        let result = project(&slant, &list, Projection::Normal).unwrap();
        assert_eq!(result.chains.len(), 2);
        assert_eq!(result.chains[0].segments[0].hit, lower);
        assert_eq!(result.chains[1].segments[0].hit, upper);
        assert_eq!(result.chains[0].end, ChainEnd::Jump);
        let kept = result.chains[0].segments[0].source.interval;
        assert!((kept.end.value() - 0.5).abs() < 1e-7);
    }

    #[test]
    fn a_face_is_not_a_wall_for_a_source_behind_it() {
        let mut model = TestModel::new();
        let flat = flat_square(&mut model);
        let face = model.face_unchecked(flat);
        let below = TrimmedCurve::segment(Point3::new(0.5, 1.0, -2.0), Point3::new(1.5, 1.0, -2.0));
        let result = project(&below, &face, down());
        assert!(matches!(result, Err(ProjectionError::Miss)));
        let result = project(&below, &face, Projection::Normal).unwrap();
        assert_eq!(result.chains[0].segments[0].hit, flat);
    }

    #[test]
    fn a_curved_face_in_the_list_is_refused() {
        let mut model = TestModel::new();
        let cylinder = add_cylinder(&mut model, Frame::xyz(), 1.0, 2.0).unwrap();
        let faces = [cylinder.end_cap, cylinder.laterals[0].face];
        let list = FaceList::new(&model, &faces);
        for mode in both_modes() {
            let result = project(&circle_over(1.0, 5.0), &list, mode);
            assert!(matches!(
                result,
                Err(ProjectionError::UnsupportedSurface {
                    surface: "cylinder"
                })
            ));
        }
    }

    #[test]
    fn a_profile_over_a_face_keeps_its_vertex_rules() {
        let mut model = TestModel::new();
        let flat = flat_square(&mut model);
        let at = |x: f64, y: f64| Point3::new(x, y, 3.0);
        let closed = add_polyline(
            &mut model,
            &[
                at(0.5, 0.5),
                at(1.5, 0.5),
                at(1.5, 1.5),
                at(0.5, 1.5),
                at(0.5, 0.5),
            ],
        )
        .unwrap();
        let open = add_polyline(&mut model, &[at(1.0, 1.0), at(3.0, 1.0), at(3.0, 2.0)]).unwrap();
        let face = model.face_unchecked(flat);
        for mode in both_modes() {
            let profile = model.profile(closed).unwrap();
            let result = project(&profile, &face, mode).unwrap();
            assert_eq!(result.chains.len(), 1);
            assert!(result.chains[0].closed());
            assert_eq!(result.chains[0].segments.len(), 4);
            assert_continuous(&result.chains[0]);

            let profile = model.profile(open).unwrap();
            let result = project(&profile, &face, mode).unwrap();
            assert_eq!(result.chains.len(), 1);
            let chain = &result.chains[0];
            assert_eq!(
                (chain.start, chain.end),
                (ChainEnd::SourceEnd, ChainEnd::Miss)
            );
            assert_eq!(chain.segments.len(), 1);
        }
    }

    #[test]
    fn an_edge_names_itself_as_the_source_of_its_segments() {
        let mut model = TestModel::new();
        let flat = flat_square(&mut model);
        let plane = Plane::new(Point3::new(1.0, 1.0, 4.0), Vector3::x(), Vector3::z());
        let key = add_circle(&mut model, plane, 0.5).unwrap();
        let edge = model.edge(key).unwrap();
        let face = model.face_unchecked(flat);
        let result = project(&edge, &face, down()).unwrap();
        assert_eq!(result.chains.len(), 1);
        assert!(result.chains[0].closed());
        assert_eq!(
            result.chains[0].segments[0].source.source,
            SegmentSource::Edge(key)
        );
    }

    #[test]
    fn a_whole_curve_is_projected_onto_a_face_list() {
        let mut model = TestModel::new();
        let flat = flat_square(&mut model);
        let faces = [flat];
        let list = FaceList::new(&model, &faces);
        let curve = circle_over(1.0, 5.0).curve().clone();
        let result = project(&curve, &list, down()).unwrap();
        assert!(result.chains[0].closed());
        let line = Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 1.0));
        let result = project(&line, &list, down());
        assert!(matches!(result, Err(ProjectionError::UnboundedSource)));
    }
}
