use super::curve_result::CurveProjection;
use super::curve_walk::{Walk, WalkEdge, project_walk};
use super::error::ProjectionError;
use super::mode::Projection;
use super::options::ProjectOptions;
use super::project::Project;
use crate::geometry::{Curve, LINEAR_TOLERANCE, Plane, PointCoincidence, TrimmedCurve};
use crate::topology::closed::Closeable;
use crate::topology::edge::Edge;
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;

/// A projection onto a plane is a parallel map, so the image of a curve is
/// exact: a line is a line, a conic a conic, and a NURBS curve the NURBS curve
/// on the mapped control points. A curve with no such image is refused, never
/// fitted.
///
/// Sources are refused when they are unbounded, when they are a helix, and
/// when the image collapses to a point or a conic's to a segment, with a named
/// [`ProjectionError`].
///
/// A directional projection keeps the half-line rule: a part of the source
/// behind the plane along the direction casts no shadow, so a source crossing
/// the plane is cut there and the chain of the part in front stops at a
/// [`Miss`](super::ChainEnd::Miss). A source wholly behind it
/// [misses](ProjectionError::Miss).
impl Project<Plane> for TrimmedCurve {
    type Mode = Projection;
    type Hit = ();
    type Output = CurveProjection<()>;

    fn project(
        &self,
        plane: &Plane,
        mode: Projection,
        options: &ProjectOptions,
    ) -> Result<CurveProjection<()>, ProjectionError> {
        let cyclic = self.start().coincides(self.end(), LINEAR_TOLERANCE);
        let walk = Walk {
            edges: vec![WalkEdge::of_curve(self.clone())],
            cyclic,
        };
        project_walk(&walk, plane, mode, options)
    }
}

/// The whole of the curve's domain, as for a [`TrimmedCurve`]; an unbounded
/// curve has no finite span and is refused.
impl Project<Plane> for Curve {
    type Mode = Projection;
    type Hit = ();
    type Output = CurveProjection<()>;

    fn project(
        &self,
        plane: &Plane,
        mode: Projection,
        options: &ProjectOptions,
    ) -> Result<CurveProjection<()>, ProjectionError> {
        if !self.domain().is_finite() {
            return Err(ProjectionError::UnboundedSource);
        }
        TrimmedCurve::whole(self.clone()).project(plane, mode, options)
    }
}

/// The edge's span as it is oriented in the view, as for a [`TrimmedCurve`];
/// a segment names the edge as its source.
impl<P: Payload> Project<Plane> for Edge<'_, P> {
    type Mode = Projection;
    type Hit = ();
    type Output = CurveProjection<()>;

    fn project(
        &self,
        plane: &Plane,
        mode: Projection,
        options: &ProjectOptions,
    ) -> Result<CurveProjection<()>, ProjectionError> {
        let walk = Walk {
            edges: vec![WalkEdge::of_edge(self)],
            cyclic: !matches!(self, Edge::Bounded(_)),
        };
        project_walk(&walk, plane, mode, options)
    }
}

/// The profile's edges in the order of its walk, each as for an [`Edge`].
///
/// A chain runs on through a vertex where exactly two ends of the profile's
/// edges meet. It stops on a [`SourceVertex`](super::ChainEnd::SourceVertex)
/// where more or fewer do, on a [`SourceEnd`](super::ChainEnd::SourceEnd) at
/// a free end, and a closed profile whose edges all have a shadow is one chain
/// with [`Closed`](super::ChainEnd::Closed) ends.
impl<P: Payload> Project<Plane> for Profile<'_, P> {
    type Mode = Projection;
    type Hit = ();
    type Output = CurveProjection<()>;

    fn project(
        &self,
        plane: &Plane,
        mode: Projection,
        options: &ProjectOptions,
    ) -> Result<CurveProjection<()>, ProjectionError> {
        let walk = Walk {
            edges: self.edges().iter().map(WalkEdge::of_edge).collect(),
            cyclic: self.is_closed(),
        };
        project_walk(&walk, plane, mode, options)
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::TAU;

    use nalgebra::Vector3;

    use super::*;
    use crate::builders::edges::{add_circle, add_line};
    use crate::builders::profiles::add_polyline;
    use crate::geometry::parameter::Fraction;
    use crate::geometry::{Interval, NurbsCurve, Point3};
    use crate::model::Model;
    use crate::projection::{ChainEnd, Exactness, SegmentSource, project, project_point};
    use crate::topology::StandardPayload;

    const SAMPLES: usize = 8;

    fn options() -> ProjectOptions {
        ProjectOptions::default()
    }

    fn tilted_plane() -> Plane {
        Plane::new(
            Point3::new(0.0, 0.0, 1.0),
            Vector3::x(),
            Vector3::new(0.0, -1.0, 1.0),
        )
    }

    fn slanted() -> Projection {
        Projection::along(Vector3::new(0.3, 0.2, -1.0))
    }

    /// Checks every segment is exact, and that its points are the point
    /// projections of the source's points at the same fractions, in 3D and
    /// through the pcurve.
    fn assert_matches_point_projection(
        projection: &CurveProjection<()>,
        source: &Curve,
        plane: &Plane,
        mode: Projection,
    ) {
        for segment in projection.segments() {
            assert_eq!(segment.fit, Exactness::Exact);
            for step in 0..=SAMPLES {
                let fraction = Fraction::new(step as f64 / SAMPLES as f64);
                let origin = source.point_at(segment.source.interval.at(fraction));
                let expected = project_point(&origin, plane, mode, &options()).unwrap();
                let found = segment.curve.point_at(fraction);
                assert!(
                    (found - expected).norm() < 1e-9,
                    "{found:?} vs {expected:?}"
                );
                let uv = segment.pcurve.point_at(fraction);
                assert!((plane.point_at(uv.x, uv.y) - found).norm() < 1e-9);
            }
        }
    }

    fn only_segment(projection: &CurveProjection<()>) -> &crate::projection::ProjectedSegment<()> {
        assert_eq!(projection.chains.len(), 1);
        assert_eq!(projection.chains[0].segments.len(), 1);
        &projection.chains[0].segments[0]
    }

    #[test]
    fn a_line_onto_a_parallel_plane_stays_a_line() {
        let span = TrimmedCurve::segment(Point3::new(0.0, 0.0, 2.0), Point3::new(4.0, 1.0, 2.0));
        for mode in [Projection::Normal, slanted()] {
            let result = project(&span, &Plane::xy(), mode, &options()).unwrap();
            let segment = only_segment(&result);
            assert!(matches!(segment.curve.curve(), Curve::Line(_)));
            assert_eq!(result.chains[0].start, ChainEnd::SourceEnd);
            assert_eq!(result.chains[0].end, ChainEnd::SourceEnd);
            assert!(!result.chains[0].closed());
            assert_eq!(segment.source.source, SegmentSource::Curve);
            assert_matches_point_projection(&result, span.curve(), &Plane::xy(), mode);
        }
    }

    #[test]
    fn a_line_onto_an_inclined_plane_stays_a_line() {
        let span = TrimmedCurve::segment(Point3::new(0.0, 0.0, 5.0), Point3::new(4.0, 1.0, 6.0));
        let plane = tilted_plane();
        for mode in [
            Projection::Normal,
            Projection::along(Vector3::new(0.2, 0.0, -1.0)),
        ] {
            let result = project(&span, &plane, mode, &options()).unwrap();
            assert!(matches!(
                only_segment(&result).curve.curve(),
                Curve::Line(_)
            ));
            assert_matches_point_projection(&result, span.curve(), &plane, mode);
        }
    }

    #[test]
    fn a_line_along_the_direction_collapses_to_a_point() {
        let span = TrimmedCurve::segment(Point3::new(1.0, 1.0, 2.0), Point3::new(1.0, 1.0, 5.0));
        let result = project(&span, &Plane::xy(), Projection::Normal, &options());
        assert!(matches!(result, Err(ProjectionError::DegenerateImage)));
    }

    #[test]
    fn a_circle_onto_a_parallel_plane_is_a_circle() {
        let source = Plane::new(Point3::new(1.0, 0.0, 3.0), Vector3::x(), Vector3::z());
        let span = TrimmedCurve::arc(source, 2.0, TAU);
        for mode in [Projection::Normal, slanted()] {
            let result = project(&span, &Plane::xy(), mode, &options()).unwrap();
            let segment = only_segment(&result);
            let Curve::Circle(circle) = segment.curve.curve() else {
                panic!("a circle onto a parallel plane is a circle");
            };
            assert!((circle.radius() - 2.0).abs() < 1e-9);
            assert!(result.chains[0].closed());
            assert_matches_point_projection(&result, span.curve(), &Plane::xy(), mode);
        }
    }

    #[test]
    fn a_circle_onto_an_inclined_plane_is_an_ellipse() {
        let source = Plane::new(Point3::new(0.0, 0.0, 5.0), Vector3::x(), Vector3::z());
        let span = TrimmedCurve::arc(source, 2.0, TAU);
        let plane = tilted_plane();
        for mode in [
            Projection::Normal,
            Projection::along(Vector3::new(0.1, 0.0, -1.0)),
        ] {
            let result = project(&span, &plane, mode, &options()).unwrap();
            let segment = only_segment(&result);
            assert!(matches!(segment.curve.curve(), Curve::Ellipse(_)));
            assert!(result.chains[0].closed());
            assert_matches_point_projection(&result, span.curve(), &plane, mode);
        }
    }

    #[test]
    fn a_circle_arc_keeps_its_span_under_a_reversed_map() {
        // Seen from below, the plane's normal turns the circle clockwise.
        let source = Plane::new(Point3::new(0.0, 0.0, 5.0), Vector3::x(), Vector3::z());
        let span = TrimmedCurve::new(Curve::circle(source, 2.0), Interval::new(2.0, 0.5));
        let flipped = Plane::new(Point3::origin(), Vector3::x(), -Vector3::z());
        let result = project(&span, &flipped, Projection::Normal, &options()).unwrap();
        assert_matches_point_projection(&result, span.curve(), &flipped, Projection::Normal);
    }

    #[test]
    fn a_nurbs_curve_maps_its_control_points() {
        let curve = Curve::Nurbs(
            NurbsCurve::interpolate(&[
                Point3::new(0.0, 0.0, 2.0),
                Point3::new(1.0, 1.0, 3.0),
                Point3::new(2.0, 0.0, 2.5),
                Point3::new(3.0, -1.0, 3.0),
                Point3::new(4.0, 0.0, 2.0),
            ])
            .unwrap(),
        );
        let plane = tilted_plane();
        for mode in [
            Projection::Normal,
            Projection::along(Vector3::new(0.0, 0.0, -1.0)),
        ] {
            let result = project(&curve, &plane, mode, &options()).unwrap();
            let segment = only_segment(&result);
            let (Curve::Nurbs(in_3d), crate::geometry::Curve2::Nurbs(in_2d)) =
                (segment.curve.curve(), segment.pcurve.curve())
            else {
                panic!("a NURBS curve projects to NURBS curves");
            };
            let source = curve.to_nurbs().unwrap();
            assert_eq!(in_3d.degree(), source.degree());
            assert_eq!(in_2d.degree(), source.degree());
            assert_eq!(in_3d.control_points().len(), source.control_points().len());
            assert_matches_point_projection(&result, &curve, &plane, mode);
        }
    }

    #[test]
    fn a_circle_seen_edge_on_is_refused() {
        let source = Plane::new(Point3::new(0.0, 0.0, 1.0), Vector3::x(), Vector3::y());
        let span = TrimmedCurve::arc(source, 1.0, TAU);
        let edge_on = project(&span, &Plane::xy(), Projection::Normal, &options());
        assert!(matches!(edge_on, Err(ProjectionError::DegenerateImage)));
        let along = project(
            &span,
            &Plane::xy(),
            Projection::along(Vector3::new(0.5, 0.0, -1.0)),
            &options(),
        );
        assert!(matches!(along, Err(ProjectionError::DegenerateImage)));
    }

    #[test]
    fn an_unbounded_curve_is_refused() {
        let line = Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 1.0));
        let result = project(&line, &Plane::xy(), Projection::Normal, &options());
        assert!(matches!(result, Err(ProjectionError::UnboundedSource)));
    }

    #[test]
    fn a_helix_has_no_exact_image() {
        let helix = TrimmedCurve::new(
            Curve::helix(crate::geometry::Frame::xyz(), 1.0, 0.5),
            Interval::new(0.0, 5.0),
        );
        let result = project(&helix, &tilted_plane(), Projection::Normal, &options());
        assert!(matches!(
            result,
            Err(ProjectionError::UnsupportedCurve { curve: "helix" })
        ));
    }

    /// A circle tilted 45 degrees about the x axis, centred half a unit above
    /// the plane `z = 0`, so it dips below it between two crossings.
    fn dipping_circle() -> Plane {
        Plane::new(
            Point3::new(0.0, 0.0, 0.5),
            Vector3::x(),
            Vector3::new(0.0, -1.0, 1.0),
        )
    }

    #[test]
    fn an_arc_dipping_behind_the_plane_makes_two_chains_ending_on_a_miss() {
        let span = TrimmedCurve::arc(dipping_circle(), 2.0, 6.1);
        let mode = Projection::along(Vector3::new(0.0, 0.0, -1.0));
        let result = project(&span, &Plane::xy(), mode, &options()).unwrap();
        assert_eq!(result.chains.len(), 2);
        assert_eq!(result.chains[0].start, ChainEnd::SourceEnd);
        assert_eq!(result.chains[0].end, ChainEnd::Miss);
        assert_eq!(result.chains[1].start, ChainEnd::Miss);
        assert_eq!(result.chains[1].end, ChainEnd::SourceEnd);
        assert_matches_point_projection(&result, span.curve(), &Plane::xy(), mode);
        // The two parts in front are the parts of the arc above the plane.
        let first = result.chains[0].segments[0].source.interval;
        let second = result.chains[1].segments[0].source.interval;
        assert!(first.start.value().abs() < 1e-9 && first.end.value() > 3.0);
        assert!(second.start.value() > 5.0 && (second.end.value() - 6.1).abs() < 1e-9);
    }

    #[test]
    fn a_whole_circle_dipping_behind_the_plane_is_one_chain_across_its_seam() {
        let span = TrimmedCurve::arc(dipping_circle(), 2.0, TAU);
        let mode = Projection::along(Vector3::new(0.0, 0.0, -1.0));
        let result = project(&span, &Plane::xy(), mode, &options()).unwrap();
        assert_eq!(result.chains.len(), 1);
        let chain = &result.chains[0];
        assert_eq!(chain.segments.len(), 2);
        assert_eq!((chain.start, chain.end), (ChainEnd::Miss, ChainEnd::Miss));
        // The chain leaves the seam where the circle began.
        let seam = chain.segments[0].source.interval;
        assert!(seam.end.value() > TAU - 1e-9 || seam.start.value() > 5.0);
        assert_matches_point_projection(&result, span.curve(), &Plane::xy(), mode);
    }

    #[test]
    fn a_curve_wholly_behind_the_plane_misses() {
        let span = TrimmedCurve::segment(Point3::new(0.0, 0.0, -2.0), Point3::new(3.0, 1.0, -1.0));
        let result = project(
            &span,
            &Plane::xy(),
            Projection::along(Vector3::new(0.0, 0.0, -1.0)),
            &options(),
        );
        assert!(matches!(result, Err(ProjectionError::Miss)));
    }

    #[test]
    fn a_line_crossing_the_plane_keeps_the_part_in_front() {
        let span = TrimmedCurve::segment(Point3::new(0.0, 0.0, 2.0), Point3::new(4.0, 0.0, -2.0));
        let mode = Projection::along(Vector3::new(0.0, 0.0, -1.0));
        let result = project(&span, &Plane::xy(), mode, &options()).unwrap();
        let chain = &result.chains[0];
        assert_eq!(result.chains.len(), 1);
        assert_eq!(
            (chain.start, chain.end),
            (ChainEnd::SourceEnd, ChainEnd::Miss)
        );
        let kept = chain.segments[0].source.interval;
        assert!((kept.end.value() - 0.5).abs() < 1e-9);
        assert_matches_point_projection(&result, span.curve(), &Plane::xy(), mode);
    }

    #[test]
    fn a_direction_parallel_to_the_plane_is_refused() {
        let span = TrimmedCurve::segment(Point3::new(0.0, 0.0, 2.0), Point3::new(4.0, 1.0, 3.0));
        let result = project(
            &span,
            &Plane::xy(),
            Projection::along(Vector3::x()),
            &options(),
        );
        assert!(matches!(result, Err(ProjectionError::Miss)));
        let in_plane =
            TrimmedCurve::segment(Point3::new(0.0, 0.0, 0.0), Point3::new(4.0, 1.0, 0.0));
        let result = project(
            &in_plane,
            &Plane::xy(),
            Projection::along(Vector3::x()),
            &options(),
        );
        assert!(matches!(result, Err(ProjectionError::RayInSurface)));
    }

    #[test]
    fn an_edge_names_itself_as_the_source_of_its_segment() {
        let mut model = Model::<StandardPayload>::new();
        let key = add_line(
            &mut model,
            Point3::new(0.0, 0.0, 2.0),
            Point3::new(3.0, 1.0, 2.0),
        )
        .unwrap();
        let edge = model.edge(key).unwrap();
        let result = project(&edge, &Plane::xy(), Projection::Normal, &options()).unwrap();
        let segment = only_segment(&result);
        assert_eq!(segment.source.source, SegmentSource::Edge(key));
        assert_matches_point_projection(&result, edge.curve(), &Plane::xy(), Projection::Normal);
    }

    #[test]
    fn a_reversed_edge_is_projected_in_the_direction_it_is_walked() {
        let mut model = Model::<StandardPayload>::new();
        let key = add_line(
            &mut model,
            Point3::new(0.0, 0.0, 2.0),
            Point3::new(3.0, 1.0, 2.0),
        )
        .unwrap();
        let edge = model.edge(key).unwrap().reversed();
        let result = project(&edge, &Plane::xy(), Projection::Normal, &options()).unwrap();
        let segment = only_segment(&result);
        assert!((segment.curve.start() - Point3::new(3.0, 1.0, 0.0)).norm() < 1e-9);
        assert!((segment.curve.end() - Point3::new(0.0, 0.0, 0.0)).norm() < 1e-9);
        assert_matches_point_projection(&result, edge.curve(), &Plane::xy(), Projection::Normal);
    }

    #[test]
    fn a_circle_edge_is_one_closed_chain() {
        let mut model = Model::<StandardPayload>::new();
        let key = add_circle(
            &mut model,
            Plane::new(Point3::new(0.0, 0.0, 4.0), Vector3::x(), Vector3::z()),
            1.5,
        )
        .unwrap();
        let edge = model.edge(key).unwrap();
        for mode in [Projection::Normal, slanted()] {
            let result = project(&edge, &tilted_plane(), mode, &options()).unwrap();
            assert_eq!(result.chains.len(), 1);
            assert!(result.chains[0].closed());
            assert_matches_point_projection(&result, edge.curve(), &tilted_plane(), mode);
        }
    }

    fn rectangle(
        model: &mut Model<StandardPayload>,
        z: f64,
    ) -> crate::topology::shape_keys::ProfileKey {
        add_polyline(
            model,
            &[
                Point3::new(0.0, 0.0, z),
                Point3::new(2.0, 0.0, z),
                Point3::new(2.0, 1.0, z),
                Point3::new(0.0, 1.0, z),
                Point3::new(0.0, 0.0, z),
            ],
        )
        .unwrap()
    }

    #[test]
    fn a_rectangle_profile_is_one_closed_chain_of_four_segments() {
        let mut model = Model::<StandardPayload>::new();
        let key = rectangle(&mut model, 1.0);
        let profile = model.profile(key).unwrap();
        for mode in [Projection::Normal, slanted()] {
            let result = project(&profile, &Plane::xy(), mode, &options()).unwrap();
            assert_eq!(result.chains.len(), 1);
            let chain = &result.chains[0];
            assert!(chain.closed());
            assert_eq!(chain.segments.len(), 4);
            let mut edges: Vec<_> = chain.segments.iter().map(|s| s.source.source).collect();
            edges.dedup();
            assert_eq!(edges.len(), 4);
            // Each segment begins where the one before it ends.
            for index in 0..4 {
                let here = chain.segments[index].curve.end();
                let next = chain.segments[(index + 1) % 4].curve.start();
                assert!((here - next).norm() < 1e-9);
            }
        }
    }

    #[test]
    fn an_open_profile_is_one_chain_between_two_source_ends() {
        let mut model = Model::<StandardPayload>::new();
        let key = add_polyline(
            &mut model,
            &[
                Point3::new(0.0, 0.0, 1.0),
                Point3::new(2.0, 0.0, 1.0),
                Point3::new(2.0, 1.0, 1.0),
            ],
        )
        .unwrap();
        let profile = model.profile(key).unwrap();
        let result = project(&profile, &Plane::xy(), Projection::Normal, &options()).unwrap();
        assert_eq!(result.chains.len(), 1);
        let chain = &result.chains[0];
        assert_eq!(
            (chain.start, chain.end),
            (ChainEnd::SourceEnd, ChainEnd::SourceEnd)
        );
        assert_eq!(chain.segments.len(), 2);
        let here = chain.segments[0].curve.end();
        assert!((here - chain.segments[1].curve.start()).norm() < 1e-9);
    }

    #[test]
    fn a_profile_crossing_the_plane_keeps_the_chain_in_front() {
        // A rectangle standing on the x axis, half below the plane.
        let mut model = Model::<StandardPayload>::new();
        let key = add_polyline(
            &mut model,
            &[
                Point3::new(0.0, 0.0, -1.0),
                Point3::new(2.0, 0.0, -1.0),
                Point3::new(2.0, 0.0, 1.0),
                Point3::new(0.0, 0.0, 1.0),
                Point3::new(0.0, 0.0, -1.0),
            ],
        )
        .unwrap();
        let profile = model.profile(key).unwrap();
        let mode = Projection::along(Vector3::new(0.0, 0.3, -1.0));
        let result = project(&profile, &Plane::xy(), mode, &options()).unwrap();
        assert_eq!(result.chains.len(), 1);
        let chain = &result.chains[0];
        assert_eq!(chain.segments.len(), 3);
        assert_eq!((chain.start, chain.end), (ChainEnd::Miss, ChainEnd::Miss));
        let curves: Vec<_> = chain.curves().collect();
        for pair in curves.windows(2) {
            assert!((pair[0].end() - pair[1].start()).norm() < 1e-9);
        }
        assert_eq!(result.curves().count(), 3);
        assert_eq!(result.pcurves().count(), 3);
    }
}
