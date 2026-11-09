//! Profiles from a curve projection.

use crate::builders::edges::{add_closed_curve_edit, add_edge_edit};
use crate::builders::errors::{EdgeCreationError, ModelEditFailure, PolylineError};
use crate::builders::profiles::add_profile_from_edges_edit;
use crate::geometry::{Curve, LINEAR_TOLERANCE, NurbsError, Periodicity, Point3, TrimmedCurve};
use crate::model::{Model, OpResult};
use crate::projection::{CurveProjection, ProjectedChain};
use crate::topology::ModelEdit;
use crate::topology::edit::ModelEditError;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, ProfileKey};
use thiserror::Error;

/// What [`add_profiles_from_projection`] refused.
///
/// Every refusal leaves the model unchanged.
#[derive(Debug, Error)]
pub enum ProjectionProfilesError {
    #[error("the projection has no chains")]
    EmptyProjection,
    /// A chain's segments each begin where the previous one ends; these do not.
    #[error("segment {segment} of chain {chain} begins {gap} away from where the one before ends")]
    BrokenChain {
        chain: usize,
        segment: usize,
        gap: f64,
    },
    /// A closed chain runs round to where it began; this one stops short.
    #[error("chain {chain} is closed but its last segment ends {gap} away from its first")]
    UnclosedChain { chain: usize, gap: f64 },
    #[error("segment {segment} of chain {chain} cannot be an edge")]
    Edge {
        chain: usize,
        segment: usize,
        #[source]
        source: EdgeCreationError,
    },
    #[error("segment {segment} of chain {chain} has no exact edge support")]
    Support {
        chain: usize,
        segment: usize,
        #[source]
        source: NurbsError,
    },
    #[error("chain {chain} cannot be joined into a profile")]
    Profile {
        chain: usize,
        #[source]
        source: PolylineError,
    },
    #[error("projection model edit failed")]
    ModelEditFailed(#[source] ModelEditFailure),
}

impl From<ModelEditError> for ProjectionProfilesError {
    fn from(error: ModelEditError) -> Self {
        Self::ModelEditFailed(ModelEditFailure::new(error))
    }
}

/// A projected chain as the profile it became.
#[derive(Clone, Debug)]
pub struct ProjectionProfile {
    pub profile: ProfileKey,
    /// The edge of each segment of the chain, in the chain's order.
    pub edges: Vec<EdgeKey>,
}

/// What [`add_profiles_from_projection`] added: one profile per chain, in the chain order.
#[derive(Clone, Debug)]
pub struct ProfilesFromProjection {
    pub profiles: Vec<ProjectionProfile>,
    revision: Option<u64>,
}

impl OpResult for ProfilesFromProjection {
    fn stamp(&mut self, revision: u64) {
        self.revision = Some(revision);
    }
}

/// Adds the edges and profiles a curve projection draws.
///
/// Each chain becomes one profile, closed when the chain is, and each segment
/// becomes one edge on the segment's 3D curve. Consecutive segments of a chain
/// share the vertex where one ends and the next begins. A profile cannot
/// branch, so chains meeting at a source vertex stay separate profiles with
/// vertices of their own.
///
/// An edge holds only its 3D curve: a pcurve is stored on the face that the
/// edge bounds, and the edges added here bound none, so the segments' pcurves
/// are not kept.
///
/// A projection without chains is refused, and so is a chain whose segments do
/// not continue one another within [`LINEAR_TOLERANCE`].
pub fn add_profiles_from_projection<P: Payload, Hit>(
    model: &mut Model<P>,
    projection: &CurveProjection<Hit>,
) -> Result<ProfilesFromProjection, ProjectionProfilesError> {
    model.transaction_result(|edit| add_profiles_from_projection_edit(edit, projection))
}

/// Refuses what cannot be drawn, then draws it inside the caller's transaction.
pub(crate) fn add_profiles_from_projection_edit<P: Payload, Hit>(
    edit: &mut ModelEdit<'_, P>,
    projection: &CurveProjection<Hit>,
) -> Result<ProfilesFromProjection, ProjectionProfilesError> {
    if projection.chains.is_empty() {
        return Err(ProjectionProfilesError::EmptyProjection);
    }
    let corners = projection
        .chains
        .iter()
        .enumerate()
        .map(|(index, chain)| chain_corners(index, chain))
        .collect::<Result<Vec<_>, _>>()?;
    let mut profiles = Vec::with_capacity(corners.len());
    for (index, (chain, corners)) in projection.chains.iter().zip(&corners).enumerate() {
        profiles.push(add_chain_edit(edit, index, chain, corners)?);
    }
    Ok(ProfilesFromProjection {
        profiles,
        revision: None,
    })
}

/// The point each segment of the chain starts from, then where the chain ends.
///
/// A segment's start and the previous one's end are one point: the earlier one.
fn chain_corners<Hit>(
    chain_index: usize,
    chain: &ProjectedChain<Hit>,
) -> Result<Vec<Point3>, ProjectionProfilesError> {
    let mut corners = vec![chain.segments[0].curve.start()];
    for (index, segment) in chain.segments.iter().enumerate() {
        if index > 0 {
            let gap = (segment.curve.start() - corners[index]).norm();
            if gap > LINEAR_TOLERANCE {
                return Err(ProjectionProfilesError::BrokenChain {
                    chain: chain_index,
                    segment: index,
                    gap,
                });
            }
        }
        corners.push(segment.curve.end());
    }
    if chain.closed() && chain.segments.len() > 1 {
        let last = corners.len() - 1;
        let gap = (corners[last] - corners[0]).norm();
        if gap > LINEAR_TOLERANCE {
            return Err(ProjectionProfilesError::UnclosedChain {
                chain: chain_index,
                gap,
            });
        }
        corners[last] = corners[0];
    }
    Ok(corners)
}

/// Adds the edge of every segment, then the profile joining them.
fn add_chain_edit<P: Payload, Hit>(
    edit: &mut ModelEdit<'_, P>,
    chain_index: usize,
    chain: &ProjectedChain<Hit>,
    corners: &[Point3],
) -> Result<ProjectionProfile, ProjectionProfilesError> {
    let whole_curve = chain.closed() && chain.segments.len() == 1;
    let mut edges = Vec::with_capacity(chain.segments.len());
    for (index, segment) in chain.segments.iter().enumerate() {
        let support =
            edge_support(&segment.curve).map_err(|source| ProjectionProfilesError::Support {
                chain: chain_index,
                segment: index,
                source,
            })?;
        let key = if whole_curve {
            add_closed_curve_edit(edit, support)
        } else {
            add_edge_edit(edit, corners[index], corners[index + 1], support)
        };
        edges.push(key.map_err(|source| ProjectionProfilesError::Edge {
            chain: chain_index,
            segment: index,
            source,
        })?);
    }
    let profile = add_profile_from_edges_edit(edit, &edges).map_err(|source| {
        ProjectionProfilesError::Profile {
            chain: chain_index,
            source,
        }
    })?;
    Ok(ProjectionProfile { profile, edges })
}

/// The support whose span between the segment's end points is the segment.
///
/// An edge keeps a whole support and reads its span off its vertices, going
/// forward on a periodic support, so a segment walked against its support's
/// parameter takes the reversed support. A NURBS support is cut to the segment
/// exactly, since its span cannot be read back from points without error.
fn edge_support(segment: &TrimmedCurve) -> Result<Curve, NurbsError> {
    let interval = segment.interval();
    let backwards = interval.delta() < 0.0;
    match (segment.curve(), segment.curve().periodicity()) {
        (curve, Periodicity::Periodic(_)) => Ok(if backwards {
            curve.reversed()
        } else {
            curve.clone()
        }),
        (curve @ Curve::Nurbs(_), _) => {
            if backwards {
                Ok(curve.trimmed_native(interval.reversed())?.reversed())
            } else {
                curve.trimmed_native(interval)
            }
        }
        (curve, _) => Ok(curve.clone()),
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::TAU;

    use nalgebra::Vector3;

    use super::*;
    use crate::builders::faces::add_polygon_with_holes;
    use crate::builders::profiles::add_polyline;
    use crate::builders::sheets::add_sheet_from_faces;
    use crate::geometry::Plane;
    use crate::geometry::parameter::Fraction;
    use crate::projection::{FaceList, ProjectOptions, Projection, project};
    use crate::topology::StandardPayload;
    use crate::topology::closed::Closeable;
    use crate::topology::shape_keys::FaceKey;

    type TestModel = Model<StandardPayload>;

    fn options() -> ProjectOptions {
        ProjectOptions::default()
    }

    fn down() -> Projection {
        Projection::along(-Vector3::z())
    }

    fn inclined_plane() -> Plane {
        Plane::new(
            Point3::new(0.0, 0.0, 1.0),
            Vector3::x(),
            Vector3::new(0.0, -1.0, 1.0),
        )
    }

    fn distance_to(plane: &Plane, point: Point3) -> f64 {
        plane.normal().dot(&(point - plane.origin())).abs()
    }

    fn circle_over(x: f64, height: f64) -> TrimmedCurve {
        let plane = Plane::new(Point3::new(x, 1.0, height), Vector3::x(), Vector3::z());
        TrimmedCurve::arc(plane, 0.8, TAU)
    }

    fn polygon(model: &mut TestModel, plane: Plane, points: &[Point3]) -> FaceKey {
        add_polygon_with_holes(model, plane, points, &[]).unwrap()
    }

    #[test]
    fn a_circle_projected_onto_a_plane_is_one_closed_profile_of_one_edge() {
        for mode in [Projection::Normal, down()] {
            let mut model = TestModel::new();
            let plane = inclined_plane();
            let projection = project(&circle_over(1.0, 5.0), &plane, mode, &options()).unwrap();
            let added = add_profiles_from_projection(&mut model, &projection).unwrap();
            assert_eq!(added.profiles.len(), 1);
            let profile = model.profile(added.profiles[0].profile).unwrap();
            assert!(profile.is_closed());
            assert_eq!(profile.edges().len(), 1);
            assert_eq!(added.profiles[0].edges.len(), 1);
            let edge = model.edge(added.profiles[0].edges[0]).unwrap();
            let segment = &projection.chains[0].segments[0].curve;
            assert!((edge.length() - segment.length()).abs() < 1e-7);
            for step in 0..8 {
                let fraction = Fraction::new(f64::from(step) / 8.0);
                assert!(distance_to(&plane, edge.trimmed_curve().point_at(fraction)) < 1e-9);
            }
        }
    }

    #[test]
    fn a_rectangle_projected_onto_an_inclined_plane_is_one_closed_profile_of_four_edges() {
        let mut model = TestModel::new();
        let at = |x: f64, y: f64| Point3::new(x, y, 3.0);
        let rectangle = add_polyline(
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
        let plane = inclined_plane();
        for mode in [Projection::Normal, down()] {
            let projection = {
                let source = model.profile(rectangle).unwrap();
                project(&source, &plane, mode, &options()).unwrap()
            };
            let added = add_profiles_from_projection(&mut model, &projection).unwrap();
            assert_eq!(added.profiles.len(), 1);
            let profile = model.profile(added.profiles[0].profile).unwrap();
            assert!(profile.is_closed());
            assert_eq!(profile.edges().len(), 4);
            assert_eq!(profile.vertices().len(), 4);
            for vertex in profile.vertices() {
                assert!(distance_to(&plane, *vertex.point()) < 1e-9);
            }
            for (edge, segment) in profile.edges().iter().zip(projection.segments()) {
                assert!((edge.length() - segment.curve.length()).abs() < 1e-9);
            }
        }
    }

    #[test]
    fn a_circle_over_a_sheet_fold_is_one_closed_profile_joined_on_the_shared_edge() {
        let mut model = TestModel::new();
        let flat = polygon(
            &mut model,
            Plane::xy(),
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
                Point3::new(2.0, 2.0, 0.0),
                Point3::new(0.0, 2.0, 0.0),
            ],
        );
        let tilt: f64 = 20.0_f64.to_radians();
        let (across, height) = (3.0 * tilt.cos(), 3.0 * tilt.sin());
        let hinged = polygon(
            &mut model,
            Plane::from_xy(
                Point3::new(2.0, 0.0, 0.0),
                Vector3::new(tilt.cos(), 0.0, tilt.sin()),
                Vector3::y(),
            ),
            &[
                Point3::new(2.0, 2.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
                Point3::new(2.0 + across, 0.0, height),
                Point3::new(2.0 + across, 2.0, height),
            ],
        );
        add_sheet_from_faces(&mut model, &[flat, hinged]).unwrap();
        let faces = [flat, hinged];
        let projection = {
            let list = FaceList::new(&model, &faces);
            project(&circle_over(2.0, 5.0), &list, down(), &options()).unwrap()
        };
        let added = add_profiles_from_projection(&mut model, &projection).unwrap();
        assert_eq!(added.profiles.len(), 1);
        let profile = model.profile(added.profiles[0].profile).unwrap();
        assert!(profile.is_closed());
        assert_eq!(profile.edges().len(), 2);
        let vertices = profile.vertices();
        assert_eq!(vertices.len(), 2);
        for vertex in vertices {
            let point = vertex.point();
            assert!((point.x - 2.0).abs() < 1e-7 && point.z.abs() < 1e-7);
            assert!(point.y > 0.0 && point.y < 2.0);
        }
    }

    #[test]
    fn a_projection_with_two_chains_gives_two_profiles_in_chain_order() {
        let mut model = TestModel::new();
        let at = |x: f64, y: f64| Point3::new(x, y, 0.0);
        let outer = [at(0.0, 0.0), at(4.0, 0.0), at(4.0, 4.0), at(0.0, 4.0)];
        let inner = [at(1.0, 1.0), at(3.0, 1.0), at(3.0, 3.0), at(1.0, 3.0)];
        let key = add_polygon_with_holes(&mut model, Plane::xy(), &outer, &[&inner]).unwrap();
        let span = TrimmedCurve::segment(Point3::new(-1.0, 2.0, 3.0), Point3::new(5.0, 2.0, 3.0));
        let projection = {
            let face = model.face_unchecked(key);
            project(&span, &face, down(), &options()).unwrap()
        };
        assert_eq!(projection.chains.len(), 2);
        let added = add_profiles_from_projection(&mut model, &projection).unwrap();
        assert_eq!(added.profiles.len(), 2);
        let starts: Vec<_> = added
            .profiles
            .iter()
            .map(|added| model.profile(added.profile).unwrap().start().point().x)
            .collect();
        assert!(starts[0].abs() < 1e-7 && (starts[1] - 3.0).abs() < 1e-7);
        for added in &added.profiles {
            let profile = model.profile(added.profile).unwrap();
            assert!(!profile.is_closed());
            assert_eq!(profile.edges().len(), 1);
        }
    }

    #[test]
    fn a_projection_without_chains_is_refused() {
        let mut model = TestModel::new();
        let projection = CurveProjection::<()> { chains: vec![] };
        let error = add_profiles_from_projection(&mut model, &projection).unwrap_err();
        assert!(matches!(error, ProjectionProfilesError::EmptyProjection));
    }
}
