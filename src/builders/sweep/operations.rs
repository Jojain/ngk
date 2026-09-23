//! Sweep operations and their edit-scoped implementation.

use std::collections::{HashMap, HashSet};
use std::f64::consts::PI;

use nalgebra::{Rotation3, UnitVector3, Vector3};
use radians::Rad64;

use crate::builders::faces::reverse_face_winding_edit;
use crate::builders::transform::rigid;
use crate::geometry::parameter::Fraction;
use crate::geometry::{
    ANGULAR_TOLERANCE, Axis3, Curve, Degree, Frame, LINEAR_TOLERANCE, NurbsCurve, NurbsSurface,
    Plane, Point2, Point3, PointCoincidence, Rigid, Surface, SurfaceOfRevolution, TrimmedCurve,
    TrimmedCurve2, make_compatible,
};
use crate::model::{Cell2, MergeTopology, Model, OpResult, StaleResult};
use crate::topology::attributes::{
    EdgeAttr, FaceAttr, ProfileAttr, SheetAttr, SolidAttr, VertexAttr,
};
use crate::topology::edge::Edge;
use crate::topology::edit::{EditKey, ModelEdit};
use crate::topology::embedding::EntityOwner;
use crate::topology::face::Face;
use crate::topology::gmap::{Dart, Dim};
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;
use crate::topology::shape_keys::{EdgeKey, FaceKey, SolidKey, VertexKey};
use crate::topology::solid::Solid;
use crate::topology::vertex::Vertex;

use super::SweepError;

/// How the section frame is carried along a smooth spine.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub enum SweepFrame {
    /// Carry the frame by the shortest rotation between successive tangents.
    #[default]
    Parallel,
    /// Use the spine tangent and principal normal as the moving frame.
    Frenet,
    /// Turn the section about an axis and slide it along it, following the
    /// spine: at every point the frame's `x` runs out from the axis and its
    /// `z` along it.
    ///
    /// Along a helix round that axis this is the screw motion itself, so a
    /// section drawn in a plane through the axis stays in one -- the way a
    /// thread profile is drawn. The section is carried rigidly rather than
    /// held perpendicular to the spine, so it need only cross the spine, not
    /// stand square to it; and since the frame is read off the spine's
    /// position alone, a junction needs no transition.
    Axial(Axis3),
}

/// How a sweep crosses a junction that is only position-continuous.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum SweepTransition {
    /// Accept only tangent-continuous junctions and add no corner geometry.
    #[default]
    Smooth,
    /// Trim the two adjacent wall groups to their common miter section.
    Straight,
    /// Turn the arriving section around the junction axis before continuing.
    Rounded,
}

/// Controls frame transport, corner treatment, and curved-spine resolution.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SweepOptions {
    /// The moving frame used within smooth spine segments.
    pub frame: SweepFrame,
    /// The treatment applied at C0 spine junctions.
    pub transition: SweepTransition,
    /// Number of intervals used on a curved spine segment.
    pub samples_per_segment: usize,
}

impl Default for SweepOptions {
    fn default() -> Self {
        Self {
            frame: SweepFrame::Parallel,
            transition: SweepTransition::Smooth,
            samples_per_segment: 8,
        }
    }
}

/// The solid and boundary faces produced by sweeping a face.
#[derive(Debug)]
pub struct FaceSweep {
    /// The assembled solid.
    pub solid: SolidKey,
    /// The source face, retained as the first cap.
    pub start_cap: FaceKey,
    /// The moved copy used as the last cap.
    pub end_cap: FaceKey,
    /// Faces in path order, and in section-loop order within each wall group.
    pub laterals: Vec<FaceKey>,
    revision: Option<u64>,
}

/// Borrowed views for a [`FaceSweep`] result.
pub struct FaceSweepView<'m, P: Payload> {
    /// The assembled solid.
    pub solid: Solid<'m, P>,
    /// The source face retained as the first cap.
    pub start_cap: Face<'m, P>,
    /// The moved copy used as the last cap.
    pub end_cap: Face<'m, P>,
    /// The lateral faces in path order.
    pub laterals: Vec<Face<'m, P>>,
}

impl OpResult for FaceSweep {
    fn stamp(&mut self, revision: u64) {
        self.revision = Some(revision);
    }
}

impl FaceSweep {
    /// Resolves this result against the model revision that committed it.
    pub fn view<'m, P: Payload>(
        &self,
        model: &'m Model<P>,
    ) -> Result<FaceSweepView<'m, P>, StaleResult> {
        StaleResult::check(self.revision, model)?;
        Ok(FaceSweepView {
            solid: model.solid_unchecked(self.solid),
            start_cap: model.face_unchecked(self.start_cap),
            end_cap: model.face_unchecked(self.end_cap),
            laterals: self
                .laterals
                .iter()
                .map(|key| model.face_unchecked(*key))
                .collect(),
        })
    }
}

/// A borrowed topological path that can drive a sweep.
///
/// Implemented by [`Edge`] and [`Profile`]. Their contextual orientation is
/// preserved, so reversing either view reverses the resulting spine. Geometry
/// is copied before the target model's transaction opens; the path itself is
/// never consumed or modified and may belong to another model.
pub trait SweepSpine {
    /// Returns the path's finite curve spans in traversal order.
    fn segments(&self) -> Vec<TrimmedCurve>;
}

impl<P: Payload> SweepSpine for Edge<'_, P> {
    fn segments(&self) -> Vec<TrimmedCurve> {
        vec![self.trimmed_curve()]
    }
}

impl<P: Payload> SweepSpine for Profile<'_, P> {
    fn segments(&self) -> Vec<TrimmedCurve> {
        self.edges()
            .into_iter()
            .map(|edge| edge.trimmed_curve())
            .collect()
    }
}

/// Sweeps `face` along an oriented edge or profile view.
///
/// The source face is the start cap and is expected to be placed at the
/// spine's first point with its normal parallel to the first tangent. Curved
/// segments are NURBS-skinned through transported copies of each boundary
/// edge. At a C0 junction, [`SweepTransition::Straight`] shares a section in
/// the tangent-bisector plane and [`SweepTransition::Rounded`] inserts an
/// exact surface-of-revolution wall group.
pub fn add_swept_face<P: Payload, S: SweepSpine + ?Sized>(
    model: &mut Model<P>,
    face: FaceKey,
    spine: &S,
    options: SweepOptions,
) -> Result<FaceSweep, SweepError> {
    let segments = spine.segments();
    model.transaction_result(|edit| add_swept_face_edit(edit, face, &segments, options))
}

/// Builds a face sweep inside an already-open transaction.
pub(crate) fn add_swept_face_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    spine: &[TrimmedCurve],
    options: SweepOptions,
) -> Result<FaceSweep, SweepError> {
    SweepPlan::read(edit, face, spine, options)?.build(edit)
}

#[derive(Clone)]
struct SectionEdge {
    key: EdgeKey,
    curve: NurbsCurve,
}

struct SectionLoop {
    edges: Vec<SectionEdge>,
}

struct SweepPlan {
    face: FaceKey,
    loops: Vec<SectionLoop>,
    wall_groups: Vec<WallGroup>,
    end_motion: Rigid,
    start_tangent: Vector3<f64>,
    end_tangent: Vector3<f64>,
}

enum WallGroup {
    AlongSpine {
        placements: Vec<Placement>,
    },
    AroundCorner {
        axis: Axis3,
        angle: f64,
        start: Rigid,
        inward: UnitVector3<f64>,
    },
}

#[derive(Clone)]
enum Placement {
    Rigid(Rigid),
    Miter {
        motion: Rigid,
        corner: Point3,
        normal: UnitVector3<f64>,
        along: UnitVector3<f64>,
    },
}

impl Placement {
    fn point(&self, point: Point3) -> Point3 {
        match self {
            Self::Rigid(motion) => motion.apply(point),
            Self::Miter {
                motion,
                corner,
                normal,
                along,
            } => {
                let point = motion.apply(point);
                point - **along * ((point - *corner).dot(normal.as_ref()) / normal.dot(along))
            }
        }
    }

    fn curve(&self, curve: &NurbsCurve) -> NurbsCurve {
        curve.map_control_points(|point| self.point(point))
    }
}

impl SweepPlan {
    fn read<P: Payload>(
        model: &Model<P>,
        face_key: FaceKey,
        spine: &[TrimmedCurve],
        options: SweepOptions,
    ) -> Result<Self, SweepError> {
        let face = model
            .face(face_key)
            .ok_or(SweepError::MissingFace { key: face_key })?;
        if face.outer_loop().is_none() {
            return Err(SweepError::FaceHasNoOuterLoop);
        }
        if spine.is_empty() {
            return Err(SweepError::EmptySpine);
        }
        if spine
            .last()
            .is_some_and(|last| last.end().coincides(spine[0].start(), LINEAR_TOLERANCE))
        {
            return Err(SweepError::SpineClosesOnItself);
        }
        for (index, pair) in spine.windows(2).enumerate() {
            if !pair[0].end().coincides(pair[1].start(), LINEAR_TOLERANCE) {
                return Err(SweepError::DisconnectedSpine {
                    junction: index + 1,
                });
            }
        }

        let loops =
            face.loops()
                .into_iter()
                .map(|boundary| {
                    boundary
                        .darts()
                        .map(|dart| {
                            let edge = Edge::from_dart(model, dart)
                                .ok_or(SweepError::MissingEdge { dart })?;
                            let key = edge.key();
                            let curve = match edge.trimmed_curve().to_curve().map_err(|source| {
                                SweepError::SectionHasNoNurbsForm { key, source }
                            })? {
                                Curve::Nurbs(curve) => curve,
                                curve => curve.to_nurbs().map_err(|source| {
                                    SweepError::SectionHasNoNurbsForm { key, source }
                                })?,
                            };
                            Ok(SectionEdge { key, curve })
                        })
                        .collect::<Result<Vec<_>, SweepError>>()
                        .map(|edges| SectionLoop { edges })
                })
                .collect::<Result<Vec<_>, _>>()?;

        let PathPlan {
            wall_groups,
            end_motion,
            start_tangent,
            end_tangent,
        } = plan_wall_groups(spine, options)?;
        let crossing = face.normal_at(0.0, 0.0).dot(&start_tangent).abs();
        match options.frame {
            SweepFrame::Axial(_) if crossing <= ANGULAR_TOLERANCE => {
                return Err(SweepError::SectionAlongSpine);
            }
            SweepFrame::Axial(_) => {}
            SweepFrame::Parallel | SweepFrame::Frenet if crossing < 1.0 - ANGULAR_TOLERANCE => {
                return Err(SweepError::SectionNotNormalToSpine);
            }
            SweepFrame::Parallel | SweepFrame::Frenet => {}
        }
        Ok(Self {
            face: face_key,
            loops,
            wall_groups,
            end_motion,
            start_tangent,
            end_tangent,
        })
    }

    fn build<P: Payload>(self, edit: &mut ModelEdit<'_, P>) -> Result<FaceSweep, SweepError> {
        let laterals_face_outward = Face::new(edit, self.face)
            .normal_at(0.0, 0.0)
            .dot(&self.start_tangent)
            > 0.0;
        let end_cap = moved_face(edit, self.face, self.end_motion)?;
        orient_caps(
            edit,
            self.face,
            end_cap,
            self.start_tangent,
            self.end_tangent,
        );

        let mut built_wall_groups = Vec::with_capacity(self.wall_groups.len());
        let mut laterals = Vec::new();
        for wall_group in &self.wall_groups {
            let mut loops = Vec::with_capacity(self.loops.len());
            for section_loop in &self.loops {
                let faces = section_loop
                    .edges
                    .iter()
                    .map(|edge| build_wall(edit, edge, wall_group))
                    .collect::<Result<Vec<_>, _>>()?;
                sew_loop_rails(edit, &faces)?;
                laterals.extend(faces.iter().flatten().map(|face| face.key));
                loops.push(faces);
            }
            built_wall_groups.push(loops);
        }

        for loop_index in 0..self.loops.len() {
            for edge_index in 0..self.loops[loop_index].edges.len() {
                let walls = built_wall_groups
                    .iter()
                    .filter_map(|group| group[loop_index][edge_index].as_ref())
                    .collect::<Vec<_>>();
                for pair in walls.windows(2) {
                    sew_edges(edit, pair[0].last_section, pair[1].first_section)?;
                }
            }
        }

        let first = built_wall_groups
            .first()
            .expect("a nonempty spine builds a wall group");
        let last = built_wall_groups
            .last()
            .expect("a nonempty spine builds a wall group");
        sew_cap(edit, self.face, first, false)?;
        sew_cap(edit, end_cap, last, true)?;

        if !laterals_face_outward {
            for lateral in &laterals {
                reverse_face_winding_edit(edit, *lateral);
            }
        }

        let shell = edit.face_attr_unchecked(self.face).outer_unchecked();
        if edit.sheet_key(shell).is_none() {
            edit.add_sheet_derived_from(vec![EditKey::Face(self.face)], SheetAttr::new(shell));
        }
        let solid = edit
            .add_solid_derived_from(vec![EditKey::Face(self.face)], SolidAttr::new(shell, None));
        Ok(FaceSweep {
            solid,
            start_cap: self.face,
            end_cap,
            laterals,
            revision: None,
        })
    }
}

struct PathPlan {
    wall_groups: Vec<WallGroup>,
    end_motion: Rigid,
    start_tangent: Vector3<f64>,
    end_tangent: Vector3<f64>,
}

fn plan_wall_groups(spine: &[TrimmedCurve], options: SweepOptions) -> Result<PathPlan, SweepError> {
    let first_tangent = tangent(&spine[0], Fraction::START)?;
    let first_point = spine[0].start();
    let mut current = match options.frame {
        SweepFrame::Parallel => initial_frame(first_point, first_tangent),
        SweepFrame::Frenet => frenet_frame(&spine[0], Fraction::START, first_tangent)?,
        SweepFrame::Axial(axis) => axial_frame(axis, first_point, Fraction::START)?,
    };
    let base = current.clone();
    let mut pending_start = Placement::Rigid(Rigid::identity());
    let mut wall_groups = Vec::new();

    for (index, segment) in spine.iter().enumerate() {
        let frames = segment_frames(segment, current.clone(), options)?;
        let incoming = frames
            .last()
            .expect("a segment has endpoint frames")
            .clone();
        let mut placements = frames
            .iter()
            .map(|frame| Placement::Rigid(Rigid::between_frames(&base, frame)))
            .collect::<Vec<_>>();
        placements[0] = pending_start.clone();

        let Some(next) = spine.get(index + 1) else {
            let end_motion = Rigid::between_frames(&base, &incoming);
            wall_groups.push(WallGroup::AlongSpine { placements });
            // An axial frame's `z` is the axis, not the spine, so the spine is
            // asked for its own direction at the two ends.
            let (start_tangent, end_tangent) = match options.frame {
                SweepFrame::Axial(_) => (*first_tangent, *tangent(segment, Fraction::END)?),
                SweepFrame::Parallel | SweepFrame::Frenet => (*base.z_dir, *incoming.z_dir),
            };
            return Ok(PathPlan {
                wall_groups,
                end_motion,
                start_tangent,
                end_tangent,
            });
        };

        // The axial frame at a junction is the one both segments read off the
        // same point, so the next group starts exactly where this one ends.
        if let SweepFrame::Axial(_) = options.frame {
            wall_groups.push(WallGroup::AlongSpine { placements });
            pending_start = Placement::Rigid(Rigid::between_frames(&base, &incoming));
            current = incoming;
            continue;
        }

        let outgoing_tangent = tangent(next, Fraction::START)?;
        let (outgoing, angle, axis) = turn_frame(&incoming, outgoing_tangent, index + 1)?;
        if angle <= ANGULAR_TOLERANCE {
            pending_start = Placement::Rigid(Rigid::between_frames(&base, &outgoing));
        } else {
            match options.transition {
                SweepTransition::Smooth => {
                    return Err(SweepError::SpineTurnsACorner {
                        junction: index + 1,
                        angle,
                    });
                }
                SweepTransition::Straight => {
                    let bisector = UnitVector3::new_normalize(*incoming.z_dir + *outgoing.z_dir);
                    let miter = Placement::Miter {
                        motion: Rigid::between_frames(&base, &incoming),
                        corner: incoming.origin,
                        normal: bisector,
                        along: incoming.z_dir,
                    };
                    *placements.last_mut().expect("endpoint placement") = miter.clone();
                    pending_start = miter;
                }
                SweepTransition::Rounded => {
                    pending_start = Placement::Rigid(Rigid::between_frames(&base, &outgoing));
                }
            }
        }

        wall_groups.push(WallGroup::AlongSpine { placements });
        if angle > ANGULAR_TOLERANCE && options.transition == SweepTransition::Rounded {
            let inward = UnitVector3::new_normalize(
                *outgoing.z_dir - *incoming.z_dir * incoming.z_dir.dot(&outgoing.z_dir),
            );
            wall_groups.push(WallGroup::AroundCorner {
                axis,
                angle,
                start: Rigid::between_frames(&base, &incoming),
                inward,
            });
        }
        current = outgoing;
    }
    unreachable!("the final segment returns the plan")
}

fn initial_frame(origin: Point3, tangent: UnitVector3<f64>) -> Frame {
    let seed = if tangent.dot(&Vector3::x()).abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    Frame::from_xz(origin, seed, tangent)
}

fn tangent(segment: &TrimmedCurve, fraction: Fraction) -> Result<UnitVector3<f64>, SweepError> {
    UnitVector3::try_new(segment.derivative_at(fraction, 1), LINEAR_TOLERANCE).ok_or(
        SweepError::SpineHasNoTangent {
            fraction: fraction.value(),
        },
    )
}

fn segment_frames(
    segment: &TrimmedCurve,
    start: Frame,
    options: SweepOptions,
) -> Result<Vec<Frame>, SweepError> {
    // A line carries a transported frame unchanged from end to end, but an
    // axial frame turns along any line not parallel to the axis.
    let intervals = match (segment.curve(), options.frame) {
        (Curve::Line(_), SweepFrame::Parallel | SweepFrame::Frenet) => 1,
        _ => options.samples_per_segment.max(2),
    };
    let mut frames = Vec::with_capacity(intervals + 1);
    let mut previous = start;
    for sample in 0..=intervals {
        let fraction = Fraction::new(sample as f64 / intervals as f64);
        let point = segment.point_at(fraction);
        let direction = tangent(segment, fraction)?;
        let frame = match options.frame {
            SweepFrame::Parallel => {
                if sample == 0 {
                    Frame::from_xz(point, previous.x_dir, direction)
                } else {
                    transported_frame(&previous, point, direction, fraction.value())?
                }
            }
            SweepFrame::Frenet => frenet_frame(segment, fraction, direction)?,
            SweepFrame::Axial(axis) => axial_frame(axis, point, fraction)?,
        };
        previous = frame.clone();
        frames.push(frame);
    }
    Ok(frames)
}

fn frenet_frame(
    segment: &TrimmedCurve,
    fraction: Fraction,
    tangent: UnitVector3<f64>,
) -> Result<Frame, SweepError> {
    let acceleration = segment.derivative_at(fraction, 2);
    let normal = acceleration - *tangent * acceleration.dot(tangent.as_ref());
    let normal =
        UnitVector3::try_new(normal, LINEAR_TOLERANCE).ok_or(SweepError::SpineDoesNotCurve {
            fraction: fraction.value(),
        })?;
    Ok(Frame::from_xz(segment.point_at(fraction), normal, tangent))
}

/// The frame at `point` whose `x` runs out from `axis` and whose `z` along it.
fn axial_frame(axis: Axis3, point: Point3, fraction: Fraction) -> Result<Frame, SweepError> {
    let radial = UnitVector3::try_new(point - axis.project(point), LINEAR_TOLERANCE).ok_or(
        SweepError::SpineMeetsTheAxis {
            fraction: fraction.value(),
        },
    )?;
    Ok(Frame::from_xz(point, radial, axis.direction))
}

fn transported_frame(
    previous: &Frame,
    point: Point3,
    tangent: UnitVector3<f64>,
    fraction: f64,
) -> Result<Frame, SweepError> {
    let rotation = Rotation3::rotation_between(previous.z_dir.as_ref(), tangent.as_ref()).ok_or(
        SweepError::SpineReverses {
            from: 0.0,
            to: fraction,
        },
    )?;
    Ok(Frame::from_xz(point, rotation * *previous.x_dir, tangent))
}

fn turn_frame(
    incoming: &Frame,
    tangent: UnitVector3<f64>,
    junction: usize,
) -> Result<(Frame, f64, Axis3), SweepError> {
    let dot = incoming.z_dir.dot(&tangent).clamp(-1.0, 1.0);
    let angle = dot.acos();
    if (PI - angle).abs() <= ANGULAR_TOLERANCE {
        return Err(SweepError::SpineReverses {
            from: junction.saturating_sub(1) as f64,
            to: junction as f64,
        });
    }
    if angle <= ANGULAR_TOLERANCE {
        return Ok((
            incoming.clone(),
            0.0,
            Axis3::new(incoming.origin, incoming.x_dir),
        ));
    }
    let axis_direction = UnitVector3::new_normalize(incoming.z_dir.cross(&tangent));
    let axis = Axis3::new(incoming.origin, axis_direction);
    let rotation = Rotation3::from_axis_angle(&axis.direction, angle);
    Ok((
        Frame::from_xz(incoming.origin, rotation * *incoming.x_dir, tangent),
        angle,
        axis,
    ))
}

struct WallFace {
    key: FaceKey,
    first_section: Dart,
    last_section: Dart,
    start_rail: Option<Dart>,
    end_rail: Option<Dart>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RoundedWallShape {
    Omitted,
    Triangle { start_on_axis: bool },
    Quad,
}

fn build_wall<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    edge: &SectionEdge,
    wall_group: &WallGroup,
) -> Result<Option<WallFace>, SweepError> {
    match wall_group {
        WallGroup::AlongSpine { placements } => {
            let mut sections = placements
                .iter()
                .map(|placement| placement.curve(&edge.curve))
                .collect::<Vec<_>>();
            make_compatible(&mut sections).map_err(|source| SweepError::Skinning {
                key: edge.key,
                source,
            })?;
            let degree =
                Degree::new(3.min(sections.len() - 1)).map_err(|source| SweepError::Skinning {
                    key: edge.key,
                    source,
                })?;
            let surface = NurbsSurface::skinned(&sections, degree).map_err(|source| {
                SweepError::Skinning {
                    key: edge.key,
                    source,
                }
            })?;
            add_skinned_wall(edit, edge.key, surface).map(Some)
        }
        WallGroup::AroundCorner {
            axis,
            angle,
            start,
            inward,
        } => {
            let start_curve = edge.curve.map_control_points(|point| start.apply(point));
            match rounded_wall_shape(edge.key, &start_curve, *axis, *inward)? {
                RoundedWallShape::Omitted => Ok(None),
                RoundedWallShape::Triangle { start_on_axis } => add_rounded_triangle_wall(
                    edit,
                    edge.key,
                    start_curve,
                    *axis,
                    *angle,
                    start_on_axis,
                )
                .map(Some),
                RoundedWallShape::Quad => {
                    add_rounded_wall(edit, edge.key, start_curve, *axis, *angle).map(Some)
                }
            }
        }
    }
}

const BOX: [Point2; 4] = [
    Point2::new(0.0, 0.0),
    Point2::new(1.0, 0.0),
    Point2::new(1.0, 1.0),
    Point2::new(0.0, 1.0),
];

fn add_skinned_wall<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    source: EdgeKey,
    skin: NurbsSurface,
) -> Result<WallFace, SweepError> {
    let named = |source_error| SweepError::Boundary {
        key: source,
        source: source_error,
    };
    let boundary = [
        Curve::Nurbs(skin.isocurve_v(0.0).map_err(named)?),
        Curve::Nurbs(skin.isocurve_u(1.0).map_err(named)?),
        Curve::Nurbs(skin.isocurve_v(1.0).map_err(named)?.reversed()),
        Curve::Nurbs(skin.isocurve_u(0.0).map_err(named)?.reversed()),
    ];
    let corners = BOX.map(|corner| skin.point_at(corner.x, corner.y));
    add_quad_wall(edit, source, Surface::Nurbs(skin), boundary, corners, BOX)
}

fn add_rounded_wall<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    source: EdgeKey,
    start: NurbsCurve,
    axis: Axis3,
    angle: f64,
) -> Result<WallFace, SweepError> {
    let motion = Rigid::rotation(axis, Rad64::new(angle));
    let end = start.map_control_points(|point| motion.apply(point));
    let start_points = [start.point_at(0.0), start.point_at(1.0)];
    let end_points = [end.point_at(0.0), end.point_at(1.0)];
    let boundary = [
        Curve::Nurbs(start.clone()),
        corner_arc(axis, start_points[1], angle),
        Curve::Nurbs(end.reversed()),
        corner_arc(axis, end_points[0], -angle),
    ];
    let uv = [
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 0.0),
        Point2::new(1.0, angle),
        Point2::new(0.0, angle),
    ];
    add_quad_wall(
        edit,
        source,
        Surface::Revolution(SurfaceOfRevolution::new(Curve::Nurbs(start), axis)),
        boundary,
        [
            start_points[0],
            start_points[1],
            end_points[1],
            end_points[0],
        ],
        uv,
    )
}

fn add_rounded_triangle_wall<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    source: EdgeKey,
    start: NurbsCurve,
    axis: Axis3,
    angle: f64,
    start_on_axis: bool,
) -> Result<WallFace, SweepError> {
    let motion = Rigid::rotation(axis, Rad64::new(angle));
    let end = start.map_control_points(|point| motion.apply(point));
    let start_points = [start.point_at(0.0), start.point_at(1.0)];
    let end_points = [end.point_at(0.0), end.point_at(1.0)];
    let surface = Surface::Revolution(SurfaceOfRevolution::new(Curve::Nurbs(start.clone()), axis));

    if start_on_axis {
        add_triangle_wall(
            edit,
            source,
            surface,
            [
                Curve::Nurbs(start),
                corner_arc(axis, start_points[1], angle),
                Curve::Nurbs(end.reversed()),
            ],
            [start_points[0], start_points[1], end_points[1]],
            [
                (Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)),
                (Point2::new(1.0, 0.0), Point2::new(1.0, angle)),
                (Point2::new(1.0, angle), Point2::new(0.0, angle)),
            ],
            true,
        )
    } else {
        add_triangle_wall(
            edit,
            source,
            surface,
            [
                Curve::Nurbs(start),
                Curve::Nurbs(end.reversed()),
                corner_arc(axis, end_points[0], -angle),
            ],
            [start_points[0], start_points[1], end_points[0]],
            [
                (Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)),
                (Point2::new(1.0, angle), Point2::new(0.0, angle)),
                (Point2::new(0.0, angle), Point2::new(0.0, 0.0)),
            ],
            false,
        )
    }
}

fn corner_arc(axis: Axis3, point: Point3, angle: f64) -> Curve {
    let projected = axis.project(point);
    let direction = if angle < 0.0 {
        -axis.direction
    } else {
        axis.direction
    };
    let radial = point - projected;
    let plane = if radial.norm() <= LINEAR_TOLERANCE {
        Plane::new(projected, Vector3::x(), direction)
    } else {
        Plane::new(projected, radial, direction)
    };
    Curve::circle(plane, radial.norm())
}

fn rounded_wall_shape(
    key: EdgeKey,
    curve: &NurbsCurve,
    axis: Axis3,
    inward: UnitVector3<f64>,
) -> Result<RoundedWallShape, SweepError> {
    let control_points = curve
        .control_points()
        .iter()
        .map(|point| point.to_cartesian())
        .collect::<Vec<_>>();
    let radial_vectors = control_points
        .iter()
        .map(|point| *point - axis.project(*point))
        .collect::<Vec<_>>();
    let Some(reference) = radial_vectors
        .iter()
        .find_map(|radial| UnitVector3::try_new(*radial, LINEAR_TOLERANCE))
    else {
        return Ok(RoundedWallShape::Omitted);
    };

    for radial in &radial_vectors {
        if radial.norm() > LINEAR_TOLERANCE && radial.dot(&reference) <= LINEAR_TOLERANCE {
            return Err(SweepError::SectionMeetsTheTurningAxis { key });
        }
        if radial.dot(&inward) > LINEAR_TOLERANCE {
            return Err(SweepError::RoundedTransitionWouldSelfIntersect { key });
        }
    }
    let start = curve.point_at(0.0);
    let end = curve.point_at(1.0);
    let start_on_axis = (start - axis.project(start)).norm() <= LINEAR_TOLERANCE;
    let end_on_axis = (end - axis.project(end)).norm() <= LINEAR_TOLERANCE;
    match (start_on_axis, end_on_axis) {
        (true, true) => Err(SweepError::SectionMeetsTheTurningAxis { key }),
        (true, false) => Ok(RoundedWallShape::Triangle {
            start_on_axis: true,
        }),
        (false, true) => Ok(RoundedWallShape::Triangle {
            start_on_axis: false,
        }),
        (false, false) => Ok(RoundedWallShape::Quad),
    }
}

fn add_triangle_wall<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    source: EdgeKey,
    surface: Surface,
    boundary: [Curve; 3],
    corners: [Point3; 3],
    uv: [(Point2, Point2); 3],
    start_on_axis: bool,
) -> Result<WallFace, SweepError> {
    let darts: [Dart; 6] = std::array::from_fn(|_| edit.add_dart());
    for index in 0..3 {
        edit.link(Dim::Zero, darts[2 * index], darts[2 * index + 1])?;
        edit.link(
            Dim::One,
            darts[2 * index + 1],
            darts[(2 * index + 2) % darts.len()],
        )?;
    }
    let sources = vec![EditKey::Edge(source)];
    for index in 0..3 {
        let dart = edit.cell_representative(darts[2 * index], Dim::Zero);
        edit.add_vertex_derived_from(sources.clone(), VertexAttr::new(dart, corners[index]));
        edit.add_edge_derived_from(
            sources.clone(),
            EdgeAttr::new(darts[2 * index], boundary[index].clone()),
        );
    }
    let pcurves = (0..3)
        .map(|index| {
            (
                darts[2 * index],
                TrimmedCurve2::segment(uv[index].0, uv[index].1),
            )
        })
        .collect::<HashMap<_, _>>();
    edit.add_profile_derived_from(sources.clone(), ProfileAttr::new(darts[0]));
    let key = edit.add_face_derived_from(
        sources,
        FaceAttr::with_pcurves(surface, darts[0], Vec::new(), pcurves),
    );
    let (last_section, start_rail, end_rail) = if start_on_axis {
        (darts[5], None, Some(darts[2]))
    } else {
        (darts[3], Some(darts[5]), None)
    };
    Ok(WallFace {
        key,
        first_section: darts[0],
        last_section,
        start_rail,
        end_rail,
    })
}

fn add_quad_wall<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    source: EdgeKey,
    surface: Surface,
    boundary: [Curve; 4],
    corners: [Point3; 4],
    uv: [Point2; 4],
) -> Result<WallFace, SweepError> {
    let darts: [Dart; 8] = std::array::from_fn(|_| edit.add_dart());
    for index in 0..4 {
        edit.link(Dim::Zero, darts[2 * index], darts[2 * index + 1])?;
        edit.link(
            Dim::One,
            darts[2 * index + 1],
            darts[(2 * index + 2) % darts.len()],
        )?;
    }
    let sources = vec![EditKey::Edge(source)];
    for index in 0..4 {
        let dart = edit.cell_representative(darts[2 * index], Dim::Zero);
        edit.add_vertex_derived_from(sources.clone(), VertexAttr::new(dart, corners[index]));
        edit.add_edge_derived_from(
            sources.clone(),
            EdgeAttr::new(darts[2 * index], boundary[index].clone()),
        );
    }
    let pcurves = (0..4)
        .map(|index| {
            (
                darts[2 * index],
                TrimmedCurve2::segment(uv[index], uv[(index + 1) % 4]),
            )
        })
        .collect::<HashMap<_, _>>();
    edit.add_profile_derived_from(sources.clone(), ProfileAttr::new(darts[0]));
    let key = edit.add_face_derived_from(
        sources,
        FaceAttr::with_pcurves(surface, darts[0], Vec::new(), pcurves),
    );
    Ok(WallFace {
        key,
        first_section: darts[0],
        last_section: darts[5],
        start_rail: Some(darts[7]),
        end_rail: Some(darts[2]),
    })
}

fn sew_loop_rails<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    faces: &[Option<WallFace>],
) -> Result<(), SweepError> {
    for index in 0..faces.len() {
        let current = faces[index].as_ref().and_then(|face| face.end_rail);
        let next = faces[(index + 1) % faces.len()]
            .as_ref()
            .and_then(|face| face.start_rail);
        if let (Some(current), Some(next)) = (current, next) {
            sew_edges(edit, current, next)?;
        }
    }
    Ok(())
}

fn sew_cap<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    cap: FaceKey,
    loops: &[Vec<Option<WallFace>>],
    end: bool,
) -> Result<(), SweepError> {
    for faces in loops {
        for wall in faces.iter().flatten() {
            let side = if end {
                wall.last_section
            } else {
                wall.first_section
            };
            let start = edit
                .point_at_dart(side)
                .ok_or(SweepError::MissingEdge { dart: side })?;
            let finish = edit
                .point_at_dart(edit.alpha(Dim::Zero, side))
                .ok_or(SweepError::MissingEdge { dart: side })?;
            let cap_dart = cap_dart_between(edit, cap, start, finish)
                .ok_or(SweepError::MissingEdge { dart: side })?;
            sew_edges(edit, cap_dart, side)?;
        }
    }
    Ok(())
}

/// Returns the cap dart running between the two endpoints in that order.
fn cap_dart_between<P: Payload>(
    model: &Model<P>,
    cap: FaceKey,
    start: Point3,
    end: Point3,
) -> Option<Dart> {
    Face::new(model, cap)
        .loops()
        .into_iter()
        .find_map(|boundary| {
            boundary.darts().find_map(|dart| {
                let here = model.point_at_dart(dart)?;
                let partner = model.alpha(Dim::Zero, dart);
                let there = model.point_at_dart(partner)?;
                if here.coincides(start, LINEAR_TOLERANCE) && there.coincides(end, LINEAR_TOLERANCE)
                {
                    return Some(dart);
                }
                (here.coincides(end, LINEAR_TOLERANCE) && there.coincides(start, LINEAR_TOLERANCE))
                    .then_some(partner)
            })
        })
}

fn sew_edges<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    survivor: Dart,
    removed: Dart,
) -> Result<(), SweepError> {
    let edge_at = |model: &Model<P>, dart: Dart| {
        Edge::from_dart(model, dart)
            .map(|edge| edge.key())
            .ok_or(SweepError::MissingEdge { dart })
    };
    let survivor_edge = edge_at(edit, survivor)?;
    let removed_edge = edge_at(edit, removed)?;
    let pairs = [
        (survivor, removed),
        (
            edit.alpha(Dim::Zero, survivor),
            edit.alpha(Dim::Zero, removed),
        ),
    ]
    .into_iter()
    .filter_map(|(keep, drop)| {
        Some((
            Vertex::from_dart(edit, keep)?.key(),
            Vertex::from_dart(edit, drop)?.key(),
        ))
    })
    .collect::<Vec<_>>();
    let closes = [survivor, removed]
        .into_iter()
        .any(|dart| edge_owned_closure(edit, dart).is_some());
    edit.sew(Dim::Two, survivor, removed)
        .map_err(|_| SweepError::SewFailed {
            dim: Dim::Two,
            first: survivor,
            second: removed,
        })?;
    if survivor_edge != removed_edge {
        edit.merge_edges_into(survivor_edge, removed_edge);
        if closes {
            edit.disown_cell(Dim::Zero, survivor);
            edit.own_cell(Dim::Zero, survivor, EntityOwner::Edge(survivor_edge));
        }
    }
    let mut merged = HashSet::<VertexKey>::new();
    for (keep, drop) in pairs {
        if keep != drop && merged.insert(drop) {
            edit.merge_vertices_into(keep, drop);
        }
    }
    Ok(())
}

fn edge_owned_closure<P: Payload>(edit: &ModelEdit<'_, P>, dart: Dart) -> Option<EdgeKey> {
    if Vertex::from_dart(edit, dart).is_some() {
        return None;
    }
    edit.orbit(dart, edit.orbit_indices(Dim::Zero))
        .find_map(
            |anchor| match edit.embedding().owner_at(Dim::Zero, anchor) {
                Some(EntityOwner::Edge(key)) => Some(key),
                _ => None,
            },
        )
}

fn moved_face<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    motion: Rigid,
) -> Result<FaceKey, SweepError> {
    let (mut moved, dart) = Face::new(edit, face).isolate();
    rigid(&mut moved, &motion);
    let moved_face = *moved.attribute_unchecked::<Cell2>(dart);
    let merged = edit.merge(moved.face_unchecked(moved_face));
    Ok(*edit.attribute_unchecked::<Cell2>(merged))
}

fn orient_caps<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    start: FaceKey,
    end: FaceKey,
    start_tangent: Vector3<f64>,
    end_tangent: Vector3<f64>,
) {
    if Face::new(edit, start)
        .normal_at(0.0, 0.0)
        .dot(&start_tangent)
        > 0.0
    {
        reverse_face_winding_edit(edit, start);
    }
    if Face::new(edit, end).normal_at(0.0, 0.0).dot(&end_tangent) < 0.0 {
        reverse_face_winding_edit(edit, end);
    }
}
