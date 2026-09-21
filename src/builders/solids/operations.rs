//! Solid operations and edit-scoped implementations.

use crate::geometry::TrimmedCurve2;
use crate::geometry::parameter::Fraction;
use crate::model::{Cell0, Cell1, Cell2, MergeTopology, Model};
use crate::model::{OpResult, StaleResult};
use crate::topology::ModelEditError;
use crate::topology::embedding::EntityOwner;
use std::collections::HashMap;

use nalgebra::Vector3;
use thiserror::Error;

use crate::{
    Payload,
    builders::errors::ClosedFaceCellError,
    builders::errors::ExtrudeError,
    builders::faces::{add_circle_edit, add_rectangle_edit, reverse_face_winding_edit},
    builders::scaffold::add_closed_face_cell,
    geometry::{
        ANGULAR_TOLERANCE, Axis2, Curve, Cylinder, Frame, LINEAR_TOLERANCE, Plane, Point2, Point3,
        Rigid, RuledSurface, Sphere, Surface, SurfacePeriodicity, Torus,
    },
    topology::{
        Dart, ModelEdit, SheetAttr, SolidAttr,
        attributes::{EdgeAttr, FaceAttr, LoopDefinition, ProfileAttr},
        edge::Edge,
        edge::Edge as EdgeView,
        edit::EditKey,
        face::Face,
        face::Face as FaceView,
        gmap::Dim,
        shape::{FaceTag, Shape},
        shape_keys::{EdgeKey, FaceKey, SolidKey},
        solid::Solid as SolidView,
    },
};

#[derive(Debug, Error)]
pub enum SphereBuildError {
    #[error("sphere radius must be positive and finite, got {radius}")]
    InvalidRadius { radius: f64 },

    #[error("failed to build the sphere's 2-cell")]
    ClosedFaceCell(#[from] ClosedFaceCellError),

    #[error("failed to commit sphere topology")]
    ModelEdit(#[from] ModelEditError),
}

#[derive(Debug, Error)]
pub enum BlockError {
    #[error("block height must be positive and finite, got {height}")]
    InvalidHeight { height: f64 },

    #[error(transparent)]
    Face(#[from] crate::builders::errors::FaceCreationError),

    #[error(transparent)]
    Extrusion(#[from] ExtrudeError),

    #[error(transparent)]
    ModelEdit(#[from] ModelEditError),
}

#[derive(Debug, Error)]
pub enum CylinderError {
    #[error("cylinder height must be positive and finite, got {height}")]
    InvalidHeight { height: f64 },

    #[error(transparent)]
    Face(#[from] crate::builders::errors::FaceCreationError),

    #[error(transparent)]
    Extrusion(#[from] ExtrudeError),

    #[error(transparent)]
    ModelEdit(#[from] ModelEditError),
}

#[derive(Debug, Error)]
pub enum TorusBuildError {
    #[error("minor radius {minor} must be positive and smaller than major radius {major}")]
    InvalidRadii { major: f64, minor: f64 },

    #[error("failed to build the torus's 2-cell")]
    ClosedFaceCell(#[from] ClosedFaceCellError),

    #[error("failed to commit torus topology")]
    ModelEdit(#[from] ModelEditError),
}

/// One lateral face and the boundary edges created while sweeping one base
/// edge of an extrusion.
#[derive(Debug, Clone, Copy)]
pub struct Lateral {
    /// The base-face edge that produced this lateral.
    pub swept_from: EdgeKey,
    /// The lateral face itself.
    pub face: FaceKey,
    /// The vertical edge at the beginning of this lateral in loop order.
    pub start_edge: EdgeKey,
    /// The vertical edge at the end of this lateral in loop order.
    pub end_edge: EdgeKey,
}

/// Everything learned while extruding a face.
#[derive(Debug)]
pub struct FaceExtrusion {
    /// The solid assembled by the operation.
    pub solid: SolidKey,
    /// The source face used as the start cap.
    pub start_cap: FaceKey,
    /// The translated copy used as the end cap.
    pub end_cap: FaceKey,
    /// Laterals in the base face's outer-loop order, followed by inner loops.
    pub laterals: Vec<Lateral>,
    revision: Option<u64>,
}

/// Borrowed views for a [`FaceExtrusion`] result.
pub struct FaceExtrusionView<'m, P: Payload> {
    pub solid: SolidView<'m, P>,
    pub start_cap: FaceView<'m, P>,
    pub end_cap: FaceView<'m, P>,
    pub laterals: Vec<LateralView<'m, P>>,
}

/// Borrowed views for one [`Lateral`] in an extrusion.
pub struct LateralView<'m, P: Payload> {
    pub swept_from: EdgeView<'m, P>,
    pub face: FaceView<'m, P>,
    pub start_edge: EdgeView<'m, P>,
    pub end_edge: EdgeView<'m, P>,
}

/// Everything learned while building a boundaryless primitive solid.
#[derive(Debug)]
pub struct ClosedSolid {
    pub solid: SolidKey,
    pub faces: Vec<FaceKey>,
    pub seams: Vec<EdgeKey>,
    revision: Option<u64>,
}

/// Borrowed views for a [`ClosedSolid`] result.
pub struct ClosedSolidView<'m, P: Payload> {
    pub solid: SolidView<'m, P>,
    pub faces: Vec<FaceView<'m, P>>,
    pub seams: Vec<EdgeView<'m, P>>,
}

impl OpResult for ClosedSolid {
    fn stamp(&mut self, revision: u64) {
        self.revision = Some(revision);
    }
}

impl ClosedSolid {
    /// Resolves the primitive result against its committed model revision.
    pub fn view<'m, P: Payload>(
        &self,
        model: &'m Model<P>,
    ) -> Result<ClosedSolidView<'m, P>, StaleResult> {
        StaleResult::check(self.revision, model)?;
        Ok(ClosedSolidView {
            solid: model.solid_unchecked(self.solid),
            faces: self
                .faces
                .iter()
                .map(|&key| model.face_unchecked(key))
                .collect(),
            seams: self
                .seams
                .iter()
                .map(|&key| model.edge_unchecked(key))
                .collect(),
        })
    }
}

impl OpResult for FaceExtrusion {
    fn stamp(&mut self, revision: u64) {
        self.revision = Some(revision);
    }
}

impl FaceExtrusion {
    /// Resolves every key in the result against `model` at the result's
    /// committed revision.
    pub fn view<'m, P: Payload>(
        &self,
        model: &'m Model<P>,
    ) -> Result<FaceExtrusionView<'m, P>, StaleResult> {
        StaleResult::check(self.revision, model)?;
        let view = |key| model.face_unchecked(key);
        Ok(FaceExtrusionView {
            solid: model.solid_unchecked(self.solid),
            start_cap: view(self.start_cap),
            end_cap: view(self.end_cap),
            laterals: self
                .laterals
                .iter()
                .map(|lateral| LateralView {
                    swept_from: model.edge_unchecked(lateral.swept_from),
                    face: view(lateral.face),
                    start_edge: model.edge_unchecked(lateral.start_edge),
                    end_edge: model.edge_unchecked(lateral.end_edge),
                })
                .collect(),
        })
    }
}

/// Adds a sphere as one boundaryless face on a spherical support.
///
/// A sphere has no boundary anywhere, so it carries no edge and no vertex: the
/// face covers its whole support, and the bigon underneath it is scaffold the
/// face owns outright. The poles stay parametric singularities of [`Sphere`] —
/// where `normal_at` is degenerate — and are not topology.
///
/// The support's own parameterization already faces outward, so the face is
/// stored unreversed.
///
/// The result exposes the generated solid and face, together with any seam
/// edges synthesized by the operation.
pub fn add_sphere<P: Payload>(
    g: &mut Model<P>,
    frame: Frame,
    radius: f64,
) -> Result<ClosedSolid, SphereBuildError> {
    g.transaction_result(|edit| add_sphere_edit(edit, frame, radius))
}

pub(crate) fn add_sphere_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    frame: Frame,
    radius: f64,
) -> Result<ClosedSolid, SphereBuildError> {
    if !radius.is_finite() || radius <= 0.0 {
        return Err(SphereBuildError::InvalidRadius { radius });
    }
    let surface = Surface::Sphere(Sphere::new(frame, radius));
    let cell = add_closed_face_cell(edit, &surface)?;
    let face = edit.add_face(FaceAttr::closed(surface, cell.anchor(), HashMap::new()));
    cell.own(edit, face);
    let shell = cell.anchor();
    edit.add_sheet(SheetAttr::new(shell));
    Ok(ClosedSolid {
        solid: edit.add_solid_derived_from(vec![EditKey::Face(face)], SolidAttr::new(shell, None)),
        faces: vec![face],
        seams: Vec::new(),
        revision: None,
    })
}

/// Adds a rectangular prism whose base starts on `frame` and rises along its
/// positive z direction.
pub fn add_block<P: Payload>(
    g: &mut Model<P>,
    frame: Frame,
    x_size: f64,
    y_size: f64,
    height: f64,
) -> Result<FaceExtrusion, BlockError> {
    g.transaction_result(|edit| add_block_edit(edit, frame, x_size, y_size, height))
}

pub(crate) fn add_block_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    frame: Frame,
    x_size: f64,
    y_size: f64,
    height: f64,
) -> Result<FaceExtrusion, BlockError> {
    if !height.is_finite() || height <= 0.0 {
        return Err(BlockError::InvalidHeight { height });
    }
    let direction = frame.z_dir.into_inner() * height;
    let face = add_rectangle_edit(edit, Plane::from_frame(frame), x_size, y_size)?;
    Ok(add_extruded_face_edit(edit, face, direction)?)
}

/// Adds a cylindrical solid whose base starts on `frame` and rises along its
/// positive z direction.
pub fn add_cylinder<P: Payload>(
    g: &mut Model<P>,
    frame: Frame,
    radius: f64,
    height: f64,
) -> Result<FaceExtrusion, CylinderError> {
    g.transaction_result(|edit| add_cylinder_edit(edit, frame, radius, height))
}

pub(crate) fn add_cylinder_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    frame: Frame,
    radius: f64,
    height: f64,
) -> Result<FaceExtrusion, CylinderError> {
    if !height.is_finite() || height <= 0.0 {
        return Err(CylinderError::InvalidHeight { height });
    }
    let direction = frame.z_dir.into_inner() * height;
    let face = add_circle_edit(edit, Plane::from_frame(frame), radius)?;
    Ok(add_extruded_face_edit(edit, face, direction)?)
}

/// Adds a torus as one boundaryless face on a [`Torus`] support.
///
/// The frame's z-axis is the revolution axis and its x-axis fixes the meridian
/// the tube's centre circle starts from. The support is closed in both
/// directions — a whole turn in the longitude and in the tube — so the face
/// covers it entirely: no edge and no vertex, over a square the face owns
/// outright, exactly as [`add_sphere`] does over a bigon.
///
/// `minor` must stay under `major`: a tube as wide as its offset reaches the
/// axis, and one wider sweeps through itself.
///
/// The result exposes the generated solid and face, together with any seam
/// edges synthesized by the operation.
pub fn add_torus<P: Payload>(
    g: &mut Model<P>,
    frame: Frame,
    major: f64,
    minor: f64,
) -> Result<ClosedSolid, TorusBuildError> {
    g.transaction_result(|edit| add_torus_edit(edit, frame, major, minor))
}

pub(crate) fn add_torus_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    frame: Frame,
    major: f64,
    minor: f64,
) -> Result<ClosedSolid, TorusBuildError> {
    if !major.is_finite() || !minor.is_finite() || minor <= 0.0 || minor >= major {
        return Err(TorusBuildError::InvalidRadii { major, minor });
    }
    let surface = Surface::Torus(Torus::new(frame, major, minor));
    let cell = add_closed_face_cell(edit, &surface)?;
    let face = edit.add_face(FaceAttr::closed(surface, cell.anchor(), HashMap::new()));
    cell.own(edit, face);
    let shell = cell.anchor();
    edit.add_sheet(SheetAttr::new(shell));
    Ok(ClosedSolid {
        solid: edit.add_solid_derived_from(vec![EditKey::Face(face)], SolidAttr::new(shell, None)),
        faces: vec![face],
        seams: Vec::new(),
        revision: None,
    })
}

/// Returns an isolated copy of `face` translated by `direction`.
///
/// Vertex positions, edge curves, and the supporting surface are translated;
/// face pcurves remain unchanged because a rigid motion preserves every
/// parameterisation. The source map is not modified.
///
/// Returns an error for a zero direction.
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
    let motion = Rigid::translation(direction);
    translated.transaction(|edit| {
        for key in vertex_keys {
            edit.vertex_attr_mut_unchecked(key).point += direction;
        }

        for key in edge_keys {
            let edge = edit.edge_attr_mut_unchecked(key);
            edge.curve = edge.curve.moved(&motion);
        }

        let translated_face = edit.face_attr_mut_unchecked(translated_face_key);
        translated_face.surface = translated_face.surface.moved(&motion);
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
    g: &mut Model<P>,
    face_key: FaceKey,
    direction: Vector3<f64>,
) -> Result<FaceExtrusion, ExtrudeError> {
    g.transaction_result(|edit| add_extruded_face_edit(edit, face_key, direction))
}

/// Builds translated caps and lateral faces, then registers the staged solid.
pub(crate) fn add_extruded_face_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face_key: FaceKey,
    direction: Vector3<f64>,
) -> Result<FaceExtrusion, ExtrudeError> {
    let bot_face = edit
        .face_attr(face_key)
        .map(|attr| attr.face(edit))
        .ok_or(ExtrudeError::MissingFace { dart: face_key })?;
    let top_face = translate_face(&bot_face, direction)?;
    let bottom_loop_darts = bot_face
        .loops()
        .into_iter()
        .map(|loop_| loop_.dart())
        .collect::<Vec<_>>();

    let top_face_dart = edit.merge(top_face.face());
    let top_face_key = *edit.attribute_unchecked::<Cell2>(top_face_dart);
    let top_face_attr = edit.face_attr_unchecked(top_face_key);
    let mut top_loop_darts = Vec::with_capacity(1 + top_face_attr.inner().count());
    top_loop_darts.push(top_face_attr.outer_unchecked());
    top_loop_darts.extend(top_face_attr.inner());

    let laterals_face_outward = orient_extruded_caps(edit, face_key, top_face_key, direction);

    let mut laterals = Vec::new();
    for (bottom_loop_dart, top_loop_dart) in bottom_loop_darts.into_iter().zip(top_loop_darts) {
        let (_, loop_laterals) = sew_extruded_loop(
            edit,
            (face_key, bottom_loop_dart),
            (top_face_key, top_loop_dart),
            direction,
        )?;
        laterals.extend(loop_laterals);
    }
    if !laterals_face_outward {
        for lateral in &laterals {
            reverse_face_winding_edit(edit, lateral.face);
        }
    }

    // The shell dart is contextual: unlike a cell representative, it must retain
    // the outward orientation established for the bottom cap.
    let outer_shell = edit.face_attr_unchecked(face_key).outer_unchecked();
    if edit.sheet_key(outer_shell).is_none() {
        edit.add_sheet_derived_from(vec![EditKey::Face(face_key)], SheetAttr::new(outer_shell));
    }
    let solid = edit.add_solid_derived_from(
        vec![EditKey::Face(face_key)],
        SolidAttr::new(outer_shell, None),
    );
    Ok(FaceExtrusion {
        solid,
        start_cap: face_key,
        end_cap: top_face_key,
        laterals,
        revision: None,
    })
}

/// Turns the two caps so they face out of the extrusion, and reports whether
/// the laterals will too.
///
/// The bottom cap must face against the run and the top with it, and exactly
/// one of them is ever wrong, because the top is a translated copy of the
/// bottom: turning whichever it is settles both.
///
/// The laterals are a third question, and this is where its answer is known.
/// A lateral's winding follows the cap loop as the *source* face stores it,
/// read before either cap is turned, so it is decided by the source's own
/// winding and by nothing the caps do afterwards. A face wound with the run —
/// the case that turns the bottom cap — grows laterals that already face out;
/// one wound against it grows them all facing in. They share the one sign, so
/// the caller turns all of them or none.
fn orient_extruded_caps<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    bottom_face: FaceKey,
    top_face: FaceKey,
    direction: Vector3<f64>,
) -> bool {
    let Some(bottom_normal_dot_direction) = edit
        .face_attr(bottom_face)
        .map(|attr| face_normal_dot_direction(edit, attr, direction))
    else {
        return true;
    };

    if bottom_normal_dot_direction > LINEAR_TOLERANCE {
        reverse_face_winding_edit(edit, bottom_face);
    } else if bottom_normal_dot_direction < -LINEAR_TOLERANCE {
        reverse_face_winding_edit(edit, top_face);
    }
    bottom_normal_dot_direction > 0.0
}

fn face_normal_dot_direction<P: Payload>(
    g: &Model<P>,
    face: &FaceAttr<P::F>,
    direction: Vector3<f64>,
) -> f64 {
    face.face(g).normal_at(0.0, 0.0).dot(&direction)
}

/// Returns the logical edges of the cap loop reached through `start`, in that
/// dart's direction.
///
/// The raw profile chain cannot answer this. A cap with a hole owns a cut
/// joining its two loops, so one alpha0/alpha1 chain carries both: walking it
/// leaves this loop part way round and continues into the other one. The face's
/// boundary walk turns across the cut instead, which is what keeps each loop a
/// list of its own.
///
/// # Panics
///
/// Panics when `start` is not a boundary dart of `face`, which no loop dart the
/// caller read off that same face is.
fn cap_loop_edges<P: Payload>(g: &Model<P>, face: FaceKey, start: Dart) -> Vec<Dart> {
    Face::new(g, face)
        .loops()
        .into_iter()
        .find_map(|boundary| Some(boundary.starting_at(start)?.darts().collect()))
        .expect("a cap loop dart lies on a boundary of its cap")
}

fn sew_extruded_loop<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    bottom: (FaceKey, Dart),
    top: (FaceKey, Dart),
    direction: Vector3<f64>,
) -> Result<(Dart, Vec<Lateral>), ExtrudeError> {
    let (bottom_face, bottom_loop_dart) = bottom;
    let (top_face, top_loop_dart) = top;
    let bottom_edges = cap_loop_edges(edit, bottom_face, bottom_loop_dart);
    let top_edges = cap_loop_edges(edit, top_face, top_loop_dart);
    if let Some((representative, face)) =
        sew_wrapping_lateral_face(edit, &bottom_edges, &top_edges, direction)?
    {
        let swept_from = edit
            .cell_key::<Cell1>(*bottom_edges.first().expect("one bottom edge"))
            .expect("wrapping lateral must have a source edge");
        let start_edge = swept_from;
        let end_edge = edit
            .cell_key::<Cell1>(*top_edges.first().expect("one top edge"))
            .expect("wrapping lateral must have a copied edge");
        return Ok((
            edit.cell_representative(representative, Dim::Three),
            vec![Lateral {
                swept_from,
                face,
                start_edge,
                end_edge,
            }],
        ));
    }

    let laterals = bottom_edges
        .iter()
        .copied()
        .zip(top_edges.iter().copied())
        .map(|(bottom_edge, top_edge)| {
            let prepared = prepare_lateral_face(edit, bottom_edge, top_edge, direction)?;
            let topology = add_lateral_face_topology(edit)?;
            let swept_from = edit
                .cell_key::<Cell1>(bottom_edge)
                .expect("lateral must have a source edge");
            let face = add_lateral_face_attributes(edit, &topology, &prepared, swept_from);
            Ok(ExtrudedFaceLateral {
                topology,
                face,
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

    let vertical_edges = laterals
        .iter()
        .map(|lateral| {
            edit.add_edge_derived_from(
                vec![EditKey::Vertex(
                    edit.cell_key::<Cell0>(lateral.topology.end_vertical)
                        .expect("vertical edge must meet a swept vertex"),
                )],
                EdgeAttr::new(
                    lateral.topology.end_vertical,
                    Curve::line(lateral.vertical_start, lateral.vertical_end),
                ),
            )
        })
        .collect::<Vec<_>>();
    let representative = laterals
        .first()
        .map(|lateral| lateral.topology.bottom_edge)
        .expect("a loop should have at least one lateral face");
    let result = laterals
        .iter()
        .enumerate()
        .map(|(index, lateral)| Lateral {
            swept_from: edit.cell_key_unchecked::<Cell1>(bottom_edges[index]),
            face: lateral.face,
            start_edge: vertical_edges[(index + vertical_edges.len() - 1) % vertical_edges.len()],
            end_edge: vertical_edges[index],
        })
        .collect();
    Ok((edit.cell_representative(representative, Dim::Three), result))
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
    edit: &mut ModelEdit<'_, P>,
    bottom_edges: &[Dart],
    top_edges: &[Dart],
    direction: Vector3<f64>,
) -> Result<Option<(Dart, FaceKey)>, ExtrudeError> {
    let ([bottom_edge], [top_edge]) = (bottom_edges, top_edges) else {
        return Ok(None);
    };
    let prepared = prepare_lateral_face(edit, *bottom_edge, *top_edge, direction)?;
    let Some(axis) = swept_period_axis(&prepared) else {
        return Ok(None);
    };

    // The swept edge at the start of the sweep and its image at the end, joined
    // by a seam the boundary walk uses twice. Without the seam the two loops sit
    // in two 2-cells that nothing connects, and the wall would be one face only
    // because its attribute said so; with it the involutions carry that fact.
    // The seam is scaffold, not shape — a swept wall closes on itself, and the
    // two copies of the source edge it would otherwise carry are the same curve
    // in the same place.
    let slots: [[Dart; 2]; 4] = std::array::from_fn(|_| [edit.add_dart(), edit.add_dart()]);
    for slot in slots {
        edit.link(Dim::Zero, slot[0], slot[1])?;
    }
    for i in 0..slots.len() {
        edit.link(Dim::One, slots[i][1], slots[(i + 1) % slots.len()][0])?;
    }
    let [seam_out, top_slot, seam_back, bottom_slot] = slots;
    edit.link(Dim::Two, seam_out[0], seam_back[1])?;
    edit.link(Dim::Two, seam_out[1], seam_back[0])?;
    let [bottom_start, _bottom_end] = bottom_slot;
    let [top_start, top_end] = top_slot;

    let uv = prepared.uv;
    let source_edge = edit
        .cell_key::<Cell1>(*bottom_edge)
        .expect("wrapping lateral must have a source edge");
    edit.add_profile_derived_from(
        vec![EditKey::Edge(source_edge)],
        ProfileAttr::new(bottom_start),
    );
    edit.add_profile_derived_from(
        vec![EditKey::Edge(source_edge)],
        ProfileAttr::new(top_start),
    );
    let face = edit.add_face_derived_from(
        vec![EditKey::Edge(source_edge)],
        FaceAttr::with_loops(
            prepared.surface.clone(),
            vec![
                LoopDefinition::wrapping(bottom_start, axis),
                LoopDefinition::wrapping(top_start, axis),
            ],
            HashMap::from([
                (bottom_start, TrimmedCurve2::segment(uv[0], uv[1])),
                (top_start, TrimmedCurve2::segment(uv[2], uv[3])),
            ]),
        ),
    );
    edit.own_cell(Dim::One, seam_out[0], EntityOwner::Face(face));

    // The swept loop runs with the sweep at the bottom and against it at the
    // top, so the top loop meets its cap through `alpha0`, as the quad path's
    // top edge does.
    sew(edit, Dim::Two, bottom_start, *bottom_edge)?;
    sew(edit, Dim::Two, top_end, *top_edge)?;
    Ok(Some((bottom_start, face)))
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
    edit: &mut ModelEdit<'_, P>,
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
    face: FaceKey,
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
    g: &Model<P>,
    bottom_edge: Dart,
    _top_edge: Dart,
    direction: Vector3<f64>,
) -> Result<PreparedLateralFace, ExtrudeError> {
    let bottom_edge_view = Edge::from_dart(g, bottom_edge)
        .ok_or(ExtrudeError::MissingEdgeCurve { dart: bottom_edge })?;
    let edge_dart = bottom_edge_view.dart();
    // The ends of the section being swept: for a closed edge they are the same
    // point, which is what makes the swept wall wrap.
    let section = bottom_edge_view.trimmed_curve();
    let (start, end) = (
        section.point_at(Fraction::new(0.0)),
        section.point_at(Fraction::new(1.0)),
    );
    let curve = bottom_edge_view.curve();
    let surface = lateral_face_surface(edge_dart, curve, start, end, direction)?;
    let uv = lateral_face_uv(&surface, curve, start, end, direction);

    Ok(PreparedLateralFace { end, uv, surface })
}

fn add_lateral_face_topology<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
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
    edit: &mut ModelEdit<'_, P>,
    topology: &LateralFaceTopology,
    prepared: &PreparedLateralFace,
    swept_from: EdgeKey,
) -> FaceKey {
    edit.add_profile_derived_from(
        vec![EditKey::Edge(swept_from)],
        ProfileAttr::new(topology.loop_dart),
    );
    edit.add_face_derived_from(
        vec![EditKey::Edge(swept_from)],
        FaceAttr::with_pcurves(
            prepared.surface.clone(),
            topology.loop_dart,
            Vec::new(),
            quad_pcurves(&prepared.uv, &topology.darts),
        ),
    )
}

pub(crate) fn lateral_face_surface(
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

pub(crate) fn lateral_face_uv(
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
                Point2::new(interval.start.value(), 0.0),
                Point2::new(interval.end.value(), 0.0),
                Point2::new(interval.end.value(), 1.0),
                Point2::new(interval.start.value(), 1.0),
            ]
        }
        // The cylinder was built on the circle's own frame, so `u` is the
        // circle's parameter unchanged; `v` is a signed height along the axis
        // rather than the ruled surface's normalized sweep fraction.
        Surface::Cylinder(cylinder) => {
            let interval = curve.interval_between(start, end);
            let height = direction.dot(&cylinder.axis());
            [
                Point2::new(interval.start.value(), 0.0),
                Point2::new(interval.end.value(), 0.0),
                Point2::new(interval.end.value(), height),
                Point2::new(interval.start.value(), height),
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

#[cfg(test)]
mod tests {
    use std::collections::HashSet;

    use nalgebra::Vector3;

    use super::add_extruded_face_edit;
    use crate::builders::faces::{add_circle, add_face, add_polygon};
    use crate::builders::test_support::LineageRecorder;
    use crate::geometry::{Plane, Point3};
    use crate::model::Model;
    use crate::topology::edit::{EditKey, Origin};
    use crate::topology::payload::StandardPayload;

    #[test]
    fn rectangle_extrusion_declares_swept_and_assembled_lineage() {
        let mut model = Model::<StandardPayload>::new();
        let profile = add_polygon(
            &mut model,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
                Point3::new(2.0, 2.0, 0.0),
                Point3::new(0.0, 2.0, 0.0),
            ],
        );
        let face = add_face(&mut model, profile).expect("face should build");
        let mut recorder = LineageRecorder::default();

        let result = model
            .transaction_with_policy(&mut recorder, |edit| {
                add_extruded_face_edit(edit, face, Vector3::z())
            })
            .expect("extrusion should commit");

        let mut expected = result
            .laterals
            .iter()
            .map(|lateral| {
                (
                    EditKey::Face(lateral.face),
                    Origin::derived(EditKey::Edge(lateral.swept_from)),
                )
            })
            .chain(std::iter::once((
                EditKey::Solid(result.solid),
                Origin::derived(EditKey::Face(face)),
            )))
            .collect::<Vec<_>>();

        let lateral_edges = result
            .laterals
            .iter()
            .flat_map(|lateral| [lateral.start_edge, lateral.end_edge])
            .collect::<HashSet<_>>();
        for edge in lateral_edges {
            let source = model
                .cell_key::<crate::model::Cell0>(model.edge_attr_unchecked(edge).dart)
                .expect("vertical edge should meet its source vertex");
            expected.push((
                EditKey::Edge(edge),
                Origin::derived(EditKey::Vertex(source)),
            ));
        }

        // A lateral's boundary loop comes from the edge that swept it, and the
        // shell from the cap it closes — neither is something the extrusion
        // invented, so neither is `New`.
        for lateral in &result.laterals {
            let loop_dart = model.face_attr_unchecked(lateral.face).outer_unchecked();
            let profile = model
                .profile_key(loop_dart)
                .expect("a lateral's boundary loop is a registered profile");
            expected.push((
                EditKey::Profile(profile),
                Origin::derived(EditKey::Edge(lateral.swept_from)),
            ));
        }
        expected.extend(
            recorder
                .created
                .iter()
                .filter(|(key, _)| matches!(key, EditKey::Sheet(_)))
                .map(|(key, _)| (*key, Origin::derived(EditKey::Face(face)))),
        );
        recorder.assert_exact(&expected);
    }

    #[test]
    fn circle_extrusion_reports_its_wrapping_lateral_face() {
        let mut model = Model::<StandardPayload>::new();
        let face = add_circle(&mut model, Plane::xy(), 1.0).expect("circle face should build");
        let mut recorder = LineageRecorder::default();

        let result = model
            .transaction_with_policy(&mut recorder, |edit| {
                add_extruded_face_edit(edit, face, Vector3::z())
            })
            .expect("circle extrusion should commit");

        assert_eq!(result.laterals.len(), 1);
        let lateral = result.laterals[0];
        assert_eq!(
            lateral.swept_from,
            model.face_unchecked(face).edges()[0].key()
        );
        assert!(recorder.created.contains(&(
            EditKey::Face(lateral.face),
            Origin::derived(EditKey::Edge(lateral.swept_from)),
        )));
    }
}
