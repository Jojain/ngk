//! Builds a [`Surgery`] into the model: cut, insert, fill, sew, register.
//!
//! The executor computes no geometry. Every point, curve and pcurve arrives in
//! the surgery; what happens here is the map: selected edges are unsewn,
//! joints are spliced into the corners they close, new faces are laid down and
//! sewn to the darts the plan names — by id, never by comparing coordinates —
//! and each piece of topology is registered with the lineage the plan gave it.
//!
//! No dart is removed, so nothing that points at the map needs repairing: the
//! surgery only ever adds darts and changes links.
//!
//! A band — the blend of a closed edge — is laid down as two one-edge loops,
//! each sewn to one face's side of the cut, and joined by a scaffold cut so
//! it occupies one 2-cell, as a lofted band is. Each of its rails is closed
//! with no corner, so the 0-cell where it closes is interior to it.

use std::collections::{BTreeSet, HashMap, HashSet};

use super::errors::BlendError;
use super::pcurve::onto_branch;
use super::surgery::{BoundKind, CornerId, NewBoundary, RailEnds, Surgery};
use crate::builders::profiles::curve_pcurve;
use crate::builders::scaffold::cut_between_loops;
use crate::geometry::parameter::Fraction;
use crate::geometry::{
    Axis2, IntersectionError, Interval, LINEAR_TOLERANCE, Point3, Surface, TrimmedCurve2,
};
use crate::model::{Cell1, Cell2};
use crate::topology::ModelEdit;
use crate::topology::attributes::{EdgeAttr, FaceAttr, LoopDefinition, ProfileAttr, VertexAttr};
use crate::topology::edge::Edge;
use crate::topology::edit::EditKey;
use crate::topology::embedding::EntityOwner;
use crate::topology::gmap::{Dart, Dim};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey, SolidKey};

/// What building a surgery added and disturbed.
#[derive(Debug, Default)]
pub(crate) struct Executed {
    /// The faces the surgery added, in surgery order.
    pub(crate) faces: Vec<FaceKey>,
    /// Existing faces whose boundary moved.
    pub(crate) changed_faces: Vec<FaceKey>,
    /// Solids holding anything the surgery touched.
    pub(crate) solids: Vec<SolidKey>,
}

/// Which way each existing face runs along what the surgery changes in it.
struct Directions {
    /// Per cut and side: the face's directed dart on the side.
    sides: Vec<[Dart; 2]>,
    /// Per joint inserted into a face: whether the face runs with the joint.
    insertions: HashMap<usize, bool>,
}

/// Builds `surgery` into the model.
pub(crate) fn execute<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    surgery: &Surgery,
) -> Result<Executed, BlendError> {
    let directions = read_directions(edit, surgery)?;
    let consumed_darts = surgery
        .consumed_vertices
        .iter()
        .filter_map(|&vertex| edit.vertex_attr(vertex).map(|attr| attr.dart))
        .flat_map(|dart| {
            edit.orbit(dart, edit.orbit_indices(Dim::Zero))
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let mut changed_faces = BTreeSet::new();
    let mut solids = BTreeSet::new();
    for cut in &surgery.cuts {
        for side in &cut.sides {
            changed_faces.insert(side.face);
            solids.extend(edit.solid_key(side.start));
        }
    }
    for joint in &surgery.joints {
        if let Some(insertion) = &joint.insertion {
            changed_faces.extend(insertion.face);
            solids.extend(edit.solid_key(insertion.after));
        }
    }

    for cut in &surgery.cuts {
        edit.remove_edge(cut.edge);
    }
    for &vertex in &surgery.consumed_vertices {
        edit.remove_vertex(vertex);
    }

    // Cut: each selected edge's two faces let go of each other.
    for cut in &surgery.cuts {
        let [first, second] = &cut.sides;
        if edit.alpha(Dim::Two, first.start) != second.start {
            return Err(BlendError::InconsistentSurgery {
                reason: "a cut's two sides are not sewn to each other",
            });
        }
        let end = edit.alpha(Dim::Zero, first.start);
        edit.unlink(Dim::Two, first.start)?;
        edit.unlink(Dim::Two, end)?;
    }

    let mut corner_darts = vec![Vec::new(); surgery.corners.len()];
    for cut in &surgery.cuts {
        for side in &cut.sides {
            if let RailEnds::Corners(corners) = side.ends {
                corner_darts[corners[0]].push(side.start);
                corner_darts[corners[1]].push(edit.alpha(Dim::Zero, side.start));
            }
        }
    }

    // Insert: every joint closing an existing corner is spliced in after the
    // corner's incoming edge, whatever follows it, so a bridge attached at
    // the corner stays attached to the corner's far side.
    let mut joint_uses = vec![Vec::<[Dart; 2]>::new(); surgery.joints.len()];
    let mut inserted_directed = HashMap::new();
    for (id, joint) in surgery.joints.iter().enumerate() {
        let Some(insertion) = &joint.insertion else {
            continue;
        };
        if edit.is_free(insertion.after, Dim::One) {
            return Err(BlendError::InconsistentSurgery {
                reason: "an insertion names a corner that is not closed",
            });
        }
        let next = edit.alpha(Dim::One, insertion.after);
        edit.unlink(Dim::One, insertion.after)?;
        let first = edit.add_dart();
        let second = edit.add_dart();
        edit.link(Dim::Zero, first, second)?;
        edit.link(Dim::One, insertion.after, first)?;
        edit.link(Dim::One, second, next)?;
        joint_uses[id].push([first, second]);
        corner_darts[joint.corners[0]].push(first);
        corner_darts[joint.corners[1]].push(second);
        if insertion.face.is_some() {
            let forward = directions.insertions[&id];
            inserted_directed.insert(id, if forward { first } else { second });
        }
    }

    // Fill: lay each new face's walk down, then sew it to what it borders.
    // A walk links each bound to the next; a band's two rails each close on
    // themselves, and the cut joining them comes once the face is registered.
    let mut walks = Vec::with_capacity(surgery.faces.len());
    for face in &surgery.faces {
        let bounds = face.boundary.bounds();
        let walk = bounds
            .iter()
            .map(|_| {
                let start = edit.add_dart();
                let end = edit.add_dart();
                edit.link(Dim::Zero, start, end).map(|()| [start, end])
            })
            .collect::<Result<Vec<_>, _>>()?;
        match face.boundary {
            NewBoundary::Walk(_) => {
                for index in 0..walk.len() {
                    edit.link(Dim::One, walk[index][1], walk[(index + 1) % walk.len()][0])?;
                }
            }
            NewBoundary::Band(_) => {
                for [start, end] in &walk {
                    edit.link(Dim::One, *start, *end)?;
                }
            }
        }
        for (bound, darts) in bounds.iter().zip(&walk) {
            let own = if bound.reversed {
                [darts[1], darts[0]]
            } else {
                *darts
            };
            if let Some(corners) = surgery.bound_corners(bound.kind) {
                corner_darts[corners[0]].push(own[0]);
                corner_darts[corners[1]].push(own[1]);
            }
            match bound.kind {
                BoundKind::Rail { cut, side } => {
                    edit.sew(Dim::Two, own[0], surgery.cuts[cut].sides[side].start)?;
                }
                BoundKind::Joint(id) => joint_uses[id].push(own),
            }
        }
        walks.push(walk);
    }
    for (id, uses) in joint_uses.iter().enumerate() {
        match uses.as_slice() {
            [_] if surgery.joints[id].insertion.is_some() => {}
            [first, second] => edit.sew(Dim::Two, first[0], second[0])?,
            _ => {
                return Err(BlendError::InconsistentSurgery {
                    reason: "a joint is not bounded by one or two faces",
                });
            }
        }
    }

    let reversed = orient_faces(edit, surgery, &walks, &directions, &inserted_directed)?;

    // Register: rails and joints first, so every new edge has its key before
    // the surviving edges are told apart from the fresh ones.
    let mut fresh_edges = HashSet::new();
    for cut in &surgery.cuts {
        for side in &cut.sides {
            let rail = edit.add_edge_derived_from(
                vec![EditKey::Edge(cut.edge)],
                EdgeAttr::new(side.start, side.rail.curve().clone()),
            );
            // Nothing meets where a closed rail closes, so that 0-cell is
            // interior to the rail rather than a corner of the shape.
            if side.ends == RailEnds::Closed {
                edit.own_cell(Dim::Zero, side.start, EntityOwner::Edge(rail));
            }
            fresh_edges.insert(rail);
        }
    }
    for (id, joint) in surgery.joints.iter().enumerate() {
        let dart = joint_uses[id][0][0];
        fresh_edges.insert(edit.add_edge_derived_from(
            joint.sources.clone(),
            EdgeAttr::new(dart, joint.curve.curve().clone()),
        ));
    }

    let mut faces = Vec::with_capacity(surgery.faces.len());
    for ((face, walk), &flip) in surgery.faces.iter().zip(&walks).zip(&reversed) {
        let directed = |index: usize| walk[index][usize::from(flip)];
        let pcurves = face
            .boundary
            .bounds()
            .iter()
            .enumerate()
            .map(|(index, bound)| {
                let pcurve = if flip {
                    bound.pcurve.reversed()
                } else {
                    bound.pcurve.clone()
                };
                (directed(index), pcurve)
            })
            .collect();
        let key = match face.boundary {
            NewBoundary::Walk(_) => {
                let seed = directed(0);
                edit.add_profile_derived_from(face.sources.clone(), ProfileAttr::new(seed));
                edit.add_face_derived_from(
                    face.sources.clone(),
                    FaceAttr::with_pcurves(face.surface.clone(), seed, Vec::new(), pcurves),
                )
            }
            NewBoundary::Band(_) => {
                let seeds = [directed(0), directed(1)];
                for seed in seeds {
                    edit.add_profile_derived_from(face.sources.clone(), ProfileAttr::new(seed));
                }
                let key = edit.add_face_derived_from(
                    face.sources.clone(),
                    FaceAttr::with_loops(
                        face.surface.clone(),
                        seeds
                            .iter()
                            .map(|&seed| LoopDefinition::wrapping(seed, Axis2::U))
                            .collect(),
                        pcurves,
                    ),
                );
                cut_between_loops(edit, key, seeds[0], seeds[1])?;
                key
            }
        };
        faces.push(key);
    }

    let corner_cells = register_corners(edit, surgery, &corner_darts)?;
    if consumed_darts
        .iter()
        .any(|&dart| !corner_cells.contains_key(&edit.cell_representative(dart, Dim::Zero)))
    {
        return Err(BlendError::InconsistentSurgery {
            reason: "a piece of a consumed vertex was left without a corner",
        });
    }

    // Pcurves of existing faces: rails and inserted joints are written as
    // planned, in each face's own direction.
    for (cut, sides) in surgery.cuts.iter().zip(&directions.sides) {
        for (side, &directed) in cut.sides.iter().zip(sides) {
            let pcurve = if directed == side.start {
                side.pcurve.clone()
            } else {
                side.pcurve.reversed()
            };
            write_pcurve(edit, side.face, directed, pcurve);
        }
    }
    for (id, joint) in surgery.joints.iter().enumerate() {
        let Some(insertion) = &joint.insertion else {
            continue;
        };
        let (Some(face), Some(pcurve)) = (insertion.face, &insertion.pcurve) else {
            continue;
        };
        let directed = inserted_directed[&id];
        let pcurve = if directed == joint_uses[id][0][0] {
            pcurve.clone()
        } else {
            pcurve.reversed()
        };
        write_pcurve(edit, face, directed, pcurve);
    }

    // Every surviving edge with an end at a moved corner keeps its support
    // and its key, but its pcurves were written for the old span.
    let mut retrim = BTreeSet::new();
    for &cell in corner_cells.keys() {
        for dart in edit
            .orbit(cell, edit.orbit_indices(Dim::Zero))
            .collect::<Vec<_>>()
        {
            if let Some(edge) = edit.cell_key::<Cell1>(dart)
                && !fresh_edges.contains(&edge)
            {
                retrim.insert(edge);
            }
        }
    }
    for edge in retrim {
        for (face, directed) in edge_face_darts(edit, edge) {
            let pcurve = retrimmed_pcurve(edit, edge, face, directed)?;
            write_pcurve(edit, face, directed, pcurve);
            changed_faces.insert(face);
        }
    }

    Ok(Executed {
        faces,
        changed_faces: changed_faces.into_iter().collect(),
        solids: solids.into_iter().collect(),
    })
}

/// Reads, before anything changes, which way each face runs along its cut
/// sides and inserted corners.
fn read_directions<P: Payload>(
    edit: &ModelEdit<'_, P>,
    surgery: &Surgery,
) -> Result<Directions, BlendError> {
    let mut loops = HashMap::<FaceKey, HashSet<Dart>>::new();
    let mut walked = |face: FaceKey| -> HashSet<Dart> {
        loops
            .entry(face)
            .or_insert_with(|| {
                edit.face_unchecked(face)
                    .loops()
                    .iter()
                    .flat_map(|loop_| loop_.darts())
                    .collect()
            })
            .clone()
    };

    let mut sides = Vec::with_capacity(surgery.cuts.len());
    for cut in &surgery.cuts {
        let mut directed = [cut.sides[0].start; 2];
        for (slot, side) in directed.iter_mut().zip(&cut.sides) {
            let darts = walked(side.face);
            let end = edit.alpha(Dim::Zero, side.start);
            *slot = if darts.contains(&side.start) {
                side.start
            } else if darts.contains(&end) {
                end
            } else {
                return Err(BlendError::InconsistentSurgery {
                    reason: "a cut side is not on its face's boundary",
                });
            };
        }
        sides.push(directed);
    }

    let mut insertions = HashMap::new();
    for (id, joint) in surgery.joints.iter().enumerate() {
        let Some(insertion) = &joint.insertion else {
            continue;
        };
        let Some(face) = insertion.face else {
            continue;
        };
        let darts = walked(face);
        let forward = if darts.contains(&edit.alpha(Dim::Zero, insertion.after)) {
            true
        } else if darts.contains(&insertion.after) {
            false
        } else {
            return Err(BlendError::InconsistentSurgery {
                reason: "an insertion's corner is not on its face's boundary",
            });
        };
        insertions.insert(id, forward);
    }
    Ok(Directions { sides, insertions })
}

/// Chooses, for every new face, whether its walk runs against its natural
/// order, so that it crosses every shared edge opposite to its neighbour.
///
/// Existing faces anchor the answer; a new face bordered only by new faces —
/// a ball's patch — is decided from them once they are.
fn orient_faces<P: Payload>(
    edit: &ModelEdit<'_, P>,
    surgery: &Surgery,
    walks: &[Vec<[Dart; 2]>],
    directions: &Directions,
    inserted_directed: &HashMap<usize, Dart>,
) -> Result<Vec<bool>, BlendError> {
    let mut joint_faces = HashMap::<usize, Vec<(usize, usize)>>::new();
    for (face_index, face) in surgery.faces.iter().enumerate() {
        for (bound_index, bound) in face.boundary.bounds().iter().enumerate() {
            if let BoundKind::Joint(id) = bound.kind {
                joint_faces
                    .entry(id)
                    .or_default()
                    .push((face_index, bound_index));
            }
        }
    }

    let mut reversed: Vec<Option<bool>> = vec![None; surgery.faces.len()];
    let neighbour_directed = |reversed: &[Option<bool>], face: usize, bound: usize| match surgery
        .faces[face]
        .boundary
        .bounds()[bound]
        .kind
    {
        BoundKind::Rail { cut, side } => Some(directions.sides[cut][side]),
        BoundKind::Joint(id) => inserted_directed.get(&id).copied().or_else(|| {
            joint_faces
                .get(&id)?
                .iter()
                .find_map(|&(other, other_bound)| {
                    (other != face)
                        .then(|| reversed[other])
                        .flatten()
                        .map(|flip| walks[other][other_bound][usize::from(flip)])
                })
        }),
    };
    let flip_against = |face: usize, bound: usize, neighbour: Dart| {
        let [start, end] = walks[face][bound];
        if neighbour == edit.alpha(Dim::Two, end) {
            Ok(false)
        } else if neighbour == edit.alpha(Dim::Two, start) {
            Ok(true)
        } else {
            Err(BlendError::InconsistentSurgery {
                reason: "a new face is not sewn to the neighbour it names",
            })
        }
    };

    loop {
        let mut progressed = false;
        for face in 0..surgery.faces.len() {
            if reversed[face].is_some() {
                continue;
            }
            for bound in 0..surgery.faces[face].boundary.bounds().len() {
                if let Some(neighbour) = neighbour_directed(&reversed, face, bound) {
                    reversed[face] = Some(flip_against(face, bound, neighbour)?);
                    progressed = true;
                    break;
                }
            }
        }
        if !progressed {
            break;
        }
    }

    let reversed = reversed
        .into_iter()
        .map(|flip| {
            flip.ok_or(BlendError::InconsistentSurgery {
                reason: "a new face borders nothing already oriented",
            })
        })
        .collect::<Result<Vec<_>, _>>()?;
    let decided = reversed.iter().copied().map(Some).collect::<Vec<_>>();
    for (face, &flip) in reversed.iter().enumerate() {
        for bound in 0..surgery.faces[face].boundary.bounds().len() {
            if let Some(neighbour) = neighbour_directed(&decided, face, bound)
                && flip_against(face, bound, neighbour)? != flip
            {
                return Err(BlendError::InconsistentSurgery {
                    reason: "a new face cannot agree with all of its neighbours",
                });
            }
        }
    }
    Ok(reversed)
}

/// Registers one vertex per planned corner, checking the map agrees.
///
/// Returns each corner's 0-cell representative.
fn register_corners<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    surgery: &Surgery,
    corner_darts: &[Vec<Dart>],
) -> Result<HashMap<Dart, CornerId>, BlendError> {
    let mut cells = HashMap::new();
    for (id, corner) in surgery.corners.iter().enumerate() {
        let Some(&first) = corner_darts[id].first() else {
            return Err(BlendError::InconsistentSurgery {
                reason: "a planned corner is named by nothing",
            });
        };
        let cell = edit.cell_representative(first, Dim::Zero);
        if corner_darts[id]
            .iter()
            .any(|&dart| edit.cell_representative(dart, Dim::Zero) != cell)
        {
            return Err(BlendError::InconsistentSurgery {
                reason: "a planned corner spans several vertices",
            });
        }
        if cells.insert(cell, id).is_some() {
            return Err(BlendError::InconsistentSurgery {
                reason: "two planned corners landed on one vertex",
            });
        }
        let attr = VertexAttr::new(first, corner.point);
        if corner.sources.is_empty() {
            edit.add_vertex(attr);
        } else {
            edit.add_vertex_derived_from(corner.sources.clone(), attr);
        }
    }
    Ok(cells)
}

/// Every face an edge bounds, with the face's directed dart on it.
fn edge_face_darts<P: Payload>(edit: &ModelEdit<'_, P>, edge: EdgeKey) -> Vec<(FaceKey, Dart)> {
    let dart = edit.edge_attr_unchecked(edge).dart;
    edit.orbit(dart, edit.orbit_indices(Dim::One))
        .filter_map(|dart| {
            let face = edit.cell_key::<Cell2>(dart)?;
            edit.face_attr_unchecked(face)
                .pcurves
                .contains_key(&dart)
                .then_some((face, dart))
        })
        .collect()
}

/// Writes a surviving edge's pcurve on `face` for the span it now has.
///
/// The edge keeps its support and only its ends moved along it, so its new
/// pcurve is part of its old one. A plane's is written afresh, exactly; any
/// other face's is cut down from the old pcurve, which keeps whichever branch
/// of a periodic direction the face's other pcurves are written on.
fn retrimmed_pcurve<P: Payload>(
    edit: &ModelEdit<'_, P>,
    edge: EdgeKey,
    face: FaceKey,
    directed: Dart,
) -> Result<TrimmedCurve2, BlendError> {
    let span = Edge::from_dart(edit, directed)
        .ok_or(BlendError::InconsistentSurgery {
            reason: "a surviving edge lost its key",
        })?
        .trimmed_curve();
    let attr = edit.face_attr_unchecked(face);
    if let Surface::Plane(plane) = &attr.surface {
        return curve_pcurve(&span, plane)
            .map_err(|error| BlendError::Pcurve(IntersectionError::from(error)));
    }
    let old = attr
        .pcurves
        .get(&directed)
        .ok_or(BlendError::UnsupportedEdge {
            edge,
            reason: "it has no pcurve on a face its end moves on",
        })?;
    let fraction_of = |point: Point3| -> Result<f64, BlendError> {
        let uv = attr
            .surface
            .param_at(point)
            .map_err(|error| BlendError::Pcurve(IntersectionError::from(error)))?;
        let uv = onto_branch(&attr.surface, uv, old.point_at(Fraction::new(0.5)));
        let fraction = old.parameter_at(uv);
        let lifted = old.point_at(fraction);
        if (attr.surface.point_at(lifted.x, lifted.y) - point).norm() > LINEAR_TOLERANCE.sqrt() {
            return Err(BlendError::UnsupportedEdge {
                edge,
                reason: "its moved end is not on its old pcurve",
            });
        }
        Ok(fraction.value())
    };
    let start = fraction_of(span.start())?;
    let end = fraction_of(span.end())?;
    Ok(old.sub(Interval::new(start, end)))
}

fn write_pcurve<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    directed: Dart,
    pcurve: TrimmedCurve2,
) {
    if let Some(attr) = edit.face_attr_mut(face) {
        attr.pcurves.insert(directed, pcurve);
    }
}
