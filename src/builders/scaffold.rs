//! Scaffold a face needs to hold together, and nothing a shape can see.
//!
//! A face whose boundary is more than one closed loop cannot be one 2-cell
//! unless something joins those loops. The join is a **cut**: one raw edge the
//! face's boundary walk uses twice, `alpha2`-linking its two uses to each other,
//! and owned by the face so the walk turns across it instead of emitting it.
//!
//! The cut is scaffold, never shape. It carries no [`EdgeKey`] and no curve, its
//! feet are raw vertices classified inside whatever they sit on, and every
//! logical query answers as if it were not there — which is what lets an annulus
//! keep two unmarked rims and a swept wall stay seamless.
//!
//! [`EdgeKey`]: crate::topology::shape_keys::EdgeKey

use crate::builders::errors::ClosedFaceCellError;
use crate::geometry::{Surface, SurfacePeriodicity};
use crate::topology::embedding::{EntityOwner, is_embedded_cell};
use crate::topology::face::Loop;
use crate::topology::gmap::{Dart, Dim};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{FaceKey, SolidKey};
use crate::topology::{ModelEdit, ModelEditError, RawEdit};

/// One end of a cut, directed from an arriving boundary dart into the cut.
/// Moving it changes only scaffold connectivity; the boundary keeps its keys.
pub(crate) struct CutAttachment {
    incoming: Dart,
    outgoing: Dart,
}

impl CutAttachment {
    /// Finds the unique cut leaving this loop in its traversal direction.
    /// Returns `None` when the loop has no cut or several attachments.
    pub(crate) fn on_loop<P: Payload>(
        edit: &ModelEdit<'_, P>,
        boundary: &Loop<'_, P>,
    ) -> Option<Self> {
        let mut cuts = boundary.darts().filter_map(|dart| {
            let incoming = edit.alpha(Dim::One, edit.alpha(Dim::Zero, dart));
            is_embedded_cell(edit.topology(), edit.embedding(), Dim::One, incoming)
                .then_some(incoming)
        });
        let incoming = cuts.next()?;
        if cuts.next().is_some() {
            return None;
        }
        Some(Self::at(edit, incoming))
    }

    /// Reads the attachment whose two uses are the darts of `incoming`'s cut.
    ///
    /// `incoming` is the cut dart a boundary walk arrives on, so the walk
    /// leaves the cut again on its `alpha2` partner. A caller that already
    /// holds that dart — a splice that just read it off a corner — names it
    /// here rather than searching a loop for it.
    pub(crate) fn at<P: Payload>(edit: &ModelEdit<'_, P>, incoming: Dart) -> Self {
        Self {
            incoming,
            outgoing: edit.alpha(Dim::Two, incoming),
        }
    }

    /// Returns a dart of the boundary this cut reaches across.
    pub(crate) fn across<P: Payload>(&self, edit: &ModelEdit<'_, P>) -> Dart {
        edit.alpha(Dim::One, edit.alpha(Dim::Zero, self.incoming))
    }

    /// Returns the dart the arriving boundary hands the walk over on.
    pub(crate) fn incoming(&self) -> Dart {
        self.incoming
    }

    /// Closes the old boundary gap and inserts the attachment after `boundary`.
    /// The destination runs in the same direction as the detached loop.
    ///
    /// `owner` is the face whose 2-cell the cut's word runs through once it is
    /// there. The cut is labelled as that face's in the same block that moves
    /// it: a cut moved across to another face's 2-cell while still naming the
    /// one it left is a labelling that does not describe the map.
    pub(crate) fn move_after<P: Payload>(
        self,
        edit: &mut ModelEdit<'_, P>,
        boundary: Dart,
        owner: FaceKey,
    ) -> Result<(), ModelEditError> {
        let old_end = edit.alpha(Dim::One, self.incoming);
        let old_next = edit.alpha(Dim::One, self.outgoing);
        let new_end = edit.alpha(Dim::Zero, boundary);
        let new_next = edit.alpha(Dim::One, new_end);
        // The corner the cut leaves keeps its vertex. A vertex is anchored on
        // one dart of its 0-cell, and that dart may be the cut's own: left
        // there, the vertex would follow the cut to the corner it moves to.
        let anchored_on_cut = edit
            .model()
            .iter_vertices()
            .filter(|(_, attr)| attr.dart == self.incoming || attr.dart == self.outgoing)
            .map(|(key, _)| key)
            .collect::<Vec<_>>();
        for vertex in anchored_on_cut {
            edit.vertex_attr_mut_unchecked(vertex).dart = old_end;
        }
        // Detaching the cut from one loop and attaching it to the other merges
        // and splits 0-cells on the way, so nothing may read the model until
        // the last link is made. The block reads nothing, and the labelling is
        // checked where it closes.
        edit.deferred(|raw| {
            for dart in [self.incoming, self.outgoing, new_end] {
                raw.unlink(Dim::One, dart)?;
            }
            raw.link(Dim::One, old_end, old_next)?;
            raw.link(Dim::One, new_end, self.incoming)?;
            raw.link(Dim::One, self.outgoing, new_next)?;
            // A label sits on the dart it was recorded at, so the old one is
            // taken off the whole cell before the new one goes on.
            raw.disown_cell(Dim::One, self.incoming);
            raw.own_cell(Dim::One, self.incoming, EntityOwner::Face(owner));
            Ok::<_, ModelEditError>(())
        })?;
        Ok(())
    }
}

/// Joins two closed boundary loops of one face with a cut the face owns.
///
/// Each loop arrives closed on itself: a run of darts whose last is `alpha1`
/// linked back to its first. The cut replaces that pair of closures with a
/// single cyclic boundary word,
///
/// ```text
/// [cut_out, second loop, cut_back, first loop]
/// ```
///
/// so the walk leaves the first loop along the cut, goes once round the second,
/// comes back along the same cut and carries on. Reading the cut *before* the
/// second loop and again *after* it is what makes the walk turn back out;
/// ordering the word any other way gives a walk that closes after one slot or
/// never terminates at all.
///
/// `first` and `second` are any dart of each loop. The cut is spliced in just
/// after each, so the loops need not be single edges: a face that already
/// carries a cut has loops several slots long, and joining a third loop to one
/// of them works the same way. The face is labelled as owner of the cut, which
/// is what hides it from [`boundary_cycles`] and lets [`recover_region`] cross
/// it.
///
/// The splice runs in a deferred block: between unlinking the two closures
/// and linking the cut's last use, the face's darts form no loop a view could
/// walk, and the block's close checks the map and the labelling once the cut
/// is whole. A caller already inside a deferred block uses
/// [`splice_cut_between_loops`].
///
/// [`boundary_cycles`]: crate::topology::embedding::boundary_cycles
/// [`recover_region`]: crate::topology::embedding::recover_region
pub(crate) fn cut_between_loops<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    first: Dart,
    second: Dart,
) -> Result<(), ModelEditError> {
    edit.deferred(|raw| splice_cut_between_loops(raw, face, first, second))
}

/// [`cut_between_loops`] inside a deferred block the caller already holds.
pub(crate) fn splice_cut_between_loops<P: Payload>(
    raw: &mut RawEdit<'_, '_, P>,
    face: FaceKey,
    first: Dart,
    second: Dart,
) -> Result<(), ModelEditError> {
    let first_end = raw.topology().alpha(Dim::Zero, first);
    let second_end = raw.topology().alpha(Dim::Zero, second);
    // Where each walk went next, read before the splice takes those links over.
    // On a one-edge loop this is the loop's own dart again, which is why the
    // single-edge case needs no special handling.
    let after_first = raw.topology().alpha(Dim::One, first_end);
    let after_second = raw.topology().alpha(Dim::One, second_end);

    for dart in [first_end, second_end] {
        raw.unlink(Dim::One, dart)?;
    }

    let cut_out = [raw.add_dart(), raw.add_dart()];
    let cut_back = [raw.add_dart(), raw.add_dart()];
    for use_ in [cut_out, cut_back] {
        raw.link(Dim::Zero, use_[0], use_[1])?;
    }

    raw.link(Dim::One, first_end, cut_out[0])?;
    raw.link(Dim::One, cut_out[1], after_second)?;
    raw.link(Dim::One, second_end, cut_back[0])?;
    raw.link(Dim::One, cut_back[1], after_first)?;

    // The two uses are the same raw edge seen from its two sides, which is what
    // `turn` follows when the walk crosses the cut.
    raw.link(Dim::Two, cut_out[0], cut_back[1])?;
    raw.link(Dim::Two, cut_out[1], cut_back[0])?;

    raw.own_cell(Dim::One, cut_out[0], EntityOwner::Face(face));
    Ok(())
}

/// Joins every boundary shell of one solid through one scaffold cut face.
///
/// Each shell contributes one raw edge. Its two incident face uses are opened
/// and attached to matching sides of two polygonal faces; those faces are then
/// sewn through `alpha3`. The sewn pair is one raw 2-cell inside the solid,
/// while turning across it keeps the original shell components distinct on the
/// boundary.
///
/// The polygon alternates a shell attachment with a connector edge. Reflecting
/// the second polygon before the `alpha3` sew aligns each attachment with the
/// other use of the same shell edge. This is the two-shell pillow construction
/// generalized to any non-empty set of shells.
///
/// Each shell edge is opened before the pillow closes it again, so the whole
/// construction is one deferred block, checked once the pillow is sewn and
/// owned.
pub(crate) fn cut_between_shells<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    solid: SolidKey,
    shells: &[Dart],
) -> Result<(), ModelEditError> {
    if shells.len() < 2 {
        return Ok(());
    }
    edit.deferred(|raw| {
        let slots = 2 * shells.len();
        let front = polygon(raw, slots)?;
        let back = polygon(raw, slots)?;

        for (index, &shell) in shells.iter().enumerate() {
            let map = raw.topology();
            let first = [shell, map.alpha(Dim::Zero, shell)];
            let second = [
                map.alpha(Dim::Zero, map.alpha(Dim::Two, shell)),
                map.alpha(Dim::Two, shell),
            ];
            raw.unlink(Dim::Two, first[0])?;
            raw.unlink(Dim::Two, first[1])?;

            let front_slot = 2 * index;
            let back_slot = reflected_slot(front_slot, slots);
            link_edge_uses(raw, first, front[front_slot])?;
            link_edge_uses(raw, second, back[back_slot])?;
        }

        for slot in (1..slots).step_by(2) {
            link_edge_uses(raw, front[slot], back[reflected_slot(slot, slots)])?;
        }

        raw.sew(Dim::Three, front[0][0], back[0][1])?;

        let owner = EntityOwner::Solid(solid);
        raw.own_cell(Dim::Two, front[0][0], owner);
        for slot in (1..slots).step_by(2) {
            raw.own_cell(Dim::One, front[slot][0], owner);
        }
        Ok(())
    })
}

fn reflected_slot(slot: usize, slots: usize) -> usize {
    (slots - slot) % slots
}

fn link_edge_uses<P: Payload>(
    raw: &mut RawEdit<'_, '_, P>,
    first: [Dart; 2],
    second: [Dart; 2],
) -> Result<(), ModelEditError> {
    raw.link(Dim::Two, first[0], second[1])?;
    raw.link(Dim::Two, first[1], second[0])?;
    Ok(())
}

fn polygon<P: Payload>(
    raw: &mut RawEdit<'_, '_, P>,
    slots: usize,
) -> Result<Vec<[Dart; 2]>, ModelEditError> {
    let uses = (0..slots)
        .map(|_| [raw.add_dart(), raw.add_dart()])
        .collect::<Vec<_>>();
    for use_ in &uses {
        raw.link(Dim::Zero, use_[0], use_[1])?;
    }
    for slot in 0..slots {
        raw.link(Dim::One, uses[slot][1], uses[(slot + 1) % slots][0])?;
    }
    Ok(uses)
}

/// The raw 2-cell a face covering a closed support occupies.
///
/// Built by [`add_closed_face_cell`] and handed to [`Self::own`] once the face
/// it belongs to has a key. The two halves travel together because they are one
/// fact: these darts are that face's cell, and every cell below it is inside
/// the face rather than bounding it.
pub(crate) struct ClosedFaceCell {
    anchor: Dart,
    /// One representative per lower-dimensional cell of the polygon.
    interior: Vec<(Dim, Dart)>,
}

impl ClosedFaceCell {
    /// Returns the dart the face anchors at.
    pub(crate) fn anchor(&self) -> Dart {
        self.anchor
    }

    /// Classifies every cell below the polygon as embedded in `face`.
    pub(crate) fn own<P: Payload>(&self, edit: &mut RawEdit<'_, '_, P>, face: FaceKey) {
        let owner = EntityOwner::Face(face);
        for &(dim, dart) in &self.interior {
            edit.own_cell(dim, dart, owner);
        }
    }
}

/// Builds the raw 2-cell a face covering `surface` occupies.
///
/// A face bounds nothing only where its support closes in both parameter
/// directions, and the 2-cell it needs is the polygon schema of the closed
/// surface that makes. Two schemas cover what this kernel builds and imports:
///
/// - **a bigon**, where one direction is periodic and the other closes by
///   collapsing to a point at each end — a sphere, capped at its poles;
/// - **a square**, where both directions are periodic — a torus.
///
/// Every edge and corner of the polygon is scaffold: the face owns all of them,
/// so its boundary walk turns across each and emits nothing. What the face
/// gains is the one 2-cell it is required to occupy, which is also the dart its
/// shell is rooted at.
pub(crate) fn add_closed_face_cell<P: Payload>(
    edit: &mut RawEdit<'_, '_, P>,
    surface: &Surface,
) -> Result<ClosedFaceCell, ClosedFaceCellError> {
    if !surface.is_closed() {
        return Err(ClosedFaceCellError::SupportNotClosed);
    }
    match surface.periodicity() {
        SurfacePeriodicity::UVPeriodic(_, _) => add_closed_square(edit),
        SurfacePeriodicity::UPeriodic(_) | SurfacePeriodicity::VPeriodic(_) => {
            add_closed_bigon(edit)
        }
        SurfacePeriodicity::None => Err(ClosedFaceCellError::ClosureNotSchematized),
    }
}

/// Builds the four-dart 2-cell a whole sphere is.
///
/// Two edges between the same two corners — the polygon with the edge word
/// `a a⁻¹` — which is the sphere written as a bigon. `V - E + F = 2`.
fn add_closed_bigon<P: Payload>(
    edit: &mut RawEdit<'_, '_, P>,
) -> Result<ClosedFaceCell, ClosedFaceCellError> {
    let d: Vec<Dart> = (0..4).map(|_| edit.add_dart()).collect();
    // The bigon: two edges, each between the same two corners.
    edit.link(Dim::Zero, d[0], d[1])?;
    edit.link(Dim::Zero, d[2], d[3])?;
    edit.link(Dim::One, d[1], d[2])?;
    edit.link(Dim::One, d[3], d[0])?;
    // Closing it. Pairing each dart with the one reading the other edge the
    // same way round is what makes this a sphere; pairing them the other way
    // round would identify the corners too and give a projective plane.
    edit.link(Dim::Two, d[0], d[3])?;
    edit.link(Dim::Two, d[1], d[2])?;
    Ok(ClosedFaceCell {
        anchor: d[0],
        // One edge and two poles.
        interior: vec![(Dim::One, d[0]), (Dim::Zero, d[0]), (Dim::Zero, d[1])],
    })
}

/// Builds the eight-dart 2-cell a whole torus is.
///
/// A square with both pairs of opposite edges identified, which is the torus
/// written as a polygon with the edge word `a b a⁻¹ b⁻¹`. The four corners
/// become one, the four edges become two, and `V - E + F = 0`.
fn add_closed_square<P: Payload>(
    edit: &mut RawEdit<'_, '_, P>,
) -> Result<ClosedFaceCell, ClosedFaceCellError> {
    let d: Vec<Dart> = (0..8).map(|_| edit.add_dart()).collect();
    for pair in [(0, 1), (2, 3), (4, 5), (6, 7)] {
        edit.link(Dim::Zero, d[pair.0], d[pair.1])?;
    }
    for pair in [(1, 2), (3, 4), (5, 6), (7, 0)] {
        edit.link(Dim::One, d[pair.0], d[pair.1])?;
    }
    // Opposite edges identified the way a translation would: the first side
    // onto the third, the second onto the fourth, each reversed as the square's
    // boundary runs round.
    for pair in [(0, 5), (1, 4), (2, 7), (3, 6)] {
        edit.link(Dim::Two, d[pair.0], d[pair.1])?;
    }
    Ok(ClosedFaceCell {
        anchor: d[0],
        // Two edges and one corner.
        interior: vec![(Dim::One, d[0]), (Dim::One, d[2]), (Dim::Zero, d[0])],
    })
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use crate::geometry::{Frame, Sphere};
    use crate::model::Model;
    use crate::topology::attributes::{FaceAttr, SheetAttr, SolidAttr};
    use crate::topology::embedding::{EntityOwner, boundary_shells, recover_region};
    use crate::topology::payload::StandardPayload;
    use crate::topology::validation::validate_cell_occupancy;

    use super::*;

    #[test]
    fn one_cut_joins_three_shells_without_joining_their_boundaries() {
        let mut model = Model::<StandardPayload>::new();
        let solid = model
            .transaction(|edit| {
                let mut roots = Vec::new();
                for (index, radius) in [3.0, 2.0, 1.0].into_iter().enumerate() {
                    let surface = Surface::Sphere(Sphere::new(Frame::xyz(), radius));
                    let root = edit.deferred(|raw| {
                        let cell = add_closed_face_cell(raw, &surface)?;
                        let face =
                            raw.add_face(FaceAttr::closed(surface, cell.anchor(), HashMap::new()));
                        cell.own(raw, face);
                        let root = if index == 0 {
                            cell.anchor()
                        } else {
                            raw.topology().alpha(Dim::Zero, cell.anchor())
                        };
                        Ok::<_, ClosedFaceCellError>(root)
                    })?;
                    edit.add_sheet(SheetAttr::new(root));
                    roots.push(root);
                }
                let solid = edit.add_solid(SolidAttr::new(roots[0], Some(roots[1..].to_vec())));
                cut_between_shells(edit, solid, &roots)?;
                Ok::<_, ClosedFaceCellError>(solid)
            })
            .expect("three closed shells should share one scaffold cut");

        validate_cell_occupancy(&model).expect("the solid should occupy one raw 3-cell");
        assert_eq!(model.iter_sheets().count(), 3);
        assert_eq!(model.solid_unchecked(solid).shells().len(), 3);
        let region = recover_region(
            model.topology(),
            model.embedding_index(),
            EntityOwner::Solid(solid),
            model.solid_unchecked(solid).dart(),
        )
        .expect("the solid region should be recoverable");
        assert_eq!(
            boundary_shells(model.topology(), model.embedding_index(), &region)
                .expect("the boundary should be closed")
                .len(),
            3
        );
    }
}
