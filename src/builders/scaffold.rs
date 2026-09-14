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

use crate::topology::face::Loop;
use crate::topology::gmap::{Dart, Dim};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::FaceKey;
use crate::topology::embedding::{EntityOwner, is_embedded_cell};
use crate::topology::{ModelEdit, ModelEditError};

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
    pub(crate) fn move_after<P: Payload>(
        self,
        edit: &mut ModelEdit<'_, P>,
        boundary: Dart,
    ) -> Result<(), ModelEditError> {
        let old_end = edit.alpha(Dim::One, self.incoming);
        let old_next = edit.alpha(Dim::One, self.outgoing);
        let new_end = edit.alpha(Dim::Zero, boundary);
        let new_next = edit.alpha(Dim::One, new_end);
        for dart in [self.incoming, self.outgoing, new_end] {
            edit.unlink(Dim::One, dart)?;
        }
        edit.link(Dim::One, old_end, old_next)?;
        edit.link(Dim::One, new_end, self.incoming)?;
        edit.link(Dim::One, self.outgoing, new_next)?;
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
/// [`boundary_cycles`]: crate::topology::embedding::boundary_cycles
/// [`recover_region`]: crate::topology::embedding::recover_region
pub(crate) fn cut_between_loops<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    first: Dart,
    second: Dart,
) -> Result<(), ModelEditError> {
    let first_end = edit.alpha(Dim::Zero, first);
    let second_end = edit.alpha(Dim::Zero, second);
    // Where each walk went next, read before the splice takes those links over.
    // On a one-edge loop this is the loop's own dart again, which is why the
    // single-edge case needs no special handling.
    let after_first = edit.alpha(Dim::One, first_end);
    let after_second = edit.alpha(Dim::One, second_end);

    for dart in [first_end, second_end] {
        edit.unlink(Dim::One, dart)?;
    }

    let cut_out = [edit.add_dart(), edit.add_dart()];
    let cut_back = [edit.add_dart(), edit.add_dart()];
    for use_ in [cut_out, cut_back] {
        edit.link(Dim::Zero, use_[0], use_[1])?;
    }

    edit.link(Dim::One, first_end, cut_out[0])?;
    edit.link(Dim::One, cut_out[1], after_second)?;
    edit.link(Dim::One, second_end, cut_back[0])?;
    edit.link(Dim::One, cut_back[1], after_first)?;

    // The two uses are the same raw edge seen from its two sides, which is what
    // `turn` follows when the walk crosses the cut.
    edit.link(Dim::Two, cut_out[0], cut_back[1])?;
    edit.link(Dim::Two, cut_out[1], cut_back[0])?;

    edit.own_cell(Dim::One, cut_out[0], EntityOwner::Face(face));
    Ok(())
}
