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

use crate::topology::gmap::{Dart, Dim};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::FaceKey;
use crate::topology::subdivision::EntityOwner;
use crate::topology::{ModelEdit, ModelEditError};

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
/// [`boundary_cycles`]: crate::topology::subdivision::boundary_cycles
/// [`recover_region`]: crate::topology::subdivision::recover_region
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
