# One boundary walk for face domains

Noticed while building the Green boundary integral in `src/measure/green.rs`.
It works as it stands; this is about code read off `UnwrappedFaceDomain` in
several places that each re-derive the same facts.

## What overlaps

1. **The loop walk is written twice.** `UnwrappedFaceDomainLoop::flatten`
   (`src/topology/unwrapped_face_domain.rs`) walks each loop as: the corners a
   pcurve turns through, then the pcurve shifted by its offset. The measure's
   `push_loop` and `Piece` (`src/measure/green.rs`) do the same walk again, as
   straight segments between corners plus offset pcurves. The corner rules
   are subtle: a capping loop's corners run out to a collapsed row and back,
   and `rebranch_across_degenerate_row` and `bring_inside` move some corners
   but not others. Two copies of that walk can drift apart.

2. **The rule for a face that nothing encloses is spread over three places.**
   Such a face (a whole sphere or torus, possibly with holes) covers its
   support's parameter rectangle, wound by its sense, or against its first
   hole when it has one:
   - `FaceTrimDomain::whole_support` (`src/builders/boolean/trim.rs`);
   - `tessellate_boundaryless_face` (`src/tessellate/face.rs`), which takes
     its winding from `Face::boundary_signed_area`;
   - `FaceDomain::of_face` (`src/measure/green.rs`), which builds the
     rectangle as a loop and computes the hole winding itself.

3. **The measure does not use the realization cache.** `FaceDomain::of_face`
   calls `UnwrappedFaceDomain::of_face` directly. `Model::realize_face`
   (`src/model/realization.rs`) caches the same domain per revision, sense
   and purpose, and `Face::boundary_signed_area` already goes through it.

## The refactor it suggests

- Give `UnwrappedFaceDomainLoop` a `pieces()` iterator that yields segment
  or offset-pcurve pieces. Build `flatten` and the polyline readers on it.
  The measure's `Piece` would then move into topology.
- Give `UnwrappedFaceDomain` one query for the whole-support case: the
  parameter rectangle and its winding, or `None` when a loop encloses the
  face. The Boolean trim, the tessellator and the measure would all call it.
- Build the measure's `FaceDomain` from `realize_face(key, sense,
  RealizationPurpose::Geometry)`, so validation and the measure share one
  cached domain on an unchanged model.

The measure would keep only what is its own: integration breakpoints (knots,
quarter turns, crossings of `v` knot rows) and the reference line.

## Not checked

- Whether the tessellator and the Boolean trim truly agree with the measure
  on the winding of a whole-support face with holes. A face the Boolean
  flipped by rewinding its holes is the case to test.
- Whether `realize_face` can be used from every place that measures. It
  takes a key and a sense, not a view; boundaryless faces are measured in the
  shell's own sense.
- Cost. A shared walk returning boxed iterators could be slower than the
  measure's current `Vec<Piece>`.
