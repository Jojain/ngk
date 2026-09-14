# Rebuilding a pcurve on a curved support

Status: **Proposed**

Healing cannot rebuild a parameter curve on anything but a plane, and never
could. Every curved fusion is reported as `PcurveNotJoinable`, so a cylinder's
rim — split by a Boolean, then healed — does not come back as the one closed
edge it started as. That is the last unpaid part of §9 of
`plan/seamless_periodic_faces.done.md`: seamless representation landed, the closed
edge it makes expressible does not yet survive a round trip through healing.

This plan owns that path. It was found while building the seamless work,
deliberately left out of it (§11.10 there), and confirmed by building it once:
the first two pieces below work, and the third is what stopped them from
landing.

## 1. Why nothing passes today

`predicates::pcurve::boundary_pcurve` fits a segment or an arc through the
lifted samples and accepts it only if `traces` agrees. `traces` compares the
*polyline through the lifted candidate* with the *polyline through the samples*.

On a curved support every lifted point sits off the chords joining the samples
by a sagitta of about `r(1 - cos(pi / n))` — 0.034 on a unit circle at twelve
samples, orders of magnitude past `LINEAR_TOLERANCE`. The test therefore
measures how many samples were taken, not whether the fit is right, and no
candidate can pass it. `traces` says so in its own doc comment.

## 2. The three pieces

### 2.1 Compare pointwise at matching fractions

Replace the two-sided polyline distance in `traces` with a pointwise comparison
at equal fractions of each span — the synchronized-halves rule the rest of the
kernel already states (`FaceImprint`, `CLAUDE.md`). Sagitta stops entering the
measurement, and a correct fit on a curved support passes.

### 2.2 Unwrap the lifted parameters first — **already in**

`param_at` answers inside the surface's own domain, so a boundary crossing a
closed direction comes back folded. `unwrap_parameters` in `pcurve.rs` undoes
that. It is in the tree already: it is right on its own terms and inert without
the rest of this plan.

### 2.3 `join_on_circle` must read its direction from the samples

Its closed branch reads the sweep direction off the vanishing vertex's angle,
which is well conditioned only while that vertex sits within half a turn of the
start. A Boolean tends to put it at exactly half a turn, where that angle is one
half turn and its sign is noise; the fused circle then comes back reversed, the
loop with it, and the face fails `validate_solid_orientation`. The samples carry
the traversal and answer for any vertex. `join_on_circle` states which reading
it makes and what it costs, and points here.

With 2.1 and 2.3 in,
`boolean_union_of_a_block_and_a_protruding_cylinder_opens_one_inner_loop` passes
and a cylinder rim heals into one closed edge.

## 3. What then fails, one layer out

`builders/boolean/trim.rs` flattens a face's loops into winding polygons. A loop
that is a *single closed pcurve* appears to flatten to a degenerate one, after
which every classification ray is rejected near the boundary and
`solid_contains_point` answers `AmbiguousClassification`.

This is the substance of the work, not a detail of it: the seamless
representation made one-edge loops ordinary, and the winding flattener has not
caught up. It is the same layer that already answers
`AmbiguousClassification` on the crossing-bores case tracked in
`plan/boolean_evaluation.md` milestone 7, though for a different reason — there
the polygons come from an uncertified NURBS branch, here from a loop with one
edge — and the two should be looked at together.

## 4. Milestones

| # | Milestone | Scope |
|---|---|---|
| 1 | Flatten a one-edge closed loop | `trim.rs` produces a non-degenerate winding polygon for a loop that is a single closed pcurve. Provable on a hand-built ring face, ahead of any healing change. |
| 2 | Pointwise `traces` + sampled direction | §2.1 and §2.3 together — they are one change, since either alone leaves a rim that heals to the wrong orientation or not at all. |
| 3 | Retarget the tests | `boolean_difference_supports_a_cylindrical_through_hole` asserts `edges.len() >= 2`, "a rim needs at least two arcs" (`tests/builders/boolean.rs`). That stops being true: state the property that survives — a rim is one closed loop on the bore wall, of one or more arcs — rather than deleting the assertion. |

## 5. Definition of done

- `boolean_union_of_a_block_and_a_protruding_cylinder_opens_one_inner_loop`
  passes with the rim a single closed edge.
- A cylinder rim split by a Boolean and healed is the one closed edge it started
  as, asserted as such.
- `traces` and `join_on_circle` no longer carry doc comments describing a
  weakness they still have, and this plan is what they stop pointing at.
- No healing path reports `PcurveNotJoinable` for a fusion that is geometrically
  a fusion.
