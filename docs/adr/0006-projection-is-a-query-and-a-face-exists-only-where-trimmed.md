---
status: accepted
date: 2026-11-09
---

# Projection is a query, and a face exists only where it is trimmed

## Context

ngk needs one public API to project points, curves, edges and profiles onto
planes, surfaces, curves, faces and lists of faces. Its first caller projects
geometry into a sketch plane; others project profiles onto faces to imprint
them.

Two questions shape the whole API. First, whether projecting writes to the
model: the natural output for a projected profile is a `Profile`, and a
`Profile` is topological, so returning one means creating edges. Second, what a
projection onto a bounded face means where the projection falls outside the
face. A ray can hit the support surface inside a hole; the foot of a
perpendicular can land past the face's boundary. One answer is to return the
closest point on the face, which then lies on its boundary. Projecting a curve
that passes over a hole under that rule drags the projected curve along the
hole's edge, which is neither a shadow nor anything a caller can use.

## Decision

Projection is a pure query. It reads the model, opens no transaction, and
returns a dedicated result: the projected geometry plus what was computed on the
way (parameters, pcurves, the face or edge hit, why a chain stops). A separate
builder, `add_profiles_from_projection`, turns a result into `Profile`s, as
`add_profiles_from_system` does for a solved constraint system.

A face exists only where it is trimmed. In both modes, a projection landing
outside a face's trim is a `Miss`:

- directional projection follows the half-line `P + t·d`, `t ≥ 0`, and stops at
  the first wall; a hit outside the trim is not a wall and the ray continues;
- normal projection takes the foot of the perpendicular on the support and keeps
  it only if it lies inside the trim.

The closest point on a trimmed face is a distance query and does not belong to
projection.

## Consequences

- Projecting never edits the model, so it can be called freely, from Python or
  from inside other builders, and composes with other queries.
- Callers who want topology pay one more call, and the result carries the
  information that call needs (pcurves, faces hit, chain ends) rather than the
  builder recomputing it.
- A curve projected across a hole comes back as two chains ending on `Miss`,
  the same in both modes, rather than one curve following the hole's edge.
- A point whose nearest foot falls in a hole is a `Miss` even though the face
  has points nearby; callers wanting those use the distance query instead.
- Equidistant feet (a point on a cylinder's axis, above the centre of a circular
  hole) are refused as `Ambiguous` rather than resolved by an arbitrary choice.
