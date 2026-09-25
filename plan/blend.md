# Blends: chamfer and fillet on one engine

A blend replaces the neighbourhood of selected edges or corners with new
geometry: a **chamfer** puts a ruled strip there, and a **fillet** puts the
surface a ball of radius `r` sweeps while it touches both faces. Both take the
same targets and differ only in the geometry they compute, so they share one
engine. This plan replaces the chamfer prototype (`src/builders/chamfer/`,
`docs/chamfer_architecture.md`) with that engine and adds fillet on top.

Status: **proposal**. The open questions at the end decide the first iteration.

## The prototype as it stands

- Four unrelated algorithms (2D corner, solid edge, solid vertex, and a
  batched one for a solid face's rim), dispatched from `ChamferTarget`.
- Solid cases split the surrounding faces with imprints, delete the patch
  touching the selection (`remove_face_patch`, which does its own locator
  repair and dart compaction), then sew replacement faces by matching endpoint
  coordinates.
- `Vec<EdgeKey>` is applied one edge at a time. Two edges sharing a vertex fail
  in either order: the first chamfer consumes the second edge's key. Only the
  profile path is simultaneous.
- It can only remove material. Splitting shrinks faces and cannot grow one, so
  a concave edge is out of reach, because its end faces have to grow.
- A solid edge's distance is measured along the neighbouring edges. That only
  equals the perpendicular setback at square corners.

## Layers

```text
fillet(model, target, radius)      chamfer(model, target, distance)
                     \                /
 1. resolve    BlendTarget -> 2D corners | 3D edge set | vertex cuts (chamfer)
 2. plan       read-only: a section per edge, a treatment per vertex, fit checks
 3. execute    cut -> re-embed -> fill, in one transaction
```

Only layer 2 knows a chamfer from a fillet.

### 1. Resolve

`BlendTarget` takes a `VertexKey`, `EdgeKey`, `ProfileKey` or `FaceKey`, or a
`Vec` or array of them, and normalizes it to a **set**: duplicates collapse
and order is dropped. Incidence decides whether an element is 2D or 3D, not the
call:

| selection | wire, or free planar face | solid |
|---|---|---|
| vertex | its corner | chamfer: corner cut; fillet: open question 3 |
| edge | refused: a 2D edge has no corner of its own | the edge |
| profile | every corner, excluding the ends of an open profile | every edge |
| face | every corner of every loop | every edge of every loop |

A face, its profiles, and the list of its edges resolve to the same set, so
they give the same result.

### 2. Plan

Everything is computed against the model as the call found it, before
anything changes:

- **Per edge, a section:** the blend surface, plus one contact curve on each of
  the two faces.
- **Per vertex the set touches, a treatment.** See [Vertices](#vertices-fans).
- **Per 2D corner:** a trim point on each edge, plus the curve of the new edge.
- **Fit checks:**
  - every trimmed edge keeps a positive length;
  - trims on the same edge do not cross;
  - every affected face's loops stay simple.

  A failure names the edge or vertex, and the model is unchanged.

### 3. Execute: cut, re-embed, fill

This replaces split, remove, then sew-by-matching:

1. **Cut.** Unsew α2 along every selected edge. Each selected edge becomes one
   edge per face. Each vertex falls apart into pieces, as described in
   [Vertices](#vertices-fans).
2. **Re-embed.** Move vertices to their planned points, replace edge curves
   with contact curves, and refresh pcurves. Faces keep their surface and
   their key. A boundary moves inward on a convex edge and outward on a concave
   one, with the same code.
3. **Fill.** Insert the run-out edges into the end faces, add blend faces and
   corner patches, and α2-sew them to darts the plan already names.

What this buys:

- Convex and concave are one path, because a face grows as easily as it
  shrinks.
- No dart is ever removed. That means no locator repair, no compaction, and no
  coordinate matching to decide what to sew.
- Faces keep their keys. The selected edge and the vertices it split are
  consumed. Contact edges, new corners and the blend face are
  `add_*_derived_from` them, so a payload policy can see where each one came
  from.
- A 2D corner blend is step 3's run-out insertion applied to a wire or a free
  face. That makes it one primitive for both, and it is what
  `chamfer_profile_corner` does today.

A Boolean formulation (subtract one tool solid per edge) gets vertex handling
for free, but it has three costs. It gives sharp intersection corners where a
ball is expected. It rests on tangent and coplanar contacts. And every call pays
for a full Boolean. It is not pursued.

### Vertices: fans

The faces around a manifold vertex form a cycle. Cutting `k` of the edges
around it splits that cycle into `k` **fans**. For `k >= 2`, each fan becomes
one corner of the result. For `k = 1`, the single fan is split instead by the
edge inserted into the end face.

| at a trihedral vertex | after the cut | chamfer | fillet |
|---|---|---|---|
| 1 selected: **run-out** | one fan of three faces; the end face gets one inserted edge | line | circle, or ellipse if the end face is oblique |
| 2 selected: **mitre** | one fan of one face, one fan of two faces | line | ellipse, in the plane bisecting the two axes |
| 3 selected: **corner** | three fans of one face | three mitres meeting at the planes' common point | spherical triangle (ball) |
| a solid vertex as the target (chamfer only) | α1 unlinked in each face, one edge inserted in each | triangle | — |

**Mitre.** The two selected edges share one face. The corner of the two-face
fan is a point on the unselected edge between those two faces, and both blends
must land on that same point. They do when the two fan faces make the same
dihedral angle with the shared face. That covers any pair of edges on a block,
and both cap edges at any vertex of a right prism. Other angles are refused in
the first iteration.

**Ball.** For a fillet, a ball exists whenever the three edges have the same
convexity:

- its centre is the point at distance `r` from all three planes;
- each cylinder ends on its cross-section through that centre;
- those three cross-sections are great circles of the sphere.

### Geometry, planar faces

Notation: `n_a` and `n_b` are the outward normals of the two faces. `s = +1`
for a convex edge and `-1` for a concave one.

- **Fillet axis.** It passes through `e0 - s r (n_a + n_b) / (1 + n_a·n_b)`,
  parallel to the edge.
- **Fillet contacts.** Offset the axis by `s r n_a` and by `s r n_b`.
- **Ball centre.** Solve `(A - v)·n_i = -s r` for the three faces.
- **Mitre plane.** It passes through `A`, contains the shared face's normal,
  and bisects the two axes.
- **Fillet run-out curve.** Intersect the cylinder with the end plane.
- **Chamfer.** Each contact line is set back `d` from the edge, perpendicular
  to it, inside its face. The strip is the plane through the two contact lines.

Pcurves:

- On planar faces, pcurves are exact (`curve_pcurve`).
- On cylinders, rulings and cross-sections are exact isolines.
- Ellipses on cylinders and great circles on spheres are fitted through
  `intersect_analytic_surfaces`, the same way Boolean sections already are.

### Code layout

```text
src/builders/blend/     engine: target.rs, plan.rs, section.rs, vertex.rs,
                        corner.rs, execute.rs, errors.rs
src/builders/chamfer/   chamfer() and TargetChamfer, thin over blend
src/builders/fillet/    fillet() and TargetFillet, thin over blend
```

Both entry points return one `BlendError`, whose variants name the offending
entity. It replaces `ChamferError`. `docs/chamfer_architecture.md` is replaced
by a blend architecture document once the engine exists.

## Ordering

- **One call is one network,** planned on the model as the call found it. The
  result depends neither on the order of the list nor on how the selection is
  spelled.
- **Separate calls express a sequence.** Each call sees the previous result.
  That is where order changes the shape, and it is the caller's choice.
- **An interaction a call cannot plan is refused,** with the vertex named. It is
  never degraded to applying the blends one at a time, which would bring back
  order dependence silently.

## First iteration (proposed)

In scope:

- The resolve, plan and execute engine. Chamfer is ported onto it and keeps
  its current coverage, including the extruded NURBS edge and the corner cut.
  `Vec<EdgeKey>` becomes simultaneous.
- Fillet with one constant radius per call.
- In 2D: corners of wires and of free planar faces, for line–line, line–arc and
  arc–arc corners. Offset intersections are closed-form for lines and circles.
- In 3D, on solids:
  - straight edges between two planar faces, convex or concave;
  - trihedral vertices with a run-out, a same-angle mitre, or a ball.
- Results `TargetFillet` and `TargetChamfer`, listing the created faces and the
  consumed edges.
- A `filleted_block` script and a playground experiment with a live radius.

Refused with a named error:

- Any non-planar face in 3D. That rules out circular edges (plane–cylinder
  torus fillets) and blending next to an earlier fillet.
- Tangent-chain propagation and smooth (G1) vertex joins.
- Vertices of valence other than 3, mixed convexity at a vertex, and mitres
  whose blends land on different points.
- A radius per edge, a variable radius, and setbacks.
- Sheets: an open shell has no material side.
- The distance–angle and two-distance chamfer variants.

A rounded box still comes out of one call: all twelve edges and eight balls.

After that, in rough order of value:

1. Circular edges between a plane and a cylinder or cone (torus sections). A
   whole rim needs its seam.
2. Tangent chains.
3. Valence-4 vertices.
4. A radius per edge.
5. Free-form sections through the sweep and skin machinery.

## Tests

- **2D.**
  - A rectangle profile, and a free rectangle face whose area is
    `ab - (4 - π) r²`.
  - A polyline containing an arc.
  - Tangency at every trim point.
  - A radius that is too large leaves the model unchanged.
- **3D.** Each case is checked for manifoldness, orientation and exact volume:
  - one block edge: `abc - (1 - π/4) r² L`;
  - the top profile: four mitres;
  - all twelve edges: the rounded-box closed form;
  - a concave edge of an L-shaped extrusion: the caps grow;
  - an oblique end face: an ellipse run-out;
  - a pocket rim: mitres at reflex corners.
- **Spelling.** The same set given as a face, as its profiles, and as shuffled
  edge lists gives identical results.
- **Refusals.** Every refusal above names its entity and leaves the model
  unchanged.
- **Chamfer.** The existing tests keep passing. Where semantics changed, they
  are reworded.

## Open questions

1. Switch the execution model from split, remove and sew-by-matching to cut,
   re-embed and fill?
2. Do plane–cylinder circular edges (torus) belong in the first iteration? They
   are what allows filleting next to an earlier fillet and rounding a
   cylinder's rim. They also bring seams, tangent chains and re-embedding on
   curved faces, which roughly doubles the iteration.
3. What does a solid vertex mean for a fillet? Either it is refused, or it
   expands to its incident edges. Chamfer keeps its corner cut either way.
