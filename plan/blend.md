# Blends: chamfer and fillet on one engine

A blend replaces the neighbourhood of selected edges or corners with new
geometry: a **chamfer** puts a ruled strip there, and a **fillet** puts the
surface a ball of radius `r` sweeps while it touches both faces. Both take the
same targets and differ only in the geometry they compute, so they share one
engine. This plan replaces the chamfer prototype (`src/builders/chamfer/`,
`docs/chamfer_architecture.md`) with that engine and adds fillet on top.

## Decisions

1. **Execution model:** cut, re-embed, fill. It replaces split, remove and
   sew-by-matching.
2. **Scope:** the first iteration stays with planar faces and straight edges.
   The abstractions below are chosen so that everything after it slots in
   without reshaping the engine:
   - torus sections;
   - variable radius;
   - free-form rolling-ball sections;
   - tangent chains;
   - more vertex configurations.
3. **Solid vertex as a fillet target:** refused. Chamfer keeps its corner cut.

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

## Pipeline

```text
fillet(model, target, radius)      chamfer(model, target, distance)
                     \                /
 resolve    BlendTarget  -> Resolution { 2D corners, 3D edge set, corner cuts }
 capture    Resolution   -> BlendNetwork    (topology only: sides, ends, rings)
 section    network edge -> EdgeSection     (per-edge geometry, from its law)
 treat      network vertex + sections -> VertexBlend  (local surgery)
 assemble   all of the above -> Surgery      (global, geometry-complete)
 execute    Surgery      -> the model        (cut, re-embed, fill; no geometry)
```

Everything up to `assemble` reads the model as the call found it. Only
`execute` writes. The laws are only read inside `section`, and the 2D corner
solver is the one other place that computes geometry from them.

## Abstractions

### `BlendTarget` → `Resolution` (resolve)

`BlendTarget` is a list of `BlendSelection { Vertex, Edge, Profile, Face }`,
built from any key, `Vec`, or array. Incidence decides whether an element is
2D or 3D:

| selection | wire, or free planar face | solid |
|---|---|---|
| vertex | its corner | chamfer: corner cut; fillet: refused |
| edge | refused: a 2D edge has no corner of its own | the edge |
| profile | every corner, excluding the ends of an open profile | every edge |
| face | every corner of every loop | every edge of every loop |

The resolution is a set: duplicates collapse and order is dropped. A face, its
profiles, and its edges therefore resolve to the same thing.

When expanding a profile or a face, flat corners and flat edges are skipped,
because they have nothing to blend. Named explicitly, they are refused.

*Extension:* tangent-chain propagation is a resolution policy that grows the
edge set. It touches nothing below.

### Law

`BlendLaw::{Chamfer(ChamferLaw), Fillet(FilletLaw)}` is the cross-section rule:
`ChamferLaw::Distance(d)` and `FilletLaw::Radius(r)`. It is read in exactly
two places: the section solvers and the 2D corner solver. Vertex treatments
never read the law, only the sections. So when two blends of different radius
meet, the treatment sees two cylinders of different radius and decides for
itself, whatever produced them.

*Extension:*

- A variable radius, a conic section, or two-distance and distance–angle
  chamfers are new law variants plus section solvers for them.
- A radius per edge is a law per network edge. Vertex treatments are
  unaffected.

### `BlendNetwork` (capture)

This is the combinatorial picture of the 3D selection. It holds no blend
geometry.

- **`NetworkEdge`:** its key, its two `EdgeSide`s (the face, and that face's
  dart at the edge's start), and the network vertices at its two ends.
- **`NetworkVertex`:** its key, and its **ring**: the faces and edges around it
  in rotation order. Each slot is an edge, whether that edge is selected, the
  face after it, and that face's two darts at the vertex. Walking the ring uses
  `turn`, so bridges and cavity scaffold are crossed, not stopped at.

Cutting `k` selected edges splits a vertex's ring into `k` **fans**. For
`k >= 2`, each fan becomes one corner. For `k = 1`, the fan is split by an edge
inserted into its faces. The ring already covers any valence, so a valence-4
vertex needs no change here.

### `EdgeSection` (section)

`EdgeSection` holds one edge's blend geometry, independent of how its ends are
treated:

- the blend surface;
- one rail support per side (the contact curve on that face);
- a `SectionForm` that records the closed form, when there is one.

Solving is dispatched over a table of face-pair and edge kinds, analytic first,
in the same shape as `intersect_analytic_*`:

| faces | edge | chamfer | fillet |
|---|---|---|---|
| plane / plane | line | `Strip` (plane) | `Cylinder` (axis, radius, convexity) |
| plane / extruded wall | translate of the wall's base | `Translated` (ruled) | refused |

The extruded-wall chamfer keeps the prototype's semantics: both rails are
translates of the edge. On the plane, the translation is perpendicular to the
edge's chord, so this is not a true constant setback.

*Extension:*

- Plane–cylinder and plane–cone circular edges become `Torus` rows.
- Free-form edges become a general row that marches the rolling ball and skins
  a NURBS surface through its cross-sections, using the sweep's skinning.
- Nothing downstream needs to know which row answered.

### `VertexBlend` (treat)

A treatment turns one vertex, and the sections meeting there, into **local
surgery** written in a fixed vocabulary:

- **corners:** the points of the vertices the treatment leaves behind;
- **joints:** new curves between two corners. A joint is either inserted
  into an existing face's corner, or shared by two new faces;
- **ends:** for each blend ending here, which corner each rail stops at, and the
  chain of joints that closes the blend face between them;
- **patches:** new faces the treatment adds, bounded by joints.

It is dispatched by configuration, closed form first:

| configuration | chamfer | fillet |
|---|---|---|
| run-out: 1 selected edge, trihedral | line on the end face | circle or ellipse on the end face |
| mitre: 2 selected edges, trihedral, same-angle | line | ellipse in the plane bisecting the axes |
| corner: 3 selected edges, trihedral, same convexity | three mitres meeting at the planes' common point | spherical triangle (ball) |
| corner cut: a solid vertex target | triangle | refused |

*Extension:*

- An asymmetric mitre, a valence-4 run-out that crosses two faces, a smooth
  (G1) join on a tangent chain, and setback patches are all new treatments.
  Each emits the same vocabulary, so none of them changes the executor.
- The closed forms here all use `SectionForm`. A general treatment would
  intersect the sections' surfaces with each other and with the end faces
  instead.

### `Surgery` (assemble) and the executor

`assemble` maps every treatment's local ids to global ones. It then builds one
blend face per network edge:

```text
rail A (end 0 -> end 1), chain at end 1, rail B reversed, chain at end 0 reversed
```

It also computes every pcurve:

- On planes, pcurves are exact (`curve_pcurve`).
- Elsewhere they are traced through the analytic section machinery:
  - exact when the curve is an isoline;
  - fitted, with a measured deviation, otherwise.

The result is `Surgery`, which is geometry-complete:

- `corners`: a point and lineage sources for each;
- `consumed_vertices`;
- `cuts`: one per selected edge, and on each side the rail's curve and pcurve;
- `joints`: a curve, plus either the existing face corner it is inserted into
  or nothing;
- `faces`: a surface, lineage sources, and a boundary made of rails and
  joints, each with its pcurve.

The executor computes no geometry. In order, it:

1. unsews α2 along every cut;
2. inserts every joint that has a corner insertion;
3. builds each new face's boundary darts and sews them to their partners,
   which it finds by id, never by coordinates;
4. orients each new face by consistency with its oriented neighbours;
5. registers identities with lineage;
6. writes points, curves and pcurves;
7. re-trims the pcurves of every surviving edge whose end moved.

It checks that each 0-cell it produced holds exactly one planned corner, so a
treatment that forgot a piece of a vertex fails before commit.

After execution, every affected face is checked for crossing loops, and
every affected solid for manifoldness and orientation. A failure rolls the call
back.

**Identity:**

- Faces keep their keys, and so do trimmed edges.
- Selected edges and blended vertices are consumed.
- Rails and the blend face derive from their edge.
- Corners, joints and patches derive from their vertex.

*Extension:* the one thing the vocabulary cannot express yet is a rail that
crosses an existing edge (a roll-over), or a face a blend consumes whole. Each
needs a new surgery operation: splitting an existing edge, or removing a face.

## Ordering

- **One call is one network,** planned on the model as the call found it. The
  result depends neither on the order of the list nor on how the selection is
  spelled.
- **Separate calls express a sequence.** Each call sees the previous result.
  That is where order changes the shape, and it is the caller's choice.
- **An interaction a call cannot plan is refused,** with the vertex named. It is
  never degraded to applying the blends one at a time.

## First iteration

In scope:

- The engine above. Chamfer is ported onto it and keeps its coverage (2D line
  corners, solid edges, the extruded NURBS edge, the corner cut, rims).
  `Vec<EdgeKey>` becomes simultaneous, and concave edges now work too.
- Fillet with one radius per call.
  - 2D: corners of wires and free planar faces, for line–line, line–arc and
    arc–arc corners.
  - 3D: straight edges between planar faces, with the treatments in the
    table above.
- `TargetFillet` and `TargetChamfer`, which list the created faces and the
  consumed edges.
- `filleted_block` script and playground experiment.
- `docs/blend_architecture.md`, which replaces `docs/chamfer_architecture.md`.

Refused with a named error:

- non-planar faces in 3D;
- tangent chains;
- vertices of valence other than 3, and mixed convexity at a vertex;
- asymmetric mitres;
- a radius per edge, a variable radius, and setbacks;
- sheets;
- the distance–angle and two-distance chamfers.

After that, in rough order of value:

1. torus sections, where a whole rim needs its seam;
2. tangent chains and the G1 join;
3. valence-4 vertices;
4. a radius per edge;
5. free-form sections.

## Tests

Each 3D case is checked for manifoldness, orientation and exact volume.

- **2D.**
  - A rectangle wire, where the length is `2(a + b) - 8r + 2πr`.
  - A free rectangle face, where the area is `ab - (4 - π)r²`.
  - A line–arc corner.
  - A radius that is too large leaves the model unchanged.
- **One block edge.** The volume is `abc - (1 - π/4) r² L`.
- **Top profile (four mitres).** The volume is
  `abc - (1 - π/4) r² P + 4 (5/3 - π/2) r³`.
- **All twelve edges.** The rounded-box closed form.
- **L-extrusion, concave edge.** The volume is `V + (1 - π/4) r² h`.
- **L-extrusion, top profile, with one reflex mitre.** Each convex corner
  subtracts `(5/3 - π/2) r³` from the removed volume, and each reflex corner
  adds it.
- **Oblique end face (ellipse run-out).** The removed volume is
  `∫∫_R (L - y cot φ) dA`.
- **Chamfer, all twelve edges.** The volume is `abc - 2d²(a + b + c) + 6d³`.
- **Spelling.** The same set given as a face, as its profiles, and as shuffled
  edge lists gives identical results.
- **Refusals.** Every refusal names its entity and leaves the model unchanged.
