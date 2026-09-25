# Blends: chamfer and fillet on one engine

A blend replaces the neighbourhood of selected edges or corners with new
geometry: a **chamfer** puts a ruled strip there, and a **fillet** puts the
surface a ball of radius `r` sweeps while it touches both faces. Both take the
same targets and differ only in the geometry they compute, so they share one
engine.

**Status:** the first iteration has landed, as `src/builders/blend/`. It is
described as built in `docs/blend_architecture.md`. This plan keeps the
reasoning behind the abstractions and what comes next.

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

## What the first iteration replaced

The chamfer prototype had four unrelated algorithms and could only remove
material:

- It split the surrounding faces, deleted the patch touching the selection,
  and sewed replacement faces back by matching coordinates.
- It chamfered edge lists one edge at a time, so two edges sharing a vertex
  failed.
- It could not grow a face, so concave edges were out of reach.

Cut, re-embed and fill removed all three limits.

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
| plane / plane | straight, whatever curve carries it | `Strip` (plane) | `Cylinder` (axis, radius, convexity) |
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

## First iteration (landed)

- The engine above. Chamfer runs on it, with the prototype's coverage plus
  adjacent and concave edges.
- Fillet with one radius per call.
  - 2D: line–line, line–arc and arc–arc corners of wires and free planar
    faces.
  - 3D: straight edges between planar faces, with run-outs (circle or
    ellipse), same-angle mitres, and balls.
- `TargetFillet` and `TargetChamfer`, the `filleted_block` script and
  playground experiment, and `docs/blend_architecture.md`.
- Tests check manifoldness, orientation and closed-form volumes. Among them: a
  block's edge, rim and every edge; a concave edge; a rim with a reflex corner;
  an oblique run-out; the chamfer's common points; that the result does not
  depend on how the selection is spelled; and every refusal.

Refused, each with a named error:

- non-planar faces in 3D;
- tangent chains;
- vertices of valence other than 3;
- mixed convexity at a vertex;
- asymmetric mitres;
- a radius per edge, a variable radius, and setbacks;
- sheets;
- the distance–angle and two-distance chamfers.

## Next

In rough order of value, each with where it slots in.

1. **Torus sections.** Add plane–cylinder and plane–cone circular edges to the
   section table. A whole rim is a closed edge, so capture must accept it and
   the blend face needs a seam; this adds `Wrapping` loops to `NewFace`.
   Re-trimming on curved faces already exists.
2. **Tangent chains.** Tangent propagation is a resolution policy. The smooth
   join is a mitre whose joint is the shared cross-section, a new treatment.
3. **Valence-4 vertices.** The ring already covers them. A run-out across two
   faces is corner insertions in both faces plus a new corner on the edge
   between them.
4. **A radius per edge.** The law is assigned per network edge. Treatments
   already compare the sections they receive.
5. **Free-form sections.** A general row in the section table, which marches
   the rolling ball and skins a NURBS surface. Treatments fall back to general
   surface intersections.
6. **Roll-over and consumed faces.** These need two new surgery operations:
   splitting an existing edge, and removing a face.
