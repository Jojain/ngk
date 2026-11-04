---
status: accepted
date: 2026-11-04
---

# Tessellation: edges first, then a constrained Delaunay triangulation per face

A solid is tessellated by cutting every edge once, in 3D, at a density that
serves every face along it, and by meshing each face against exactly those
points: a constrained Delaunay triangulation of the face's parameter domain
built on exact predicates, then refined in 3D until it follows the surface.
Triangles take their winding from the face's stored boundary, never from a
loop's area. The result is one closed, consistently oriented surface whose
neighbouring faces share their boundary vertices bit for bit. This replaces
slab cuts across the parameter domain and ear clipping of the cut pieces.

Slab cuts put new vertices on a face's boundary wherever a slab line crossed
it, and a neighbour that did not have them was left with a T-junction. Each face
also lifted the edge's samples through its own surface, so two faces agreed on
a shared boundary point only to rounding. 42 of the 231 brep-bench cases failed
on that alone with volume and area exact, and a full-turn revolved face came out
with inward triangles, because its winding was read from the signed area of a
loop that is closed across a seam.

## Considered options

- **Weld the output.** Merge vertices within a tolerance after meshing.
  Rejected: it hides T-junctions under a tolerance that has to be chosen per
  model, and a consumer that does not weld sees cracks.
- **A triangulation crate (`spade`).** Not taken: the metric refinement, the
  seam scaffold with two copies sharing vertices, and the poles are NGK's own
  in either case, and the triangulation itself is small. Only the exact
  predicates are a dependency, the `robust` crate, a port of Shewchuk's
  adaptive `orient2d` and `incircle` with no further dependencies.
- **OCCT's BRepMesh structure with inexact predicates.** The structure
  (edges first and reused by faces, incremental Delaunay in parameter space,
  refinement to a deflection) is what this adopts. Its two known weaknesses are
  not: an isotropic triangulation in raw `(u, v)`, which leaves slivers in 3D
  where the parameterisation stretches, and predicates that can decide a
  near-degenerate case wrongly.

## Consequences

- A boundary segment is never split by one face alone. A face that needs a
  finer boundary gets it from the edge, which is cut at the density of the
  most demanding face along it (including the rate at which each incident
  surface's normal turns along it, so a straight ruling of a curved face is
  cut), so both faces take the new vertices. Seams the parameter domain
  synthesises belong to one face and are cut by that face alone.
- The triangulation can refuse: a face whose boundary segments cross, or whose
  pcurves do not lie on the edges it is built from, yields an error naming the
  face. A model whose pcurves disagree with its edges is no longer meshed
  approximately.
- `Tessellation::mesh` keeps a vertex per face corner, so that each corner
  carries the normal of its own face; positions on shared edges are identical
  to the bit, so joining on exact coordinates closes the surface.
- Refinement stops on three measures: the angle between the normals at a
  triangle's corners, chord sag, and a minimum angle in 3D. The minimum angle is
  a goal that refinement works toward and gives up on where a boundary segment
  would have to be split, not a guarantee.
