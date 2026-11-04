# Tessellation

How a shape becomes triangles and polylines. Code: `src/tessellate/`. Decision:
[ADR 0002](../adr/0002-tessellation-edge-first-constrained-delaunay.md).

A [`Tessellation`] is what a viewer draws and what an exporter writes: every
face's triangles in one indexed buffer, each face a range of it; every edge's
polyline; every vertex's point. The triangles of a solid join, on exact
coordinates, into a closed, consistently oriented surface.

## Edges first

`Discretisation` (`edges.rs`) cuts each edge once, in its stored direction, and
every later reader gets the same points, bit for bit. The cut serves every face
along the edge, so it is the finest of:

- the edge's own turning: at most one `segments`-th of a turn of the tangent
  between two samples (`curve::segments_for`);
- each incident surface's bend along it: the edge is cut so that the normal of
  every incident surface turns at most the angular step between two samples
  (`size::normal_turn_rate`), which is what cuts a straight ruling of a
  cylinder or a cone, whose tangent never turns;
- the normals sampled along it, halving an interval until neither surface turns
  more than the step across it;
- every place a pcurve of an incident face starts or ends along it.

The ends of an edge are the stored positions of its vertices. A point of an edge
has an identity, `SharedPoint`: a vertex, a sample of an edge, or a point inside
one face (a seam it synthesised, a pole). The mesh of a face names its boundary
points by those identities, which is what lets a test, and the face's own
assembly, tell that two faces share a point rather than merely agree on it.

`tessellate_edge` and the polylines of a `Tessellation` read the same samples,
so every point an edge is drawn at is a vertex of every face along it.

## A face

`face.rs` reads the face the way its stored boundary states it. A face with a
boundary says which way it points in how that boundary winds as stored; the sense
a shell reads it with says the same thing a second time and is not applied. A
face with no loop at all has only that sense, and a face nothing encloses but
that carries holes is wound against them.

1. **Boundary rings.** Each loop of the face's `UnwrappedFaceDomain` is walked
   pcurve by pcurve. Each pcurve contributes the samples of its edge, placed on
   the pcurve where the surface puts them, and with the identity the edge gave
   them. Stretches of the domain's boundary that lie on no edge are cut here:
   the seam the domain made to open a periodic support, and a row that collapses
   to a pole. A seam is cut once from its end lower along its axis and read by
   both sides, so its two copies share their 3D points; points joined by a
   collapsed row are one shape point however many parameter points they take. A
   face nothing encloses takes its support's parameter rectangle, opened where
   no hole lies, as its outer ring.
2. **Constrained Delaunay triangulation** (`cdt.rs`) in parameters scaled by the
   surface's average first fundamental form, so triangles come out near their
   3D shape. Boundary rings are inserted as vertices and forced in as
   constraints; inside and outside are told apart by the parity of the
   constraints crossed. Every orientation and in-circle decision is Shewchuk's
   adaptive exact predicate (`robust`), so near-collinear and near-cocircular
   inputs are decided correctly.
3. **Refinement in 3D.** A triangle is split at its circumcentre (its centroid
   where that would encroach) when the normals at its corners and centre turn
   more than the angular step, or the surface stands off it by more than the
   matching chord sag; then a triangle with an angle under 20° in 3D is split at
   its circumcentre. A point that would fall inside the diametral circle of a
   boundary segment is not inserted: boundary segments are never split by one
   face alone.
4. **Output.** Triangles that collapse onto one shape point (the fan round a
   pole) are dropped; the rest are wound from the face's orientation. A pole
   has no normal of its own and takes the mean of its triangles'.

A face that cannot be meshed is an error naming it (`TessellationError`); a mesh
that is not the face would be read as the face.

## Limits

- A face whose pcurves do not lie on the edges it is bounded by cannot be meshed
  consistently: the edge says one thing and the face's own geometry another, and
  the boundary segments cross. It is reported, not approximated.
- The minimum angle is a refinement goal, not a guarantee, since boundary
  segments are not split by the face. A face much longer than the step is cut
  along its ruling at most `MAX_SEAM_SEGMENTS` and `MAX_SAMPLES` pieces.
