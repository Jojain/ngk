# Projection: what comes next

Projection of points, curves, edges and profiles onto planes, surfaces, curves, edges
and faces is built and described in
[projection.md](../docs/architecture/projection.md); the profile builder is in
[builders.md](../docs/architecture/builders.md#profiles-from-a-projection), and
[ADR 0006](../docs/adr/0006-projection-is-a-query-and-a-face-exists-only-where-trimmed.md)
records why projection is a query and a face exists only where it is trimmed. This plan
keeps what is still open.

## Curves onto surfaces and non-planar faces

A curve, an edge or a profile projects onto a plane and onto planar faces. Onto a face
whose support is not a plane it is refused with `UnsupportedSurface`, and there is no
projection of a curve onto a bare `Surface` at all. The image on such a surface has no
closed form, so it needs a NURBS fit to a tolerance, reported as
`Exactness::Approx { error }`, with an error and no result when the fit does not reach
the tolerance. `Approx` exists in the result type and no code path produces it yet.

- **Default fit tolerance.** `LINEAR_TOLERANCE` is `1e-9`, which may be too tight for a
  fit to reach on general curves. Measure what a fit achieves before settling the
  default of `ProjectOptions`.
- **Directional projection onto 2D curves inside a sketch plane**, if a caller needs it.

## Points

- **Directional projection onto a torus, a ruled surface, a surface of revolution or a
  NURBS surface** is refused with `UnsupportedSurface`: only the surfaces the kernel
  intersects with a line in closed form (plane, sphere, cylinder, cone) have one. The
  perpendicular works on every surface.
- **`Ambiguous` is not detected on a ruled or NURBS surface, nor on a NURBS curve.**
  They are searched numerically, and equally near feet there are returned as one.
- **Closest point on a trimmed face** would land on the face's boundary. It is a
  distance query, not a projection, and belongs to `measure` once a caller needs it.

## Curves

- **A NURBS curve whose image folds back onto a segment is not refused.** A conic whose
  plane holds the direction of projection is `DegenerateImage`, and so is a NURBS curve
  whose mapped control points all coincide; a NURBS curve whose image retraces itself
  along a line is returned as it is.
- **Chains touching at a point stay separate profiles.** `add_profiles_from_projection`
  gives each chain its own vertices, so two chains that meet at a point do not share a
  vertex. A `Profile` cannot branch, so a chain end `ChainEnd::SourceVertex` where more
  or fewer than two ends meet is reachable only from a profile that revisits a vertex.
- **Pcurves are computed and not stored.** A pcurve lives on the face whose boundary an
  edge is, and the edges `add_profiles_from_projection` draws bound none, so there is no
  slot for the segment's pcurve. An imprint builder that consumes a projection needs
  one, or reads the pcurves from the result.

## Python

- A span of a curve cannot be a source: Python has no trimmed curve class, so a curve
  source is projected over its whole domain, and a part of one goes through an `Edge`.

## Not gaps

- A `Jump` in `Normal` mode across a fold is expected: the nearest-foot map is
  discontinuous where the nearer face changes.
