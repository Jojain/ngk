# Projection

Projecting points, curves, edges and profiles onto planes, surfaces, curves, edges
and faces, under `src/projection/`. It sits next to [`measure`](measure.md): both
read geometry and the model and write neither, so projection opens no
transaction. The decisions behind it are
[ADR 0006](../adr/0006-projection-is-a-query-and-a-face-exists-only-where-trimmed.md);
what is not done yet is in [plan/projection.md](../../plan/projection.md).

A projection is a query with a dedicated result, never a shape. The builder that
turns a curve projection into profiles is a separate call,
[`add_profiles_from_projection`](builders.md#profiles-from-a-projection).

## The `Project` trait

There is one entry point, `project(source, target, mode, &options)`, which calls
`Project<Target>::project` on the source. The trait is implemented once per
(source, target) pair and has three associated types:

| Type | Meaning |
|---|---|
| `Mode` | How the source is carried. `Projection` for a two-dimensional target; `()` for a curve or an edge, which has only the perpendicular, so a directional projection onto one cannot be written. |
| `Hit` | What the target reports as touched: `()` for pure geometry (`Plane`, `Surface`, `Curve`), `FaceKey` for a face or a `FaceList`, `EdgeKey` for an edge. |
| `Output` | `PointProjection<Hit>` for a point, `CurveProjection<Hit>` for a curve, an edge or a profile. |

`Projection` is `Normal` (the foot of the perpendicular on the target) or
`Along(UnitVector3)` (the first hit of the half-line from the source in that
direction; `Projection::along` normalizes and panics on the zero vector).
`ProjectOptions` holds one `tolerance`, the distance within which two points
coincide, `LINEAR_TOLERANCE` by default: feet that close in distance to the source
are equally near, and a hit that far behind the source's origin is still on the
half-line. `FaceList::new(&model, &[FaceKey])` is a list of faces of one model,
projected onto as a whole.

`project_point` calls `project` for a point and keeps only the projected point. A
result carries what the algorithm computed on the way and nothing more: the `(u, v)`
of a projection onto a plane are not returned, as they follow from the point.

### Supported pairs

N is `Projection::Normal`, D is `Projection::Along`, and the entries for a curve
or an edge target are `()`.

| Source | `Plane` | `Surface` | `Face` | `FaceList` | `Curve` | `Edge` |
|---|---|---|---|---|---|---|
| `Point3` | N, D | N, D | N, D | N, D | perpendicular | perpendicular |
| `Curve`, `TrimmedCurve`, `Edge`, `Profile` | N, D | not offered | N, D, planar support only | N, D, planar supports only | not offered | not offered |

A curve onto a curve is not offered in any mode: in 3D a ray and a curve meet only
by coincidence, and where one curve crosses another seen along a direction composes
from projecting both into a plane and intersecting there. A curve onto a surface that
is not a plane is not offered either (see below for a face).

Projecting a region is not a projection: a face, or a filled profile, is an imprint
builder consuming the projected boundary.

## A face exists only where it is trimmed

A face is its support surface cut by its trim. What lands outside the trim, in a hole
or past the boundary, is a `Miss` in both modes; the boundary itself is inside.

- **Along.** The ray is the half-line `P + t d`, `t >= 0`, and stops at the first wall
  it hits. A hit on the support outside the trim is not a wall and the ray goes on. A
  point already behind the wall casts no shadow. Only the target's faces cast shadows.
- **Normal.** The foot of the perpendicular on the support, kept when it is inside the
  trim; otherwise `Miss`. Onto a `FaceList`, each face's foot is kept if inside its
  trim, then the nearest of them wins. Feet equally near at different points are
  `Ambiguous`; the same foot on two faces, as on an edge they share, is one answer.
  For `Along`, a wall on an edge two faces share is one point and the face earlier in
  the list is reported.
- **Edge.** The foot on the support curve must fall within the edge's bounds, else
  `Miss`; the reported parameter is on the edge's span.

The decision is [ADR 0006](../adr/0006-projection-is-a-query-and-a-face-exists-only-where-trimmed.md).
The closest point on a trimmed face, which would land on its boundary, is a distance
query and not a projection.

## Point projection

`PointProjection<Hit>` is the projected `point`, the `parameters` on the target
(`TargetParameters::Curve(t)` or `Surface((u, v))`) when computed and `None` when a
caller can recompute them, the `distance` from the source to the point, and the
`hit`.

- **Plane**: closed form in both modes. A direction parallel to the plane is a `Miss`,
  or `RayInSurface` when the point lies in the plane; a plane behind the point along
  the direction is a `Miss`.
- **Surface**: `Normal` is the surface's nearest point. `Along` is closed form for the
  surfaces the kernel intersects with a line analytically (plane, sphere, cylinder, a
  cone that is not degenerate) and is refused with `UnsupportedSurface` for the
  others. A hit that touches without crossing is `GrazingRay`; a ray in the surface
  is `RayInSurface`.
- **Curve** and **Edge**: the foot on the support curve, with the curve parameter.
- **Face**, **FaceList**: as above, over the support the face has.

`Ambiguous` is the same rule on every target, whatever its representation: a point
with two feet farther apart than the tolerance and equally near within it. The
analytic surfaces and curves are checked in closed form: the centre of a sphere, the
axis of a cylinder, cone, torus or revolution, the centre of a circle, the major axis
of an ellipse inside its evolute. A ruled or NURBS surface and a NURBS curve are
searched numerically, so the rule is applied to the local nearest points the search
finds. The search samples the domain, starts a Newton refinement from every sampled
local minimum of the distance, and keeps each one it converges to; a ruled surface
searches its profile curve across the ruling direction. A NURBS cylinder is therefore
refused on its axis exactly as the analytic one is. The foot that is returned is
still the one the surface's own nearest-point search reports; the local nearest
points only decide whether it is unique. At most 32 local minima of a surface's
sample grid are refined, so a point equally near more places than that is judged on
the nearest 32.

## Curve projection

A curve, an edge or a profile projects to a `CurveProjection<Hit>`: chains of
segments, never glued into one curve.

```text
CurveProjection<Hit>  chains: Vec<ProjectedChain<Hit>>
ProjectedChain<Hit>   segments: Vec<ProjectedSegment<Hit>>, start: ChainEnd, end: ChainEnd
ProjectedSegment<Hit> curve: TrimmedCurve, pcurve: TrimmedCurve2, hit: Hit,
                      source: SourceSpan, fit: Exactness
```

A segment is the image of one span of the source on one part of the target. `curve`
is the piece in 3D and `pcurve` the same piece in the target's parameter space; both
run the way the source was walked and the point at a given fraction of one is the
point at that fraction of the other. `source` names what was projected
(`SegmentSource::Curve`, or `SegmentSource::Edge(EdgeKey)`) and the directed native
span of its support the segment covers. `hit` is `()` for a plane and the `FaceKey` of
the face the segment lies on for a face target. The thin helpers narrow a result:
`CurveProjection::segments`, `curves` (the 3D curves) and `pcurves` (the 2D curves),
and the same `curves` and `pcurves` on a `ProjectedChain`.

### Chains and `ChainEnd`

Chains are stored because the algorithm knows a join is continuous when it computes
it. `ChainEnd` says why a chain begins or stops where it does:

| `ChainEnd` | Meaning |
|---|---|
| `Closed` | The chain runs round to its own other end. `ProjectedChain::closed()` is both ends `Closed`. |
| `SourceEnd` | The source itself ends here. |
| `SourceVertex(VertexKey)` | The source meets itself at this vertex: more or fewer than two ends of the walk meet there, so every chain there stops on it. |
| `Miss` | The projection stops existing here: the source passes behind the target or past it. |
| `Jump` | The projection leaps to a different part of the target here. |

Two consecutive parts with a shadow are one chain when they meet inside a span, or at
a vertex where exactly two ends of the walk meet, and the target says they continue.
A part with a shadow next to one without ends the chain on a `Miss`. A closed walk
all of whose parts are continuous is one chain with `Closed` ends. The walk of a
`Profile` is its edges in order, closed when the profile is; an `Edge` is one span,
oriented as in its view; a `Curve` is its whole domain and an unbounded one is
refused. A source cut by a `Miss`, a `Jump` or a source vertex gives several chains
in one result.

### Exactness

`Exactness` is `Exact` when the segment is the image itself and `Approx { error }`
when it is a fit off the image by at most `error`. Every projection built today is
`Exact`, so no path produces `Approx`.

### Onto a plane

Both modes are parallel maps: each point moves along one fixed direction (the normal,
or the given one) until it meets the plane. An affine map takes a line to a line, a
conic to a conic and a NURBS curve to the NURBS curve on the mapped control points
with the same weights, so every image is exact in 3D and in the plane's `(u, v)`.

A directional projection keeps the half-line rule: a part of the source behind the
plane along the direction casts no shadow, so a source crossing the plane is cut
there and the chain of the part in front stops on a `Miss`. A source wholly behind
the plane is a `Miss`.

### Onto faces

On each face whose support is a plane the image of a span is exact and is cut where it
crosses the face's trim; a part is kept where its midpoint is on the face. Where
several faces cover a parameter one wins as it does for a point: the first wall of a
directional projection, the nearest foot of a perpendicular one, the earlier face of
the list on equal reach. Winners are read off parts of the span cut where two faces
are equally far, which is where the span crosses a plane.

The winning parts are chained. Two parts continue each other when they are on the same
face, or when they meet on an edge the two faces share in the model. Otherwise the
chain stops on a `Jump`; where no face takes the source, on a `Miss`. A face whose
support is not a plane refuses the whole target with `UnsupportedSurface`, and a list
that takes no part of the source is a `Miss`.

In `Normal` mode the nearest-foot map is discontinuous across a fold, so a chain can
stop on a `Jump` where the nearer face changes, and beside a falling fold a part has
no foot. That is the answer, not a gap.

## Refusals

Where the answer is not unique or not defined, projection refuses with a named
`ProjectionError` rather than picking.

| Error | When |
|---|---|
| `Miss` | Nothing lies where the projection looks: the half-line meets no part of the target, or only behind its origin; a foot outside a trim or an edge's bounds. |
| `Ambiguous` | Several feet are equally near within tolerance. |
| `GrazingRay` | The ray touches a surface without crossing it. |
| `RayInSurface` | The ray lies in the surface. |
| `UnsupportedSurface { surface }` | A directional point projection onto a torus, ruled surface, surface of revolution or NURBS surface; a curve onto a face whose support is not a plane. |
| `UnsupportedCurve { curve }` | A source curve with no exact image: a helix onto a plane. |
| `UnboundedSource` | The source has no finite span. |
| `DegenerateImage` | The source collapses under the projection: to a point, or, for a conic whose plane holds the direction, to a segment that retraces itself; and for a NURBS curve, to one point. |
| `IncompleteCrossings` | The crossings of the source with a plane could not all be found, so the part in front is not certain. |
| `Nurbs`, `Intersection`, `Trim`, `TrimCrossing` | A NURBS operation, an intersection or a face's trim read that the projection relies on refused its input; each carries its cause. |

## Profiles from a projection

`add_profiles_from_projection` turns a `CurveProjection` into profiles, one per chain,
with one edge per segment. It lives with the other profile builders; see
[builders.md](builders.md#profiles-from-a-projection). The pcurves of the segments are
computed by the projection and are not stored by it. The facade
`modeling::profiles::from_projection` does the same in a fresh model.

## Python

`ngk.projection` exposes `project(source, target, along=None, tolerance=None)` and
`project_point` (the same arguments for a point source, returning the `Point`). The
source is a `Point` (or a 3-tuple), a curve (`Line`, `Circle`, `Ellipse`, `Helix`,
`NurbsCurve`, taken over its whole domain), an `Edge` or a `Profile`; the target is a
`Plane`, another surface, a curve, an `Edge`, a `Face` or a list of faces of one model.
`along` is a `Vector`; it is a `ValueError` for a curve or an edge target, and a pair
outside the table is a `TypeError`.

The result is a `PointProjection` or a `CurveProjection` with the same fields as in
Rust: `ProjectedChain`, `ProjectedSegment`, `SourceSpan`, `ChainEnd` (a `kind` string
and the `vertex` of a `"source_vertex"` end), `Exactness`, and the spans as
`TrimmedCurve` and `TrimmedCurve2`. The hit of a point or a segment is the `Edge` or
`Face` itself, or `None`. Refusals are `ProjectionError`, a `ValueError`, with
`MissError`, `AmbiguousError`, `GrazingRayError`, `RayInSurfaceError`,
`UnsupportedSurfaceError` and `UnsupportedCurveError` deriving from it; the other
variants raise `ProjectionError` with their cause in the message.
`ngk.modeling.profiles.from_projection` draws a curve projection in a fresh model and
returns one `ProjectionProfile` (the profile and the edge of each segment) per chain.

## Where the code is

| File | Holds |
|---|---|
| `project.rs`, `mode.rs`, `options.rs`, `error.rs` | The trait, `project`, `project_point`, `Projection`, `ProjectOptions`, `ProjectionError` |
| `result.rs`, `curve_result.rs` | `PointProjection`, `TargetParameters`; `CurveProjection`, chains, segments, `ChainEnd`, `Exactness` |
| `point_plane.rs`, `point_surface.rs`, `point_curve.rs`, `point_edge.rs`, `point_face.rs`, `point_face_list.rs` | Point projections, one file per target |
| `ambiguity.rs` | Where the nearest point of an analytic target is not unique |
| `plane_carrier.rs`, `curve_plane.rs` | The affine map onto a plane and the curve, edge and profile projections onto it |
| `curve_walk.rs` | A source walked span by span, and the cutting of parts into chains |
| `curve_face_clip.rs`, `curve_face_rank.rs`, `curve_face_list.rs` | Planar faces, the face that wins a part, and curves onto faces and lists |
