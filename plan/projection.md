# Projection

A unified public API for projecting points and curves onto geometry and model
entities. The first caller is Nales, which projects points and curves into a
sketch plane to use them as references; other callers project profiles onto
faces to imprint or cut them. [ADR 0006](../docs/adr/0006-projection-is-a-query-and-a-face-exists-only-where-trimmed.md)
records the two decisions that shape everything below.

## Shape of the API

Projection lives in a new top-level module, `src/projection/`, next to
`measure/`: it reads geometry and the model (faces, edges, adjacency) and writes
neither. It is a pure query and opens no transaction.

- **One entry point**, dispatched by a trait on the (source, target) pair. The
  trait carries two associated types:
  - `Mode`: the `Projection` enum (`Normal | Along(direction)`) for
    two-dimensional targets, `()` for curve and edge targets, which only have a
    normal mode. Asking for a directional projection onto a curve cannot be
    written.
  - `Hit`: what the target reports as touched — `()` for pure geometry
    (`Plane`, surface, curve), `FaceKey` for a face or a list of faces, `EdgeKey`
    for an edge.
- **Rich results, thin helpers.** The base call returns everything the
  algorithm computed on the way that a caller may use. It computes nothing
  extra just to return it: what a caller can recompute from the result (for
  example `(u, v)` on a plane from the 3D point) is left out. Helpers narrow the
  result to one specialised type (just the point, just the curves) so that a
  caller does not handle the full structure everywhere.
- **Python**: `ngk.projection.project(source, target, along=None, ...)`;
  `along` on a curve target raises.

### Sources and targets

| Source                    | Plane | Surface | Face | List of faces | Curve | Edge |
| ------------------------- | ----- | ------- | ---- | ------------- | ----- | ---- |
| Point                     | N, D  | N, D    | N, D | N, D          | N     | N    |
| Curve, `Edge`, `Profile`  | N, D  | N, D    | N, D | N, D          | —     | —    |

N is normal, D is directional (along a fixed direction). Curve onto curve is
not offered in any mode: in 3D a ray and a curve meet only by coincidence, and
"where A crosses B seen along d" composes from projecting both into a plane and
intersecting in 2D.

A `Profile` source projects edge by edge; its vertices give the connectivity, so
a branching profile needs no special case. Projecting a region (a face, a
filled profile) is not a projection: it is an imprint builder consuming the
projected boundary.

The list-of-faces target is any list of `FaceKey`; a `Sheet`, a solid and an
explicit selection all produce one.

## Semantics

**A face exists only where it is trimmed.** What lands outside a face's trim —
in a hole or past its boundary — is a `Miss`, in both modes.

- **Directional**: the ray is the half-line `P + t·d`, `t ≥ 0`, and stops at the
  first wall it hits. A hit on the support outside the trim is not a wall; the
  ray continues. A point already behind the wall casts no shadow. Only the
  target's faces cast shadows; the rest of the model is ignored.
- **Normal**: the foot of the perpendicular on the nearest support. Outside the
  trim, `Miss`. On a list of faces, each face's nearest foot is kept if inside
  its trim, then the nearest of those. On an `Edge`, the foot on the support
  curve must fall within the edge's bounds.
- **Refusals**, each a named error: `Miss`; `Ambiguous` when several feet are
  equally near within tolerance (a sphere's centre, a point on a cylinder's
  axis, a point above the centre of a circular hole); a grazing ray, or a ray
  lying in the surface.

The closest point *on a trimmed face* — which would land on its boundary — is a
distance query, not a projection. It belongs to `measure` once a caller needs
it.

## Results

A point projection returns the projected point, the parameters on the target
(`t` or `(u, v)`) when computed, the distance, and the `Hit`.

A curve projection never glues its pieces into one curve:

```rust
struct CurveProjection<Hit> { chains: Vec<ProjectedChain<Hit>> }

struct ProjectedChain<Hit> {
    segments: Vec<ProjectedSegment<Hit>>,
    start: ChainEnd,
    end: ChainEnd,           // closed() is derived from the ends
}

struct ProjectedSegment<Hit> {
    curve: Curve3,            // on the target
    pcurve: Curve2,           // computed on the way, so returned
    hit: Hit,
    source: (EdgeKey or curve, Interval),
    fit: Exactness,           // Exact | Approx { error }
}

enum ChainEnd { Closed, SourceEnd, SourceVertex(VertexKey), Miss, Jump }
```

Chains are stored, not derived afterwards: the algorithm knows a crossing is
continuous when it computes it, and deriving it later would redo that work with
tolerance comparisons.

A chain continues while the shadow passes from one face to an adjacent face of
the target through their shared edge. It breaks when the shadow misses the
target, when two faces are not sewn (even across a gap within tolerance), or
when the hit jumps from a near wall to a farther one at a silhouette (or, in
normal mode, in a concave region). Continuity comes from topology, never from a
geometric coincidence.

`ChainEnd` says why a chain stops. Three chains of a T-shaped profile end on
the same `SourceVertex`, which tells the conversion builder to join them there.

### Exactness

The result is exact when it has an analytic form (a line onto a plane, a circle
onto a parallel plane). Otherwise it is a NURBS fit to a tolerance passed in the
options, defaulting to the kernel's tolerance, and the segment reports the error
reached. A fit that does not reach the tolerance is an error, never a degraded
result.

## From projection to profiles

`add_profiles_from_projection(&projection)` is a builder, following the
`add_profiles_from_system` pattern: it turns a projection result into `Profile`s
in the model, with edges already carrying their pcurves on the faces hit, ready
for an imprint. It joins chains where `ChainEnd::SourceVertex` says so.

## Phase 1

- All point projections in the table above.
- Curve, `Edge` and `Profile` onto a plane, both modes.
- Curve onto a list of faces, both modes, accepted on the two-face case: a
  circle projected onto a sheet made of a flat face and a face inclined by 20°
  sharing an edge gives one closed chain of two segments, joined on the shared
  edge.
- `add_profiles_from_projection`.
- Tests in Rust first; Python integration tests where useful.

## Later

- Curve onto surfaces and faces in the general case beyond what phase 1 needs.
- Directional projection onto 2D curves inside a sketch plane, if a caller needs
  it.
- Closest point on a trimmed face, in `measure`, if a caller needs it.

## Open

- **Default fit tolerance.** `LINEAR_TOLERANCE` is `1e-9`, which may be too tight
  for a NURBS fit to reach on general curves. Check what a fit actually achieves
  before settling the default.
- **`docs/architecture/projection.md`** is written with the code, not before.
