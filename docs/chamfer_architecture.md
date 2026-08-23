# Chamfer Algorithm Architecture

This document describes the current chamfer builder in
`src/builders/chamfer.rs`: its public selection model, geometry preparation,
topology mutation phases, supported cases, and intended extension points.

The chamfer operation mutates one `GMap` transactionally:

```rust
chamfer(&mut g, target, distance) -> Result<(), ChamferError>
```

It deliberately returns no topology handles. A successful call means the
local map was updated; a failure rolls the complete edit back.

## Design Goals

The implementation follows these rules:

- Public callers select domain cells with stable keys, never raw darts.
- Singular and plural selections share the same internal representation.
- Geometry is validated and prepared before destructive topology changes.
- The edit preserves a closed, manifold, outward-oriented solid when the input
  is a supported solid boundary.
- Darts remain internal traversal and sewing locators.
- The operation is atomic through `GMap::transaction`.

The key-versus-dart distinction follows the GMap model. A dart describes a
local part of every incident cell, while a vertex or edge is the complete cell
orbit containing that dart. `VertexKey` and `EdgeKey` therefore identify the
user's selection; the builder resolves appropriate darts only when it performs
an alpha-level edit.

## Public Selection Model

`ChamferTarget` has three normalized variants:

| Input form | Internal target | Meaning |
| --- | --- | --- |
| `EdgeKey` | `Edges(vec![edge])` | One solid boundary edge |
| `Vec<EdgeKey>` or `[EdgeKey; N]` | `Edges(edges)` | Several solid boundary edges |
| `VertexKey` | `Vertices(vec![vertex])` | One profile or solid vertex |
| `Vec<VertexKey>` or `[VertexKey; N]` | `Vertices(vertices)` | Several profile or solid vertices |
| `ProfileKey` | `Profile(profile)` | Every corner or boundary edge of one profile |

`VertexKey` dispatch is incidence-based:

- A vertex with no incident face is treated as a standalone 2D profile corner.
- A vertex with incident faces is treated as a solid-boundary vertex.

`ProfileKey` is kept separate because a solid face profile needs a batched rim
operation. Expanding it into sequential edge calls would invalidate adjacent
edge keys after the first mutation.

## Global Pipeline

```text
stable cell selection
        |
        v
normalize singular keys to vectors
        |
        v
start GMap transaction and validate distance
        |
        v
dispatch by target and incidence
        |
        v
prepare geometry and capture affected topology
        |
        v
split supporting faces with synchronized 3D/UV imprints
        |
        v
detach and remove the selected face patch
        |
        v
construct replacement profile(s) and face(s)
        |
        v
alpha1/alpha2-sew replacement boundaries
        |
        v
reconcile cell identities and commit, or roll back on error
```

The prepare/mutate separation is especially important for adjacent cells.
Preparation reads all original incidences and computes all offsets before any
split can invalidate them.

## Shared Building Blocks

### Face imprints

`FaceImprint` stores synchronized model-space and face-parameter-space curves.
`split_face_by_imprints_staged` uses the pcurve to split the face domain while
retaining exact 3D boundary geometry.

Two imprint paths currently exist:

- Planar lines use a 3D line and its exact line in plane UV coordinates.
- Translated NURBS boundaries use exact planar projection or parameters
  recovered on a ruled surface and checked at sample points.

### Patch splitting

`split_chamfer_face` divides one face and returns:

- The patch containing the selected source edge or vertex, which must be
  removed.
- The section edge on the surviving face, which becomes a boundary of the new
  chamfer face.

### Patch removal

`remove_face_patch` treats the affected faces as one connected set of darts. It:

1. Finds survivor-side darts across the section edges.
2. Identifies edges, vertices, and profiles whose complete cell orbits are
   contained in the removed patch.
3. Alpha2-unlinks the patch from surviving faces.
4. Repairs section-cell, sheet, and solid locator darts before deletion.
5. Removes obsolete attributes and isolated darts.
6. Returns remapped survivor boundary darts for replacement sewing.

A lower-dimensional cell is removed only when its full orbit belongs to the
patch. Section edges and their endpoint vertices keep their domain identity.

### Replacement sewing

Replacement polygons are created as independent profiles and faces. Boundary
darts are matched by geometric endpoints within `LINEAR_TOLERANCE`, oriented to
the same start point, and alpha2-sewn into the survivor shell.

## Standalone 2D Vertex Chamfer

The 2D path accepts a `VertexKey` and internally resolves one dart in its
0-cell orbit. The two alpha1-linked corner occurrences are ordered as the end
of the incoming edge and the start of the outgoing edge.

The operation then:

1. Validates that both incident profile edges are straight.
2. Computes one offset point along each incident edge.
3. Alpha1-unlinks the original corner, splitting its 0-cell into two vertices.
4. Moves the two resulting vertex attributes to the offset points.
5. Rebuilds the shortened line geometry of both source edges.
6. Inserts a new line edge and alpha1-sews it between the offset vertices.

A standalone `ProfileKey` collects all eligible `VertexKey` values and applies
this corner operation to each one. Open-profile endpoints are excluded.

## Solid Edge Chamfer

A supported solid edge must have exactly two incident faces. Its two endpoints
must each contribute one additional planar endpoint face.

Preparation computes a four-sided replacement:

- Two trim boundaries on the faces incident to the selected edge.
- Two trim boundaries on the endpoint faces.

The four surrounding faces are split, the patches touching the selected edge
or endpoint vertices are removed, and one replacement quadrilateral is sewn to
the four surviving section edges.

For a straight edge, the replacement is planar. For a supported NURBS edge,
the two long boundaries are translated copies of the source curve and the
replacement support is a ruled surface.

## Solid Vertex Chamfer

The current solid-vertex path supports a manifold trihedral corner:

- Three incident straight edges.
- Three incident planar faces.

One offset point is computed on each edge. Each incident face is split between
its two offset points. The three corner patches are removed and an
outward-oriented triangular face is sewn to the three survivor section edges.

## Complete Solid Profile Chamfer

The solid-profile path handles an entire planar cap rim as one operation. The
current first-pass domain is:

- A closed, convex, straight-edged outer loop.
- One planar target face with no inner loops.
- One planar side face per profile edge.
- One non-profile solid edge leaving every rim vertex.

Preparation captures all profile edges, side faces, outward normals, lower
side-wall offsets, and inset cap corners before mutation.

The cap inset is computed in the target plane's UV space by intersecting
consecutive inward-offset support lines. The result is rejected if it becomes
degenerate, inverted, or non-convex.

Mutation proceeds as follows:

1. Split each side face along the lower bevel boundary.
2. Treat the original cap and all upper side strips as one patch.
3. Remove that complete patch.
4. Add the inset cap.
5. Add one bevel quadrilateral per original profile edge.
6. Sew each bevel to its lower side-face survivor and to the inset cap.
7. Sew neighboring bevels along their diagonal corner edges.

This batched structure is what makes adjacent rim edges safe. Sequential edge
chamfers cannot preserve the original neighboring selection after the first
edge is replaced.

## Current Geometry Support

| Selection | Supported geometry | Replacement |
| --- | --- | --- |
| Standalone vertex | Two straight profile edges | Straight edge |
| Standalone profile | Straight open or closed profile | One straight edge per eligible corner |
| Solid edge | Straight edge with planar surrounding faces | Planar quadrilateral |
| Extruded NURBS solid edge | NURBS edge with planar/ruled incident faces and planar endpoint faces | Ruled quadrilateral |
| Solid vertex | Straight trihedral corner with planar faces | Planar triangle |
| Solid profile | Convex straight planar outer rim | Inset planar cap plus planar bevel ring |

Unsupported geometry returns `ChamferError` and leaves the source map
unchanged.

## Multiple Selections

Singular and plural inputs are folded into the same vector-backed target, but
that does not make every selection simultaneously batchable.

- Disjoint `Edges` are currently applied sequentially inside one transaction.
- `Vertices` are currently applied sequentially inside one transaction.
- A solid `Profile` has a dedicated simultaneous algorithm because all of its
  edges are adjacent.

A future generalized selection planner should group connected selected cells,
prepare each connected component against the original topology, resolve shared
setbacks at selected vertices, and then execute each component as one patch
replacement.

## Transaction and Identity Guarantees

The entire call executes inside `GMap::transaction`. Any validation, split,
sew, or reconciliation failure restores the original map.

The edit preserves these intended invariants:

- Public selections use stable cell keys.
- Darts are temporary traversal and sewing locators.
- Surviving section cells retain their keys where reconciliation permits.
- Removed cells do not leave stale locator darts.
- Sheet and solid roots point to survivor darts before patch compaction.
- Replacement faces are sewn into the same shell.
- Solid face normals are oriented outward.

## NURBS Extension Direction

Extending a complete profile chamfer to curved boundaries should preserve the
same topology pipeline. The main new work belongs in geometry preparation:

1. Offset each boundary curve in the target surface parameter space.
2. Intersect consecutive offset curves to compute exact inset corners or trim
   parameters.
3. Build corresponding lower boundaries on adjacent surfaces.
4. Construct blend surfaces between the inset and lower curves.
5. Feed those curves and surfaces into the existing split/remove/sew phases.

The topology algorithm therefore does not need to be completely replaced for
NURBS. Its preparation data must become curve- and surface-based instead of
assuming line segments and planar quadrilaterals.

## Architectural Follow-ups

`remove_face_patch` currently performs low-level attribute-locator repair in
the builder. That behavior belongs in a reusable typed topology-edit primitive
so future fillet, shell, draft, and boolean operations can replace connected
face patches without duplicating raw GMap bookkeeping.

Useful future abstractions are:

- `PreparedCellSelection` for immutable incidence capture.
- `FacePatch` and `SurvivorBoundary` typed views.
- A topology-layer `remove_patch` operation that owns locator repair and dart
  remapping.
- Geometry-specific chamfer planners that produce a common replacement-patch
  description.
- A generalized connected-selection planner for adjacent edges and vertices.

## Code Map

- `src/builders/chamfer.rs`: selection dispatch and all current algorithms.
- `src/builders/faces.rs`: face imprinting and splitting.
- `src/builders/profiles.rs`: profile construction and planar pcurves.
- `src/topology/edit.rs`: transactional edit and identity reconciliation.
- `tests/builders/chamfer.rs`: core behavior and topology invariants.
- `src/scripts/chamfered_rectangle.rs`: 2D profile-versus-vertex example.
- `src/scripts/chamfered_block.rs`: 3D profile-versus-vertex example.
- `src/scripts/chamfered_wavy_edge.rs`: NURBS solid-edge example.
