# Topology Identity and Orientation Design

This document records the adopted design for stable topology identity,
orientation-sensitive views, shared cells, and mutation safety.

The central rule is:

```text
identity = XKey
default orientation = reference dart stored in XAttr
contextual orientation = dart carried by the topology view
```

An entity keeps one durable key even when different traversals observe it with
opposite orientations. Oriented uses are derived from darts and do not receive
their own durable keys unless they eventually need persistent data.

## Problem

Raw darts are oriented traversal locators. They are also short-lived: topology
operations such as split, chamfer, imprint, sew, and unsew may delete or replace
them.

Keys are the durable identities users and modeling operations should store.
However, several topology views expose orientation-sensitive behavior:

- `Edge::start`, `Edge::end`, and tangents depend on edge direction.
- `Profile` traversal and tangents depend on profile direction.
- `Face::normal_at` depends on trim orientation relative to the support surface.
- Solid outward normals depend on how a shared face is reached from a shell.

The API must preserve stable identity while carrying the exact local traversal
context internally.

## Orientation Layers

The model has three orientation layers.

### 1. Geometry Orientation

Geometry has its own intrinsic parameter direction.

- A `Curve` has a parameter direction.
- A `Surface` has a UV parameterization and a support-surface normal.

This layer does not know about topology ownership or traversal context.

### 2. Default Entity Orientation

Each durable topology entity has a default orientation stored through a live
locator dart in its attribute.

Examples:

- `EdgeKey -> EdgeAttr { dart, curve, ... }`
- `ProfileKey -> ProfileAttr { dart, ... }`
- `FaceKey -> FaceAttr { outer_loop, inner_loops, surface, pcurves, ... }`
- `SheetKey -> SheetAttr { dart, ... }`

The locator is not an arbitrary canonical representative. It is a live dart
that preserves the entity's chosen default orientation. A view built from a key
starts from that reference orientation.

### 3. Contextual Use Orientation

A use is a local traversal or incidence of an entity inside another entity.
Its orientation is relative to the entity's default locator.

Examples:

- A profile traverses a shared edge forward or reversed.
- A face loop traverses a shared edge forward or reversed.
- Two shells reach the same shared face from opposite volumes.

Uses are normally transient views. The traversed dart is enough to preserve the
context, so there is no need to store a separate `Orientation` field or create
`EdgeUseKey` and `FaceUseKey` identities.

## Topology Views

`Edge`, `Face`, `Profile`, and `Sheet` combine a stable key with a contextual
dart. The essential shape is:

```rust
pub struct Edge<'g, P> {
    gmap: &'g GMap<P>,
    key: EdgeKey,
    dart: Dart,
}

pub struct Face<'g, P> {
    gmap: &'g GMap<P>,
    key: FaceKey,
    dart: Dart,
}
```

Construction from a key uses the attribute's reference dart:

```text
gmap.edge(edge_key) -> Edge { key, dart: EdgeAttr.dart }
gmap.face(face_key) -> Face { key, dart: FaceAttr.outer_loop }
```

Construction during traversal preserves the dart that was actually reached:

```text
Edge::from_dart(gmap, dart) -> Edge { resolved key, dart }
Face::from_dart(gmap, dart) -> Face { resolved key, dart }
```

`reversed()` keeps the same key and replaces the contextual dart with
`alpha0(dart)`. Orientation-sensitive operations derive `Same` or `Reversed`
from the contextual dart only when they need it.

Traversal APIs therefore return correctly oriented views without exposing an
extra public use type:

```rust
profile.edges() -> Vec<Edge<'g, P>>
face.edges() -> Vec<Edge<'g, P>>
sheet.faces() -> Vec<Face<'g, P>>
```

## Shared Face Semantics

`FaceKey` identifies one complete topological 2-cell.

When two volumes are 3-sewn, `alpha3` identifies their two initial boundary
faces as one shared 2-cell. The resulting interface therefore has one
`FaceKey`, one `FaceAttr`, and one set of support and trimming data.

```text
volume A -- contextual dart d ------\
                                      FaceKey F -> FaceAttr
volume B -- contextual dart alpha3(d) /
```

The two volumes do not need different face identities. Their shell traversals
produce `Face` views with the same key and contextual darts that resolve to the
appropriate relative orientation. Consequently, the face normal is outward or
inward according to the traversal without duplicating the face entity.

`FaceAttr.outer_loop` defines the shared face's default orientation. It does
not identify a volume-specific side.

```rust
FaceKey -> FaceAttr {
    outer_loop: Dart,
    inner_loops: Vec<Dart>,
    surface: Surface,
    pcurves: HashMap<Dart, Curve2>,
    data: P::F,
}
```

This design intentionally means that persistent data attached to `FaceAttr`
belongs to the shared face, not to one face-volume incidence. If persistent
per-volume-side data becomes necessary, it should be modeled as explicit
incidence/use data rather than by duplicating `FaceKey`.

There is no separate `Facet` or raw-2-cell attribute layer. `FaceAttr` is the
storage boundary for the face's topology-facing geometry and payload.

### Sewing Faces That Already Have Keys

Before a 3-sew, the two independent boundary faces may each have their own
`FaceKey`. Once sewing identifies them as one 2-cell, topology reconciliation
must select one surviving key, merge payloads according to the edit policy, and
remove the consumed key. Rebuilding the face index must never leave two keys
for the same 2-cell.

## Edge Semantics

`EdgeKey` identifies one durable 1-cell with a default direction.

```rust
EdgeKey -> EdgeAttr {
    dart: Dart,
    curve: Curve,
    data: P::E,
}
```

The intended invariant is:

```text
EdgeAttr.curve parameter direction follows EdgeAttr.dart.
```

`gmap.edge(edge_key)` can therefore answer `start`, `end`, and tangent queries
in a stable default orientation. When a profile or face loop reaches the edge
in the opposite direction, `Edge::from_dart` returns the same `EdgeKey` with the
opposite contextual dart. `start` and `end` then naturally swap.

No `EdgeUseKey` is needed unless an edge use eventually owns persistent data.

## Profile and Sheet Semantics

Profiles and sheets are durable keyed entities with oriented reference darts:

```rust
ProfileKey -> ProfileAttr { dart, data }
SheetKey -> SheetAttr { dart, data }
```

Views created from keys start at these reference darts. Views created during a
traversal keep the reached darts. Reversing a profile or sheet traversal changes
the orientation of the returned edge or face views without changing their
keys.

## Orientation Derivation

`Orientation::{Same, Reversed}` remains a useful derived value, but it is not
stored in topology views.

`GMap::cell_orientation_from_seed` compares a contextual dart with a reference
dart inside the same cell orbit. Moving through a lower-dimensional involution
reverses the cell orientation; moving through a higher-dimensional incidence
preserves its intrinsic orientation.

This lets typed views retain the exact traversal dart while centralizing the
rules used by `edge_orientation_at_dart` and `face_orientation_at_dart`.

## Stored Data Versus Derived Uses

Store durable entities and their default locators:

- `VertexKey -> VertexAttr`
- `EdgeKey -> EdgeAttr`
- `ProfileKey -> ProfileAttr`
- `FaceKey -> FaceAttr`
- `SheetKey -> SheetAttr`
- `SolidKey -> SolidAttr`

Derive oriented uses during traversal:

- `profile.edges()` preserves each traversed edge dart.
- `face.edges()` preserves the oriented boundary traversal.
- `sheet.faces()` preserves each face's sheet-relative traversal dart.
- solid shell traversal delegates to an oriented `Sheet`/`ShellRef` view.

Do not add `XUseKey` until a use needs durable identity or persistent data.

## Indexing Requirements

Indexes resolve keys from darts without using raw darts as durable public
handles.

- canonical 0-cell representative to `VertexKey`
- canonical 1-cell representative to `EdgeKey`
- profile representative to `ProfileKey`
- canonical 2-cell representative to `FaceKey`
- canonical 3-cell representative to `SheetKey`

Canonical representatives define cell identity for indexing. They do not
define public orientation: the live locators stored in attributes do that.

For faces, the canonical 2-cell orbit includes `alpha3`. Both volume-side darts
of a sewn interface must therefore resolve to the same `FaceKey`. Index rebuild
must reject duplicate keys for a single resulting cell unless an explicit merge
event selects the survivor.

## Pcurves

Pcurves live in the shared `FaceAttr`.

The stored pcurve for a boundary dart follows that dart's oriented boundary
traversal. When a face view requests the pcurve from the opposite orientation,
the centralized lookup returns the reversed curve value.

Callers should not manually inspect `alpha0`, `alpha2`, or `alpha3` combinations
to orient pcurves.

## Mutation Safety

If an operation deletes a dart stored in `VertexAttr.dart`, `EdgeAttr.dart`,
`ProfileAttr.dart`, `FaceAttr.outer_loop`, `FaceAttr.inner_loops`,
`SheetAttr.dart`, `SolidAttr`, or a pcurve key, the durable key can remain valid
only if topology reconciliation repairs the locator.

Topology mutation therefore goes through the centralized edit pipeline. It
reconciles live locators, semantic split/merge events, payload policy, and all
`dart_to_*` indexes before validating and committing the result.

The reconciliation step must verify that remapped darts still belong to the
expected cell and preserve the intended default orientation unless the edit
explicitly reverses the entity.

## Required Invariants and Tests

- Every attribute locator dart is live.
- Every locator belongs to the cell identified by its key.
- Every topological cell has at most one key of its dimension.
- Both darts related across `alpha3` on a sewn face resolve to the same
  `FaceKey`.
- Traversing that shared face from opposite volume contexts produces opposite
  relative orientations and normals.
- `EdgeAttr.curve` direction matches `EdgeAttr.dart`.
- `FaceAttr.outer_loop` and inner loops are closed.
- Pcurve keys resolve to valid oriented boundary darts.
- Every index can be rebuilt from attributes after topology edits.

## Non-goals

- Do not add `EdgeUseKey`, `FaceUseKey`, or `ProfileUseKey` unless a use needs
  durable identity or persistent data.
- Do not create separate `FaceKey` values for the two volume-side uses of one
  `alpha3`-shared face.
- Do not add a separate `Facet` or raw-2-cell attribute layer.
- Do not encode face holes into artificial GMap topology just to avoid
  `FaceAttr.inner_loops`.
- Do not use canonical representatives as default public orientation.
- Do not expose raw mutable access to orientation locator fields from builders.

## API Examples

Default views use the stored orientation:

```rust
let edge = gmap.edge(edge_key)?;
edge.start();
edge.end();

let face = gmap.face(face_key)?;
face.normal_at(u, v);
```

Traversal returns contextual views:

```rust
for edge in profile.edges() {
    edge.start(); // profile-relative start
    edge.end();   // profile-relative end
}

for face in shell.faces() {
    face.normal_at(u, v); // shell-relative normal, shared FaceKey
}
```

Reversing changes the view, not the identity:

```rust
let reversed = face.reversed();
assert_eq!(face.key(), reversed.key());
```

Keys preserve identity. Attributes preserve live default orientation. Views
carry contextual darts. `Orientation` is derived only when an operation needs
to compare a contextual use with the default orientation.
