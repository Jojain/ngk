# Topology Orientation Refactor Plan

This document records the intended refactor for stable topology identity,
orientation-sensitive views, shared faces, and mutation safety.

## Problem

Raw darts are oriented traversal locators. They are also short-lived: topology
operations such as split, chamfer, imprint, sew, and unsew may delete or replace
them.

Keys are the durable identities users and modeling operations should store.
However, several topology views expose orientation-sensitive behavior:

- `Edge::start`, `Edge::end`, and tangents depend on edge direction.
- `Profile` traversal and tangents depend on profile direction.
- `Face::normal` depends on trim orientation relative to the support surface.
- Solid outward normals depend on how a face is used by a shell or solid.

The refactor must make these orientations explicit and stable without making
callers manage raw dart lifetimes.

## Orientation Layers

The model has three orientation layers.

### 1. Geometry Orientation

Geometry has its own intrinsic parameter direction.

- A `Curve` has a parameter direction.
- A `Surface` has a UV parameterization and a surface normal.

This layer does not know about topology ownership or traversal context.

### 2. Default Entity Orientation

Each durable topology entity has a default orientation stored through its
`XAttr` locator dart.

The locator is not just any representative dart. It is the current live dart
that preserves the entity's chosen orientation.

Examples:

- `EdgeKey -> EdgeAttr { dart, curve, ... }`
- `FaceKey -> FaceAttr { outer_loop, inner_loops, surface, pcurves, ... }`
- future `ProfileKey -> ProfileAttr { dart, ... }`

Views built from keys use this default orientation.

### 3. Use Orientation

A use is a local traversal or incidence of an entity inside another entity.
It is relative to the default orientation.

Examples:

- A profile traverses an edge forward or reversed.
- A face loop traverses an edge forward or reversed.
- A shell uses a face with the same or opposite orientation.

Uses are usually derived views, not stored identities.

## Public View Shape

Keep the public names simple. `Face` and `Edge` carry an `Orientation` instead
of introducing public `FaceUse` and `EdgeUse` types immediately.

```rust
pub enum Orientation {
    Same,
    Reversed,
}

pub struct Edge<'g, P> {
    gmap: &'g GMap<P>,
    key: EdgeKey,
    orientation: Orientation,
}

pub struct Face<'g, P> {
    gmap: &'g GMap<P>,
    key: FaceKey,
    orientation: Orientation,
}
```

Default key lookup returns `Orientation::Same`.

```rust
gmap.edge(edge_key) -> Edge { key, orientation: Same }
gmap.face(face_key) -> Face { key, orientation: Same }
```

Traversal APIs derive the appropriate orientation.

```rust
profile.edges() -> impl Iterator<Item = Edge<'g, P>>
face.outer_loop().edges() -> impl Iterator<Item = Edge<'g, P>>
shell.faces() -> impl Iterator<Item = Face<'g, P>>
```

Each view exposes:

- `key()`
- `orientation()`
- `reversed()`
- orientation-sensitive methods that apply `orientation`
- explicit geometry/default methods when useful

## Face Semantics

Do not introduce a separate attribute layer for raw GMap 2-cell orbits.
`FaceKey -> FaceAttr` is the face storage boundary.

```rust
FaceKey -> FaceAttr {
    outer_loop: Dart,
    inner_loops: Vec<Dart>,
    surface: Surface,
    pcurves: HashMap<Dart, Curve2>,
    data: P::F,
}
```

`FaceKey` identifies one oriented CAD face side. `FaceAttr.outer_loop` is the
current live oriented root of that side. `FaceAttr.surface` and
`FaceAttr.pcurves` are the support geometry and trimming data for that face.

This intentionally duplicates geometry when two sewn solids have opposite sides
of the same geometric interface. That is acceptable for the first design:

- the common CAD-facing identity is the oriented `FaceKey`;
- `gmap.face(face_key)` can always rebuild a stable oriented `Face`;
- `solid.faces()` can return plain `Face` views with the correct solid-relative
  orientation;
- there is no extra raw-2-cell key that callers or builders must keep in sync.

The raw GMap 2-cell orbit may still span both sides when two volumes are sewn
through `alpha3`. That shared orbit is a topology relation, not a stored CAD
face identity. If algorithms need to inspect the shared interface, they can do
so by traversing the GMap cell orbit or by following `alpha3` from one face side
to the opposite side.

Shared support storage can be introduced later only if duplication becomes a
real problem. It should not be part of the first orientation refactor.

## Edge Semantics

`EdgeKey` identifies a durable edge with a default direction.

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

So `gmap.edge(edge_key)` can answer `start`, `end`, and `tangent` in a stable
default orientation. When a profile or loop traverses the edge in the opposite
direction, it returns an `Edge` view with `Orientation::Reversed`.

No `EdgeUseKey` is needed initially. Add one only if edge uses need durable
identity or own persistent data.

## Profile Semantics

`Profile` can remain a transient dart-based view while profiles are only
temporary traversal results.

If profiles become durable user/modeling entities, add:

```rust
ProfileKey -> ProfileAttr {
    dart: Dart,
}
```

Then `ProfileAttr.dart` is the current live oriented root, not a canonical
representative. `gmap.profile(profile_key)` can then answer start/end/tangent in
a stable direction.

## Stored Data Versus Derived Uses

Store durable entities and their locators:

- `VertexKey -> VertexAttr`
- `EdgeKey -> EdgeAttr`
- `FaceKey -> FaceAttr`
- `SolidKey -> SolidAttr`
- optional future `ProfileKey -> ProfileAttr`

Derive uses during traversal:

- `profile.edges()` derives edge orientation from the traversed dart.
- `face.outer_loop().edges()` derives edge orientation from the loop dart.
- `shell.faces()` derives face orientation from the shell traversal dart.

Do not add `XUseKey` until a use needs durable identity or persistent data.

## Indexing Requirements

The map needs indexes that resolve keys from darts without using raw darts as
durable handles.

Existing or required indexes:

- cell representative to `VertexKey`
- cell representative to `EdgeKey`
- face-side or loop representative to `FaceKey`

Canonical representatives are useful for indexes, but must not define
user-facing orientation.

`XAttr.dart` or `FaceAttr.outer_loop` defines the default orientation.

For faces, the index must resolve a traversed side to the correct `FaceKey`
without merging the opposite side reached through `alpha3`.

## Pcurves

Pcurves live in `FaceAttr`.

The stored pcurve for a dart follows that dart's oriented boundary traversal.
When a view requests the pcurve from the opposite traversal direction, return a
reversed curve view/value.

The pcurve lookup should be centralized so callers do not manually inspect
`alpha0`/`alpha3` combinations.

## Mutation Safety

The biggest risk is stale locators. If an operation deletes the dart stored in
`EdgeAttr.dart`, `FaceAttr.outer_loop`, an inner loop, or a pcurve key, the key
can remain valid only if the locator is repaired.

Topology mutation code must not update attributes ad hoc.

Introduce a centralized topology edit/remap pipeline:

```rust
TopologyEdit {
    dart_remaps: old_dart -> new_dart,
    removed_keys,
    new_keys,
}
```

Then one internal `GMap` method applies the edit and reconciles:

- `VertexAttr.dart`
- `EdgeAttr.dart`
- `FaceAttr.outer_loop`
- `FaceAttr.inner_loops`
- future `ProfileAttr.dart`
- `SolidAttr` locators
- `FaceAttr.pcurves` keys
- `dart_to_*` indexes

The reconciliation step should validate that remapped darts still belong to the
expected cell/side and preserve the expected orientation unless the edit
explicitly says an entity was reversed.

## Refactor Steps

1. Introduce `Orientation`.

   Add a small orientation enum with helpers such as `flip`, `apply_vector`, and
   `compose`.

2. Change `Edge` view to key plus orientation.

   `GMap::edge(EdgeKey)` returns `Orientation::Same`. Traversals that currently
   create `Edge::new(gmap, dart)` should resolve `EdgeKey` and derive
   `Orientation`.

3. Make `EdgeAttr.dart` orientation-preserving.

   Stop treating it as a canonical representative for public orientation.
   Keep separate representative-based indexes for lookup.

4. Change `Face` view to key plus orientation.

   `GMap::face(FaceKey)` returns `Orientation::Same`. Shell/solid traversal
   derives `Orientation` relative to `FaceAttr.outer_loop`.

5. Collapse face geometry into `FaceAttr`.

   Keep raw 2-cell orbits as topology only. Store `surface`, `pcurves`, and
   face payload directly on `FaceAttr`.

6. Centralize pcurve orientation lookup.

   Replace direct pcurve map access in higher-level code with a method that
   returns direct or reversed pcurves according to the requested oriented dart.

7. Add side-to-face indexing.

   Ensure `GMap` can resolve a traversed face-side dart to the correct
   `FaceKey` without merging the alpha3-opposite side.

8. Introduce topology edit remapping.

   Start with a minimal remap table and a reconciliation method for locators and
   indexes. Migrate split/chamfer/imprint code to emit/apply remaps instead of
   manually mutating attrs.

9. Add invariant checks.

   Provide debug/test helpers that verify:

   - every attr locator dart is live;
   - every locator belongs to the expected cell/side;
   - `EdgeAttr.curve` direction matches `EdgeAttr.dart`;
   - `FaceAttr.outer_loop` and inner loops are closed;
   - pcurve keys resolve to valid oriented boundary darts;
   - every index can be rebuilt from attrs.

10. Decide later whether `ProfileKey` is needed.

   Keep `Profile` transient for now unless durable profile identity is needed.
   If introduced, make `ProfileAttr.dart` an orientation-preserving live locator
   and include it in the topology edit remap pipeline.

## Non-goals For The First Pass

- Do not add `EdgeUseKey`, `FaceUseKey`, or `ProfileUseKey` unless a use needs
  durable identity or own data.
- Do not add a separate raw-2-cell attribute layer unless shared support
  storage becomes a demonstrated need.
- Do not encode face holes into artificial GMap topology just to avoid
  `FaceAttr.inner_loops`.
- Do not use canonical dart representatives as default public orientation.
- Do not expose raw mutable access to orientation locator fields from builders.

## Desired End State

The stable API should feel like:

```rust
let edge = gmap.edge(edge_key)?;
edge.start();
edge.end();
edge.tangent_at(t);

let face = gmap.face(face_key)?;
face.normal_at(u, v);

for edge in profile.edges() {
    edge.tangent_at(t); // local traversal orientation
}

for face in shell.faces() {
    face.normal_at(u, v); // shell-relative orientation
}
```

Keys preserve identity. Attrs preserve live default orientation. Views carry
`Orientation` when they represent a local use. Raw darts remain internal
traversal tools, not durable user-facing handles.
