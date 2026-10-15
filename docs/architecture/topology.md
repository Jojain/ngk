# Topology

The map, the entities read off it, how scaffold is embedded, and how identity
and orientation are carried. Terms are defined in
[CONTEXT.md](../../CONTEXT.md); this document says how the code realizes them.

## Map and entities

- **`Dart`**: oriented traversal locator. Short-lived; may be destroyed by any
  edit.
- **`Dim::{Zero, One, Two, Three}`** ↔ α0..α3 ↔ vertex / edge / face /
  (sheet | solid). `GMAP_INVOLUTION_COUNT = 4`.
- **Keys are the durable identity**: `VertexKey`, `EdgeKey`, `ProfileKey`,
  `FaceKey`, `SheetKey`, `SolidKey` (slotmap `new_key_type!`). Public APIs
  select entities by key, never by dart.
- **Attributes** (`topology/attributes.rs`) store geometry and user payload per
  entity: `VertexAttr`, `EdgeAttr`, `FaceAttr` (surface, boundary, pcurves),
  `ProfileAttr`, `SheetAttr`, `SolidAttr`.
- **Typed views**: `Vertex`, `Edge`, `Face`, `Profile`, `Sheet`, `Solid`, plus
  `Shape<K, P>` (owned model + primary key, read with `model()` /
  `model_mut()` / `into_model()`). Traverse with these, not raw darts.
- **`Payload` trait**: a type-level bundle of user data per dimension, bounded
  only by `Clone + 'static`, plus `type Policy: EditPolicy<Self> + Default`, the
  rule that maintains it ([transactions.md](transactions.md)).
  `StandardPayload` is `()` everywhere with `Policy = PreservePayload`.
  Everything is generic over `P: Payload`.
- **Profiles and sheets are registered explicitly** (`add_profile` /
  `add_sheet`); commit rejects faces or solids referencing unregistered
  components.

Identifiers follow the vocabulary rule: `Model::cells(Dim::Two)` and
`cell_representative` are map-side, `iter_faces` and `face_attr` are
logical-side, and `Dim` never appears in a logical signature.

## Corners and edges

An edge's kind is decided by how many distinct corners it has, not by the shape
of its curve.

| Kind | Corners | `Edge` variant | Corner access |
|---|---|---|---|
| bounded | 2 distinct | `Edge::Bounded(BoundedEdge)` | `BoundedEdge::vertices()` returns two |
| marked | 1, both start and end | `Edge::Marked(MarkedEdge)` | `MarkedEdge::corner()` returns one |
| unmarked | 0 | `Edge::Unmarked(UnmarkedEdge)` | none to ask for |

Corner access is **total on each type**: no `Option` to unwrap, nothing to
forget.

Marked and unmarked are both closed, but the enum is flat rather than nested on
purpose: a site that must tell them apart cannot write one pattern that silently
covers both. A site that genuinely means "closed" writes
`Edge::Marked(_) | Edge::Unmarked(_)`, which is still exhaustive-checked, or
asks `Closeable::is_closed()`. `vertices_at_dart` folds `start == end` to
`None`, so a marked edge is never `Bounded`.

An unmarked edge's closing point is a 0-cell embedded in the edge, and
`point_at_dart` derives its position from the curve. **Ask `point_at_dart` for a
position; ask the vertex store only when the identity of a vertex matters.**

| Count | Where |
|---|---|
| distinct corners | `BoundedEdge::vertices`, `MarkedEdge::corner`, `EdgeCore::has_corner_at` |
| corner visits | `Loop::corners`, `LoopCorner` |

`LoopCorner` pairs the dart arriving with the one leaving, which is what tells
two visits to one vertex apart; asking it for a `vertex()` collapses them again.
One dimension up needs no such machinery: a loop's darts are its oriented edges,
one for one, so an edge a loop runs twice (a seam) appears twice in
`Loop::darts()`.

### Cutting

| Cut a… | You get |
|---|---|
| bounded edge | two bounded edges |
| unmarked edge | one marked edge: the 0-cell is materialized where the cut was asked for, and no edge is created |
| marked edge | two bounded edges |

Marking is a pure relabel: an unmarked edge already has its two darts and its
one 0-cell orbit, so materializing costs no darts and no links. Only the
vertex's stored point says where it now sits, which is why a marked edge spans
`[t, t + period]` from its corner rather than from the curve's own domain start.

### Hazards that have already bitten

- **Predicates over corners.** "Every vertex satisfies P" is vacuously true of
  an edge or face that has none.
- **A span's ends are not always the edge's ends.** Ask the corner, not the
  parameterization:
  - *The pcurve's start.* A pcurve of a closed edge is a closed loop in
    parameter space, anchored wherever it was built; a marked edge begins at
    its corner, elsewhere on that loop. `pcurve.point_at(0.0)` is a boundary
    corner only for a bounded edge.
  - *The parameter domain's ends.* "Too close to `domain.start`, so this cut is
    degenerate" is right for a bounded or marked edge. An unmarked edge's span
    is its whole support, and its ends are where the curve closes: a place to
    put a corner like any other. Refusing there silently drops a junction.

### Homonyms, deliberate and safe

`BBox::corners`, `UnwrappedFaceDomain::corners` and `snap_boundary_corner` mean
points, not topology; they return `Point3` / `Point2`, never a view.
`exchange::step::topology::export::Corner` is a corner of the exported file: it
is `Vertex(VertexKey)` or `Closure(EdgeKey)`, because `EDGE_CURVE` has no
spelling without two ends. That type is private to the exporter.

## Boundaries: profile, loop, sheet, shell

**Connected means connected through logical cells only.** A profile is the
`alpha0`/`alpha1` walk and a sheet the `alpha0`/`alpha1`/`alpha2` walk, and when
a step lands on a scaffold cell the walk **turns across it** and carries on:

- a profile turns across a raw edge the face owns: a bridge, a periodic seam;
- a sheet turns across a raw face the solid owns: a cut face inside a cavity.

Turning is local. At dimension `n`, a step onto a scaffold `n`-cell is crossed
with `alpha(n + 1)` and the walk resumes. `turn` (`topology/embedding/walk.rs`)
is that primitive at every dimension: `turn(map, index, Dim::One, dart)` walks a
profile past a bridge edge around the 0-cell they share, and
`turn(map, index, Dim::Two, dart)` walks a sheet past a cut face around the
1-cell they share. Nothing else re-derives it.

**Turning across is not stopping at.** An annulus's inner rim is an unmarked
circle whose closure runs through the bridge: `alpha1` of the rim's far end is a
bridge dart. Stop there and a closed circle reads as an open one-edge chain.
Turn across it and the walk comes back onto the same rim, which is what makes
the annulus's two rims two profiles. A raw `alpha0`/`alpha1` component of a
bridged face runs through every loop the face has, so "which loop is this edge
on" has an answer only on the logical walk.

**A loop is a walk over a profile.** A profile says which edges. A loop says how
a face runs along them:

- *Sense.* An edge has up to four darts; a face sees two. Which two is a
  question about the face's region, so a standalone wire has no side to pick.
- *Repetition.* A face can run along one edge twice (a slit, or a marked edge
  passed on both sides). The profile holds that edge once; the loop emits it
  twice.

## Embedding (`src/topology/embedding/`)

A raw cell carrying no entity of its own dimension is **embedded** in the
entity of higher dimension whose interior contains it. Records are one
`EmbeddedCell` per orbit, anchored at a representative dart, and every one names
an owner of **strictly greater** dimension than the cell, so "has a record" and
"is embedded" are one question.

An entity's own cell never appears in a record: it is read off the entity's
anchor, so a shape with nothing embedded stores no records and is still fully
classified. `recover_region` walks an entity out of the map from that anchor.
`boundary_cycles` gives a face its oriented loops by turning across the cells it
owns. `boundary_shells` does the same one dimension up, though `Solid::shells`
still reads the shell roots stored on `SolidAttr` rather than calling it.

### One entity, one cell of its own dimension

Each vertex, edge, face and solid occupies **exactly one** raw cell of its own
dimension. The converse does not hold: a raw cell need carry no entity, and any
number of lower-dimensional cells may be embedded in one entity. Profiles and
sheets are aggregates, so the rule does not reach them.

This is a **commit invariant**, checked by
`validation::validate_cell_occupancy`, not a construction rule: a transaction
may break it freely (a Boolean partitions a face and puts it back together), but
a commit that leaves an entity spanning two cells of its own dimension, or none,
is rejected.

What follows from it:

- **One key, one orbit.** An entity's anchor identifies its whole
  same-dimensional topology, so nothing collects an entity's cells and nothing
  floods.
- **Records only classify embedded cells**, which are lower-dimensional by
  definition, and the record type refuses any other.
- **A raw cell between two raw cells of the dimension above is real topology.**
  A 1-cell with a different 2-cell on each side is either an edge between two
  faces, or it must be removed so the two raw faces merge. A bridge has only one
  incident 2-cell, so removing it merges nothing: that is what makes it interior.
- **A raw split forces a logical split; a raw merge forces a logical merge.**
  Commit checks the correspondence, so a partition cannot hide beneath one key.

What it costs, each required before commit:

- a face with holes needs one bridge edge per hole;
- a cylindrical face needs its seam;
- a whole sphere needs a seam/cut representation rather than several quads;
- a whole torus needs two cuts;
- a solid with a cavity needs a scaffold face joining its outer and inner
  shells, so that it is one raw 3-cell;
- STEP import must synthesize these before committing, and cannot keep one
  `ADVANCED_FACE` as several disconnected raw faces.

The rule turns a missing bridge into a named error at commit instead of a face
that silently reports half its boundary.

## Identity and orientation

```text
identity               = the entity's key
default orientation    = the anchor stored in its attribute
contextual orientation = carried by the view
```

An entity keeps one key even when traversals observe it with opposite
orientations. Oriented uses are derived during traversal and get no key of their
own.

Three orientation layers:

1. **Geometry**: a curve's parameter direction; a surface's (u, v) and its
   support normal. Knows nothing about topology.
2. **Default entity orientation**: the live anchor in the entity's attribute. A
   view built from a key starts there. `EdgeAttr`'s curve parameter direction
   follows its anchor dart.
3. **Contextual orientation**: a use of an entity inside another (a loop
   running a shared edge backwards, two shells reaching one shared face from
   opposite sides). The view carries it: `EdgeCore`, `Profile`, `Sheet` and
   `Solid` hold the dart actually reached; `Face` holds a `sense:
   Orientation` relative to its default.

`reversed()` keeps the key and flips the context. `Orientation::{Same,
Reversed}` is derived where an operation needs to compare a use with the
default: `Model::edge_orientation_at_dart`, `face_orientation_at_dart`, both
over `cell_orientation_from_seed`.

**A face shared by two volumes is one face.** When two volumes are 3-sewn,
`alpha3` identifies their boundary 2-cells, so the interface has one `FaceKey`,
one `FaceAttr`, one surface and one set of pcurves. Each shell's traversal
yields a `Face` view with the same key and the sense its side sees, so the
normal comes out outward from either solid. Payload on `FaceAttr` therefore
belongs to the shared face, not to one side; per-side data would need explicit
incidence data, not a second key. Sewing two faces that already have keys
reconciles them to one survivor at commit
([transactions.md](transactions.md#identity-reconciliation)).

**Pcurves live in the shared `FaceAttr`**, keyed by oriented boundary dart.
`Face::pcurve(dart)` returns the curve for the orientation asked; callers never
inspect `alpha0` / `alpha2` / `alpha3` combinations to orient one.

### Deliberately not done

- No `EdgeUseKey`, `FaceUseKey` or `ProfileUseKey` until a use needs durable
  identity or persistent data.
- No separate `FaceKey` for the two sides of an `alpha3`-shared face.
- No `Facet` or raw-2-cell attribute layer: `FaceAttr` is the storage boundary
  for a face's geometry and payload.
- Canonical cell representatives define identity for indexing, never public
  orientation.
- Builders get no raw mutable access to anchor fields; anchors are repaired by
  the transaction's reconciliation.
