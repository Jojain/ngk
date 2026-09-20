# AGENTS.md — NGK (Nales Geometry Kernel)

Working memory for agents (Claude Code, Codex, and others) in `D:\Projets\ngk`.
Read this first, then the pointed-to docs.

Shared skills live in `skills/` at the repo root — `.claude/skills` is a local
directory junction onto it, so skill content is edited in one place only.
The junction is machine-local (not tracked in git); recreate it after a fresh
clone with:

```powershell
New-Item -ItemType Junction -Path ".claude\skills" -Target "skills"
```

## What this project is

NGK is an experimental **CAD geometric kernel written in Rust**, built around
**generalized maps (3-GMaps)** instead of a classical winged-edge / OCCT-style B-Rep.

The core bet: represent topology combinatorially first (darts + α involutions),
and attach geometry (points, curves, surfaces) as *payload attributes* on cells.
Sewing, unsewing, extrusion, splitting and cell traversal then become explicit,
provable operations on the map rather than ad-hoc record surgery.

Goal is a **robust, state-of-the-art kernel for CAD** — not a history-based
parametric modeler. Operations mutate one map and return explicit handles.

- Crate: `ngk`, edition 2024, MIT. `cdylib` + `rlib`, plus an `ngk` binary.
- Deps: `nalgebra`, `slotmap`, `serde`, `thiserror`, `radians`.
- Features: `python` (pyo3/maturin), `wasm` (wasm-bindgen).
- Author: Jojain (romain.ferru@gmail.com). Pages demo: https://jojain.github.io/ngk/

## Layering (bottom → top)

| Layer | Path | Role |
|---|---|---|
| `topology` | `src/topology/` | The pure GMap (darts, α0..α3, orbits), the embedding classification over it, entity records and typed views |
| `model` | `src/model.rs` | `Model<P>`: one GMap plus the entity stores, geometry, payloads, labels and derived indexes keyed against it; the transaction boundary |
| `geometry` | `src/geometry/` | Pure math: points, curves, surfaces, NURBS, intersections, bbox, tolerance |
| `builders` | `src/builders/` | Low-level topology construction (`&mut Model<P>`), one transaction each |
| `modeling` | `src/modeling/` | Thin user-facing standalone shape builders (`block`, `revolve`, …) |
| `healing` | `src/healing/` | Removes topology that carries no shape (`i`-removal passes over `builders::removal`) |
| `tessellate` | `src/tessellate/` | Geometry/BRep → polylines + indexed meshes |
| `viz` | `src/viz/` | `VizScene` assembly, dart/α overlays, debug viewer, ocp_vscode bridge |
| `scripts` | `src/scripts/` | Named exploration scenes, registered in `SCRIPTS` |
| `bindings` | `bindings/{common,python,wasm}` | pyo3 + wasm-bindgen surfaces |
| `visualization/` | React + R3F + Vite | Playground consuming the wasm build |

`Model<P>` owns everything a shape is: `topology: GMap`, the six entity
slotmaps, the `Embedding` classification, a `revision` counter, and the derived
lookups over all of it. `GMap` itself imports no geometry, no payload and no
key type — it is darts and involutions. Nothing outside `model.rs` and
`topology/edit.rs` can reach `&mut GMap`: `Model::topology()` is read-only and
every mutation goes through a transaction. `docs/model_api.md` predates this
and is a design note, not a description of the tree.
`plan/one_logical_cell_one_raw_cell.done.md` records the final cell-occupancy
invariant; `plan/logical_topology_over_gmap.done.md` is the completed wider
migration record.

## Topology core — key concepts

- **`Dart`** — oriented traversal locator. Short-lived; may be destroyed by any edit.
- **`Dim::{Zero,One,Two,Three}`** ↔ α0..α3 ↔ vertex/edge/face/(sheet|solid).
  `GMAP_INVOLUTION_COUNT = 4`.
- **Keys are the durable identity**: `VertexKey, EdgeKey, ProfileKey, FaceKey,
  SheetKey, SolidKey` (slotmap `new_key_type!`). Public APIs select cells by key,
  never by dart.
- **Attributes** (`topology/attributes.rs`) store geometry + user payload per cell:
  `VertexAttr{dart, point, data}`, `EdgeAttr{dart, curve, data}`, `FaceAttr`
  (surface + pcurve loops), `ProfileAttr`, `SheetAttr`, `SolidAttr`.
- **Orientation triple** (`docs/topology_orientation_refactor.md`):
  `identity = XKey` · `default orientation = reference dart in XAttr` ·
  `contextual orientation = dart carried by the view`.
  `Orientation::{Same,Reversed}` composes and applies to vectors/scalars.
- **Typed views** — `Vertex`, `Edge`, `Face`, `Profile`, `Sheet`, `Solid`, plus
  `Shape<K, P>` (owned model + primary handle, read with `model()` /
  `model_mut()` / `into_model()`). *Traverse with these, not raw darts.*
- **`Payload` trait** — type-level bundle of user data per dimension, bounded
  only `Clone + 'static`, plus `type Policy: EditPolicy<Self> + Default`, the
  rule that maintains it. `StandardPayload` = `()` everywhere with
  `Policy = PreservePayload`. Everything is generic over `P: Payload`; there is
  no `DefaultPayload` bound any more — see "Transactions" below.
- **Profiles = face boundary loops; Sheets = solid shells.** They must be
  **registered explicitly** (`add_profile` / `add_sheet`); commit rejects faces
  or solids referencing unregistered components.

### Vocabulary: cells and entities

Two layers, two sets of words, and they are never mixed. Getting this wrong is
how a reader comes to believe the map has vertices.

| Layer | Write | Prefer |
|---|---|---|
| **GMap** — what the involutions hold | `0-cell`, `1-cell`, `2-cell`, `3-cell`; or `raw vertex`, `raw edge`, `raw face`, `raw cell` | the `N-cell` form, most of the time |
| **Logical** — what a shape is made of | `vertex`, `edge`, `face`, `profile`, `sheet`, `solid` | the bare noun |

**The one rule: a bare noun is always the logical entity.** A "face" is a
`FaceKey` and its `FaceAttr`. A thing the map holds is a 2-cell, or a raw face,
and it is **never** just "a face" — in any context, however obvious the
surrounding code makes it. Dropping the qualifier is the single mistake this
vocabulary exists to prevent, because it is how a reader comes to believe the
map has vertices.

Both spellings of the map-side noun are legitimate and mean the same thing.
`N-cell` is the better default: it is shorter, it carries the dimension, and it
cannot be misread. Reach for `raw edge` when a sentence gains from naming the
layer in words — "the bridge is a raw edge the face owns" reads better than the
same sentence with `1-cell` — and use `raw cell` when the dimension is not the
point.

Going the other way, write `logical vertex` only where one sentence names both
layers and the contrast is its whole point — "the 0-cell where a circle closes
is not a logical vertex" — and drop the qualifier again as soon as the sentence
is over. The bare noun already means the logical one.

This applies to doc comments, inline comments, test names, error messages and
plan prose alike. Identifiers already follow it: `Model::cells(Dim::Two)` and
`cell_representative` are map-side, `iter_faces` and `face_attr` are
logical-side, and `Dim` never appears in a logical signature.

### Corners

A **corner** is a vertex in the role of bounding something. Corners are always
vertices — that is the whole content of the word, and it is what makes the
following true:

- A 0-cell an edge owns is **not** a corner. A whole circle closes somewhere,
  and `point_at_dart` will tell you where, but nothing meets there, so the
  circle has no corner and is *unmarked*.
- **Cutting adds a corner.** A cut promotes the 0-cell at the cut to a vertex;
  whether the edge then separates depends on whether it already had one.
- An edge's corner count is what names its shape: **bounded** 2, **marked** 1,
  **unmarked** 0. See the table below.

Two counts, and they differ on purpose:

| Count | Means | Where |
|---|---|---|
| **distinct corners** | how many different vertices bound the thing | `BoundedEdge::vertices`, `MarkedEdge::corner`, `EdgeCore::has_corner_at` |
| **corner visits** | how many times a walk arrives at one | `Loop::corners`, `LoopCorner` |

A marked edge has **one** corner and a walk along it passes that corner
**twice** — it is both the start and the end. A loop that meets one vertex twice
has one vertex and two corners. `LoopCorner` therefore pairs the dart arriving
with the one leaving, which is what tells two visits apart; asking it for a
`vertex()` collapses them again.

The same split one dimension up needs no machinery. A loop's darts *are* its
oriented edges, one for one, because an edge occupies exactly one raw cell and
the walk cannot land on part of one. An edge a loop runs twice — a seam — simply
appears twice in `Loop::darts()`.

**Two homonyms, deliberate and safe.** `BBox::corners`,
`UnwrappedFaceDomain::corners` and `snap_boundary_corner` mean *points*, not
topology, and their types say so — they hand back `Point3`/`Point2`, never a
view. And `exchange::step::topology::export::Corner` is a corner **of the
exported file**, not of the model: it is `Vertex(VertexKey)` or
`Closure(EdgeKey)`, because `EDGE_CURVE` has no spelling without two ends, so a
closure point that is no corner here must still be written as a vertex there.
That type is private to the exporter, where the file's vocabulary is the one in
force.

### Edges: bounded, marked, unmarked

An edge's kind is decided by **how many distinct corners it has**, not by the
shape of its curve. Three cases, and these are the words for them:

| Term | Corners | Example | `Edge` variant |
|---|---|---|---|
| **bounded** | 2 distinct | a segment, an arc | `Edge::Bounded(BoundedEdge)` |
| **marked** | 1, which is both its start and its end | a circle someone cut | `Edge::Marked(MarkedEdge)` |
| **unmarked** | 0 | a circle as built | `Edge::Unmarked(UnmarkedEdge)` |

Corner access is **total on each type**: `BoundedEdge::vertices()` returns two,
`MarkedEdge::corner()` returns one, and `UnmarkedEdge` offers no way to ask. No
`Option` to unwrap, and nothing to forget.

**Marked and unmarked are both closed**, but the enum is flat rather than nested
on purpose: a site that must tell them apart should not be able to write one
pattern that silently covers both. A site that genuinely means "closed" writes
`Edge::Marked(_) | Edge::Unmarked(_)`, which is still exhaustive-checked, or asks
`Closeable::is_closed()`. `vertices_at_dart` folds `start == end` to `None`, so a
marked edge is never `Bounded` — a closed edge with a deliberate corner is still
closed.

An unmarked edge's closing point is not nothing: it is a 0-cell, classified
as interior to the edge, and `point_at_dart` derives its position from the curve.
It is simply not a corner anything meets at. **Ask `point_at_dart` for a
position; ask the vertex store only when the identity of a logical vertex is
what matters.**

Cutting an edge adds a corner. Whether the edge *separates* depends on whether
it already had one, which makes one rule cover all three:

| cut a… | you get |
|---|---|
| bounded edge | two bounded edges |
| **unmarked** edge | **one marked edge** — the 0-cell is materialized where the cut was asked for, and no edge is created |
| marked edge | two bounded edges |

Marking is a pure relabel of the map: an unmarked edge already has its 2 darts
and its one 0-cell orbit, so materializing costs no darts and no links. Only the
vertex's stored point says where it now sits, and the span derivation reads it —
which is why a marked edge spans `[t, t + period]` from its corner rather than
from the curve's own domain start.

Two hazards follow from all this, and both have already bitten:

- **Predicates over corners.** "Every vertex satisfies P" is vacuously true of an
  edge or face that has none.
- **A span's ends are not always the edge's ends.** Four separate places used
  one as a proxy for the other, and each was right only until an edge could be
  unmarked. Two forms to watch for:
  - *The pcurve's start.* A pcurve is a closed loop in the face's parameter
    space, anchored wherever it was built; a marked edge begins at its *corner*,
    somewhere else on that loop. `pcurve.point_at(0.0)` is a boundary corner
    only for a bounded edge.
  - *The parameter domain's ends.* "Too close to `domain.start`, so this cut is
    degenerate" is right for a bounded or marked edge, whose span ends at a
    corner. An unmarked edge's span is its whole support, and the ends are where
    the curve *closes* -- a place to put a corner like any other. Refusing there
    silently drops a junction, which is how a tangency landing on a circle's
    parameterization origin goes missing.

  In both forms: ask the corner, not the parameterization.

### Boundaries: profile, loop, sheet, shell

These four name **boundary components of logical entities**, never components of
the map. That is their whole definition, and everything else follows from it.

| Word | Is | Bounds |
|---|---|---|
| **profile** | a maximal connected set of edges | a face |
| **loop** | a closed profile | a face |
| **sheet** | a maximal connected set of faces | a solid |
| **shell** | a closed sheet | a solid |

A profile and a sheet are the same idea one dimension apart, and both carry a
stable key of their own. A profile exists without a face — an open wire is a
profile, and so is a closed one lying in space bounding nothing.

**Connected means connected through logical cells only.** The traversal is the
`alpha0`/`alpha1` walk for a profile and the `alpha0`/`alpha1`/`alpha2` walk for
a sheet, and when a step lands on a **scaffold** cell — a raw cell owned by
something of higher dimension, which
[Embedding](#embedding-srctopologyembedding) names — the walk **turns
across it** and carries on:

- a profile turns across a raw edge the face owns: the bridge to a hole, a
  periodic seam;
- a sheet turns across a raw face the solid owns: a cut face inside a cavity.

Turning is local. At dimension `n`, a step onto a scaffold `n`-cell is crossed
with `alpha(n + 1)` and the walk resumes — no region, no classification of the
surrounding entity, nothing but the involutions and the owner of the one cell
being crossed. `turn` is that primitive and is the same one at every dimension:
`turn(map, index, Dim::One, dart)` walks a profile past a bridge edge around the
0-cell they share, and `turn(map, index, Dim::Two, dart)` walks a sheet past a
cut face around the 1-cell they share. Nothing else should re-derive it.

**Turning across is not the same as stopping at.** An annulus's inner rim is an
unmarked circle whose closure now runs *through* the bridge: `alpha1` of the
rim's far end is a bridge dart. Stop there and a closed circle reads as an open
one-edge chain. Turn across it and the walk comes back onto the same rim, which
is what makes the annulus's two rims two profiles rather than one.

**Why it matters that the walk turns rather than crosses straight on.** A raw
`alpha0`/`alpha1` component of a bridged face runs through *every* loop the face
has, so the outer rim and the hole share one component. Asked of the raw map,
"which loop is this edge on" has no answer. Asked of the logical walk, it has
one.

#### A loop is a walk over a profile, not the profile

A profile says *which* edges. A loop says *how a face runs along them*, and the
two are not the same list:

- **Sense.** An edge has up to four darts; a face sees two of them. Which two is
  a question about the face's region, not about the profile, so a profile alone
  cannot answer it. A standalone wire has no face and therefore no side to pick.
- **Repetition.** A face can run along one edge twice — a slit, or a marked
  edge passed on both sides. The profile holds that edge once; the loop's walk
  emits it twice.

So a loop *names* its profile and is read from one face's side. Connectivity
belongs to the profile; orientation and repetition belong to the loop.

### Embedding (`src/topology/embedding/`)

A raw cell carrying no logical entity of its own dimension is **embedded** in
the entity of higher dimension whose **interior** contains it: a cylinder's seam
edge lies in its wall face, and the 0-cell where a circle's parameterization
closes lies in the circle rather than being a logical vertex. Records are one
`EmbeddedCell` per orbit, anchored at a representative dart, and every one of
them names an owner of **strictly greater** dimension than the cell — so "has a
record" and "is embedded" are one question.

Where an entity's *own* cell is never appears in a record: it is read off the
entity's anchor, so a shape with nothing embedded stores no records at all and
is still fully classified. `recover_region` walks an entity out of the map from
that anchor, and because an entity occupies exactly one raw cell of its own
dimension, that walk is one orbit and nothing floods.

`turn` is the one traversal primitive — it steps around a shared boundary cell
with the sewing involutions, passing over embedded cells. `boundary_cycles`
gives a face its oriented loops by turning across the cells it owns rather than
emitting them. `boundary_shells` does the same one dimension up for a solid's
boundary components, though `Solid::shells` still reads the shell roots stored
on `SolidAttr` rather than calling it.

#### One entity, one cell of its own dimension

Each vertex, edge, face and solid occupies **exactly one** raw cell of its own
dimension — not zero, and not two. The converse does not hold: a raw cell need
carry no entity at all, and any number of **lower**-dimensional ones may be
embedded in a single entity — a bridge to a hole, a periodic seam, the 0-cell
where a circle closes, a cut face inside a cavity. Profiles and sheets are
aggregates of entities rather than entities with a cell of their own, so the
rule does not reach them.

`validation::validate_cell_occupancy` is the commit check; see
[plan/one_logical_cell_one_raw_cell.done.md](plan/one_logical_cell_one_raw_cell.done.md)
for the completed construction and validation contract.

This is a **commit invariant**, not a construction rule. A transaction may break
it freely — a Boolean partitions a face and puts it back together — but a commit
that leaves an entity spanning two cells of its own dimension, or none, is
rejected.

What follows from it:

- **One key, one orbit.** An entity's anchor identifies its whole
  same-dimensional topology, so nothing ever has to collect an entity's cells.
  `recover_region` is an orbit walk plus the orientation it reads off it, and
  nothing floods.
- **Records only ever classify embedded cells**, which are lower-dimensional by
  definition. "Recorded as owned by an entity of higher dimension" and "is
  embedded" become the same statement, and the record type refuses any other.
- **A raw cell between two raw cells of the dimension above is real topology.**
  A 1-cell with a different 2-cell on each side is either a logical edge between
  two logical faces, or it must be removed so the two raw faces merge. This is
  the book's removal operation read as an invariant: removing an i-cell merges
  the two incident (i + 1)-cells *when they exist*, and a bridge has only one
  incident 2-cell, so removing it merges nothing. That is precisely what makes a
  bridge interior rather than boundary.
- **A raw split forces a logical split; a raw merge forces a logical merge.**
  Commit is where that correspondence is checked, so a partition can never hide
  beneath one key.

What it costs. Each of these must exist before commit, and is not an
optimisation:

- a face with holes needs one bridge edge per hole;
- a cylindrical face needs its seam;
- a whole sphere needs a seam/cut representation rather than several quads;
- a whole torus needs two cuts;
- a solid with a cavity needs a scaffold face joining its outer and inner shells,
  so that it is one raw 3-cell;
- STEP import must synthesise those before committing, and can no longer keep one
  `ADVANCED_FACE` as several disconnected raw faces.

The rule is what makes a missing bridge a **named error at commit** instead of a
face that silently reports half its boundary. That is the same trade the rest of
this file asks for: refuse rather than approximate.

### Transactions (read `src/topology/edit.md` — short and essential)

`Model::transaction` / `transaction_with_policy` is the atomic boundary for a
modeling operation. The closure receives a **`ModelEdit`** — the only public
mutation capability (`add_dart`, `remove_dart`, `link`, `unlink`, `sew`,
`own_cell`, plus attribute create/remove/split/merge declarations).

- One public builder = one transaction; composite builders pass the same
  `&mut ModelEdit` down to the matching `_edit`-suffixed operation.
- The public wrapper is a one-line transaction boundary. The `_edit` operation
  owns validation and construction, and is `pub(crate)` so kernel operations
  can compose without opening a nested transaction. The suffix names the
  parameter that makes it composable (`&mut ModelEdit`, already open),
  not just "this one is private" — a leading underscore did that but also
  hid the item from `dead_code`, so an orphaned `_edit` function left behind
  after its public wrapper is deleted is now reported like any other item.
- Operation results are stamped only after commit with `transaction_result`;
  their view methods reject an unstamped or stale result.
- `mod.rs` files are module manifests: they declare modules and re-export
  their API. Non-trivial logic belongs in named child files; only exceptional
  code small enough not to justify its own file may remain in `mod.rs`.
- Any error, validation failure, identity-reconciliation failure or payload
  policy failure restores the full transaction-start snapshot. Panics are **not** caught.
- **Lineage**: `add_*` (`Origin::New`) / `add_*_split_from` (`Origin::Split`,
  same kind) / `add_*_derived_from` (`Origin::Derived`, one or more sources of
  any kind) / `merge_*_into` (explicit survivor) / `remove_*` (nothing
  inherits). `remove_*` and `merge_*_into` each record their own event, so
  commit rejects a transaction-start attribute that goes missing without one
  explaining it (`ModelEditError::UnexplainedRemoval`).
- **`EditPolicy`** has three hooks per kind: `*_created(key, origin, before)`
  returns the new payload, `*_merged` folds a consumed payload into the
  survivor, `*_consumed` disposes of one nothing inherits. At commit, every net
  externally-visible creation, merge and consumption calls its hook once, in
  declaration order, with `Origin`'s sources resolved against the
  transaction-start snapshot.
  **Which policy runs is a property of the payload, not of the call site.**
  `Model::transaction` runs `P::Policy::default()`, so a payload that named its
  own policy gets it from every builder in the kernel with nothing said
  anywhere. `StandardPayload::Policy = PreservePayload` (clones a `Split`'s
  source, keeps the merge survivor, drops on consume, defaults `New`/`Derived`),
  which is why a payload naming it must have `Default` at every dimension — an
  obligation discharged at the `impl Payload` and carried by no builder
  signature. `Model::transaction_with_policy` overrides it for one transaction,
  which is what a policy carrying parameters in or state out needs, since
  `P::Policy` is default-constructed and dropped at commit. See
  `plan/payload.md`.
- **Identity reconciliation** picks one surviving key per final cell;
  transaction-start keys beat transaction-local ones. Local keys may vanish at commit.
- Commit order: raw gmap axioms, embedding records, required registrations,
  lineage, identity reconciliation, the unexplained-removal check, payload
  policy, then `revision += 1`.
- Derived dart→key maps and the dart→owner index are lazy caches on `Model`,
  invalidated on every mutation and never serialized.

## Geometry

- 2D: `Curve2` (`Line2`, `Circle2`, `Ellipse2`, `NurbsCurve2`), `TrimmedCurve2`,
  2D intersections (used for pcurves/imprints).
- 3D: `Curve` (`Line`, `Circle`, NURBS), `Surface` (`Plane`, `Cylinder`,
  `RuledSurface`, `SurfaceOfRevolution`, `NurbsSurface`), `BBox`, `Frame`, `Interval`.
- Intersections: curve/curve, curve/surface, surface/surface with `IntersectionOptions`.
- Tolerances: `LINEAR_TOLERANCE`, `ANGULAR_TOLERANCE`; point equality via
  `PointCoincidence::coincides`.
- **Analytic-first dispatch, certified NURBS fallback** (`intersections/analytic/`):
  a recognized pair is answered in closed form *before* anything is converted or
  decomposed; everything else takes the NURBS solver, which is also the
  differential-test oracle. `intersect_analytic_*` return `Option<Result<..>>`:
  `None` declines the pair (fall back), `Ok(..Empty)` certifies disjointness.
  A pair in the table that reaches a case the `Curve` types cannot carry also
  declines. Covered today: surface/surface plane×{plane,sphere,cylinder} and
  sphere×sphere; curve/surface line×{plane,sphere,cylinder,cone} and
  circle×{plane,sphere}; curve/curve line×line, line×circle, circle×circle.
  Not covered: anything involving a cone surface pair, cylinder×cylinder,
  circle×{cylinder,cone}.
- **A parameter carries the space it lives in** (`geometry/parameter.rs`).
  `Param<S>` wraps one `f64` with a marker, and `Interval<S>` is a range of
  them: **`Native`** is the support's own parameterization — a line's affine, a
  conic's radians, a NURBS curve's knot domain — and **`Normalized`** is a
  traversal fraction of *a named span*, `0` its start and `1` its end. The
  aliases are `NativeParam` and `Fraction`; `Interval` alone means
  `Interval<Native>`, because most spans in the kernel are native.
  `Interval::at` and `Interval::fraction_of` are the **only** two conversions
  between them, and each is asked of the span the fraction is a fraction of, so
  "a fraction of what" always has an answer at the call site.
  - **Only a `TrimmedCurve`/`TrimmedCurve2` produces or consumes a `Fraction`.**
    A bare `Curve` has no fractions, because it has no span to be a fraction of.
    That is why there is no `Curve::trimmed`: naming a portion of a support is
    `trimmed_native`, in the support's own parameters. A fraction spent against
    the wrong reference is a bug no brand can see, and this is what removes the
    chance of writing one.
  - **A `Fraction` is not confined to `[0, 1]`.** A solver hit just off the end
    of a span, a clipped overlap, a point projected past an edge — each is a
    real value outside the unit interval, and refusing or clamping it would
    destroy the news that the hit was off the span. Ask `is_inside_unit` where
    that matters.
  - **Surfaces are not branded yet, and not because they are safe.** A surface
    has two parameter spaces as much as a curve does — its analytic `(u, v)`
    and the knot domain of `to_nurbs_over`, which is why `ParamMap` and
    `Reparam` exist. What it lacks is a *fraction*: there is no trimmed surface,
    so `Normalized` has nothing to say about one, and its second space is
    `Knot`, which is not built. So `u` and `v` stay `f64`, as does the `Point2`
    of its parameter space; `domain()` is still `Interval<Native>`, and reading
    an endpoint out to evaluate takes `Param::value`. The NURBS evaluator and
    the subdivision solvers are plain `f64` for the same reason — one space
    each, unwrapped at their edges. See `plan/parameter_units.md` stage 4 for
    the `Uv<S>` shape this wants.
- **A `Curve` is an unbounded support, never trimmed to the cell carrying it.**
  There is no `Curve::Bounded` variant: a line runs to infinity, a circle closes.
  Which part is meant is said by **`TrimmedCurve`** (`geometry/dim3/trimmed.rs`)
  — support + an `Interval` of its *native* parameters, kept as one value so the
  halves cannot drift apart. It normalizes traversal (fraction 0 → 1 over the
  span), unwraps periodic branches, and answers `contains` / `length` / `sub` /
  `reversed`. `to_curve()` is the cut-down copy: exact, but NURBS, so reach for
  it only when a curve that *is* the section is required (fitting a pcurve).
- **2D mirrors 3D exactly.** A `Curve2` is an unbounded support in a surface's
  parameter space — `Line2` extrapolates, `Circle2` and `Ellipse2` close — with
  *native* parameters (a line's affine, a conic's angle in radians, a NURBS
  curve's own knot domain); nothing is renormalized to `[0, 1]`.
  **`TrimmedCurve2`** (`geometry/dim2/trimmed.rs`) is support + `Interval`, with
  the same API as `TrimmedCurve`. Both carry the same shorthand constructors —
  `segment`, `arc`, `ellipse_arc` (each anchoring the support so the span is
  just the sweep) and `whole` (a support that already *is* the section, such as
  an interpolated NURBS) — so a common span is never spelled out as a support
  and an interval side by side. **Every pcurve is a `TrimmedCurve2`**: unlike an edge, it has
  no bounding vertices to derive a span from, since a face stores no 2D vertex
  positions. 2D intersection takes two spans, not two supports — the
  subdivision search needs control polygons, which an infinite support has none
  of — and returns fractions of each span.
- **The span is derived only on an edge.** `EdgeAttr` stores no interval;
  `Edge::trimmed_curve()` derives it from the bounding vertices plus the view's
  orientation. Everything without vertices — `AnalyticSection`,
  `SurfaceIntersectionBranch`, `FaceImprint`, `IntersectionSpan` — must carry
  it: the minor and major arc between two points share both endpoints, so
  endpoints alone name neither, and direction is load-bearing besides.
- **A solver answers for the support.** Clip its result to the cells' own spans
  (`TrimmedCurve::contains`) before treating it as a contact, or an edge picks up
  hits from the part of its line no edge occupies.
- `FaceImprint`'s two halves are **synchronized** — same fraction, same point —
  which constrains the support: a `Circle` spans its arc in angle, the rational
  quadratic its pcurve is fitted from does not, so an imprinted arc carries NURBS.
- A section's 3D curve is exact; its **pcurve is exact only where a closed form

## Working rules

- **Deleting behaviour means deleting or adapting its tests.** A change is not
  finished while a test still asserts, names, or explains the thing that was
  removed. Grep for the concept by name, not just for compile errors: a test that
  still passes can be testing nothing, and one whose name or comment describes the
  old design is worse than no test, because it teaches the next reader something
  false. Three outcomes, in order of preference — the test states a property that
  survives, so keep it and fix the wording; the property moved to another shape,
  so retarget it there; the property is genuinely gone, so delete the test. Never
  leave the fourth. The same goes for doc comments, error-variant docs, `#[ignore]`
  reasons, and plan files: an `#[ignore]` whose stated reason no longer applies
  must be re-checked, since the test may now pass or fail for a new reason.
- **Never `git checkout`, `git restore`, or `git stash` a file to undo your own
  edit.** The working tree holds uncommitted work that is not recoverable. Undo
  by making the inverse edit, and use a marker comment you can grep for when
  adding temporary instrumentation.
- **A comment describes the code as it is, never a plan for it.** Doc comments
  and inline comments state what the thing does, why it is shaped that way, and
  what it refuses — all in the present tense, all checkable against the code
  next to them. They must not reference a plan document, a milestone, a stage
  number, or a decision id (`plan/foo.md`, "stage 4", "D7", "arrives later",
  "not implemented yet"), and must not describe behaviour that does not exist
  yet. Keep the *reason* and drop the schedule: "a cylinder wall closes on
  itself, which STEP cannot spell without a synthesized seam" is factual and
  stays true; "…which needs seam synthesis (stage 4)" is a promise that rots the
  moment the plan moves, and the reader cannot tell from the code whether it
  still holds. Plans belong in `plan/`, which is where they can be revised in
  one place. This applies to module docs, error-variant docs, test comments and
  commit-adjacent prose alike.
