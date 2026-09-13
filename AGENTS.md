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
| `topology` | `src/topology/` | The pure GMap (darts, α0..α3, orbits), the subdivision classification over it, entity records and typed views |
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
slotmaps, the `Subdivision` labelling, a `revision` counter, and the derived
lookups over all of it. `GMap` itself imports no geometry, no payload and no
key type — it is darts and involutions. Nothing outside `model.rs` and
`topology/edit.rs` can reach `&mut GMap`: `Model::topology()` is read-only and
every mutation goes through a transaction. `docs/model_api.md` predates this
and is a design note, not a description of the tree; `plan/logical_topology_over_gmap.md`
is the live plan.

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
- **`Payload` trait** — type-level bundle of user data per dimension;
  `StandardPayload` = `()` everywhere. Most types are generic over `P: Payload`.
- **Profiles = face boundary loops; Sheets = solid shells.** They must be
  **registered explicitly** (`add_profile` / `add_sheet`); commit rejects faces
  or solids referencing unregistered components.

### Subdivision (`src/topology/subdivision/`)

Every raw cell is labelled with the logical entity whose **interior** contains
it: a cylinder's seam edge belongs to its wall face, a circle's closure vertex
belongs to the circle. Labels are one `OrbitOwnership` record per orbit, anchored
at a representative dart; the darts an entity covers are never stored, they are
walked out of the map by `recover_region`. `turn` is the one traversal
primitive — it steps around a shared boundary cell with the sewing involutions,
passing over cells a higher-dimensional entity owns. `boundary_cycles` gives a
face its oriented loops and `boundary_shells` gives a solid its boundary
components, both by turning across interior cuts rather than emitting them.

### Transactions (read `src/topology/edit.md` — short and essential)

`Model::transaction` / `transaction_with_policy` is the atomic boundary for a
modeling operation. The closure receives a **`ModelEdit`** — the only public
mutation capability (`add_dart`, `remove_dart`, `link`, `unlink`, `sew`,
`own_cell`, plus attribute create/remove/split/merge declarations).

- One public builder = one transaction; composite builders pass the same
  `&mut ModelEdit` down to private `*_staged` helpers.
- Any error, validation failure, identity-reconciliation failure or payload
  policy failure restores the full transaction-start snapshot. Panics are **not** caught.
- **Lineage**: `add_*` (fresh) / `add_*_split_from` (derived) / `merge_*_into`
  (explicit survivor). At commit, merge chains resolve and `EditPolicy`
  (e.g. `PreservePayload`) runs only on net externally-visible changes.
- **Identity reconciliation** picks one surviving key per final cell;
  transaction-start keys beat transaction-local ones. Local keys may vanish at commit.
- Commit order: raw gmap axioms, subdivision labels, required registrations,
  lineage, identity reconciliation, payload policy, then `revision += 1`.
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
