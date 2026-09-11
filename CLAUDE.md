# CLAUDE.md — NGK (Nales Geometry Kernel)

Working memory for agents in `D:\Projets\ngk`. Read this first, then the pointed-to docs.

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
| `topology` | `src/topology/` | The GMap: darts, α0..α3, cells, keys, attributes, transactional editing |
| `geometry` | `src/geometry/` | Pure math: points, curves, surfaces, NURBS, intersections, bbox, tolerance |
| `builders` | `src/builders/` | Low-level topology construction (`&mut GMap`), one transaction each |
| `modeling` | `src/modeling/` | Thin user-facing standalone shape builders (`block`, `revolve`, …) |
| `healing` | `src/healing/` | Removes topology that carries no shape (`i`-removal passes over `builders::removal`) |
| `tessellate` | `src/tessellate/` | Geometry/BRep → polylines + indexed meshes |
| `viz` | `src/viz/` | `VizScene` assembly, dart/α overlays, debug viewer, ocp_vscode bridge |
| `scripts` | `src/scripts/` | Named exploration scenes, registered in `SCRIPTS` |
| `bindings` | `bindings/{common,python,wasm}` | pyo3 + wasm-bindgen surfaces |
| `visualization/` | React + R3F + Vite | Playground consuming the wasm build |

`src/model.rs` holds an embryonic `Model<P>` (owns one persistent `GMap`) —
the target design lives in `docs/model_api.md` and is **not implemented yet**.

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
  `Shape<K, P>` (owned map + primary handle). *Traverse with these, not raw darts.*
- **`Payload` trait** — type-level bundle of user data per dimension;
  `StandardPayload` = `()` everywhere. Most types are generic over `P: Payload`.
- **Profiles = face boundary loops; Sheets = solid shells.** They must be
  **registered explicitly** (`add_profile` / `add_sheet`); commit rejects faces
  or solids referencing unregistered components.

### Transactions (read `src/topology/edit.md` — short and essential)

`GMap::transaction` / `transaction_with_policy` is the atomic boundary for a
modeling operation. The closure receives a **`TopologyEdit`** — the only public
mutation capability (`add_dart`, `remove_dart`, `link`, `unlink`, `sew`, plus
attribute create/remove/split/merge declarations).

- One public builder = one transaction; composite builders pass the same
  `&mut TopologyEdit` down to private `*_staged` helpers.
- Any error, validation failure, identity-reconciliation failure or payload
  policy failure restores the full transaction-start snapshot. Panics are **not** caught.
- **Lineage**: `add_*` (fresh) / `add_*_split_from` (derived) / `merge_*_into`
  (explicit survivor). At commit, merge chains resolve and `EditPolicy`
  (e.g. `PreservePayload`) runs only on net externally-visible changes.
- **Identity reconciliation** picks one surviving key per final cell;
  transaction-start keys beat transaction-local ones. Local keys may vanish at commit.
- Derived dart→key maps are one lazy `DerivedCellIndexes` cache, invalidated on mutation.

## Geometry

- 2D: `Curve2`, `Line2`, `NurbsCurve2`, 2D intersections (used for pcurves/imprints).
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
