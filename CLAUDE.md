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
- **NURBS-first policy**: analytical types stay, but new algorithms convert to
  NURBS and operate there. Analytic fast paths are later optimizations only.

## Feature status (as of Sept 2026)

- ✅ GMap core, transactional editing, identity/orientation model, validation.
- ✅ Builders: profiles, faces, edges, sheets, solids, revolve, sweep, **chamfer**
  (large, documented in `docs/chamfer_architecture.md`).
- ✅ Tessellation + viz + debug viewer; wasm & Python bindings; script registry.
- 🚧 **Booleans — active rewrite.** The old implementation was deleted
  ("start with a fresh implementation") and redesigned on branch `boolean`.
  `src/builders/boolean/` now = broad_phase, contacts, graph (`IntersectionNetwork`
  of events/spans/regions), imprint, operand, result (`BooleanPreparation`,
  `BooleanLineage`, `BooleanSide`). Currently it computes **contacts + two-sided
  B-Rep splitting**; region classification/assembly is the next step.
  Design refs: `docs/boole_paper_ngk_integration.md`, `docs/boolean_algorithm_guide_fr.md`.
- 🚧 **Shape healing** — `src/builders/removal.rs` (`i`-removal, Defs. 58–59 of the
  GMap book) plus `src/healing/` (two passes: fuse cosurfacial faces, then fuse
  cocurvilinear edges). Removing an edge the same face bounds twice rejoins that
  face's boundary; a removal that would split it into two loops is refused.
  Wired into `boolean` behind `BooleanOptions::heal` (opt-in — flipping the
  default needs four raw-count tests updated). Plan: `plan/shape_healing.md`.
- 🚧 `Model` API — design only (`docs/model_api.md`).
- 🚧 Face tessellation uses per-surface shortcuts; real constrained Delaunay is
  still `// TODO: real CDT` in `src/tessellate/face.rs`.

## Conventions (from `skills/ngk-project`, `skills/test-first-workflow`)

- **Break APIs freely.** Active development: migrate all call sites, no
  compat shims, no deprecated aliases, no placeholder glue.
- **Views over darts.** In builder code never touch GMap attributes directly;
  if the typed view API can't express it, *propose the missing API* rather than
  reaching down.
- Top-level `use` imports, no long qualified paths inside function bodies.
- `thiserror` for every error enum. Convenience constructors (`Curve::line(a,b)`).
- Rustdoc `///` on every non-obvious public item; concise comments on private ones.
- Keep nesting ≤ 3 levels where reasonable.
- **Tests live in `tests/`, mirroring `src/`** (`src/topology/gmap.rs` →
  `tests/topology/`). Integration-style, not inline `mod tests`.
  No tests for `visualization/` or `src/scripts/`.
- **Test-first**: assert the *desired* behavior, watch it go red, fix, go green.
  Name tests after the stable invariant, never after the bug.
- Adding a script ⇒ file in `src/scripts/` + `mod` decl + `SCRIPTS` entry
  (and the matching `visualization/src/experiments/registry.ts` entry if it's
  meant to show in the frontend).

## Commands (PowerShell, repo root)

```powershell
cargo fmt
cargo clippy --all-targets --all-features
cargo test --all-targets --all-features
```

Frontend (`visualization/`): `npm install`, `npm run dev` (rebuilds wasm then Vite),
`npm run build`, `npm run typecheck`.
Python bindings: `.\build.ps1` (cargo build + uv venv + `maturin develop`).

Environment notes: `rg` is blocked on this Windows setup — use `Get-ChildItem
-Recurse -File`, `Select-String`, `Get-Content`. Don't fight `npm run wasm:build`
if Windows permissions block it; report and continue with non-wasm checks.

## Reference material

- `docs/chamfer_architecture.md` — chamfer pipeline, selection model, guarantees.
- `docs/topology_orientation_refactor.md` — identity/orientation design (adopted).
- `docs/model_api.md` — target `Model` / shape API (design note, not current code).
- `docs/boole_paper_ngk_integration.md`, `docs/boolean_algorithm_guide_fr.md` — boolean design.
- `src/topology/edit.md` — transaction & lineage contract.
- `private_doc/Combinatorial_Maps_Book/Combinatorial_Maps_Book.md` — **authoritative
  GMap theory**; read targeted chunks only (it is huge). Also `private_doc/`:
  BOOLE.pdf, the NURBS Book, Hoffmann's solid modeling, OCCT BOPAlgo notes,
  and internal architecture reviews.
- `skills/` — project skills: `ngk-project`, `gmap-reference`, `test-first-workflow`.
