# Overview

NGK is an experimental CAD geometry kernel written in Rust, built around
**generalized maps (3-GMaps)** instead of a classical winged-edge or OCCT-style
B-rep.

The core bet: represent topology combinatorially first (darts and α
involutions), and attach geometry (points, curves, surfaces) and user payload to
the logical entities read off the map. Sewing, unsewing, extrusion, splitting
and cell traversal are then explicit, checkable operations on the map rather
than ad hoc record surgery.

The goal is a robust kernel for CAD, not a history-based parametric modeler.
An operation mutates one model and returns explicit keys describing what it
did.

- Crate `ngk`, edition 2024, MIT; `cdylib` + `rlib`, plus an `ngk` binary.
- Dependencies: `nalgebra`, `slotmap`, `serde`, `thiserror`, `radians`.
- Features: `python` (pyo3/maturin), `wasm` (wasm-bindgen).

Vocabulary is defined in [CONTEXT.md](../../CONTEXT.md).

## Layering, bottom to top

| Layer | Path | Role |
|---|---|---|
| `topology` | `src/topology/` | The pure GMap (darts, α0..α3, orbits), the embedding classification over it, entity records and typed views |
| `model` | `src/model.rs` | `Model<P>`: one GMap plus the entity stores, geometry, payloads and derived indexes keyed against it; the transaction boundary |
| `geometry` | `src/geometry/` | Pure math: points, curves, surfaces, NURBS, intersections, bounding boxes, tolerances |
| `solver` | `src/solver/` | Stateful 2D geometric constraint systems, independent of topology |
| `builders` | `src/builders/` | Topology construction over `&mut Model<P>`, one transaction each |
| `modeling` | `src/modeling/` | Owned-`Shape` facade over `builders`: each constructor allocates a model and delegates one operation |
| `healing` | `src/healing/` | Removes topology that carries no shape (removal passes over `builders::removal`) |
| `measure` | `src/measure/` | Length, area, volume, centroid and inertia of entities |
| `tessellate` | `src/tessellate/` | Geometry and B-rep to polylines and indexed meshes |
| `exchange` | `src/exchange/` | STEP import and export |
| `viz` | `src/viz/` | Scene assembly, dart/α overlays, debug viewer, ocp_vscode bridge |
| `scripts` | `src/scripts/` | Named exploration scenes, registered in `SCRIPTS` |
| `bindings` | `bindings/{common,python,wasm}` | pyo3 and wasm-bindgen surfaces |
| `visualization/` | React + R3F + Vite | Playground consuming the wasm build |

## The model owns everything

`Model<P>` owns everything a shape is: `topology: GMap`, the six entity
slotmaps, the `Embedding` classification, a `revision` counter, and the derived
lookups over all of it. `GMap` itself imports no geometry, no payload and no key
type: it is darts and involutions.

Nothing outside `model.rs` and `topology/edit.rs` can reach `&mut GMap`.
`Model::topology()` is read-only and every mutation goes through a transaction;
see [transactions.md](transactions.md).

## Subsystem documents

- [topology.md](topology.md): the map, entities, embedding, identity and
  orientation.
- [transactions.md](transactions.md): the edit capability, commit, lineage and
  payload policy.
- [geometry.md](geometry.md): supports and spans, parameter spaces,
  intersection dispatch.
- [solver.md](solver.md): 2D constraints, editable state, solutions and numerical diagnostics.
- [builders.md](builders.md): builder naming and result contract.
- [measure.md](measure.md): length, area, volume and inertia by boundary
  integral.
- [blend.md](blend.md): the chamfer and fillet engine.
