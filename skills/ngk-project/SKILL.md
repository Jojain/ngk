---
name: ngk-project
description: Project working rules for the ngk repository. Use when Codex edits, reviews, tests, or explains code in D:\Projets\ngk, including the Rust core library, GMap/topology code, geometry/modeling/tessellation modules, wasm exports, visualization/debug tooling, example scripts, or integration tests.
---

# NGK Project

## Orientation

Treat `ngk` as a Rust project with two cooperating surfaces:

- The root crate is the core library and CLI. Most implementation lives under `src/` with domains such as `topology`, `geometry`, `modeling`, `tessellate`, `builders`, `scripts`, `viz`, and `wasm`.
- `visualization/` is a frontend helper for examples, display, and visual debugging. Keep visualization work in service of inspecting or demonstrating the core behavior, not as the source of core domain logic.

For GMap/combinatorial map work, also use the `gmap-reference` skill and consult targeted book/reference chunks when topology semantics matter.

## Code Workflow

Before editing, inspect the owning module, nearby tests, and existing naming patterns. Keep changes scoped to the requested behavior and respect the separation between the core crate and visualization helper.

This repository is in active development mode. Do not preserve old public APIs for compatibility when a better shape is emerging. It is expected and acceptable to break APIs, move fast, and update all dependent code to the new API in the same change. Do not add placeholder glue, compatibility wrappers, deprecated aliases, or temporary bridges whose main purpose is to keep old call sites working. The policy is to migrate old code to the new API rather than carry both APIs forward.

Geometry algorithms should default to a NURBS-first implementation strategy for now. Curves and surfaces may still be modeled and exposed as analytical variants such as lines, circles, planes, cylinders, ruled surfaces, and surfaces of revolution, but algorithmic work should convert them to NURBS and operate on that general representation first. Since NURBS generalize the analytical curve and surface types we currently need, getting robust NURBS algorithms working is the priority; direct analytical algorithms are later optimizations for known special cases, not a reason to block or fork the first-pass behavior. Do not remove or break existing analytical representations or behavior while adding the NURBS path.

On this Windows development setup, `rg` is be blocked by execution policy or filesystem restrictions. Use native PowerShell commands such as `Get-ChildItem -Recurse -File`, `Select-String`, and `Get-Content` instead of retrying `rg`.

After each code modification, run the cargo cleanup loop from the repository root:

```powershell
cargo fmt
cargo clippy --all-targets --all-features
cargo test --all-targets --all-features
```

If a change touches wasm or `visualization/`, also run the relevant frontend checks from `visualization/` when package scripts exist, normally:

```powershell
npm run build
```

Exception: do not run or request elevation for `npm run build` / `npm.cmd run build` / `npm run wasm:build` when the wasm build is blocked by Windows permissions or Cargo lock access. The user runs that build locally; report that it was skipped and continue with non-wasm checks when useful.

Report any command that cannot be run or fails for environment reasons.

## Rust Style

Use top-level imports for every external type, trait, function, or module path used repeatedly in a function. Do not write long qualified namespace paths inside function bodies when a `use` item can make the code clear.

Prefer:

```rust
use crate::topology::gmap::GMap;

fn build() {
    let map = GMap::new();
}
```

Avoid:

```rust
fn build() {
    let map = crate::topology::gmap::GMap::new();
}
```

Follow existing module exports and keep `mod.rs` files updated when adding Rust modules. Keep public API changes intentional and minimal.

Comment functions and methods unless their purpose is completely obvious and a comment would only restate the name or implementation. Give every non-obvious public API function and method a proper `///` rustdoc comment that clearly explains its purpose and documents important behavior, arguments, return values, errors, or panics when relevant. Private functions and methods may use a concise comment, including a single-line comment, that explains what they do or why they exist. Treat the no-comment exception narrowly: omit a comment only when it would be genuinely redundant.

Use `thiserror` as the project's default error-handling tool. Whenever a new error enum is needed, derive `thiserror::Error` and model variants explicitly instead of using ad hoc string errors or placeholder catch-all errors.

Prefer public convenience constructors over spelling enum variants and inner constructors directly. For example, create straight 3D curves with `Curve::line(start, end)` instead of `Curve::Line(Line::new(start, end))`, and apply the same pattern for similar geometry helper constructors when they exist.

Prefer dedicated topology view traversal over low-level dart traversal whenever the typed API can express the intent. For example, use `sheet.vertices().map(|v| v.dart)`, `face.outer_loop()`, `profile.edges()`, or similar domain views instead of manually walking darts, calling `cell_representative`, and reconstructing cells from raw dart orbits. Drop to raw darts only for algorithms that genuinely need alpha-level control.

When editing topology in the builder-layer API, always use topological views to retrieve the appropriate darts. Never access GMap attributes directly from builder code. If the existing view API cannot express the needed traversal or lookup, propose the missing typed topology API to the user instead of relying on lower-level GMap attribute access.

Prefer keeping routine nesting to three indentation levels or fewer. Going deeper is acceptable when the flatter alternative would make the code worse, but the general philosophy is to reduce nesting with early returns, small helper functions, or clearer data tables. Do not add empty helper functions just to satisfy a number; use judgment.

## Tests

Place all tests in a separate `tests/` folder structure that mirrors the project structure under `src/`.

Examples:

- Tests for `src/geometry/dim3/nurbs/surface.rs` belong under `tests/geometry/dim3/nurbs/`.
- Tests for `src/topology/gmap.rs` belong under `tests/topology/`.
- Tests for `src/builders/solids.rs` belong under `tests/builders/`.

Prefer integration-style tests in `tests/` over inline module tests. Add or update tests near the mirrored path for every behavior change unless the user explicitly asks for a mechanical-only edit.

For geometry point equality in tests, prefer the project coincidence helpers over ad hoc norm thresholds. Import `PointCoincidence` and assert `actual.coincides(expected, LINEAR_TOLERANCE)` for `Point3` comparisons unless a test intentionally needs a different tolerance or diagnostic.

## Visualization And Scripts

Rust exploration scripts live in `src/scripts/` and are exposed through `src/scripts/mod.rs`. When adding a Rust script:

- Add the new file in `src/scripts/`.
- Add the module declaration in `src/scripts/mod.rs`.
- Add an entry to the `SCRIPTS` registry with a stable id, title, and `run` function.

Visualization experiments live in `visualization/src/experiments/` and are exposed through `visualization/src/experiments/registry.ts`. When adding a frontend visualization script or experiment:

- Add the experiment directory/component under `visualization/src/experiments/`.
- Add the matching lazy-loaded entry to `visualization/src/experiments/registry.ts`.
- Keep ids stable and consistent with any Rust-side script ids where the frontend calls into wasm.

If a new Rust script is intended to be visible in the frontend, update both registries in the same change.
