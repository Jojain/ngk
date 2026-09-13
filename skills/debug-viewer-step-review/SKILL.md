---
name: debug-viewer-step-review
description: Use when visually reviewing or debugging ngk's STEP import/export (src/exchange/step/) or any other ngk geometry — especially when a face, edge, or vertex looks wrong after a STEP round-trip, or when a topology bug (bad sew, wrong alpha link) is suspected. Covers launching ngk's built-in browser-based debug viewer (`npm run debug` in `visualization/`, listening on 127.0.0.1:3941), sending geometry to it from Rust (`ngk::viz::debug_viewer::show`/`show_with_options`, `cargo run --example step_import_viewer`) or Python (`ngk.debug.show`), and reading the resulting 3D scene, dart/alpha overlays, and per-face UV inspector. Use this whenever the user mentions the debug viewer, wants to *see* a shape rather than just check its numbers, or a build123d/ngk assertion fails and the next step is to look at what actually got built.
---

# Debug Viewer STEP Review

The debug viewer is a browser app that receives live ngk geometry over HTTP
and renders it in 3D. It has **no file picker** — it cannot open a `.step`
file by itself. Something must load or build the geometry first (an ngk
example, a test, a script) and explicitly send it over. The one hard rule:
**start the viewer before running anything that sends to it** — the send is
a live TCP connect with a 1-second timeout, so it fails silently fast if
nothing is listening yet.

This complements [build123d-step-testing](../build123d-step-testing/SKILL.md):
that skill answers "is this numerically correct" (volume, bbox, counts);
this one answers "what does it actually look like, and which specific
face/edge/dart is wrong" — the natural next step once an assertion fails
and you need to see why.

## Launching the viewer

```bash
cd visualization
npm run debug
```

This rebuilds the wasm bundle (`wasm-pack build --target web --features
wasm` from the repo root) and serves the viewer on `127.0.0.1:3941` — the
port every Rust/Python sender expects by default. Plain `npm run dev` does
*not* use this port, so don't use it here unless you also set
`NGK_DEBUG_VIEWER_PORT` to match.

Then open the debug viewer experiment itself:

```
http://127.0.0.1:3941/#/debug-viewer
```

Drive this page with the Browser tool (`mcp__Claude_Browser__*`): navigate
to the URL, then use `read_page`/screenshots to see the rendered scene and
inspector panel, and click elements to select/inspect them.

## Sending a STEP file to the viewer

The purpose-built path for STEP review — imports and shows in one step,
so nothing sits between the importer's output and what's on screen:

```bash
# every .step file under tests/exchange/foreign/files/
cargo run --example step_import_viewer
# just specific file(s)
cargo run --example step_import_viewer -- tests/exchange/foreign/files/torus.step
```

It also prints face/edge/vertex/skipped counts to the terminal, giving a
quick cross-check against what you see in the browser before you even
start clicking around.

## Sending arbitrary ngk geometry

Useful for the other direction — checking a shape *before* it's exported to
STEP, or debugging a failing test in place. From anywhere ngk geometry is
in scope:

```rust
use ngk::viz::debug_viewer::{show, show_with_options, DebugViewerOptions};

show(&shape)?;   // GMap, a Vertex/Edge/Profile/Face/Sheet/Solid, geometry
                 // (Point3/Vector3/Plane/Curve/Surface), or a Vec of any of these

show_with_options(&shape, &DebugViewerOptions {
    name: "after_cut".into(),
    ..Default::default()
})?;
```

Dropping a bare `show(&g);` inline in a failing test (see
`tests/builders/revolve.rs:883`) is an accepted way to debug it, not just a
throwaway example pattern. One restriction worth knowing up front: topology
transfer only supports `StandardPayload` maps — a custom payload type can't
be reconstructed by the browser's statically compiled wasm module.

Override host/port via `NGK_DEBUG_VIEWER_PORT` or the fields on
`DebugViewerOptions` if 3941 is unavailable.

## Sending from Python

```python
import ngk.debug as debug

debug.show(gmap_or_cell, name="my_shape")  # GMap, or a vertex/edge/profile/
                                             # face/sheet/solid cell, or a list
debug.clear()                               # empty the viewer's history
```

## Reading the viewer

- **Timeline**: each `show(...)` call is one named, timestamped entry.
  Click one to inspect it. Hit "Clear" (or call `debug.clear()`) between
  runs so old shapes from an earlier debugging session don't linger and get
  mistaken for the current one.
- **3D scene**: toggle vertices/edges/faces, adjust sizes/colors/opacity,
  orbit/pan/zoom like a CAD viewer. A dart/alpha overlay mode shows raw
  darts and per-involution α0–α3 links, with a one-click preset to flip
  from B-rep view to pure combinatorial-map view — reach for this when the
  suspected bug is topological (a bad sew, a wrong alpha link) rather than
  a geometric one.
- **Selection**: click a vertex/edge/face/dart to select it, hover to
  preview; the Inspector panel follows the selection.
- **Inspector**: a face shows loop/edge/vertex counts, surface type,
  pcurve count, and a UV-space SVG of its pcurves — click a pcurve there to
  jump to its 3D edge. This is the tool for pinning down exactly which face
  went wrong after a round-trip, since it shows both the 3D result and the
  underlying parametric surface data side by side. Edges show endpoints,
  length, and incident faces; darts show their alpha table.

Don't confuse this with `src/viz/ocp_vscode.rs`, which is a separate bridge
to the `ocp_vscode`/OCP CAD Viewer VS Code extension.

## Where this fits in the repo

- The viewer lives in `visualization/` alongside every other visualization
  experiment (`visualization/src/experiments/`), sharing the same
  `SceneShell`/`VizSceneView` rendering infrastructure — it's not
  STEP-specific plumbing, just the experiment most useful for STEP review.
- Rust-side entry points: `src/viz/debug_viewer.rs` (the sender API),
  `examples/step_import_viewer.rs` (the STEP-specific driver).
- Treat a `show(...)` call the same way you'd treat a `println!` used for
  debugging: fine to leave in a test while you're actively chasing a bug,
  but not something that belongs in a merged PR unless the test's whole
  point is visual inspection.
