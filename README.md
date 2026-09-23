# NGK

NGK is the Nales Geometry Kernel: an experiment in building a geometry kernel
around generalized maps, or GMaps.

The project explores how geometric modeling operations can be represented by a
combinatorial topological structure first, with geometry attached as payload
data. In a GMap, shapes are decomposed into small topological elements called
darts. Involutions link those darts across dimensions, so vertices, edges,
faces, shells, and higher-dimensional cells are described by traversal through
the map rather than by ad hoc mesh records. This makes operations such as
sewing, unsewing, extrusion, and cell traversal explicit in the topology, while
coordinates and geometric data stay layered on top.

This repository currently contains:

- A Rust kernel under `src/`.
- Python bindings built with Maturin.
- WebAssembly bindings for browser experiments.
- A React Three Fiber visualization app under `visualization/`.

NGK is pre-alpha software. The public API, file compatibility, and modeling
coverage will change as the kernel evolves.

## Python package

Install the minimal native package with:

```bash
pip install ngk
```

The optional OCP viewer bridge is kept out of the core dependency set. Install
it when you want to display shapes through `ocp_vscode`:

```bash
pip install "ngk[ocp]"
```

The Python API follows the kernel's domain structure. Primitive solid builders
and the current solid Boolean operations live under `ngk.modeling`; the latter
are intentionally solid-only for now.

```python
from ngk.geometry import Frame, Point, Vector
from ngk.modeling import booleans, solids
from ngk.viz import ocp

frame = Frame.from_xy(
    Point(0, 0, 0),
    Vector(1, 0, 0),
    Vector(0, 1, 0),
)
base = solids.block(40, 30, 20, frame=frame)
tool = solids.cylinder(8, 30)
result = booleans.cut(base, tool)

ocp.show(result)
```

`ngk.viz.debug.show` can send objects to the NGK debug viewer, but the viewer is
not bundled with the wheel: start it separately from an NGK checkout before
using that bridge.

The current Python surface is useful for primitive solids, solid Booleans,
topology inspection, experimental STEP exchange, and visualization. It is not
yet a complete build123d or OCCT replacement.

## Experiments

The visualization playground is published with GitHub Pages:

https://jojain.github.io/ngk/

The NGK Sandbox is published from the same Pages site:

https://jojain.github.io/ngk/sandbox/

It displays small interactive experiments for inspecting generated geometry and
topology-backed modeling operations. The app is intentionally a playground: it
is useful for trying ideas, checking behavior visually, and making the kernel's
internal structures easier to reason about.

## Local Development

From the repository root, the Rust checks are:

```bash
cargo fmt
cargo clippy --all-targets --all-features
cargo test --all-targets --all-features
```

To run the visualization app locally:

```bash
cd visualization
npm install
npm run dev
```

See `visualization/README.md` for details about adding new experiments.

## Python releases

Prepare the next version with the repository helper:

```bash
uv tool install --editable ./release
bump
```

To let the tool create the commit and annotated tag from a clean `master`
checkout, use `--release` directly:

```bash
bump patch --release
git push origin master --follow-tags
```

The `--release` command changes `Cargo.toml` and `pyproject.toml`, refreshes
`uv.lock`, commits those files, and creates `vX.Y.Z`; it never pushes or uploads
to PyPI itself.

## Architecture Notes

- [Chamfer algorithm](docs/chamfer_architecture.md)
- [Model API direction](docs/model_api.md)
- [Topology identity and orientation](docs/topology_orientation_refactor.md)

## License

NGK is licensed under the MIT License. See `LICENSE`.
