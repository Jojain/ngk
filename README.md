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
- WebAssembly bindings for browser experiments.
- A React Three Fiber visualization app under `visualization/`.

## Experiments

The visualization playground is published with GitHub Pages:

https://jojain.github.io/ngk/

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

## License

NGK is licensed under the MIT License. See `LICENSE`.
