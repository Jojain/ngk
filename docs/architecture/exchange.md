# File exchange

`exchange` sits above modeling and tessellation. STEP preserves the B-rep;
STL exports a sampled surface. Kernel geometry and topology do not depend on
either format.

## STL

`exchange::stl` exposes `write_stl`, `stl_to_bytes` and, outside wasm,
`write_stl_file`. Each accepts a typed `Tessellate` view and
`StlWriteOptions`. The default samples a turn in 64 segments and requires a
closed, consistently oriented surface. Set `require_closed` to false to write
an open face or sheet. The format is binary STL: an 80-byte header, a
little-endian triangle count and 50 bytes per facet, with zero attribute bytes.
Coordinates are model coordinates; STL records no unit or placement metadata.

The writer uses the common tessellator, rounds positions to finite `f32`, and
computes each flat normal from those rounded positions. Invalid indices,
empty meshes and degenerate facets are errors. Facets whose distinct positions
round onto one another are omitted as contracted edges. The resulting surface
is checked again: every undirected edge must occur exactly twice in opposite
directions. This detects cracks, T-junctions and inconsistent winding at the
precision the consumer receives. It does not test self-intersection or certify
the solid's geometry. Geometry and validation complete before writing, including
before opening a destination file; I/O failures can still leave partial output.

Python exposes `ngk.exchange.stl.stl_to_bytes(solid, segments=64)` and
`write_stl(solid, path=None, segments=64)`. The browser binding exposes
`stlToBytes(solid, segments?)`, returning a `Uint8Array`, with the same default
sampling and closed-surface validation. Browser callers choose their download
filename. The benchmark runner uses this exporter rather than its own writer.

```rust
use ngk::exchange::stl::{StlWriteOptions, write_stl_file};

let shape = ngk::modeling::solids::cylinder(5.0, 10.0)?;
write_stl_file("cylinder.stl", &shape.solid(), &StlWriteOptions::default())?;
# Ok::<(), Box<dyn std::error::Error>>(())
```
