# Offset Architecture

Offsets live in two layers:

- `geometry/dim3/offset.rs` moves a support: `Surface::offset(d)` along its
  normal, `Curve::offset(D, d)` along `T × D`. See
  [geometry.md](geometry.md#offsets-geometrydim3offsetrs).
- `builders/offset/` moves an entity: an edge, profile, face, sheet or solid.
  `modeling::offset` wraps it for owned shapes (`offset` for faces, sheets and
  solids, `offset_along` for edges and profiles).

```rust
offset_edge(&mut model, edge, direction, d)       -> EdgeOffset
offset_profile(&mut model, profile, direction, d) -> ProfileOffset
offset_face(&mut model, face, d)                  -> FaceOffset
offset_sheet(&mut model, sheet, d)                -> SheetOffset
offset_solid(&mut model, solid, d)                -> SolidOffset
```

## Copy, then re-embed

**Every offset is a new entity beside its source.** The source is merged into
the same model as a copy (`ModelEdit::merge` from a clone of the model) and the
copy is moved. The source keeps its key and geometry, and an operation that
needs both — thicken, shell — has both. A copy shares nothing with what
surrounded its source, so it is closed under incidence: moving it tears
nothing.

**Moving keeps the topology** (`reembed.rs`). An offset that loses no edge and
no face has its source's topology, so it is written the way a rigid motion is:
plan every new surface, curve, point and pcurve from the model as the call
found it, then write them. No dart changes. What would need new topology is
refused by name instead.

## How each entity moves

- **Face**: its surface is offset by `d` signed by which way the face faces its
  support (`face_sign`), so a positive distance moves every face of a solid
  outward.
- **Edge**, decided by its faces:
  - *preserved* — a wire edge, a free boundary, or an edge its two faces
    continue each other across (normals agree all along it). It moves along
    its own normal keeping its parameterization (`on_surface.rs`, exact for
    curves on planes, lines with a constant normal, and rims the surface turns
    about), so its pcurves stay valid.
  - *sharp* — its faces meet at an angle. The edge becomes the intersection of
    the two moved faces (analytic table first), the branch nearest where its
    middle goes, run the old way; a circle keeps its old phase. This is the
    intersection join; a rolling-ball join adds faces, which re-embedding
    cannot.
- **Vertex**, decided by its edges:
  - all preserved: along the normal;
  - a preserved edge among sharp ones: slid along that edge's moved support
    to where it meets every moved face around (the end of a round running into
    a face), checked to lie on every preserved edge's moved support too;
  - two faces along one sharp edge (a rim's corner): the nearest point of the
    edge's new support;
  - otherwise: the point where all its faces' moved surfaces meet, refined by
    Newton from their moved tangent planes;
  - on a wire: where its two moved edges meet, or along the normal where they
    continue each other.
- **Pcurves**: kept for a preserved edge whose corners moved along the normal;
  rewritten on the moved surface otherwise, shifted onto the branch the old
  pcurve lived on.

## Refused

| Error | When |
|---|---|
| `EdgeSupport`, `FaceSupport` | a support has no offset: a radius consumed, a fold, a tangent along `D` |
| `EdgeVanishes` | the moved neighbours meet past an edge's far end (the old middle is no longer inside the new span) |
| `CornerGap` | two moved wire edges no longer meet |
| `UndecidedVertex` | the moved faces and edges around a corner do not meet at one point |
| `SharpEdge`, `SharpEdgeIntersection` | the two moved faces along a sharp edge do not meet near it |
| `Pcurve` | a moved edge has no pcurve on a moved face |
