# One logical cell, one raw cell — done

Status: **Complete (2026-09-15).**

This focused plan closed the final invariant of the logical-topology migration:

> Every logical vertex, edge, face and solid occupies exactly one raw cell of
> its own dimension. Profiles and sheets are aggregates, so the rule does not
> apply to them.

The converse is intentionally false. A raw cell may have no logical entity of
its own dimension; in that case it is embedded in the higher-dimensional entity
whose interior contains it. Examples are a circle closure 0-cell, a periodic
seam edge, a bridge edge joining face loops, and a cut face joining the shells
of a solid.

## Final architecture

- `GMap` contains only darts and involutions.
- `Model<P>` owns logical attributes, stable keys, geometry, payloads,
  embedding records, transactions, and derived indexes.
- An entity anchor identifies its one same-dimensional orbit. Region recovery
  is an oriented orbit walk, not a flood over several cells.
- `Embedding` stores only lower-dimensional cells inside higher-dimensional
  entities. An entity's own cell is derived from its anchor.
- `turn` is the common boundary primitive. A face boundary turns across a
  face-owned bridge edge; a solid boundary turns across a solid-owned cut face.
- Face loops and solid shells remain distinct boundary components even when
  their owner occupies one raw cell.
- Darts are contextual orientation locators. Logical keys remain the durable
  identities.

## Completed work

### One orbit per entity

The old machinery for reconstructing one entity from several raw cells was
removed. Logical edge occurrences no longer group consecutive raw edges, and
`recover_region` no longer crosses same-dimensional cells. Face, edge, vertex,
and solid views are rooted on real topology.

`validation::validate_cell_occupancy` checks the invariant. Transaction commit
runs it after identity reconciliation and embedding validation, when staged
splits and merges have reached their final identities. A violation is returned
as `ModelEditError::InvalidCellOccupancy`, and the complete transaction rolls
back.

### Embedding, not subdivision

The classification layer is `topology::embedding`:

| Final name | Meaning |
|---|---|
| `Embedding` | Stored lower-dimensional interior cells |
| `EmbeddedCell` | One classified raw-cell orbit |
| `EmbeddingIndex` | Derived dart-to-owner lookup |
| `is_embedded` / `is_embedded_cell` | Scaffold classification predicates |

An embedding record can only name an owner of strictly greater dimension than
the cell. Same-dimensional ownership comes from the logical attribute anchor.

### Boundaryless faces

Every boundaryless face now stands on a finite raw 2-cell built by
`builders::scaffold::add_closed_face_cell`:

- sphere-like closure: a four-dart bigon, one embedded edge, two embedded
  poles, Euler characteristic 2;
- torus-like closure: an eight-dart square with opposite sides identified, two
  embedded edges, one embedded vertex, Euler characteristic 0.

The builder dispatches from `Surface::periodicity` and refuses a closed support
without a known schema. Sphere and torus construction, full-turn revolution of
a closed edge, and STEP boundaryless-face import share this path.

`FaceBoundary::{Loops, Closed}` keeps the anchor and boundary state together.
`Face::dart`, `Sheet::dart`, `Solid::dart`, and their attribute equivalents are
total. The former `ShellRoot` enum, face-root fallbacks, optional darts, and
their merge and validation branches are gone.

### Faces with holes

`cut_between_loops` splices two closed boundary walks with one face-owned raw
edge used twice. The face remains one 2-cell, while `boundary_cycles` turns over
the bridge and returns the original loops separately. Annuli, holed polygons,
periodic bands, face imprinting, and healing use this representation.

### Solids with cavities

`cut_between_shells` is the dimension-three analogue. It opens one edge on each
boundary shell, spans all shells with two reflected polygons, sews the polygons
through `alpha3`, and classifies the resulting raw face and connector edges
inside the solid. One cut handles the outer shell and any number of void
shells.

This has three important consequences:

- all material belongs to one raw 3-cell;
- `boundary_shells` turns across the cut and still returns distinct outer and
  inner shells;
- the cut has no face key, surface, or public boundary occurrence.

Boolean result assembly and STEP import create this scaffold whenever a solid
has inner shells. STEP seam healing runs inside the import transaction before
the solid cut is attached, because a synthetic periodic seam ceases to be a
pure face seam once a cut face turns through the same raw edge. The hollow
sphere test fixture builds the same topology directly with `ModelEdit`; there
is deliberately no shape-specific hollow-sphere builder, because production
modeling obtains that shape by Boolean difference.

Sheets were corrected at the same time: a sheet is indexed and reconciled by
its logical boundary component, not by the containing 3-cell. Its faces are
enumerated from that component. During an intentionally open staged edit, the
view temporarily falls back to the raw component until the replacement closes.

### Healing correctness found by the invariant

The last healing regression did not require a new two-component rejoin rule.
Several removals in one transaction could resolve through a face key already
declared absorbed. That alias retained only the shrinking inner boundary and
forgot the outer one, eventually leaving a self-looping slit.

The repair follows staged face-merge lineage to the final survivor and keeps
the survivor's complete, orientation-adjusted boundary list on aliases until
commit removes them. `imprinted_face_inner_loop_gets_removed` now heals the
filled island to one face and four edges.

Other defects exposed during the migration were also fixed: cross-model merge
preserves boundaryless faces, a healing pass no longer writes through a stale
face identity, and seam demotion preserves the old face winding.

## Acceptance evidence

- Ordinary primitives, annuli, boundaryless sphere and torus faces, imported
  seams, Boolean cavities, and hollow STEP round trips satisfy the occupancy
  validator.
- A solid with three boundary shells occupies one raw 3-cell and still reports
  three boundary components.
- A face deliberately made to reference a second raw 2-cell is rejected by
  name at commit and the added topology and pcurve are rolled back.
- A face without an anchor is unrepresentable by `FaceBoundary`.
- The complete `cargo test --all-targets --all-features` run passes.
- Formatting, diff checks, and all-target/all-feature Clippy are part of the
  final verification for this completion.

The broader migration record is
[logical_topology_over_gmap.done.md](logical_topology_over_gmap.done.md).
