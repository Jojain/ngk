# Logical topology over a pure GMap — done

Status: **Complete (2026-09-15).**

This document is the compact completion record for the logical-topology
refactor. The former file was an implementation journal: it mixed superseded
designs, milestone transcripts, temporary exclusions, and current contracts.
Those historical branches have been removed so this plan describes the kernel
that now exists.

The final invariant and its detailed evidence are recorded in
[one_logical_cell_one_raw_cell.done.md](one_logical_cell_one_raw_cell.done.md).

## Outcome

NGK now has three explicit layers:

1. **Pure GMap** — darts, `alpha0..alpha3`, orbits, cells, sewing, and removal.
2. **Logical Model** — stable entity keys, geometry, payloads, profiles, sheets,
   embedding, transactions, lineage, and validation.
3. **Derived realization and consumers** — cached geometric realizations,
   tessellation, Booleans, STEP, bindings, and visualization.

The GMap does not contain vertices, edges, faces, or solids. Those bare nouns
always mean logical entities. Map-side objects are 0-cells through 3-cells, or
raw cells where the contrast matters.

## Decisions that survived the experiment

### Stable logical identity over combinatorial topology

`Model<P>` owns one `GMap` and six keyed stores: vertex, edge, profile, face,
sheet, and solid. `Shape<K, P>` owns a model plus its primary logical key.
Public modeling operations use keys and typed views; a dart is a short-lived
orientation locator and can be replaced by any edit.

Vertices, edges, faces, and solids each occupy exactly one raw cell of their own
dimension. Profiles and sheets are keyed aggregates rather than cell-owning
entities. Internal topology may be richer than public shape topology because a
raw cell can instead be embedded in a higher-dimensional owner.

### Atomic mutation and explicit lineage

All mutation goes through `Model::transaction` and `ModelEdit`. One public
builder owns one transaction; nested construction uses staged helpers.

Commit validates the raw map, required aggregate registrations, lineage,
identity reconciliation, embedding, and logical cell occupancy before payload
policy runs. Any failure restores the topology, entity stores, embedding,
payloads, revision, and caches together.

Logical splits and merges declare lineage. Reconciliation preserves existing
identities where possible and requires an explicit survivor for collisions
between pre-existing keys. Payload callbacks observe net logical change, not
temporary topology created and consumed inside one operation.

### Boundaries are logical walks

A profile is a connected set of edges; a loop is one face-oriented closed walk
over a profile. A sheet is a connected set of faces; a shell is one oriented
closed boundary component of a solid.

Boundary traversal uses `turn` to pass embedded scaffold:

- a face-owned bridge joins raw topology without joining logical loops;
- a solid-owned cut face joins raw volume without joining logical shells.

This keeps holes, periodic seams, cavities, and handles in pure combinatorial
connectivity while hiding artificial cells from normal shape queries.

### Closed and periodic topology

Closed edges are classified by logical corner count:

- bounded: two distinct corners;
- marked: one deliberate corner visited twice;
- unmarked: no logical corner.

A curve is an unbounded support. `TrimmedCurve` and `TrimmedCurve2` carry the
chosen native-parameter span where endpoints alone cannot. Edge spans are
derived from topology and traversal orientation; pcurves and intersection
sections carry explicit spans.

Periodic faces do not expose permanent seam edges. Their unwrapped parameter
domains synthesize cuts for algorithms and exchange. Boundaryless faces stand
on finite surface-specific raw 2-cell schemas. STEP can therefore add the seams
its file format requires without changing NGK's logical identity.

### Geometry stays above the GMap

Logical attributes own points, support curves, surfaces, pcurves, and payloads.
The GMap remains geometry-free. Model-owned realizations are derived,
revision-aware caches; mutation invalidates them and rollback cannot publish a
stale result.

Boolean intersection and classification operate on logical geometry. Imprints
may temporarily partition topology inside a transaction, but the committed
result must again satisfy logical identity and cell occupancy. Healing uses the
book's removal operation under an explicit policy and reports refusals rather
than approximating geometry.

## Milestones closed

| Milestone | Result |
|---|---|
| M0 — baseline | Existing capabilities and known unsupported cases were separated from migration regressions. |
| M1 — feasibility | Hand-built edge, circle, cylinder, holed face, sphere, torus, handle, and cavity scaffolds proved the topology and oriented walks. |
| M2 — pure core and model | Logical data left GMap; `Model`, `ModelEdit`, embedding, rollback, and derived indexes became authoritative. |
| M3 — logical views | Typed views, orientation, profiles, loops, sheets, shells, correspondence, and realization caches were migrated. |
| M4 — construction | Primitives, extrusion, revolution, holes, periodic domains, boundaryless faces, handles, and cavities build the new representation. |
| M5 — editing and healing | Splitting, imprinting, removal, chamfer, merge/copy, lineage, and payload policy operate transactionally. |
| M6 — Booleans | Preparation, intersection network, selection, assembly, cavity construction, lineage, and default healing use the logical model. |
| M7 — consumers and STEP | Tessellation, visualization data, TCV, examples, Python/WASM compilation, serialization, and STEP import/export use the new model. |
| M8 — closure | Superseded connectivity machinery and terminology were removed; occupancy is a commit invariant; the full Rust suite is green. |

## What was deliberately removed

- payload-bearing or geometry-bearing GMap cells;
- public mutation of `GMap` outside a model transaction;
- logical entities spanning several same-dimensional raw cells;
- same-dimensional ownership records and region flooding;
- `ShellRoot` face fallbacks and optional dart anchors;
- grouped edge occurrences whose purpose was to reconstruct one edge from
  several 1-cells;
- permanent logical seams on periodic faces;
- compatibility wrappers for the replaced topology API.

## Current validation contract

A committed model must satisfy all of the following:

- the involutions form a valid GMap;
- every embedding record names one existing raw-cell orbit and an owner of
  strictly greater dimension;
- every face boundary has a registered profile and every solid shell a
  registered sheet;
- lineage and identity reconciliation are unambiguous;
- every vertex, edge, face, and solid occupies exactly one same-dimensional
  raw cell;
- payload policy succeeds on the final net logical changes.

Construction may violate these temporarily inside a transaction. Commit is the
boundary where all statements must be true together.

## Verification and handoff

The completion baseline is the full all-target/all-feature Rust suite, including
builders, topology, healing, Booleans, STEP, tessellation, visualization data,
bindings compilation, examples, and benches. Focused regressions cover
multi-shell cuts, cavity orientation and round trips, transaction rollback on
occupancy failure, boundaryless faces, bridged loops, and staged healing.

No remaining work belongs to this refactor. Future improvements—broader
free-form intersections, additional analytical pairs, curved-support pcurve
rebuild, Boolean robustness and performance, richer exchange profiles, and
public API polish—are independent plans. They should preserve the contracts in
this completion record rather than reopen the logical/raw topology split.
