# Experiment: logical topology over a pure GMap subdivision

Status: **In progress** — M0, M1 and M2 complete.

Implementation guide: section 1 fixes the architecture, section 2 defines the
milestone gates, and section 5 supplies the implementation sequence, concrete
fixtures, API contracts, and evidence required for each milestone. Complete a
gate before beginning the next broad migration. The prototype in M1 is a
feasibility test, not evidence that arbitrary cavity construction is solved.

## 1. Objective and fixed architectural decisions

Refactor the existing kernel in place so that **GMap supplies all computational connectivity**, while a separate logical topology exposes meaningful, stable modeling entities.

The experiment must preserve currently working modeling, geometry, exchange, tessellation, and binding capabilities. Breaking internal and public APIs is allowed. Do not maintain an old implementation, compatibility wrappers, or a permanent experimental feature flag.

### Ownership and APIs

- Promote `Model<P>` to own the pure GMap, logical slotmaps, subdivision classification, geometric correspondence, and derived caches.
- `Shape<K, P>` owns a `Model<P>` plus its primary logical key. Replace its map accessors with `model()`, `model_mut()`, and `into_model()`.
- Retain keyed logical vertices, edges, faces, solids, profiles, and sheets, including their payload capabilities.
- Profiles and sheets are keyed aggregates above GMap. Face loops and solid shells are derived oriented boundary components, not authoritative membership lists.
- Public modeling operations consume or edit logical entities. Retain the existing standalone builder style and owned-shape Boolean inputs.
- Ordinary users get read-only subdivision diagnostics. Mutation of a model occurs through one atomic `ModelEdit` transaction; staged computational editing remains internal.
- Keep `Dart` and dimensions 0–3 in the geometry-free GMap. Raw cell identity is an orbit, not a public entity key.

### Subdivision classification

Classify each computational cell by the logical entity whose interior contains it. Ownership is constant across each raw cell orbit, and the owner's dimension cannot be lower than the raw cell's dimension.

Examples:

- Cylinder seam edge → wall face.
- Circle closure vertex → circular edge.
- Internal cutting face → solid.
- Actual boundary edge → logical edge.

Store classification as orbit-associated records with representative darts and derived lookup indexes. Do not store authoritative lists of all darts or patches belonging to an entity.

Recover logical regions through oriented traversal across internal subdivisions:

- Edges continue through vertices internal to that edge.
- Faces continue across edges internal to that face.
- Solids continue across faces internal to that solid using α3.

Preserve occurrence orientation and repeated uses. Ownership labels alone do not replace these incidence distinctions.

### Geometry and identity

- Logical supports, real vertex locations, and logical edge-use pcurves are authoritative model data.
- Artificial computational cells may remain geometrically abstract.
- Algorithms request a lazy, cached realization only for computational pieces they actually interpret geometrically.
- Persist any correspondence necessary to preserve a chosen logical section or modeling result. Such information must not exist only in a disposable cache.
- Keep full closed edges whole; reverse existing native spans without independently wrapping their endpoints.
- Internal refinement preserves logical keys and payloads.
- A logical split retains the original key on the result containing the reference anchor. Capture the survivor before deleting or replacing that anchor; create keys and lineage for other results.
- Logical merges explicitly name their survivor and apply payload policy.
- Serialization preserves logical keys within the serialized model. Cross-model import returns remapping; keys are not globally unique identifiers.

## 2. Milestones and acceptance gates

### M0 — Record the baseline and migration ledger

Before changing implementation:

- Record the repository revision, existing uncommitted changes, test results, existing ignores, and environment failures.
- Inventory working capabilities across topology, construction, extrusion/revolution/sweep, chamfer, splitting/imprinting, Booleans, healing, validation, STEP, tessellation, Python, WASM, and visualization.
- Record existing Boolean timing ceilings and representative operation timings.
- Capture small representative fixtures and their geometric invariants. Do not use raw dart counts as feature-parity criteria.
- Add a migration ledger to this plan: capability, baseline evidence, milestone, temporary exclusion, replacement test, and completion evidence.

**Gate:** previously working behavior is distinguishable from known failures and unimplemented features. No existing unsupported case is silently counted as a regression or as newly supported.

### M1 — Prove the architecture before broad migration

Build a small vertical prototype using the existing involution and orbit implementation.

Prove these configurations:

- Open edge and closed circle.
- Capped cylinder: three logical faces, two edges, zero vertices.
- Planar face with a hole.
- Whole sphere and torus: one logical face, zero edges and vertices.
- A solid with a handle.
- A solid with a closed cavity, traversable from one logical-solid anchor through computational connectivity.

For faces with holes, connect the computational boundary through artificial cuts. For cavities and handles, construct genuine combinatorial volume subdivisions with internal face adjacencies; do not identify unrelated boundary faces or fill a void to make traversal succeed.

Include the boundary-use traversal that reconstructs separate logical loops and shells. A filtered list of raw cells is insufficient.

**Gate:** raw involution validation, oriented logical traversal, and expected topology all pass without the old face-loop or shell-root connectivity indirections.

If full dimension-3 connectivity cannot be demonstrated correctly, stop the migration and document the failed experiment. Do not quietly relax the agreed pure-core objective.

### M2 — Extract the pure core and introduce `Model<P>`

Refactor the current implementation rather than rewriting its algorithms:

- Move logical attributes, payload handling, geometry, and logical indexes out of GMap.
- Reuse dart storage, involutions, orbit traversal, and combinatorial edit primitives.
- Introduce orbit ownership records, logical entity stores, and ownership-aware traversal.
- Move transactions to `ModelEdit`, covering topology, logical stores, correspondence, identity reconciliation, and cache invalidation atomically.
- Separate computational refinement from logical split/merge operations.
- Migrate `Shape` and the foundational typed views to the model owner.

**Gate:** rollback restores all authoritative state; orphaned entities, conflicting orbit ownership, and invalid references are rejected. Raw refinement and representative replacement preserve logical identities.

### M3 — Establish logical views, uses, and geometric correspondence

- Implement logical vertex, edge, face, solid, profile, and sheet traversal.
- Remove raw darts from ordinary public views; use logical keys and explicit contextual orientation.
- Represent logical boundary occurrences independently of raw dart identity so pcurves survive computational re-subdivision.
- Preserve loop roles, periodic wrapping, capping, and pcurve orientation.
- Implement ordered boundary walks and deterministic first-seen enumeration. Hash tables serve lookup/deduplication, never output ordering.
- Implement revisioned realization caches outside GMap. Relevant edits invalidate them; failed transactions cannot leave reusable stale realizations.
- Keep Boolean spatial classification on logical geometry unless an operation explicitly requests an embedded computational partition.

**Gate:** cold-cache and warm-cache queries agree; changing an artificial cut or refining its cells preserves logical geometry, identities, adjacency, and oriented boundary cycles.

### M4 — Migrate construction and full-dimensional connectivity

Migrate existing builders and standalone modeling functions to create classified subdivisions:

- Vertices, bounded and closed edges, profiles, faces, and sheets.
- Planar faces with holes and curved periodic domains.
- Blocks, cylinders, spheres, cones where currently supported, and tori.
- Extrusion, revolution, sweep, and existing placements.
- Solids with handles and cavities.

Use constructor knowledge to establish artificial-cell ownership. Preserve explicitly requested subdivisions and real junctions.

Replace authoritative face-hole and cavity-shell connectivity storage with computational cuts. Retain derived loop/shell views and required geometric domain information.

**Gate:** supported constructors match their baseline geometric behavior; logical cylinder/sphere/torus counts satisfy the new contract; cavity orientation and material containment remain correct.

### M5 — Migrate editing, identity, and healing

Migrate edge splitting, face imprinting, sewing/unsewing, removal, chamfer, and existing copy/merge operations.

Distinguish:

- Computational subdivision: inherits logical ownership without allocating public entities.
- Logical subdivision: creates public boundaries, keys, and lineage.
- Computational cleanup: changes the scaffold only.
- Logical healing: merges meaningful entities under the existing explicit healing policy.

Handle the closed-curve progression correctly:

- No marked points → vertex-free logical edge.
- One deliberate marked point → one closed edge through a logical vertex.
- Two distinct marked points → two logical arcs.

**Gate:** every incident use and pcurve changes consistently; payload callbacks run only for logical changes; cache invalidation and serialization remain correct after editing.

### M6 — Migrate the Boolean pipeline

Retain the existing analytic dispatch, NURBS fallback, intersection network, tolerances, and diagnostics.

Assign responsibilities explicitly:

1. Candidate selection and intersections use logical geometry.
2. Intersection realization refines the computational subdivision.
3. Existing geometric classification determines selected result regions.
4. Assembly establishes logical ownership, entities, and lineage.

An intersection crossing an artificial cut must not automatically create a logical vertex or fragment. Real result boundaries are promoted deliberately.

Migrate preparation APIs as well as final union, intersection, and difference. Preserve default healing semantics.

**Gate:** baseline working Boolean configurations pass, including their geometry and manifold assertions. Add seam-crossing and computational-refinement invariance cases. Preserve existing termination ceilings; report timing and memory changes.

### M7 — Restore all consumers and exchange

- Tessellation consumes logical geometry and boundary uses, requesting temporary cut realizations where needed.
- Normal rendering and picking expose logical entities; explicit subdivision diagnostics display raw cells and owners.
- Migrate Rust visualization, TCV, examples, Python, and WASM together with their changed APIs.
- STEP export realizes a suitable exchange subdivision without mutating logical identity.
- STEP import classifies verified artificial topology as internal while retaining intended boundaries; do not classify every smooth edge as artificial.
- Migrate serialization to the new model layout. No backward-format compatibility is required.

**Gate:** STEP round trips preserve shape, holes, cavities, and logical adjacency; rebuilt bindings expose the new topology correctly; normal display has no artificial seam lines or closure markers.

### M8 — Close the experiment and assess it

- Remove superseded hybrid connectivity code, migration shims, and obsolete API wording.
- Resolve every experiment-created test exclusion.
- Update plan status and architecture documentation to describe the final implementation.
- Run complete validation and compare the capability ledger and measurements with M0.
- Record whether the design fulfilled its promises: pure computational connectivity, stable logical identities, seamless public topology, and restored feature coverage.

Do not mark the experiment complete with remaining migration regressions. A negative architectural result is reported explicitly, not hidden behind additional exceptions.

## 3. Validation and temporary test policy

Write tests for intended behavior before each changed capability. Retain low-level GMap invariants separately from logical modeling invariants.

Temporary exclusions are allowed only when documented in the ledger:

- State why the old assertion conflicts with the new representation.
- Assign the milestone that restores or replaces its coverage.
- Keep a runnable subset for each completed milestone.
- If a removed API prevents compilation, temporarily exclude the affected test module explicitly; `#[ignore]` alone does not solve compile failures.
- Never weaken geometric correctness, orientation, rollback, or manifoldness assertions merely to obtain a green run.

Mandatory acceptance cases include:

- Logical counts for cylinder, sphere, and torus.
- Ordered loops, reversed views, repeated uses, and face holes.
- Handles and enclosed cavities with pure computational connectivity.
- Anchor deletion, logical split/merge lineage, and payload preservation.
- Whole-circle and seam-crossing parameter spans.
- Cold/warm caches and cut relocation yielding equivalent logical results.
- Cross-model copy/remapping and serialization.
- Baseline construction, Boolean, healing, tessellation, and STEP behavior.

For completed migration checkpoints, run formatting, diff checks, relevant focused tests, and the project's all-target/all-feature Clippy and test checks. Rebuild Python before binding checks. Run frontend typechecking and builds; report environmental blockers separately and follow the existing Windows build-permission rule.

Do not add visualization-specific tests; validate those consumers through builds and direct execution.

## 4. Experiment boundaries and defaults

- This is an in-place replacement built from existing code, not a second kernel.
- Full pure computational connectivity through dimension 3 is required.
- Profiles and sheets retain slotmap identities and payloads.
- New parallel execution is out of scope; immutable traversal must remain compatible with future parallel use.
- Stable cyclic order is required. Stable enumeration positions across arbitrary logical edits are not promised.
- Existing unsupported features need not be implemented, except the new architectural acceptance cases above.
- No arbitrary performance-regression allowance is assumed. Preserve existing hard ceilings and document measured changes before judging the experiment successful.

## 5. Implementation manual

### Shared implementation contracts

These contracts apply throughout the milestones. Names below are target names;
helper signatures may follow established Rust conventions, but their ownership
and semantics must not drift during implementation.

#### Module responsibilities

| Location | Final responsibility | Reuse or migrate |
|---|---|---|
| `src/topology/gmap.rs`, `dart.rs` | Geometry-free darts, involutions, raw orbits and combinatorial validation | Existing slotmap storage and alpha operations |
| `src/model.rs` | `Model<P>` owner, lookup, transactions, import and serialization boundary | Existing embryonic Model; stores currently inside GMap |
| `src/topology/subdivision/` | Orbit ownership, oriented raw occurrences, region traversal, cut construction and correspondence | New interpretation layer built around existing orbits |
| `src/topology/attributes.rs`, typed view modules | Logical records and key-based oriented views | Existing geometry, payloads and public view behavior |
| `src/topology/edit.rs` | `ModelEdit`, reconciliation and internal staged refinement | Existing rollback, edit events and payload policies |
| `src/topology/validation.rs` | Logical and embedding checks composed with raw validation | Existing geometric and manifold predicates |
| `src/builders/` | Geometry-aware construction and editing on `ModelEdit` | Existing builders, never a second implementation family |

Keep pure-core and logical validation separate even if the initial extraction
uses adjacent files. GMap must not import geometry, logical keys, Payload,
Profile, Sheet, or Model. A core edit may report changed darts/orbits; it must
not allocate an EdgeKey or choose a geometric tolerance.

#### Persistent state versus derived state

The following is a structural sketch, not standalone compilable code:

```rust
struct Model<P: Payload> {
    topology: GMap,
    entities: LogicalEntities<P>,
    subdivision: Subdivision,
    correspondence: GeometricCorrespondence,
    revision: u64,
    caches: DerivedCaches,
}

enum EntityOwner {
    Vertex(VertexKey),
    Edge(EdgeKey),
    Face(FaceKey),
    Solid(SolidKey),
}

struct OrbitOwnership {
    dimension: Dim,
    representative: Dart,
    owner: EntityOwner,
}
```

- Persist darts, logical slotmaps, ownership representatives, oriented logical
  use records, domain metadata, and necessary correspondence constraints.
- Derive dart-to-orbit indexes, reverse incidence indexes, boundary walks,
  region membership and artificial geometric realizations. Do not serialize
  these caches.
- Profiles and sheets are aggregates, not `EntityOwner` alternatives. Their
  records anchor oriented logical chains/components and carry user payloads.
- A raw GMap has formal orbits in every dimension even for a standalone wire
  or sheet. Record the active modeling dimension of each component. Require
  ownership only for its active cells and their lower-dimensional incidences;
  do not invent solid entities for the formal 3-orbit of an isolated edge.
  Reject dangling ownership records; permit unlabeled ambient formal orbits.
- One raw cell has one owner in this experiment. Distinct logical entities
  cannot occupy the same raw cell interior. Deliberate overlapping objects use
  separate subdivisions until a modeling operation reconciles them.

#### Oriented occurrences and boundary metadata

An orbit identifies a cell, not every way a parent uses it. Keep an internal
`EdgeUseKey` slotmap for logical edge occurrences. Each record identifies the
logical edge, logical face, orientation, pcurve and a relocatable oriented
subdivision anchor. A repeated edge on the same face has distinct use keys.
Computational refinement preserves these keys and updates their anchors.

Store no authoritative ordered vector of uses on a face. Derive ordering and
loop membership from the subdivision. Keep loop domain metadata (outer, inner,
wrapping, capping) attached to a relocatable boundary occurrence; that metadata
does not serve as an alternative connectivity graph. Closed boundaryless faces
have no such records. A face reversal returns a view with flipped sense;
it does not rewrite persistent pcurves as a side effect of reading.

For alpha3-shared logical faces, retain parent-side orientation separately
from the face's default orientation. A face-use or shell traversal state must
include its parent context; a `HashSet<FaceKey>` is insufficient for validating
all occurrences. Distinct entities are deduplicated only in APIs promising
distinct entities.

#### Traversal and refinement contract

Implement region walks with a queue/stack and visited oriented states local to
the walk. Enumerate raw neighbors in a fixed alpha order. For an entity of
dimension k, begin with a raw k-cell and cross a boundary (k-1)-cell only when
that boundary is owned by the same logical entity. Use the existing appropriate
sewing involution, preserving side and orientation; do not implement a flood
fill across all alpha links indiscriminately.

Boundary extraction walks the frontier of the recovered region and bypasses
internal cuts. For a face, it must turn across the paired artificial-cut
occurrence instead of emitting that cut, ultimately yielding separate cycles.
Group consecutive computational pieces belonging to the same logical edge use
before returning a logical boundary walk. Do not join nonconsecutive repeated
uses merely because their EdgeKey matches.

Raw edits return a `RefinementDelta`: affected dimension/orbit anchors,
surviving and created representative candidates, deleted darts, and ordered
piece correspondence when supplied by the geometric caller. The subdivision
layer consumes this delta before exposing staged logical views. Initially
rebuild indexes globally for correctness; incremental rebuilding is not a
milestone requirement.

### M0 implementation — baseline evidence and runnable checkpoints

**Start with:** the test entrypoints in `tests/*.rs`, mirrored test directories,
`Cargo.toml`, `benches/booleans.rs`, binding tests and examples, and
`visualization/package.json`. Use current source and execution as authority,
not the completion labels in older plans.

1. Record `git status --short` and the starting revision. Preserve unrelated
   edits. Do not use restore/stash to manufacture a clean baseline.
2. Run `cargo test --all-targets --all-features` and
   `cargo clippy --all-targets --all-features`. Record exact failing tests,
   compile failures and environment blockers, not just a total.
3. Inventory registered tests and existing `#[ignore]` annotations. If the
   broad run is blocked, run smaller working targets and mark the remainder
   unverified. An unverified capability cannot later be called preserved
   solely because no baseline failure was recorded.
4. Run representative binding examples using the current rebuild workflow;
   record Python commands, frontend typecheck and available viewer checks.
5. Run existing benchmark scenes under their watchdogs. Record profile,
   machine, input dimensions, timings and solver counters; use the same setup
   at M6/M8. Do not introduce unrelated optimization changes.
6. Add the ledger below and populate it with actual test names and evidence.

#### Recorded baseline

Revision `6878266` on branch `topology`. The working tree at the time of
recording held two staged files, both in `plan/`: a modified `README.md` and
this new plan. No source, test or binding file was modified, and nothing was
restored or stashed to obtain that state.

Machine: AMD Ryzen 7 9700X (8C/16T), 31.2 GB RAM, Windows 11 Pro 10.0.26200.
Toolchain: cargo 1.95.0, rustc 1.95.0. Profile: `dev` (`opt-level = 0`,
`debug = 2`) for every timing below, because `--all-targets` builds the bench
target in test mode.

Commands, exactly as run from the repository root:

```powershell
cargo test --all-targets --all-features
cargo clippy --all-targets --all-features
powershell -NoProfile -ExecutionPolicy Bypass -File bindings\python\run.ps1
.venv\Scripts\python.exe -m pytest bindings\python\tests -q
cd visualization; npm run typecheck
```

`cargo test --all-targets --all-features` exits 0. 682 Rust tests pass, none
fail and none are ignored — a repository-wide search for `#[ignore]` returns
nothing, so there is no pre-existing exclusion to distinguish from an
experiment-created one.

| Target | Passing |
|---|---|
| `src/lib.rs` unit tests | 25 |
| `tests/builders.rs` | 145 |
| `tests/exchange.rs` | 188 |
| `tests/geometry.rs` | 188 |
| `tests/healing.rs` | 31 |
| `tests/modeling.rs` | 30 |
| `tests/scripts.rs` | 1 |
| `tests/tcv.rs` | 4 |
| `tests/tessellate.rs` | 2 |
| `tests/topology.rs` | 63 |
| `tests/viz.rs` | 5 |

`cargo clippy --all-targets --all-features` exits 0 with 26 distinct warnings
(22 on the lib, 1 on the `ngk` binary, 3 on the `builders` test target). Two of
them name state this experiment removes: an unread `map` field on `Model<P>`,
and two unused `Shape` inner accessors.

`npm run typecheck` in `visualization/` passes. `npm run build` and
`npm run wasm:build` were not run: the wasm build is blocked by Windows
permissions on this machine and is the user's to run. Compilation under the
`wasm` and `python` features is still covered, because `--all-features` builds
both.

The Python extension rebuilds and `bindings/python/examples/explore_block.py`
runs. `pytest` itself was absent from `.venv` and was installed with
`uv pip install pytest` to take this measurement.

##### Known failures at baseline

These fail before any experiment work. None may later be counted as a
regression, and none may be counted as newly supported if it starts passing.

1. `bindings/python/tests/test_block.py` fails at collection, not in a test:
   the committed file ends with three stray lines that bind a `pathlib.Path`
   and then evaluate `c.s`, raising `AttributeError` on import. The 7 tests in
   that file are therefore unmeasured, not passing.
2. `bindings/python/tests/test_step.py::test_geometry_that_cannot_be_written_raises_rather_than_writing_garbage`
   fails with `DID NOT RAISE ValueError`. The assertion describes a kernel that
   refuses to export a cylinder wall; export now synthesizes the seam, so the
   test names behaviour the tree no longer has.
3. `benches/booleans.rs` scene `sphere_union_sphere` fails its watchdog run with
   `Boolean tolerance policy contains an invalid or non-finite budget`, and is
   reported as not measured.
4. `benches/booleans.rs` scene `orthogonal_cylinders` fails after 10.3 s with
   `sewing endpoints disagree for span IntersectionSpanId(0)`, and is reported
   as not measured.

The remaining 23 Python tests, in `test_exploration.py`, `test_public_modules.py`,
`test_step.py` and `test_tcv.py`, pass.

##### Baseline timings

Bench scenes under their own watchdogs, `dev` profile, one watchdog run each.
These are the numbers M6 and M8 compare against; they are not release figures.

| Scene | Ceiling | Wall clock | Profile total | Assembly stage | Solver calls |
|---|---|---|---|---|---|
| `block_union_block` | 5 s | 169.3 ms | 163.8 ms | dominant | 0 s/s, 0 c/s |
| `block_union_cylinder` | 30 s | 241.4 ms | 188.7 ms | dominant | 0 s/s (2 analytic), 0 c/s (48 analytic) |
| `block_difference_cylinder` | 30 s | 299.5 ms | 249.3 ms | 202.7 ms | 0 s/s (2 analytic), 0 c/s (48 analytic), 16 2D c/c |
| `block_union_sphere` | 30 s | 73.5 ms | 70.0 ms | 56.0 ms | 0 s/s (6 analytic), 0 c/s (41 analytic), 6 2D c/c |
| `sphere_union_sphere` | 30 s | fails in 2.2 ms | — | — | — |
| `orthogonal_cylinders` | 30 s | fails in 10.31 s | — | — | — |

Assembly dominates every scene that completes. That is the stage M4's scaffold
builder and M6's assembly step replace, so it is the number to watch.

##### Representative fixture inventory

Fixtures that already exist and stand in for the shapes this plan names. No new
measurement feature was added to record the baseline.

| Shape | Where it comes from |
|---|---|
| Block | `ngk::modeling::solids::block` / `block_at` |
| Cylinder | `ngk::modeling::solids::cylinder` / `cylinder_at` |
| Sphere | `ngk::modeling::solids::sphere` / `sphere_at` |
| Torus | `ngk::modeling::solids::torus` / `torus_at` |
| Holed slab | `tests/exchange/foreign/files/holed_slab.step`, `ngk::modeling::faces::polygon_with_holes` |
| Cavity | `tests/support/hollow.rs::hollow_sphere` |
| Open wire | `ngk::modeling::profiles::polyline` |
| Marked circle | `ngk::modeling::edges::circle` with `tests/topology/edge_split.rs` |
| Seamed periodic faces | `tests/support/seamed.rs` |
| Custom payloads | the payload-policy tests in `tests/topology/edit.rs` |
| Boolean pairs | `benches/booleans.rs` scenes, `tests/builders/boolean.rs` |
| STEP round trips | `tests/exchange/step_round_trip.rs`, `tests/exchange/foreign/files/*.step` |

#### Migration ledger

| Capability | Baseline command/test | Baseline result | Owner milestone | Temporary exclusion | Replacement/equivalent assertion | Final evidence |
|---|---|---|---|---|---|---|
| Raw gmap axioms and orbits | `tests/topology.rs` `gmap`, `topology::gmap::tests` | 10 pass | M2 | none | same tests, on a `GMap` holding no logical stores | `validate_gmap` takes `&GMap`; the merge tests moved to `tests/topology/model.rs` |
| Orbit ownership and region recovery | none — new capability | n/a | M1 | n/a | `tests/topology/subdivision.rs` | 23 tests pass; see the M1 result |
| Transactions, lineage, rollback | `tests/topology.rs` `edit`, `transaction` | 24 pass | M2 | none | same tests retargeted at `ModelEdit` | 24 pass, plus 11 in `model_state.rs` |
| Derived cell indexes | `tests/topology/indexes.rs` | 2 pass | M2 | none | ownership index rebuild tests | 2 pass; warm/cold and deserialize-rebuild cases added |
| Typed views | `tests/topology.rs` `edge`, `face`, `profile`, `sheet` | 23 pass | M3 | none | same tests against `(&Model, key, sense)` views | |
| Face loops, holes, periodic domains | `tests/topology/unwrapped_face_domain.rs`, `tests/topology/planar.rs` | 12 pass | M3 | none | frontier-walk loop extraction | |
| Serialization | `tests/topology/serialization.rs` | 2 pass | M7 | none | versioned Model layout round trip | |
| Validation | `tests/topology/validation.rs` | 3 pass | M2 | none | raw checks on `GMap`, logical checks on `Model` | split into `GMapValidationError` and `ModelValidationError` |
| Edge/profile/face/sheet construction | `tests/builders.rs` `faces`, `profiles`, `solids` | 58 pass | M4 | none | same builders on `ModelEdit` | |
| Extrusion, revolution, sweep | `tests/builders/revolve.rs`, `tests/modeling/*` | 49 pass | M4 | none | same tests, classified scaffolds | |
| Edge splitting | `tests/topology/edge_split.rs` | 7 pass | M5 | none | logical versus computational split tests | |
| Removal and chamfer | `tests/builders/removal.rs`, `tests/builders/chamfer.rs` | 28 pass | M5 | none | same tests | |
| Healing | `tests/healing.rs` | 31 pass | M5 | none | same tests | |
| Boolean pipeline | `tests/builders/boolean*.rs` | 55 pass | M6 | none | same tests plus seam-crossing and cut-invariance cases | |
| Boolean timing ceilings | `benches/booleans.rs` | 4 of 6 scenes measured | M6 | none | same scenes, same ceilings | |
| Tessellation | `tests/tessellate.rs`, `tests/tcv.rs` | 6 pass | M7 | none | same tests on logical geometry | |
| STEP import/export/round trip | `tests/exchange.rs` | 188 pass | M7 | none | same tests | |
| Visualization scene assembly | `tests/viz.rs` | 5 pass | M7 | none | same tests; viewers checked by execution | |
| Scripts registry | `tests/scripts.rs`, `scripts::*::tests` | 11 pass | M7 | none | same tests | |
| Python bindings | `pytest bindings/python/tests` | 23 pass, 1 fail, 7 uncollected | M7 | none | same tests after `maturin develop` | |
| WASM bindings and frontend | `cargo clippy --features wasm`, `npm run typecheck` | both pass | M7 | none | same checks | |

Retain small deterministic fixtures for block, cylinder, sphere, torus, holed
slab, cavity, open wire, marked circle, custom payloads, representative working
Boolean pairs, and STEP round trips. Compare geometry using existing tolerance
helpers, containment, bounds and existing measure APIs. Do not add a new mass
properties feature just to establish a baseline. Raw counts remain meaningful
only for explicitly low-level tests.

**Deliverables:** populated ledger, repeatable command list, known-failure list
and representative fixture inventory. No feature baseline is inferred from an
older memory or plan.

### M1 implementation — a bounded proof using current raw operations

This milestone precedes extracting every store. Exercise the existing raw dart
operations with no domain attribute registration; add the proposed subdivision
records and walkers beside them. Test-only construction helpers are permitted.
They must call the existing core rather than duplicate alpha manipulation.
Promote the successful helpers during M2/M4; do not leave a second kernel.

Build fixtures in this order:

1. **Segment and circle.** Segment endpoints own real vertices. On the circle,
   classify the closure vertex inside the edge. Verify zero logical vertices,
   one cyclic edge, both orientations and termination of all walks.
2. **Cylinder.** Build a seamed combinatorial cylinder and classify the vertical
   seam in the wall and both closure vertices in their rim edges. Check the
   cap and wall rim uses have consistent opposite boundary orientations.
3. **Annular face.** Connect outer and inner boundaries by an artificial bridge
   used twice. Verify extraction yields two cycles and never emits the bridge.
   Include two holes to detect an implementation that hardcodes one bridge.
4. **Sphere and torus.** Use finite combinatorial subdivisions, not zero-dart
   sentinels. A cube boundary classified entirely in one spherical face is a
   suitable abstract sphere scaffold. Use a periodic surface subdivision for
   the torus. All artificial boundary cells belong to their face; a torus
   volume scaffold must represent the solid handle, not a filled ball.
5. **Concrete volume proof.** Construct a 3×3×3 arrangement of block cells with
   the center omitted. Sew coincident material faces by alpha3. Classify every
   internal face and its interior subdivisions in one logical solid. Leave
   material/void boundary faces on the logical boundary. This gives 26 material
   cells, one connected solid and two boundary shell components.
6. **Handle proof.** Use a ring of conforming block cells around an open shaft.
   Confirm one material region and a connected genus-one boundary, distinct
   from the two-shell cavity fixture.

For the block fixtures use exact grid coordinates to establish the expected
embedding independently of ownership traversal. Check a point in the omitted
center is outside material, a point in a retained block is inside, and an
external point is outside. Check raw boundary Euler characteristics where the
fixture is a genuine cell decomposition: sphere shells 2, torus shell 0.
Never calculate Euler characteristic from seamless logical entity counts.

Test incorrect labels as well: internal-face ownership assigned to an exterior
face, a label crossing an intended public boundary, and disconnected patches
given one logical key must be rejected or detected by the appropriate validator.

**Evidence required:** actual fixture construction, alpha validity, occurrence
walks, expected logical counts and boundary components. Drawings or a mocked
`faces()` result are not a proof. M1 establishes feasibility of the mechanism;
general shell ingestion is a separate M4 requirement.

#### M1 result — the gate is met

The prototype lives in `src/topology/subdivision/`, beside the existing GMap
and built entirely on its `add_dart` / `link` / `sew` primitives. The fixtures
are in `tests/support/scaffold.rs` and the proofs in
`tests/topology/subdivision.rs`; every fixture registers no domain attribute at
all, so nothing in these walks can be reading a stored loop seed or shell root.

What the layer is:

- `ownership.rs` — `EntityOwner` (vertex, edge, face, solid), `OrbitOwnership`
  records anchored at a representative dart, the `Subdivision` that holds them,
  and the `OwnershipIndex` derived from a subdivision plus the map it labels.
- `walk.rs` — `turn`, the one traversal primitive. It steps to the next raw
  cell around a shared boundary using the sewing involutions, passing over
  cells a higher-dimensional entity owns. Every path it takes is an odd number
  of alpha steps, which is what lets a caller track which way round it is
  reading the map while it turns.
- `region.rs` — `recover_region`, which walks an entity's whole extent out of
  the labels, and records a sense per dart as it goes.
- `boundary.rs` — `boundary_cycles` for a face's oriented loops,
  `boundary_shells` for a solid's boundary components with their raw Euler
  characteristic, `boundary_vertices` for an edge's ends, and
  `BoundaryCycle::logical_uses` grouping consecutive raw pieces into logical
  edge uses.

What the fixtures prove:

| Fixture | Result |
|---|---|
| Segment | one logical edge, two logical vertices, two boundary ends |
| Circle | one logical edge, zero logical vertices, no boundary; reversing the anchor reverses every dart's sense |
| Capped cylinder | three logical faces, two logical edges, zero logical vertices; the wall's two loops never emit the seam; each cap's rim use is the `alpha0`-`alpha2` partner of the wall's |
| Face with one bridged hole | two cycles, the bridge never emitted |
| Face with two bridged holes | three cycles, still one raw face |
| Sphere (cube surface) | one logical face over six raw quads, no loop |
| Torus (periodic square) | one logical face, no edge, no vertex, one raw vertex and two raw edges |
| 3×3×3 minus centre | 26 material cells reached from one anchor through `alpha3`; two shells, of 54 and 6 raw faces, each with Euler characteristic 2 |
| 3×3×1 ring | 8 material cells, one shell of 32 raw faces with Euler characteristic 0 |

The volume fixtures are genuine combinatorial subdivisions: unit cubes sewn to
their neighbours by `alpha3` at coincident faces, with every buried face, edge
and corner classified in the solid and every free face left on its boundary.
Nothing is coned to a point and no unrelated boundary faces are identified. The
embedding is checked against the grid the cells were built from, independently
of the traversal: the omitted centre is outside the material, a retained cell is
inside, and a point beyond the block is outside.

The wrong-label cases are rejected as the plan requires:

- an interior label on an exterior face fails with `InteriorBoundaryNotShared`,
  because nothing lies across it;
- a label crossing a public boundary fails with `ForeignCell`, naming the
  entity found on the far side;
- two disconnected patches under one key fail with `Disconnected`;
- two entities claiming one raw cell fail at index build with
  `ConflictingOwnership`, an owner of lower dimension than its cell with
  `OwnerBelowCell`, and a record anchored off the map with `DanglingRecord`.

One correction the prototype forced, which the later milestones inherit: a
boundary walk cannot seed itself from whichever frontier dart the region walk
happened to reach first. Seeding a cylinder wall's two rims independently wound
one of them backwards. The region walk therefore carries a sense — flipped by
every involution step and by every crossing of an interior cut, which is always
an odd number of steps — and only darts sharing the anchor's sense seed a loop.
That is what `every_loop_of_a_face_is_wound_the_same_way` pins down.

Evidence: `cargo test --all-targets --all-features` exits 0 with 705 passing,
none failing and none ignored — the M0 baseline of 682 plus 23 new subdivision
tests, with no existing test changed. `cargo clippy --all-targets --all-features`
exits 0 with the same 26 warnings M0 recorded. `cargo fmt` and
`git diff --check` are clean.

### M2 implementation — move ownership without losing transactions

**Start with:** `gmap.rs` stores and `AttributeStore`/`CellDim`, `edit.rs`
transaction commit and lineage, `shape.rs`, and `model.rs`.

Perform the extraction in this order:

1. Move public entity slotmaps and their typed lookup into Model. Preserve the
   existing key types and slotmap generation semantics. Remove the association
   `Cell1::Key = EdgeKey` from the raw core; raw cells no longer name logical
   slotmap entries directly.
2. Move geometry/payload-dependent validation and cell identity reconciliation
   to Model. Keep alpha involution/commutation checks callable on a bare GMap.
3. Move `transaction` and `transaction_with_policy` to Model and rename the
   public edit capability to `ModelEdit`. Reuse the existing snapshot strategy;
   do not add a new incremental undo engine during extraction. Preserve the
   existing behavior that panics are not caught.
4. Give `ModelEdit` internal access to core mutation and ownership updates as
   one operation. No public `&mut GMap` escape hatch on Model or Shape.
5. Migrate Shape ownership and foundational view constructors. Update dependent
   signatures mechanically in the same checkpoint; temporarily excluded tests
   cannot be used to conceal a noncompiling production module.

Commit ordering is: apply staged deltas/re-anchor, check raw topology, reconcile
logical identities, validate ownership/uses/references, apply net logical
payload policy, advance revision and invalidate caches. A failure in any step
restores authoritative state. Validate geometric constraints needed by the
operation without demanding an embedding of all abstract internal cells.

A reference dart is not sufficient evidence that its logical region survived.
Check that it still has the right owner and orientation. Before destructive
edits, save ordered survivor candidates or the explicit replacement mapping;
never recover a deleted source by searching for an arbitrary unrelated dart.

Tests: merge two source identities with and without explicit survivor lineage;
split/refine with payload counters; delete the anchor while preserving its
entity; fail validation and payload policy after allocation; rollback then
serialize; edit a model with prewarmed caches. Assert no stale key becomes a
valid reference to a different entity after normal deletion/reallocation.

**Checkpoint:** the core imports no logical/geometry modules, foundational
targets compile and pass, and no Model mutation bypasses the atomic boundary.

#### M2 result — the gate is met

`GMap` is now the pure core. Its whole import list is `std::collections`, `serde`
and `super::dart`: no geometry, no `Payload`, no key type, no `Profile`, no
`Sheet`, no `Model`. It is no longer generic — there is nothing left in it for a
payload parameter to describe. What it holds is `alphas` and `free_slots`, and
what it answers is orbits: `alpha`, `is_free`, `orbit`, `orbit_indices`,
`cell_representative`, `incident_cells`, `cells`, `adjacent_cells`, plus the
crate-internal `add_dart`, `remove_dart`, `compact`, `link_raw`, `unlink_raw`,
`point_alpha` and `is_sewable`.

`Model<P>` in `src/model.rs` owns everything else: the `GMap`, the six entity
slotmaps, the `Subdivision`, a `revision` counter, and two lazy caches — the
dart-to-key indexes and the dart-to-owner lookup. `Cell0`–`Cell3`, `CellDim`,
`AttributeStore`, `CellKeyLookup`, `MergeHandle`, `MergeTopology` and
`TopologyMerge` moved with it, which is what removes `Cell1::Key = EdgeKey` from
the raw core: a raw cell no longer names a slotmap entry, the model does.

`TopologyEdit` is `ModelEdit`, over `&mut Model<P>`, and `TopologyEditError` is
`ModelEditError`. The snapshot strategy is the existing one, unchanged; panics
are still not caught. `ModelEdit` gains one operation, `own_cell`, so a
subdivision label is staged and rolled back exactly like everything else.

There is no public `&mut GMap` anywhere. `Model::topology()` returns `&GMap`;
the only public methods on `Model` that take `&mut self` are `transaction` and
`transaction_with_policy`. `Shape<K, P>` owns a `Model<P>` and reads it through
`model()`, `model_mut()` and `into_model()`.

Commit order is now explicit, and each step restores the transaction-start
snapshot whole on failure:

1. the raw gmap axioms, checked on `Model::topology()` alone;
2. the subdivision labels, which must describe that map;
3. shell re-rooting and required profile/sheet registration;
4. edit-event lineage, then identity reconciliation;
5. payload policy on net externally-visible changes;
6. `revision += 1` and cache invalidation.

Validation is split to match. `validate_gmap(&GMap) -> Result<(), GMapValidationError>`
reads no entity and no geometry, so it answers for a bare map;
`validate_solid_manifold`, `validate_solid_orientation` and their `all_` forms
take `&Model<P>` and return `ModelValidationError`, which carries the raw error
as one of its variants rather than pretending to be it.

Eleven tests in `tests/topology/model_state.rs` cover the gate:

- a new model is at revision 0 and every commit advances it; a failed one does not;
- a commit carries the subdivision with the map, and a rollback restores both;
- a cell two entities disagree about is rejected at commit with
  `InvalidSubdivision`, and so is a record anchored off the map;
- a label survives the renumbering that removing darts causes — the record
  follows the cell, not the number it had;
- a warm index and a cold rebuild answer identically after an edit;
- a rolled-back model serializes to exactly the state it kept;
- a deserialized model rebuilds both derived lookups from its stores;
- a key from a removed entity never resolves to the one that replaced it.

The seven merge and isolate tests that lived inside `gmap.rs` moved to
`tests/topology/model.rs`, where the behaviour now lives, rather than being
deleted with the file.

Evidence: `cargo test --all-targets --all-features` exits 0 with 716 passing,
none failing and none ignored — the 705 after M1 plus these 11, with no existing
test removed or weakened. `cargo clippy --all-targets --all-features` exits 0
with 25 warnings against the baseline's 26: no new lint, and the unread `map`
field on the old embryonic `Model<P>` is gone. `cargo fmt` and `git diff --check`
are clean. `npm run typecheck` passes. The Python extension rebuilds and its
suite is back to the M0 baseline exactly — 23 passing, the one recorded stale
STEP assertion failing, `test_block.py` still uncollectable — after renaming the
binding class from `GMap` to `Model` and the entity accessor from `.gmap` to
`.model`, which the tests were updated for in the same change.

`AGENTS.md` and `src/topology/edit.md` were rewritten where they described the
old layout. `docs/model_api.md` is left alone and is now marked in `AGENTS.md`
as a design note predating this work, not a description of the tree.

### M3 implementation — public traversal and lazy geometry

**Start with:** current edge/face/profile/sheet/solid views, pcurve lookup and
`UnwrappedFaceDomain`. Keep the existing geometry utilities and tolerances.

Implement the public view foundation as `(&Model<P>, key, sense)` plus parent
context only where an occurrence needs it. Preserve narrowed bounded/closed
edge behavior; a closed edge with a deliberate vertex is still closed. Public
`vertices()` enumerates logical vertices, never computational closure points.

Introduce internal use keys before migrating pcurve readers. Transfer every
existing directed-dart pcurve to its distinct logical occurrence; keep the
dart only as a relocatable private anchor. A raw split produces several raw
pieces for one use; it must not allocate several logical use records unless
the logical edge itself splits. Reconstruct loop order from the frontier walk.

Implement profile and sheet lookup without a second membership graph. A profile
anchors an oriented logical chain; a sheet anchors an oriented connected face
component. Preserve their independent payloads and apply aggregate lineage
when edits actually split/merge those components. Parent solid boundary walks
must not cross internal volume faces as if they were exterior sheet faces.

Define a lazy realization request around purpose and revision, such as
`realize_face(face, purpose)` and `realize_edge(edge, purpose)`. Results are owned
or immutably shared values, never references invalidated by later cache writes.
Default artificial cuts use the support's native domain origin; alternative
valid cuts remain internal choices and are exercised by invariance tests.

Cache keys include model revision, logical entity, purpose and any tolerance
or cut choice that affects the result. Invalidate globally initially. On load,
start empty. Same-revision reads must not race a shared mutable traversal mark;
use local visited sets and the existing safe cache synchronization pattern.

Retain persistent pcurves/domain constraints where multiple geometric sections
share endpoints. A disposable realization must never guess the major/minor
arc or change a model's selected region. Continue deriving ordinary bounded
edge intervals through existing geometry rules; if a new case is ambiguous,
represent the required section choice explicitly rather than silently choosing
another arc. Cache reconstruction must reproduce that choice.

Tests: annular and two-hole loops, reversed face and shell views, repeated
edge occurrences with different pcurves, marked and vertex-free circles,
anchor relocation, cold/warm serialization equivalence, cut changes preserving
3D points and normals. Check cyclic order modulo starting point when a scaffold
change relocates an anchor; do not require arbitrary list index stability.

### M4 implementation — construction and arbitrary boundary ingestion

Migrate bottom-up: edge/profile, face, sheet, extrusion/revolution/sweep, solid
primitives, then shell-based import and result construction. Existing analytic
supports and NURBS fallback remain unchanged. Geometry-specific scaffold
construction belongs in builders, not in raw edit primitives.

For each builder, separate creation of logical identities/supports from creation
and labeling of its scaffold, within one transaction. Register intended public
boundaries first; label closure points, bridges and periodic seams as internal.
Reuse known sweep/revolution correspondences to avoid rediscovering geometry.

Face scaffolds use oriented boundary cycles and deterministic cut systems.
For planar holes use noncrossing bridges in the known domain; for periodic
faces reuse unwrapping to choose cuts. If the raw scaffold is abstract, record
that fact and do not pass its paths to geometric code without realization.
Poles are not automatically public vertices; preserve intentional singular
features according to the logical constructor contract.

Add a shared internal boundary-to-solid scaffold builder, used by STEP import
and Boolean assembly as well as primitives. Its input is oriented logical
boundary uses with their known grouping into input solids; its output is
connected classified computational material cells. Input shell lists are
construction data, not persistent alternate connectivity.

This builder is an explicit research deliverable: extend the M1 templates to
supported boundary topologies and preserve their boundary subdivision when
adding interior adjacencies. General cavities must not be implemented by
coning all boundary shells to a singular point or sewing cavity walls to outer
walls. Refine boundary scaffolds consistently when necessary, preserving
logical keys. Validate boundary components, genus, material/void orientation
and all internal face pairings after construction. Before proceeding to M5,
document and test the actual construction algorithm and its supported topology
class here. If it cannot cover the baseline inputs, report the gate as failed;
do not leave an unspecified fallback in the implementation.

Acceptance includes the curved hollow-sphere STEP fixture, holed slab, multiple
cavities, a handle, and the existing primitive/frame matrix. The scaffold need
not be a spatial volume mesh, but boundary geometry and material interpretation
must remain available to existing containment algorithms. Raw internal volumes
cannot be used as spatial classification regions without their own embedding.

### M5 implementation — operation families and promotion rules

**Start with:** `builders/edges.rs`, `faces.rs`, `removal.rs`, `chamfer.rs`,
the healing passes, and existing topology copy/merge helpers.

Migrate in three passes:

1. Raw refinement and re-anchoring, including alpha sewing/unsewing and
   subdivision-only cleanup, with no logical key or payload changes.
2. Logical edge/face split and merge, profile/sheet consequences, pcurve
   propagation, and cross-model copy.
3. Composite imprint, chamfer and healing operations using those primitives.

Core split functions return deltas. Geometry-aware builders supply any real
intersection locations and interval correspondence. `ModelEdit` applies
ownership and use-anchor changes before returning a staged typed view.
No caller should separately mutate raw topology and later remember to repair
the logical store.

A logical split has an explicit source key and ordered resulting sections.
Capture the source-anchor side before refinement. If the split point is the
anchor, preserve the key on the outgoing section in the source's default
orientation. Closed-edge first marking retains the same edge key and creates
one vertex; a second distinct marking creates two arcs. Splitting again at an
existing logical vertex must follow the existing intended no-op/error contract,
not create coincident duplicate entities.

Promotion is operation intent, never `raw degree != 2` alone. A seam crossing
introduced only to trace a geometric intersection remains internal; a user
split or a retained result junction becomes logical. Preserve explicitly
retained vertices and payloads during healing. Shape-changing logical healing
continues to require the existing policy; deleting an artificial scaffold cell
is only allowed when raw validity and region connectivity remain intact.

Copy/import allocates destination logical keys and remaps all references,
including use keys, ownership, profile/sheet anchors and correspondence. Copy
isolated logical entities with the complete scaffold required for their own
dimension; do not copy a partial solid scaffold and leave outside-owned cells.
Keep links outside an extracted object's scope free where appropriate.

Acceptance matrix: split in either orientation, split at a periodic cut,
first/second circle marking, face imprint across a cut, real junction at a
closure point, copy of a boundaryless logical face, cavity solid copy, custom
payload split/merge counts, and failure halfway through each composite edit.

### M6 implementation — preserve the network, replace realization boundaries

**Start with:** Boolean `operand`, `broad_phase`, `contacts`, `graph`, `imprint`,
`trim`, `classify`, `select` and `assemble` modules. Keep their current numerical
contracts unless a logical-identity change requires a deliberate migration.

1. Change operand import to Model remapping. `OperandCells` contains logical
   keys; artificial cells never become candidate public contacts.
2. Keep prepared surfaces, support intersections, tolerances and analytic/NURBS
   dispatch on logical supports. Reuse prepared geometry across raw patches.
3. Keep event/span identities in the intersection network independent of raw
   darts. Add private mappings from network events to refined occurrences.
4. Realize imprints through M5 computational edits. One network span may cross
   several raw pieces without becoming several logical edges.
5. Preserve existing geometric region classification and selection. Avoid a
   new volume meshing/classification solver. Tangencies are not automatically
   transverse selection boundaries.
6. Assemble retained logical regions, promote their actual boundaries, and
   construct the result scaffold through M4. Assign lineage once per final
   logical change, then apply requested healing.

Keep preparation results useful independently of final Boolean assembly. Their
diagnostics report logical contacts and may additionally expose raw refinement
counts under explicit debug fields. Add timing for subdivision realization and
logical assembly so a slowdown is attributable.

Run the same supported operand pairs for union/intersection/difference and
both operand orders where semantics permit. Include disjoint, contained,
coplanar, curved, cut-crossing and existing tangent cases. Expected unsupported
cases retain explicit errors; newly failing baseline cases block completion.

Metamorphic checks: refine or relocate artificial cuts on either input before
the Boolean, then compare geometry, logical boundary structure and validity.
Do not demand identical newly allocated slotmap numbers across independent
runs. Assert source lineage is preserved within each run. Verify no extra
surface/surface solves arise merely from adding raw patches to one face.

### M7 implementation — consumers, serialized state and exchange

Migrate consumers in dependency order: logical exploration bindings, tessellation,
STEP bridge, Python/WASM wrappers, then examples and viewers. Reuse existing
geometry conversion and meshing implementations, replacing their topology
inputs rather than rewriting their numerical routines.

Tessellation must request whole logical edge curves and ordered logical face
domains. Share boundary sampling across incident face uses. Artificial UV cuts
may duplicate working samples but must not create cracks, incorrect normals,
or public pickable edges. Rendering diagnostics may show raw cells only when
requested; ordinary picking returns logical keys.

For STEP, preserve the parser, semantic IR and geometry mapping. Export creates
a suitable temporary seamed boundary from logical geometry and available
realizations. Do not assume any abstract raw seam is exportable. Share synthetic
closure vertices and exported edges across the corresponding oriented uses;
the exporter must not change Model keys or force persistent new public entities.

Import first respects the file's oriented incidences, then classifies verified
periodic artifacts. Imported intentional splits remain logical unless the
requested healing policy merges them. Replace old physical seam-removal calls
with correct ownership changes/scaffold maintenance where appropriate. Use the
M4 solid scaffold builder for shell groups, including BREP_WITH_VOIDS.

Serialize the authoritative model and slotmap generations, skip caches, and
rebuild/validate derived indexes after deserialization. Compare logical queries
and identity references, not a cache's JSON representation. Add a format version
for the new layout and reject unsupported versions explicitly; no old-layout
migration is required.

Rebuild Python using the existing maturin workflow before executing binding
tests. Update WASM shared exploration data and frontend types together. Exercise
the normal viewer and explicit computational diagnostics with cylinder, sphere,
holed face and cavity examples. Record build/permission failures separately;
they are not evidence that a stale extension passed.

### M8 implementation — remove scaffolding and write the verdict

Search the entire repository for old contracts, not just compiler errors:
`GMap<P>`, `TopologyEdit`, `ShellRoot`, face-root merge handles, raw-dart pcurve
keys, old Shape accessors, and comments asserting public entities equal orbits.
Retain a term only where it correctly describes low-level diagnostics or
historical plan context. Update exports, bindings, examples, tests and docs in
the same final migration.

Audit every ledger row and every experiment-created ignore/module exclusion.
For obsolete tests, either retarget the surviving property or remove the test
with the removed behavior identified in the ledger. No broad test-directory
exclusion may remain. Pre-existing unsupported-case ignores remain visible and
must not be misreported as experiment coverage.

Run the final checks:

```powershell
cargo fmt
git diff --check
cargo clippy --all-targets --all-features
cargo test --all-targets --all-features
```

Also rerun M0's benchmark scenes, rebuilt binding checks, STEP round trips and
frontend verification under the documented environment constraints. Keep
geometry tolerances and watchdog ceilings unchanged unless an independently
justified decision is documented; do not widen them to conceal a regression.

Write a final assessment in this plan with: capability parity table, residual
baseline failures, actual performance measurements, examples of simplified
algorithms, new interpretation-layer complexity, and any unsupported topology
discovered. If an architectural gate failed, identify the smallest reproducer
and the violated contract. Do not automatically restore an older working tree
or relax the pure-core requirement. The experiment's outcome may be negative,
but its status and evidence must be unambiguous.
