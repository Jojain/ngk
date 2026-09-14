# Experiment: logical topology over a pure GMap subdivision

Status: **In progress** — M0, M1 and M2 complete. M3 holds the realization
cache; its label-derived views wait on construction, for the reason in
[Ordering correction](#ordering-correction-classification-precedes-public-views).
M4's two classification slices are complete: a built rectangle and a built
cylinder are classified end to end, a circle is unmarked through construction,
cutting, healing, Booleans and STEP in both directions, and the suite is green.
M4 slice 3 is **in progress and the tree is not green**: the annulus is bridged,
the cylinder wall is seamed, `Face::loops()` is derived from the map, and every
single-cycle face passes. 33 tests remain, in the clusters listed below — 708 passing against a
736-passing baseline.
See [M4 slice 3](#m4-slice-3--the-bridge-proven-on-an-annulus).

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

struct Subdivision {
    // One shelf per cell dimension, each keyed by a representative dart of the
    // labelled orbit. The dimension is where an entry sits, not a field beside
    // it that could disagree; keying by the anchor makes labelling it twice a
    // correction rather than a second opinion, which is what promoting a
    // scaffold cell to a logical one needs. `BTreeMap` because enumeration and
    // serialization order are properties callers are entitled to.
    cells: [BTreeMap<Dart, EntityOwner>; 4],
}
```

An owner's dimension is greater than **or equal to** the cell's, not exactly one
above it. Ownership skips scaffold: only logical entities own, so a buried
corner inside a solid has no intermediate raw edge or face to belong to and is
labelled with the solid directly. A whole sphere over a cube scaffold owns its
six quads, twelve edges and eight corners alike. The equal case is the majority
of every labelling — a logical face owns its own raw quads.

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

#### Stored state this experiment removes

Four fields in the current tree contradict "a logical entity does not know its
neighbours". The experiment is not finished while any of them stands, and the
list is short enough to audit by reading six struct definitions:

| Field | Why it goes | Replaced by |
|---|---|---|
| `FaceAttr.loops` seed darts | an authoritative boundary list | `boundary_cycles`; `LoopKind` domain metadata stays |
| `SolidAttr.inner_shells` | an authoritative cavity list | `boundary_shells` |
| `ShellRoot::Face { face, sense }` | a stored logical→logical reference | nothing — see below |
| `FaceAttr.pcurves` keyed by `Dart` | a key that dies on refinement | keyed by `EdgeUse` |

`ShellRoot::Face` should disappear rather than move. It exists only because a
boundaryless face currently has *no darts at all*, leaving a sheet nothing to
point at. Under a real scaffold a whole sphere is six raw quads with darts like
any other face, so the anchor becomes an ordinary dart. That is a falsifiable
prediction: if the variant survives M4, the scaffold is wrong, not the
prediction.

When those four are gone, no `*Attr` holds another entity's key or any list of
raw cells, and the only dart in any attribute is one anchor meaning "this way
round".

There is also a duplicate: `DerivedCellIndexes`/`CellKeyLookup` and
`OwnershipIndex` are two dart→logical-key mechanisms. The first works only
where a raw cell equals a logical cell, which makes it the degenerate case of
the second. Delete it and reimplement `CellKeyLookup` on ownership — but only
once builders label, since before that the ownership index is empty.

#### Ordering correction: classification precedes public views

This plan ordered M3 (public views) before M4 (construction). For everything
that reads ownership, that is backwards, and the cost has already been paid
once.

`EntityOwner` appears in six files, all of them the subdivision layer and its
`model.rs`/`edit.rs` plumbing. **No builder, no importer and no modeling
function writes a single ownership label.** Every `Model` produced by real code
carries an empty `Subdivision`. M1 proved the mechanism on hand-built test
scaffolds; the production kernel is unclassified.

So a public view derived from labels has nothing to derive from, and any
attempt to write one must fall back to the old dart-keyed storage to return
anything at all. That is exactly what happened: see the M3 record below.

The correction is to stop sweeping M3 horizontally and drive one shape all the
way through instead:

1. **Plumbing slice — planar rectangle.** No scaffold at all; every raw cell is
   logical. Proves builder → `ModelEdit::own_cell` → commit validation →
   `OwnershipIndex` → walk, with no seam to confuse a failure.
2. **First real slice — cylinder.** Seam edge owned by the wall, closure
   vertices owned by their rim edges. The first case where logical differs from
   raw in production, built by the real builder rather than by hand.
3. Derive `Face::loops()` from `boundary_cycles` for those shapes and delete the
   stored seeds. Migrate directly; do not build machinery to run the stored and
   derived representations side by side. This is an experiment, and maintaining
   two answers costs more than re-deriving one.
4. Then the pcurve re-key, which by then has real labels underneath it.
5. Then the refinement-invariance test below, as a standing gate.

The remaining M3 items keep their contracts; only their position moves. M4's
builder migration is no longer a later milestone that consumes M3's views — it
is the thing M3's views wait on.

#### The invariance test that makes a shadow graph impossible

Promote this from a clause in the M3 gate to the guard rail the rest of the
migration is written against, and add it before M4 rather than after:

> Refine the scaffold everywhere — add computational vertices and cuts
> throughout a model — and assert every logical query returns an identical
> answer: same edges per face, same loop order modulo starting point, same
> pcurves, same shells, same geometry.

Any stored adjacency fails this the moment it is added, because refinement
invalidates raw cells and a shadow graph cannot follow. A rule that is enforced
only by remembering it is one that eventually stops being enforced.

#### Oriented occurrences and boundary metadata

An orbit identifies a cell, not every way a parent uses it. A dart *is* an
oriented use of an edge by a face — that is what a dart in a 2-gmap means — and
for as long as one logical edge is one raw edge, a dart is a faithful name for
one. What breaks that is internal refinement: once a logical edge may contain
vertices interior to it, one logical use spans a **run** of darts, and no single
dart in the run names the whole thing. That, and only that, is why a use needs a
name of its own.

The name is a derived value, not an allocated identity:

```rust
/// One oriented occurrence of a logical edge on a face.
/// Constructed from the walk; never allocated, never persisted as an identity.
struct EdgeUse { edge: EdgeKey, sense: Orientation }
```

Do **not** introduce an `EdgeUseKey` slotmap. A generated key would need an
anchor dart per record, re-anchoring on every refinement, and a reverse index to
keep in sync — the exact cost this key has none of, since it mentions no raw
cell at all. The earlier draft of this section prescribed that slotmap; it was
tried, and the attempt is recorded under M3 below.

Pcurves stay on `FaceAttr`, keyed by `EdgeUse`. The location was always right —
a pcurve is expressed in that face's parameter space and is meaningless without
`FaceAttr.surface`, face reversal touches surface and curves atomically, and
deleting a face drops its pcurves with no cross-key sweep. Only the key changes:

```rust
pcurves: HashMap<Dart, TrimmedCurve2>       // was: dies on refinement
pcurves: HashMap<EdgeUse, TrimmedCurve2>    // is: refinement cannot touch it
```

**`sense` is relative to the face's default boundary walk, not to the traversal
in hand.** Getting this wrong gives a double-reversal bug that only appears on
seams. An edge used once by a face has one entry; a reversed `Face` view looks
up *the same key* and reverses the returned curve. An edge used twice by one
face — a real seam edge, with two distinct UV images — has two entries,
`(e, Same)` and `(e, Reversed)`, because a coherent boundary cycle necessarily
runs the two sides opposite ways. Reversing a view never flips keys; it reverses
curves and loop order. Storage and reading stay separate, which is what the dart
key failed at.

An occurrence index in walk order was considered instead of `sense` and
rejected: it depends on where the walk starts, so re-anchoring a loop seed could
silently renumber it. Sense depends only on two stored anchors — the edge's
default dart and the face's default orientation — and is invariant to walk
start.

Two commit-time invariants keep this table an attribute rather than a second
connectivity graph:

1. every key corresponds to a use the default boundary walk produces — no orphans;
2. every logical use the walk produces on a face whose support does not make the
   pcurve exactly derivable has one — no gaps.

Never expose an iterator over `pcurves` publicly; `pcurve_of(use)` only. What
cannot be enumerated cannot be mistaken for adjacency. If a second insert under
one `(edge, sense)` is ever attempted, reject it rather than choosing — that is
the case which would prove an occurrence discriminator is genuinely needed.

Store no authoritative ordered vector of uses on a face. Derive ordering and
loop membership from the subdivision. Keep loop domain metadata (outer, inner,
wrapping, capping): no walk can recover which axis a periodic loop spans or
which degeneracy caps it, so that part of `LoopKind` is real persistent data.
The loop **seed darts** beside it are not. Closed boundaryless faces have no
such records. A face reversal returns a view with flipped sense; it does not
rewrite persistent pcurves as a side effect of reading.

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
| Orbit ownership and region recovery | none — new capability | n/a | M1 | n/a | `tests/topology/subdivision.rs` | 24 tests pass; see the M1 result and the M3 storage rework |
| Transactions, lineage, rollback | `tests/topology.rs` `edit`, `transaction` | 24 pass | M2 | none | same tests retargeted at `ModelEdit` | 24 pass, plus 11 in `model_state.rs` |
| Derived cell indexes | `tests/topology/indexes.rs` | 2 pass | M2 | none | ownership index rebuild tests | 2 pass; warm/cold and deserialize-rebuild cases added |
| Typed views | `tests/topology.rs` `edge`, `face`, `profile`, `sheet` | 23 pass | M3, after the M4 slices | none | same tests against `(&Model, key, sense)` views | |
| Face loops, holes, periodic domains | `tests/topology/unwrapped_face_domain.rs`, `tests/topology/planar.rs` | 12 pass | M3, after the M4 slices | none | frontier-walk loop extraction | |
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

**Read [Ordering correction](#ordering-correction-classification-precedes-public-views)
first.** Every item here that reads ownership waits on the two construction
slices described there. The contracts below are unchanged; their position is.

**Start with:** current edge/face/profile/sheet/solid views, pcurve lookup and
`UnwrappedFaceDomain`. Keep the existing geometry utilities and tolerances.

Implement the public view foundation as `(&Model<P>, key, sense)` plus parent
context only where an occurrence needs it. Preserve narrowed bounded/closed
edge behavior; a closed edge with a deliberate vertex is still closed. Public
`vertices()` enumerates logical vertices, never computational closure points.

Re-key pcurves to `EdgeUse { edge, sense }` as described above, keeping the
table on `FaceAttr`. Transfer every existing directed-dart pcurve to its logical
occurrence; no dart survives in the key, so there is no anchor to relocate. A
raw split produces several raw pieces for one use and must leave the stored
pcurve untouched — `split_face_pcurves` survives, but stops firing for
computational refinement and runs only on a real logical split, where a
geometry-aware builder knows the split parameter. Reconstruct loop order from
the frontier walk.

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

#### M3 progress — revisioned geometry realization

Work resumed from revision `af79a86` (`milestone 0 1 and 2`), where the prior
milestones are already committed. The only pre-existing untracked path was
`.claude/`; it was left untouched. This continuation makes no commit.

`src/model/realization.rs` adds `Model::realize_edge` and `realize_face`, keyed
by model revision, logical key, orientation and consumer purpose. Edge results
are shared immutable `TrimmedCurve` values; face results own the support and
the oriented `UnwrappedFaceDomain`. They reuse the existing span and domain
algorithms, retaining chosen native edge intervals and persistent pcurves.
Face-normal winding queries now use the cached face domain.

The caches are outside GMap and omitted from serialization. Clone and load
start cold. Every mutation discards cached geometry, including edits inside a
transaction before revision advances. Commit clears staged results; rollback
restores a cold snapshot. Callers may retain an `Arc` through later edits
without its geometry changing. Computation runs outside the publication lock,
with local traversal state; concurrent readers share the published result.
The existing dart-to-key index initialization now uses `OnceLock::get_or_init`
to remove its check-then-set race under concurrent cold reads.

Nine tests in `tests/topology/realization.rs` cover cold/warm identity, reversed
face domains and native arc sections, purpose isolation, repeated staged
geometry edits, operation-error and commit-validation rollback, immutable
support snapshots, missing-key errors, serialization/clone equivalence and
concurrent cold reads. The first six were run before implementation and failed
to compile because the realization API did not exist; they pass after it.

This is **not the M3 gate**. Pcurve occurrence migration, frontier-based public
traversal, aggregate views, and cut/refinement invariance remain outstanding.
Realizations currently use the existing domain placement rules, with no
configurable tolerance or cut; support-native default cuts and alternative-cut
checks remain part of that work. No M4 migration or test exclusion was
introduced.

Validation: `cargo test --all-targets --all-features` exits 0 with 725 passing,
zero failing and zero ignored tests (716 inherited plus nine new tests).
`cargo clippy --all-targets --all-features` exits 0 with the same 25 warnings;
formatting and diff checks pass. The benchmark harness still reports the two
recorded baseline failures: `sphere_union_sphere` rejects its budget and
`orthogonal_cylinders` rejects span-0 sewing endpoints (9.86 s in this run).
Python was not rebuilt or rerun for this core checkpoint, and no frontend
files changed. The Windows-blocked wasm/frontend builds remain skipped.

#### M3 record — an abandoned `EdgeUseKey` migration, and the storage rework

An attempt was made to implement the `EdgeUseKey` slotmap this plan used to
prescribe. It is recorded here because the way it failed is the evidence behind
the [ordering correction](#ordering-correction-classification-precedes-public-views),
not merely an abandoned branch.

The attempt wrote the consumer half — `Loop::edge_uses`, `Face::pcurve` rebuilt
around occurrences, `pcurve_for_edge`, and the call sites in `builders/faces.rs`,
`builders/sheets.rs` and three test files — against a `Model::edge_use_at` /
`unique_edge_use` API and a `topology::edge_use` module that were never written.
The tree did not compile.

What is diagnostic is the shape of `Face::pcurve` as it was left: a four-tier
fallback that tried the raw dart map across `alpha0`/`alpha2`, then an
occurrence record, then a scan of the pcurve map filtered by logical edge, then
a uniqueness check — with a debug `eprintln!` on the final miss. That is not
carelessness. With no builder writing ownership labels, an occurrence lookup can
never succeed on a real model, so every path had to end in the old dart-keyed
map. The fallbacks were load-bearing.

Two conclusions were drawn and are now in the contracts above: a use needs a
name only because one logical use spans a run of darts, and that name should be
a derived `EdgeUse { edge, sense }` rather than an allocated key; and no view
derived from labels can be written before something writes labels.

The attempt was reverted by inverse edit, leaving the six files identical to
`af79a86`. `Subdivision` was then reworked: `Vec<OrbitOwnership>` became one
`BTreeMap<Dart, EntityOwner>` per cell dimension, so the dimension is where an
entry sits rather than a field beside it, enumeration and serialization are
deterministic, and labelling one anchor twice is a replacement rather than a
second entry — which is what scaffold-to-logical promotion needs and the old
storage made impossible without a commit error. `ConflictingOwnership` now means
only what it can still mean: two *different* anchors meeting on one orbit. Two
tests that had asserted the conflict by labelling one anchor twice were
rewritten to use two anchors, and `relabelling_an_anchor_replaces_what_it_said`
was added for the promotion case.

Per-dimension owner types — `EdgeOwner`, `FaceOwner`, `SolidKey` — were designed
and deferred, not rejected. They would delete `SubdivisionError::OwnerBelowCell`
outright by making the label unrepresentable. Revisit when M4 builders provide
statically-known call sites; the ladder is `owner.dimension() >= cell.dimension()`,
which is a `>=` and not a `+1`.

Validation: `cargo test --all-targets --all-features` exits 0 with 726 passing,
zero failing and zero ignored. `cargo clippy --all-targets --all-features` exits
0 with the same 25 distinct warnings, none in changed files; `cargo fmt` and
`git diff --check` pass. Python, benchmarks and the frontend were not rerun for
this core-only checkpoint.

### M4 implementation — construction and arbitrary boundary ingestion

**This milestone now starts before M3 finishes.** Its first two steps are the
planar-rectangle and cylinder slices in the
[ordering correction](#ordering-correction-classification-precedes-public-views);
M3's derived views wait on them, not the other way round. Take the rest of this
section in its existing order once those two shapes are classified end to end.

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

#### M4 slice 1 result — the classification is two halves, and one is derived

The planar-rectangle slice is done, and it changed the design rather than just
exercising it.

The first attempt labelled each entity's own cell at `ModelEdit::add_vertex` /
`add_edge` / `add_face`. That is wrong, and the tree said so in three distinct
ways before the reason was clear:

1. **Commit validated the classification too early.** `validate_subdivision`
   ran before `reconcile_transaction_attributes`, and a builder that lays down
   one vertex per face corner and lets commit merge the coincident ones is
   holding several keys on one cell *on purpose*. Validation moved to after
   reconciliation.
2. **`extend_remapped` translated anchors but not owners.** A copied record kept
   the source model's `VertexKey`, which names a different vertex in the
   destination. It had never been exercised because nothing populated the
   subdivision. Fixed by `OwnerRemap`, and the merge routine now keeps the
   vertex/edge/face/solid key maps it had been discarding.
3. **An anchor drifts from the attribute it duplicates.** `Face(7v1)` was
   labelled at `Dart(49)` and, after its loops were rewritten mid-edit, had seed
   `Dart(112)` — while `Dart(49)`'s 2-cell had become another face's. The stored
   anchor was a second copy of where the face is, and nothing kept it in step.

The third is the design finding. The classification has two halves:

- **Derived.** An entity contains the cell its own anchor sits in. The
  attribute already says where that is, so storing it again is a loose pair that
  must be re-anchored on every edit and silently claims a foreign cell the first
  time it is not. `Model::entity_anchors` reads it from the stores;
  `OwnershipIndex::build_with_anchors` applies it before the stored records, so
  a stored record contradicting one is reported as the conflict it is.
- **Stored.** Everything *else* an entity contains: a closure point inside an
  edge, a seam inside a face, a buried corner inside a solid. This is what
  `Subdivision` holds, and it is genuinely authoritative.

Consequences that fell out:

- A shape with no scaffold stores **no labels at all** and is still fully
  classified. A rectangle's `subdivision()` is empty and every raw cell has an
  owner.
- Removing darts now re-anchors labels onto a surviving dart of the same orbit,
  chosen before the map changes. Entity attributes are guaranteed by their
  callers to reference only surviving darts; an ownership anchor names an orbit
  and carries no such guarantee.
- Labelling a cell as interior to an edge while a logical vertex still sits
  there is now rejected as `ConflictingOwnership`. That is a real contradiction
  — a closure point has no `VertexKey` — and the old storage could not see it.
  `tests/topology/model_state.rs` models the promotion properly: the vertex is
  removed in the same breath as the label is written.

Evidence: `tests/topology/classification.rs` asks a builder-produced rectangle
the questions `subdivision.rs` asks hand-labelled fixtures — every raw cell
names its entity, nothing is stored, and the frontier walk comes back with one
cycle of four distinct logical edges that the model actually holds.
`cargo test --all-targets --all-features` exits 0 with 729 passing, zero failing
and zero ignored. `cargo clippy --all-targets --all-features` exits 0 with the
same 25 distinct warnings, none in changed files. `cargo fmt` and
`git diff --check` pass.

Known gap, deliberately left: reconciliation removes merged-away entities
directly from the stores, so a *stored* label naming one would be orphaned.
Nothing writes stored labels in production yet, so nothing can hit it. Slice 2
(the cylinder seam) is the first code that will, and transferring such labels to
the merge survivor belongs there.

#### M4 slice 2 result — vertex-free circles, and where the integration stops

The cylinder slice went much further than expected, and stopped at one place
worth naming precisely.

**What the cylinder already was.** Probing it first was the single most useful
step: a built cylinder is *already* 3 faces and 2 edges with **no seam edge at
all**. The wall carries two `Wrapping` loops. The only deviation from this
plan's contract was its **2 closure vertices**, one per rim circle. So the
change was not "classify the seam" but "stop registering a vertex where a
circle's parameterization closes".

`add_circle_staged` now labels that point inside the edge instead of
registering a `VertexKey`. A cylinder is 3 faces, 2 edges, **0 vertices**.

**Five consumers had to be migrated**, each reading a vertex attribute for
something that is not a vertex question:

| Consumer | Was | Now |
|---|---|---|
| `viz::gmap::build_dart` | endpoints from vertex attrs, so a vertex-free edge emitted *no darts at all* | falls back to the curve's own domain |
| `revolve::RevolvedSourceVertex` | required a `VertexKey` | key is `Option`; the point comes from `Model::point_at_dart` |
| `validate_shell_orientation` | reference point from a vertex, else `domain_center` | a circular cap has neither, so the rim's own curve answers |
| `edges::split_*` | span from vertex points | `edge_reference_interval`: vertices when present, the curve's domain otherwise |
| `boolean::assemble::loop_traversal` | `bounded_unchecked`, which panics on a closed edge | keys are `Option`; nothing to merge where no vertex exists |

`Model::point_at_dart` is the shared derivation: the vertex's point when one
marks the dart, the curve's closure point otherwise. Ask it for a *position*;
ask the vertex store only when the *identity* of a logical vertex matters.

**Where it stopped.** The union of a block with a cylinder **tangent** to its
faces failed (`heal: false`, so healing is not involved). The result fails the
**winding** half of `validate_shell_orientation` — for a block face's edge, the
neighbour across `alpha0(alpha2(dart))` is not in the shell. Not the volume-sign
half; both *operand* shells validate with correct positive volumes. The
closed-span branch added to `sew_pair` is never reached in this test, so span
sewing is not the cause.

The cause was read as a promotion question this plan defers to M5: a tangency is
a **retained result junction**, and the rim circle's closure point is exactly
where that junction falls. Before this change the closure point happened to
carry a `VertexKey`, so the junction existed by accident. That much was right,
and it is not an argument for restoring the vertex. *Where* the promotion
belongs was wrong; the completion below says where it actually is.

Evidence, **corrected**: this entry first recorded 727 passing and 2 failing.
That count came from a run that stops at the first failing target. The real
figure was **716 passing, 13 failing, 0 ignored**, and the eleven unrecorded
failures were ordinary consumer migration rather than deferred design questions.
`cargo clippy --all-targets --all-features` holds at the same 25 distinct
warnings; `cargo fmt` and `git diff --check` pass.

#### M4 slice 2 completion — the promotion belongs to the split

The thirteen failures were three groups, and only the middle one was a design
question at all.

**A sixth consumer, missed.** The STEP exporter refused a vertex-free closed
edge outright: `edge_corners` dealt in `Vertex` views, and a circle has none to
hand over. A corner of an exported file is now a `Corner` — `Vertex(VertexKey)`
or `Closure(EdgeKey)` — resolved and cached by the cell it stands for rather
than by a key that may not exist. `TopologyError::ClosedEdge` is deleted: with
the closure point written as a vertex of the file alone, the case it named is
unrepresentable.

The import side is the same fact read backwards. `EDGE_CURVE` names two ends, so
a file has to give a circle somewhere to start, and reading that back as a
logical vertex would make an imported cylinder a different shape from a built
one. `demote_closure_vertices` runs after the seam pass and classifies inside
its edge every vertex left alone on a single closed edge. A vertex any second
edge reaches is a real junction and is left alone, which is what keeps a
deliberately marked circle marked.

**Where the promotion goes.** Splitting an edge at one parameter means "make two
edges here", and two edges need two junctions between them. A whole circle
offers only one — the cut — because the place its parameterization closes is
interior to the edge. That place is the other junction, so **the split is the
operation whose intent promotes it**. `promote_closure_point` does this in both
split paths: `disown_cell` on the 0-cell, then a vertex at the point
`Model::point_at_dart` derives. This is the M5 rule applied rather than
deferred — promotion is operation intent, and a split states that intent
completely.

That one change fixed five of the thirteen failures, the tangent Boolean among
them. The junction this plan said "has to be chosen rather than inherited" is
chosen by the operation that creates the need for it, which is the only place
that knows.

`ModelEdit::disown_cell` is new, and is the counterpart `own_cell` had been
missing: it unlabels one cell without touching anything else its owner claims.
`Subdivision::disown_at` removes one anchor, and `Model::disown_cell` offers
every dart of the orbit, since an entry sits on whichever dart the labeller
happened to hand over.

**Tests that named the deleted vertex.** Four, each retargeted rather than
relaxed. `closed_edge_darts_resolve_opposite_orientations` and the foreign
cylinder import now assert the absence.
`the_lone_vertex_of_a_closed_edge_is_preserved` kept its property — a 0-removal
declines a vertex with no pair to fuse — and moved to the shape that still has
one: a split rim, healed back into one circle, leaves exactly that lone corner.
The tangent union's counts lost one vertex, which is the far cap's rim closing
with nothing meeting it; its Euler note adds that closure point back rather than
counting it as a corner.

One failure is worth naming on its own. `bottom_faces` in
`tests/builders/removal.rs` selected planar faces whose vertices *all* sit at
z = 0 — vacuously true of a face with no vertices, so the cylinder's far cap
counted as a bottom face. It asks the support now. A predicate over a collection
that is allowed to be empty is a standing hazard in a kernel where cells need
not have corners.

Two tests were added to `tests/topology/classification.rs`, where slice 1's
evidence lives: a built circle classifies its closure point inside the edge, and
splitting that circle promotes the point so that neither junction is interior to
an edge any more.

Evidence: `cargo test --all-targets --all-features` exits 0 with **731 passing,
zero failing and zero ignored** (729 inherited plus the two new tests).
`cargo clippy --all-targets --all-features` exits 0 with the same 25 distinct
warnings, none in changed files. `cargo fmt` and `git diff --check` pass.
Python, benchmarks and the frontend were not rerun for this core-only
checkpoint.

#### M4 slice 2 addendum — a cut adds a corner; separation is a consequence

The completion above promoted a circle's closing point whenever the circle was
cut, so that "a split leaves two edges" stayed true. That was wrong, and a probe
said so: a circle cut **twice** came back as three edges and three corners,
where the answer is two of each. The invented corner also sat wherever the
parameterization happened to start — which is the accident slice 2 existed to
delete, reintroduced one stage later.

**The rule is that a cut adds a corner. Whether the edge separates depends on
whether it already had one.**

| edge cut | result |
|---|---|
| bounded | two bounded edges |
| unmarked | **one marked edge**, no edge created |
| marked | two bounded edges |

Cutting an unmarked edge is a **relabel of the map**: it already holds its two
darts and its single 0-cell orbit, so materializing that orbit as a corner costs
no darts and no links. Only the vertex's stored point says where the corner now
sits, and the span derivation reads it — a marked edge spans `[t, t + period]`
from its corner, which `Curve::interval_between` already answers for two
coincident points. `parameter_interval` had to stop asking `vertices_at_dart`,
which folds a marked edge's coincident ends to `None`.

`EdgeSplit` became an enum — `Separated { first, second, vertex }` and
`Marked { edge, vertex }` — because the flat struct could not say that nothing
was created. Its three internal consumers each wanted a different question
answered, which is why the field access hid the distinction: `vertex()`,
`created()`, `continuation()` and `edges()` are now separate.

The vocabulary is in `AGENTS.md`: **bounded**, **marked**, **unmarked**, with
`Edge::is_unmarked()` carrying it in code. `Edge::Closed` stays the umbrella over
the last two, because both *are* closed — a closed edge with a deliberate corner
is still closed.

#### A face's loops live in disconnected raw cells

Step 3 was probed before it was written, and it does not hold up: the frontier
walk cannot yet see a multi-loop face, because **a face's raw scaffold is one
disconnected 2-cell per stored loop**, and an entity can derive ownership of
only the one its own anchor sits in.

Asking every production shape for its raw 2-cells and their owners gives a rule
with no exceptions:

| shape | logical faces | loops per face | raw 2-cells | unowned |
|---|---|---|---|---|
| rectangle | 1 | 1 | 1 | 0 |
| disc | 1 | 1 | 1 | 0 |
| block | 6 | 1 each | 6 | 0 |
| **annulus** | 1 | **2** (outer, inner) | 2 | **1** |
| **cylinder** | 3 | 1, 1, **2** (wrapping pair) | 4 | **1** |
| sphere | 1 | **0** | **0** | 0 |
| torus | 1 | **0** | **0** | 0 |

One raw 2-cell per loop, and every loop after the first is unowned. Slice 1
established that an entity owns the cell its own anchor sits in and stores only
what it *else* contains; a face has one anchor, so it derives one cell, and
`recover_region` walks by involutions and turns across cells the entity owns
— it has no way to reach a cell that touches the region nowhere.

So for the cylinder wall, `cell_key` answers `FaceKey(3v1)` for both `Dart(4)`
and `Dart(6)` while `OwnershipIndex` answers `Some(Face(3v1))` and **`None`**.
`boundary_cycles` returns one cycle of one edge where the wall has two rims, and
the annulus likewise returns its outer rim alone.

**The consequence is stronger than "step 3 is blocked".** `FaceAttr.loops` is
not merely an authoritative ordering of boundaries that duplicates something
derivable. Today it is the *only* thing that makes a multi-loop face one face:
delete it and the annulus becomes two unrelated rims, the cylinder wall two
unrelated circles. It cannot be deleted before something else connects them.

That something is the scaffold M4 already owes:

- **A hole needs a bridge.** An annulus becomes one 2-cell when a bridge edge
  runs from the outer rim to the inner one, owned by the face as an interior
  cut. `boundary_cycles` then turns across it and emits the two cycles — which
  is exactly the case its doc comment describes, and exactly what the M1
  hand-built fixtures proved. Nothing in production builds one.
- **A wrapping pair needs a seam.** The cylinder wall's two rims join into one
  2-cell only through an edge running between them, owned by the wall. Slice 2
  observed that a built cylinder has no seam edge and read that as "there is no
  seam to classify". The probe says the opposite: without the seam the wall's
  second rim is not in the wall's region at all, and the wall is one face only
  because its loop list says so. The seam is not optional scaffolding; it is the
  connectivity.
- **A boundaryless face needs one too.** A sphere and a torus have *no raw
  2-cell whatsoever*, which is the `ShellRoot::Face` prediction this plan
  already recorded, met exactly.

These are one fact, not three: **a face's scaffold must be connected, and every
face whose boundary is not a single cycle currently has no connected scaffold.**
The bridge endpoints are raw 0-cells owned by the rim edges rather than logical
vertices, so a bridged annulus keeps both its circles unmarked — the machinery
slice 2 built for closure points is what makes this representable.

**Ordering.** This is the plan's own ordering correction recurring one level
down. The correction moved M3's views behind M4's *labelling*; step 3 now has to
move behind M4's *scaffold construction* for the same reason — a derived answer
waits on the thing it derives from. Take M4's face-scaffold work
(`add_annulus`, the cylinder wall, then the boundaryless primitives) before
steps 3, 4 and 5, and the refinement-invariance gate after it, where it can
finally be written against shapes that have something to refine.

Evidence: a throwaway probe over `modeling::faces` and `modeling::solids`,
reading `Model::ownership()` against `Model::cell_key::<Cell2>` and
`boundary_cycles`. It was deleted rather than kept: it asserts the gap rather
than a behaviour that should survive, and the behaviour it would assert is the
one the scaffold work is about to create. The suite is unchanged at **736
passing, zero failing, zero ignored**.

#### M4 slice 3 — the bridge, proven on an annulus

The scaffold work started, and the smallest shape that needs it settled the
construction. **This slice is in progress: the tree is not green.** What follows
is what is established, and what is left.

**The construction.** A face whose boundary is more than one cycle is built as
one cyclic *boundary word* whose slots alternate rim and bridge:

```
[bridge_out, inner rim, bridge_back, outer rim]
```

`alpha0` links the two darts of each slot, `alpha1` links consecutive slots, and
the bridge's two uses are `alpha2`-linked to each other — `bridge_out[0]` to
`bridge_back[1]` and `bridge_out[1]` to `bridge_back[0]`. Reading the bridge
*before* the hole and again *after* it is what makes the walk turn out of the
hole and back onto the outer rim instead of circling the hole for ever; that
ordering is the whole trick, and getting it wrong gives a walk that never
terminates or one that closes after a single slot.

The labelling is three lines: the bridge's 1-cell is owned by the face, and each
rim's closure 0-cell is owned by that rim's edge. Then:

- the annulus is **one raw 2-cell** of eight darts, where it was two disconnected
  cells of two;
- every raw cell of it has an owner, with nothing left unclassified;
- `boundary_cycles` returns **two** cycles of one logical edge each;
- the bridge is never emitted, because the walk turns across it;
- there are **no logical vertices**: a bridge foot is where a rim closes, so both
  circles stay unmarked, exactly as a whole circle should.

Five tests in `tests/topology/bridged_face.rs` assert those, and they pass.

Suite at the end of this slice: **708 passing, 33 failing, 0 ignored**, against a
736-passing baseline. `cargo fmt` and `git diff --check` pass, and clippy adds
no new lint.

**`Face::loops()` is now derived, and it had to be.** Bridging the annulus
forced step 3 in the same change rather than after it, which is worth recording
because the plan had them as separate steps. The reason is that a profile cannot
name a bridged face's loops: once the boundary reaches the hole along a bridge,
the raw `alpha0`/`alpha1` chain runs through *every* loop the face has, so the
two profiles the annulus used to register collapse into one, and
`outer_loop().edges()` answers 2 where the answer is 1. Only turning across the
face's own cuts separates them again. The step is not "derive the loops *then*
bridge"; a bridge and a derived walk are one change.

`Loop` is therefore no longer `Closed<Profile>` plus a kind. It carries the
walk's darts and offers `occurrences()` — one dart per oriented edge use,
collapsing the run of raw darts a refinement leaves inside one logical edge —
with `edges()`, `vertices()` and `corners()` built on that. The list is
refinement-invariant where a profile's dart list is not.

**What the migration cost, and what it caught.** `MergeTopology for Face`
collected its darts from the loops, which was only ever right because a profile
walk happened to enumerate every dart of a single-cycle face. A boundary walk
names one dart per occurrence, so copying a face silently dropped most of it and
`isolate` panicked. It reads the face's **region** now — every dart the face
covers, cuts included — which is the honest answer to a different question than
`loops()` answers. That one fix took the failure count from 169 to 95.

**Where it stands.** Every face whose boundary is a single cycle is green: the
block, the disc, the rectangle, and their export, boolean and healing paths.
The remaining 95 failures are, without exception, faces whose boundary is more
than one cycle and whose builder has not been given a seam yet — the cylinder
wall and every revolved band (`add_full_revolved_band_face` still lays down two
independent closed one-edge loops in two disconnected 2-cells), and everything
downstream of them: boolean on curved solids, STEP round-trips of cylinders,
seam removal and healing.

**The cylinder wall took the same word, and it worked.** Its builder is
`solids::sew_wrapping_lateral_face` — a cylinder is an extruded circle, not a
revolve — and it laid down exactly the annulus's mistake: two closed one-edge
loops in two 2-cells, the swept edge and its image at the far end of the sweep.
Given the seam it becomes **3 2-cells where it had 4**, the wall is one of them,
every cell is owned, the wall derives its **two** wrapping cycles, and the
cylinder still has **0 vertices** — both rims stay unmarked. Failures went
94 → 86.

**A false start worth recording, twice over.** `add_full_revolved_band_face` was
given the same word first, on the assumption that a wrapping pair is the annulus
with its rims at the ends of a sweep. Failures went 95 → 96 and revolve's own
nine did not move, so it was reverted. Re-applied later *with a probe attached*,
the probe said the band still had **4 darts** where the word makes 8 — the
function was never running. `add_revolved_edge` for a whole turn routes through
`add_full_revolved_edge_face` → `add_full_revolved_open_edge_face`, and
`add_full_revolved_band_face` serves a different caller.

The lesson is the one slices 1 and 2 already paid for: **probe which builder
actually produces the shape before changing one.** Reading the call graph and
believing it cost two attempts; one `println!` of the dart count settled it.

**Why revolve is harder than the two that are done.** The annulus and the
cylinder wall both *create* all their darts, so the word can be substituted
wholesale. `add_full_revolved_open_edge_face` instead **reuses the source edge's
own darts** as one of its two loops, through `consume_source_edge_as_closed_loop`
— the sweep consumes the profile rather than copying it. The seam has to be
woven into darts that already exist and already carry a logical edge, which is a
different operation from laying down a fresh word. That is the next piece of
real work, and it is where a shared helper should be designed rather than a
third copy written.

**The helper exists, and the word turned out to be a splice.**
`builders/scaffold.rs` holds `cut_between_loops(edit, face, first, second)`. It
does not build a word from scratch: every loop constructor in the tree already
produces a closed loop, so the operation is to **splice a cut into two loops
that already exist**, which works whether their darts were freshly made or
adopted from a consumed source edge. Reading `alpha1` past each loop's far end
*before* unlinking is what generalizes it — on a one-edge loop that reads back
the loop's own dart, which is why the simple case needs no special handling, and
on a face that already carries a cut it reads the next slot of a longer word, so
a third loop joins a face that already has two.

Applied at four sites so far: `add_annulus_staged`, `sew_wrapping_lateral_face`
(the cylinder wall), `add_full_revolved_open_edge_face`, and STEP import's
`sew_shell` for every bound after the outer one.

**What the migration exposed, which is the more valuable half.** Deriving
`Face::loops()` made `Model::ownership()` a dependency of a query builders call
*constantly*, and that surfaced a rule slice 1 had only half-stated:

- **Two anchors on one cell are not a conflict.** A builder that lays a vertex
  at every corner and lets commit fuse the coincident ones is holding several
  keys on one cell on purpose; identity reconciliation owns that question.
  `build_with_anchors` now lets the first anchor win and keeps conflicting
  *stored* records an error, since a stored record is the authoritative half and
  is the only one that can lie. This alone took the failure count 83 → 68.
- **`merge_topology` must ask the map, never the classification.** Copying a
  face happens half way through an edit, where the classification is allowed to
  be inconsistent and a region walk has nothing to report — and where a wrong
  answer silently copies part of a face. It reads the 2-cell orbit of every loop
  seed instead.

**An imprinted island needs a cut like any other hole.** `split_face_by_imprints`
was creating an inner loop and pushing it onto the face with nothing joining it,
so the derived walk found only the outer boundary. `finish_closed_imprint_split`
now cuts from the face's existing boundary to the new island. That fixed the
whole `faces` group.

#### The Boolean failure was a half-freed face, and bisection found it

The Boolean group stood at 18 failures and resisted every fix aimed at
scaffolding or orientation. It failed on **plain boxes**, with
`assemble::sew_pair` reaching `edit.sew(Dim::Two, da, db)` on a `da` that was
already `alpha2`-linked. Three guesses — asking the map for the face's dart
directly, turning each derived walk to agree with its stored seed, restoring the
old merge dart order — changed nothing and were reverted.

What found it was **bisection, not reasoning**. A temporary switch made
`Face::loops()` return the old stored-seed loops; Boolean went 24/18 → 34/8, so
the derivation was the cause. A second probe then computed both forms side by
side on every call and compared them: **identical**, every time — same loops,
same kinds, same edges, same darts. So the value was not the channel; something
that *consumed* the new shape of a `Loop` was.

`Loop::darts()` is that consumer. A derived loop names **one dart per oriented
occurrence**; the profile walk it replaced enumerated **every** dart of the
boundary. `ModelEdit::remove_faces` collected the darts to free with
`boundary.darts()` — so removing a face freed half of it, and left the other
half sewn to neighbours that no longer had anything on the other side. That is
exactly the `da` the Boolean tripped over. It now asks `Face::region_darts()`,
which is the whole face.

That one line took the suite 680 → 697 and Boolean 18 → 4.

**The general lesson, worth more than the fix.** Deriving a value that used to be
stored changes its *shape*, not just its provenance, and every caller that
treated the old shape as "all of it" is now silently reading a subset. The two
sites found this way — `remove_faces` and `MergeTopology for Face` — were both
asking "what is this face made of", which `loops()` never answered and only
appeared to. A boundary walk names what bounds a face; `region_darts` names what
it is made of. Sites that confuse them fail silently, which is why both
survived their first review.

**Cuts added since:** seam removal's ring case (removing a seam takes away an
edge, not the connectivity, so the two halves it was hiding get a cut), and
`add_polygon_with_holes`.

**An extrusion attempt, reverted.** The holed-cap cluster was tried next, on the
reading that `sew_extruded_loop` walks the raw profile chain — which, now that a
cap with a hole reaches it along a cut, runs straight through the cut and into
the other loop. The change read each cap's loops through the boundary walk
instead, anchored and rotated to the stored seeds so the two caps pair edge for
edge. It took the suite **708 → 679** and was reverted by inverse edit.

The reading is still believed correct; the replacement was not. Pairing two caps
is not the same problem as listing one cap's edges, and the rotation silently
changed which lateral face each edge got, which breaks the ordinary no-hole case
that was working. Whoever takes this next should separate the two: first make
one cap's loop yield its logical edges, and check a *block* still extrudes
before touching how the caps are paired.

**Where it stands: 708 passing, 33 failing.** The remaining clusters, in size
order: extruding a face with a hole (`extruded_holed_pentagon`,
`hollow_cylinder`, `extruded_face_with_a_hole`, the annulus shaft validation) —
the extrusion's cap faces carry the source's holes and have no cuts yet; Boolean
over *curved* solids (4) and its healing (3); `removal` (5); the boundaryless
torus and sphere (4), which is still the `ShellRoot::Face` gap and needs the M4
scaffold builder rather than a cut; and four dart-level tests in `model`, `face`
and `gmap` that assert the old dart lists directly.

**One flaw introduced and fixed in the same slice.** `Face::boundary_walks` first
swallowed a region-recovery or walk error and returned no loops. That is the
fallback pattern this plan already condemned once under the abandoned
`EdgeUseKey` attempt: it turns "this face's scaffold does not hold together"
into "this face has no boundary", which is a shape nobody built. Both sites now
name the face and the error instead.

Only then can the stored seeds go: `FaceAttr.loops` still carries the
`LoopKind` metadata, and `Face::loops()` pairs each derived cycle with the
stored definition whose seed the cycle passes through. That pairing is the last
use of a seed dart, and deleting it is the end of this milestone, not its
beginning.

#### The finding: a span's ends are not the edge's ends

Four separate places used a parameterization's own ends as a proxy for where the
edge begins and ends. Each was correct until an edge could be unmarked, and each
failed differently enough that none of them looked like the others:

1. `realize_edge_spans` matched a fragment to a span by its **bounded**
   endpoints, so a marked edge — one corner that is both its ends — never
   matched, and the Boolean saw a span with no second side.
2. `loop_boundary_edges` read each boundary corner off `pcurve.point_at(0.0)`.
   The face then believed its corner was where the pcurve started while the edge
   said otherwise, and the cut that should have followed was refused as
   degenerate.
3. `boundary_edge_at_uv` rejected a parameter near pcurve fraction 0 or 1 as
   "at an edge end", refusing to find the rim at the one point needing a cut.
4. `split_edge_at_points` and `check_split_parameter` rejected a cut near the
   ends of the edge's span as degenerate. For an unmarked edge those ends are
   where the *curve closes*, not corners — so a contact landing there was
   silently dropped. This is the one that matters most in practice: a builder
   puts a circle's parameterization origin somewhere meaningful, so a tangency
   tends to land exactly on it. The block/cylinder tangency touches each rim at
   the origin and at a quarter turn; the first contact was being discarded.

Rejected along the way: **re-anchoring a pcurve's interval to follow the
corner.** It works for a `Circle2` and breaks for a closed NURBS pcurve, which
reports no periodicity — `NurbsCurve2::point_at` clamps rather than wraps, so a
span shifted past the seam silently returns the endpoint. A pcurve's anchoring is
its own; the corner is the corner's.

That last point names real work outside this experiment. A closed NURBS is
`is_closed()` but not `Periodicity::Periodic`, so **a span on one that crosses
its seam is unrepresentable**, and every circle becomes a closed NURBS once it is
a pcurve. Nothing here needs it; it is written up in
[Periodic supports](periodic_supports.md) so the ordering it demands is on the
record before someone reaches for it.

Evidence: `cargo test --all-targets --all-features` exits 0 with **735 passing,
zero failing and zero ignored**. `cargo clippy --all-targets --all-features`
exits 0 with the same 25 distinct warnings, none in changed files. `cargo fmt`
and `git diff --check` pass.

#### The three shapes became three variants

The vocabulary — bounded, marked, unmarked — is now the type, not a convention:

```rust
pub enum Edge<'a, P> {
    Bounded(BoundedEdge<'a, P>),    // two distinct corners
    Marked(MarkedEdge<'a, P>),      // one, which is both start and end
    Unmarked(UnmarkedEdge<'a, P>),  // none
}
```

`ClosedEdge::vertex() -> Option<Vertex>` is gone, and with it the branch a
caller could forget to take. Corner access is total on each type:
`BoundedEdge::vertices()` gives two, `MarkedEdge::corner()` gives one, and
`UnmarkedEdge` offers no way to ask.

**Flat rather than nested, and the evidence for that is in this plan.** The first
attempt nested the two closed shapes under `Edge::Closed(ClosedEdge)`, so that a
caller meaning only "closed" could keep writing one pattern. `revolve.rs` was
such a caller — and it was wrong: a whole turn of a *marked* profile sweeps its
corner into an edge that bounds the result, while an unmarked profile sweeps a
boundaryless torus. The nested enum let that site keep compiling. Flattening
turns it into a compile error at the one place that can decide, which is the
entire point of moving the distinction into the type. Nothing took `ClosedEdge`
as a parameter either; every use of it was inside a pattern.

Sites that genuinely mean "closed" write `Marked(_) | Unmarked(_)`, which stays
exhaustive-checked, or ask `Closeable::is_closed()`.

`EdgeCore::has_corner_at(parameter, tolerance)` covers the other half. Every
"would this cut land on an end of the edge?" test used to reconstruct the answer
from the ends of the edge's *span*; this answers it from the corners, is total
over all three shapes, and compares points rather than parameters, so a caller
asking it never branches at all. Two of the four proxy sites collapse to it.

`revolve`'s marked case is refused by name rather than approximated:
`RevolveError::MarkedProfileRevolve`. Consuming the source loop as if it were
unmarked would delete a corner the caller placed deliberately and hand back a
shape nobody asked for. The refusal is tested; the swept-boundary torus it
describes is not built.

Evidence: `cargo test --all-targets --all-features` exits 0 with **736 passing,
zero failing and zero ignored**. `cargo clippy --all-targets --all-features`
exits 0 with the same 25 distinct warnings. `cargo fmt` and `git diff --check`
pass.

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
