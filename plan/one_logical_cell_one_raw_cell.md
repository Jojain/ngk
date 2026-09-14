# One logical cell, one raw cell

Status: **active — mid-flight, tree is red.** Steps 1-5 are done and the
sphere/torus representation is decided and built. 36 tests across five targets
fail, all from one unfinished cause: three remaining sites still build a
boundaryless face with an empty loop list, which now panics. See
[section 9](#9-where-this-stands). Extracted from
[logical_topology_over_gmap.md](logical_topology_over_gmap.md), which stays the
record of the larger migration. Where that plan assumes a logical entity may
span several raw cells, this one overrides it.

The rule is an architecture revision, not a milestone: it changes what the model
is allowed to hold, so it lands before further migration work rather than after.
Almost all of the work is deletion.

## 1. The rule

> Every logical vertex, edge, face and solid occupies **exactly one** raw cell
> of its own dimension. Not zero, not two.

The converse does **not** hold, and that asymmetry is the point. A raw cell need
not carry a logical entity of its own dimension. Such a cell is **embedded** in
the entity of higher dimension whose interior contains it — a periodic seam
inside a face, a bridge to a hole, the 0-cell where a circle closes, a cut face
inside a cavity. Embedding is what lets a purely topological seam be hidden from
everything above the map, which is the whole reason the classification exists.

Profiles and sheets are aggregates of entities rather than entities with a cell
of their own, so the rule does not reach them.

Two consequences name the work:

- Nothing above the map ever has to reconstruct a logical cell from several raw
  ones. **One key, one orbit.**
- The classification only ever records embedded cells, whose owner is of
  strictly greater dimension. "Has a record" and "is embedded" become one
  question, and the store is misnamed: nothing is subdivided.

## 2. What this makes impossible, and what that deletes

Each row is a state the invariant forbids, followed by the machinery that exists
only to survive it. All of it goes.

| Forbidden state | Deletes |
|---|---|
| One logical edge across several raw edges | `Loop::occurrences`, `Loop::continues_run`, `BoundaryCycle::logical_uses`, `LogicalEdgeUse` |
| One logical face across several raw faces | `Model::logical_sheet_darts`, whose doc comment already states the now-illegal assumption |
| A face with **no** raw cell | `ShellRoot` as an enum, `ShellRoot::{Face, at_face}`, `reroot_shells_at_darts`, `ModelEditError::MissingSheetRegistrationAtFace`, `Model::{sheet_key_at_face, solid_key_at_face, shell_sheet}`, `Option` on `FaceAttr::seed` / `Face::dart` / `Solid::dart`, and the three `dart_unchecked` twins |
| An entity's own cell carrying a stored record | `SubdivisionError::OwnerBelowCell`, and the dimension test inside `is_scaffold` |
| An entity reaching a second cell of its own dimension | `RegionError::{InteriorBoundaryNotShared, ForeignCell, UnlabelledCell, Disconnected, NoAnchor}`, `recover_all_regions` |

`recover_region` survives but collapses: an entity's region is one orbit, so the
walk is `orbit(anchor, orbit_indices(d))` plus the sense it flips on each step,
with no turning at all. `turn` itself survives — the **boundary** walk still has
to step over embedded cells — but the **region** walk stops needing it.

## 3. Where occurrence goes

`Loop::occurrences()` answers two questions at once, and the invariant kills one
of them:

- **Grouping** — several consecutive raw darts that are one logical edge. Now
  impossible. `occurrences()` becomes `darts()` verbatim, so it goes and its
  callers read `darts()`.
- **Repetition** — one face running along one logical edge twice. Still legal:
  a seamed sphere is one logical edge on one raw edge that its face's boundary
  walks up and back down. One key, one cell, two visits — all three consistent.

So the grouping machinery goes and the word survives only where repetition is
the subject. `LoopCorner` keeps its two darts for that reason: a `VertexKey`
still cannot tell two visits of one vertex apart.

`Loop::edges()` also tightens. The walk turns across embedded cells and never
emits one, so every emitted dart has a logical edge: the `filter_map` becomes a
`map`, and a dart without an edge is a bug to report rather than a row to drop.

**Decision required.** Whether a *committed* face may walk one logical edge
twice at all. Recommend **yes**: it is what a seamed STEP face carries on
import, and healing is the pass that removes it. Forbidding it would mean
refusing an import that has not yet been healed. If that answer changes, the
`LoopCorner` shape changes with it and nothing else here does.

## 4. Renaming

`Subdivision` is misnamed under the rule. Rename to **`Embedding`**, the word
for what it records, and avoid `Scaffold` for the store because
`builders/scaffold.rs` and `tests/support/scaffold.rs` already hold that name
for construction helpers. "Scaffold cell" stays as the prose noun for one
embedded cell.

| Now | Becomes |
|---|---|
| `topology/subdivision/` | `topology/embedding/` |
| `Subdivision` | `Embedding` |
| `OrbitOwnership` | `EmbeddedCell` |
| `OwnershipIndex` | `EmbeddingIndex` |
| `SubdivisionError` | `EmbeddingError` |
| `is_scaffold`, `is_scaffold_cell` | `is_embedded`, `is_embedded_cell` |
| `Model::subdivision()` | `Model::embedding()` |

`EmbeddedCell` carries the invariant in its type: its owner's dimension is
strictly greater than the cell's, so `OwnerBelowCell` becomes unrepresentable
rather than checked.

## 5. Enforcement

The check is cheap and direct, and replaces the region-flood check earlier
milestones assumed. For each entity, every dart its attribute names must lie in
**one** orbit of the entity's own dimension, and there must be at least one:

- **face** — every `loops[].seed()` and every `pcurves` key in one 2-cell;
- **solid** — every shell root in one 3-cell;
- **edge, vertex** — a single anchor, so nothing to compare; a second cell would
  need a stored record, which `EmbeddedCell` now forbids by type.

Two error cases, each naming the entity per *refuse rather than approximate*:
one for an entity spanning two cells, carrying the key and both representatives;
one for an entity with no cell, carrying the key.

It lands in two stages. First as a standalone validator with a test that
inventories what currently violates it — that is evidence, and it keeps the tree
green. It moves into `commit_model_transaction`, after
`validate_required_domain_attributes` and before lineage, only once the
inventory is empty.

## 6. Open question — boundaryless faces (sphere and torus)

**Blocked: discuss with Romain before implementing.**

`solids::add_sphere` and `solids::add_torus` build a face with **no darts at
all** ([`builders/solids.rs`](../src/builders/solids.rs) — `FaceAttr::with_loops`
with an empty loop list). That is the "not zero" half of the rule, and it is the
root of `ShellRoot::Face` and of every `Option<Dart>` on `Face` and `Solid`.

What such a face should actually hold is undecided. Sketch of the space, none of
it settled:

- **One dart, all alphas free.** Its 0-, 1-, 2- and 3-cell orbits are all that
  single dart, so every orbit exists and the gmap axioms hold trivially. Cheapest
  possible answer, but the cell carries no structure that says which surface
  direction is which.
- **Two darts**, α0-linked, giving a degenerate closed edge to anchor on.
- **A real seam**, matching what a STEP import carries: a meridian and pole cuts
  for the sphere, two cuts for the torus, all embedded so no logical boundary is
  exposed. Most structure, most construction work, and the one that makes a
  built sphere and a healed imported sphere the same map.

The decision changes what step 6 of the sequence builds and nothing else, so
everything before it proceeds independently. **Do not pick one unilaterally.**

## 7. Sequence

Steps 1–5 are independent of section 6 and each leave the tree green. Steps 6–8
wait on that decision.

1. **Inventory.** Add the section 5 validator as a standalone function plus a
   test that runs it over the fixture suite and reports violators. Not wired
   into commit. This also answers empirically whether anything today produces a
   logical edge across several raw edges — if nothing does, step 2 is safe.
2. **Delete the refinement machinery.** Row 1 of the section 2 table;
   `occurrences` becomes `darts`; `Loop::edges` stops filtering.
3. **Collapse `recover_region` to an orbit walk.** Row 5.
4. **Rename per section 4**, and make `EmbeddedCell` enforce strictly-higher
   owner. Row 4.
5. **Rebuild the dart-to-solid index.** With one 3-cell per solid,
   `logical_sheet_darts` is one orbit per solid and its cross-component flood is
   dead weight. Row 2.
6. **[BLOCKED — see section 6] Give every face a raw cell.**
7. **Collapse `ShellRoot` to a bare `Dart`**, and make `Face::dart` and
   `Solid::dart` total. Row 3. Depends on step 6.
8. **Wire the check into commit.** Depends on the inventory being empty.
9. **Vocabulary sweep.** AGENTS.md's classification and corner sections, module
   docs, error-variant docs. Per the working rules, a test that still names the
   removed design is kept and reworded, retargeted, or deleted — never left to
   pass vacuously.

## 8. Acceptance

- Every fixture in the suite commits with each entity occupying exactly one cell
  of its own dimension: sphere, torus, cylinder wall, capped cylinder, annulus,
  bridged face, seamed import before healing, cavity solid, solid with a handle.
- A hand-built face whose two loop seeds lie in different 2-cells is rejected at
  commit by name, and so is a face with no dart.
- `Loop::darts().len() == Loop::edges().len()` for every face in the suite —
  the property that replaces `occurrences()`.
- A seamed sphere reports one logical edge and two darts on its loop; healing it
  reports a face with no logical boundary and a raw 2-cell that still exists.
- Searching `src/` for "occurrence" returns only repetition-related prose.

## 9. Where this stands

Done and verified:

1. **Inventory** — `validation::{cell_occupancy_violations, validate_cell_occupancy}`
   plus `tests/topology/cell_occupancy.rs`.
2. **Refinement machinery deleted** — `Loop::occurrences` and `continues_run` are
   gone, `BoundaryCycle::logical_uses`/`LogicalEdgeUse` replaced by `edge_keys`.
   Provably behaviour-free: the derived edge index registers one 1-cell
   representative per `EdgeKey`, so `continues_run`'s "same key, different cell"
   was unsatisfiable and `occurrences()` already equalled `darts()`.
3. **`recover_region` collapsed** to one orbit walk; five `RegionError` variants
   and `recover_all_regions` deleted.
4. **Renamed to `Embedding`**, and a stored record now requires an owner of
   strictly greater dimension than its cell (`OwnerNotAboveCell`). The fixture
   harness grew the same two-halves split a real model has: anchors apart from
   records.
5. **Dart-to-solid index rebuilt** — `logical_sheet_darts` gone; `sheet_darts` is
   one orbit. The dart-to-face index is one lookup per face rather than one per
   loop seed.
6. **Sphere and torus built.** `FaceAttr` carries its boundary as one value
   (`FaceBoundary::{Loops, Closed}`) so an anchor can never drift from the loops
   it came from. `solids::sphere` is the four-dart bigon with its two edges
   coherently identified — one face, one embedded edge, two embedded poles,
   χ = 2. `solids::torus` is the eight-dart square with both pairs of opposite
   edges identified — one face, two embedded edges, one embedded vertex, χ = 0.
   Both report `(darts, 0 vertices, 0 edges, 1 face)`.
7. **Healing demotes rather than deletes.** Removing the last boundary of a face
   no longer empties the map: the seam and its corners stay as cells embedded in
   the face, which lands exactly on the four-dart sphere above. A healed seamed
   sphere and a built one are now the same map.

### What is left, and why the tree is red

Three sites still call `FaceAttr::with_loops` with an empty loop list, which now
panics by design — each needs the anchor dart its own context knows:

- `builders/revolve.rs` — the full-turn revolve of a closed edge;
- `exchange/step/topology/import.rs` — an imported boundaryless face;
- whatever the Boolean assembler hands a boundaryless operand.

Everything failing traces to those three: 11 in `builders`, 13 in `exchange`,
2 in `healing`, 6 in `modeling`. The 4 in `topology` are the pre-existing voxel
fixtures below.

Then, still untouched:

- **Step 7** — collapse `ShellRoot` to a bare `Dart`. Now unblocked: every face
  has a dart, so `ShellRoot::Face`, `at_face`, `reroot_shells_at_darts`,
  `MissingSheetRegistrationAtFace`, `sheet_key_at_face`, `solid_key_at_face` and
  `shell_sheet` all have nothing left to do.
- **Step 8** — wire `validate_cell_occupancy` into `commit_model_transaction`.
- **The voxel fixtures.** `tests/support/scaffold.rs` builds a cube-as-one-face
  and solids as 26 or 8 alpha3-sewn cubes. The rule forbids both: a solid is
  exactly one raw 3-cell, so **a solid can never be built from voxels**. Four
  tests assert the old shape and need the fixtures rebuilt as B-rep, which for
  the cavity means the scaffold face joining outer and inner shells.
