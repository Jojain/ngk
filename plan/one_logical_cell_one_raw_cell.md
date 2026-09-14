# One logical cell, one raw cell

Status: **active — steps 1-7 and 9 done, one test red.** The rule is built and
the tree runs it everywhere except two places, both named in
[section 9](#9-where-this-stands): a solid with a cavity still spans two raw
3-cells, which is why step 8 is not wired into commit, and one healing rejoin
leaves a face on two. Extracted from
[logical_topology_over_gmap.md](logical_topology_over_gmap.md), which stays the
record of the larger migration. Where that plan assumes a logical entity may
span several raw cells, this one overrides it.

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
| A face with **no** raw cell | `ShellRoot` as an enum, `ShellRoot::{Face, at_face}`, `reroot_shells_at_darts`, `validate_shell_roots`, `ModelEditError::{MissingSheetRegistrationAtFace, DanglingShellRoot, ShellRootNotAtDart}`, `ModelValidationError::SolidShellSurfaceOpen`, `Model::{sheet_key_at_face, solid_key_at_face, solid_key_at, shell_sheet}`, `MergeHandle` and `TopologyMerge::with_faces`, `Sheet::boundaryless_face`, `Option` on `FaceAttr::seed` / `Face::dart` / `Solid::dart` / `Sheet::dart` / `SheetAttr::dart`, and every `dart_unchecked` twin |
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

- **face** — its anchor, every `loops[].seed()` and every `pcurves` key in one
  2-cell;
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
inventory is empty. It is still in the first stage: the inventory holds one
entry, and section 9 says what it is.

## 6. Boundaryless faces — settled

A face covering a closed support stands on the **polygon schema of that
surface**, every cell of it embedded in the face:

- **a bigon** where one parameter direction is periodic and the other closes by
  collapsing to a point — a sphere, four darts, one embedded edge and two
  embedded poles, χ = 2;
- **a square** where both directions are periodic — a torus, eight darts, two
  embedded edges and one embedded vertex, χ = 0.

`builders::scaffold::add_closed_face_cell` is the one place that builds either;
it dispatches on `Surface::periodicity` and refuses a support it has no schema
for rather than guessing one. `solids::{add_sphere, add_torus}`, the full-turn
revolve of a closed edge and the STEP importer's `VERTEX_LOOP` face all go
through it, so a built sphere and a healed imported sphere are the same map.

`FaceAttr` carries its boundary as one value — `FaceBoundary::{Loops, Closed}` —
so an anchor can never drift from the loops it came from, and `FaceBoundary::edit`
hands out a guard that settles back into whichever variant the edit left. A face
that bounds nothing still answers `seed()`, which is what makes `Face::dart`,
`Solid::dart` and `SheetAttr::dart` total.

## 7. Sequence

1. ~~**Inventory.**~~ `validation::{cell_occupancy_violations,
   validate_cell_occupancy}` plus `tests/topology/cell_occupancy.rs`.
2. ~~**Delete the refinement machinery.**~~ Row 1 of the section 2 table.
3. ~~**Collapse `recover_region` to an orbit walk.**~~ Row 5.
4. ~~**Rename per section 4.**~~ Row 4.
5. ~~**Rebuild the dart-to-solid index.**~~ Row 2.
6. ~~**Give every face a raw cell.**~~ Section 6.
7. ~~**Collapse `ShellRoot` to a bare `Dart`.**~~ Row 3. `MergeHandle` went with
   it: it existed for the same reason one layer up.
8. **Wire the check into commit.** Still waiting on the inventory being empty,
   which today means cavity solids — see section 9.
9. ~~**Vocabulary sweep.**~~ Every test that named the removed design is
   reworded, retargeted or deleted.

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

Steps 1-7 and 9 are done and verified. `cargo test --all-targets --all-features`
reports **one** failure, and `--all-features` compiles again — the bindings'
`SharedFace`, `SharedSheet`, `SharedShell` and `SharedSolid` each carried an
`Option<Dart>` plus a fallback sense for the boundaryless case, and all four are
now a bare `Dart`.

What the later steps changed beyond the table in section 2:

- **`recover_region` refuses by name**, so a stale key reaches a named error
  rather than a flood that papered over it. That surfaced two real defects, both
  fixed: `Model::merge` dropped a boundaryless face outright (its `retain_mapped`
  emptied the anchor), and the healing edge pass wrote a rebuilt boundary onto a
  face key an earlier fusion in the same transaction had already absorbed.
- **A demoted face anchors along its old winding.** A bounded face states which
  way it points in its boundary's winding; a boundaryless one states it in its
  anchor, where the reading is the support's own normal by definition. So the
  anchor a seam removal leaves has to be the dart that read the support the way
  the winding did (`removal::unbounded_anchor`), or every shell holding the face
  silently turns over — which is how a round-tripped cavity came back as solid
  material.
- **`validate_oriented_shell_volume` refuses an empty shell** instead of
  indexing `faces[0]`.

### What is left

**A solid with a cavity spans two raw 3-cells, and step 8 waits on it.**
Its outer shell and its void are two closed surfaces with nothing between them,
so the `alpha0`/`alpha1`/`alpha2` walk from either never reaches the other.
`tests/topology/cell_occupancy.rs::a_solid_with_a_cavity_spans_two_raw_cells` is
the inventory entry. Wiring `validate_cell_occupancy` into
`commit_model_transaction` today costs six red tests, all of them this one
shape: two boolean results that register a cavity, and four STEP hollow-solid
tests.

What it needs is the **cut face** section 2 already names — a raw 2-cell the
solid owns, used twice by its boundary and `alpha3`-linked to itself there, so
the walk turns across it and the two shells stay two. The shape is settled and
built: `tests/support/scaffold.rs::cavity_solid` is the smallest one, a pillow
whose two squares are identified —

> outer bigon, inner bigon, two squares joining them along two vertical edges;
> identify the squares corner-for-corner and each bigon closes into a sphere,
> leaving 4 vertices, 4 edges, 3 faces and 1 volume, χ = 2.

and `handle_solid` is the same medicine for genus: a cube with its two opposite
x-faces identified, one shell of genus one. Both replace the old voxel fixtures,
which built a solid from 26 or 8 `alpha3`-sewn cubes and which the rule forbids
outright. What is missing is the *builder*: nothing in `builders/`, the boolean
assembler or the STEP importer synthesises such a face for a real cavity, so
`tests/support/hollow.rs` still glues two spheres together by hand.

**One healing rejoin leaves a face on two raw 2-cells.**
`tests/builders/removal.rs::imprinted_face_inner_loop_gets_removed` is the only
failing test. A rectangle with a filled square island heals to 1 face but 5
edges, and the last island edge is refused with `WouldUnboundFace`. Traced: at
the refusal the map holds 16 darts in two 2-cells — the rectangle at `Dart(0)`,
and at `Dart(8)` the last island edge together with the bridge that used to join
the rectangle to the hole. The bridge is still there and still owned by the
face; it has simply ended up looping the leftover slit onto itself rather than
onto the outer loop. So `MergePlan::loops`'s rejoin does not keep the cut
attached as the hole's boundary shrinks.

The fix is a design decision rather than a repair: `MergePlan::loops` currently
refuses a two-component split outright, on the grounds that "which of them bounds
the face from outside" cannot be answered combinatorially. Under this rule the
answer may be that neither does and the two are joined by a cut, the way `Ring`
already treats a seam removal's two halves — but that changes what a rejoin is
allowed to produce, so it wants deciding rather than assuming.
