# Shape healing — redundant topology removal

Status: **In progress** — milestones 0–4 are implemented (`src/builders/removal.rs`,
`src/healing/`, `tests/builders/removal.rs`, `tests/healing/`). See
[§12](#12-what-shipped) for where the implementation departs from this design.

## 1. Problem

Boolean evaluation (and any imprint-based operation) leaves topology that is
combinatorially valid but semantically redundant:

- an edge split in two leaves two edges whose curves are the *same support*
  restricted to adjacent parameter intervals, joined by a degree-2 vertex that
  carries no shape information;
- an imprint that turns out to be tangent, or a contact that falls exactly on an
  existing face, splits a face into fragments that all lie on the *same*
  surface, separated by edges that carry no shape information.

`tests/builders/boolean.rs::block_fused_with_cylinder_tangent_to_block_faces` is
the reference case: the cylinder is tangent to two block faces, the tangency
contact imprints those faces, and the result keeps the imprint edges even though
both sides of each imprint lie on the same plane.

Goal of this first step: given a valid solid, remove every cell whose removal
does not change the shape, and end with a topology where no such cell remains.

Explicit **non-goals** for this step: gap closing / tolerant sewing,
self-intersection repair, sliver and small-face removal, degenerate (zero-length
edge, zero-area face) cleanup, and tolerance-driven vertex snapping. Those are
*contraction*-flavoured or geometry-repair problems and get their own passes
later (§11, milestone 5).

## 2. The single theoretical operation

The book (`private_doc/Combinatorial_Maps_Book`, §6.2, Defs. 58–59, Algs. 50–51)
gives exactly the primitive we need — **`i`-removal**:

> removing an `i`-cell `C` merges the (at most two) `(i+1)`-cells incident to it.

- **0-removal** of a vertex merges its two incident **edges**.
- **1-removal** of an edge merges its two incident **faces**.

So "fusing faces" and "fusing edges" are not separate operations to design: they
are the *consequences* of removing the separating cell. The whole healing module
therefore needs exactly **two passes** over exactly **one** topological
primitive. This is the central architectural decision.

Removability (Def. 58), specialized to our 3-GMap:

- `i = n-1 = 2` (faces) is always removable — not used here;
- for `i ∈ {0, 1}`: `∀d ∈ C, α(i+1)∘α(i+2)(d) = α(i+2)∘α(i+1)(d)`.

Removal (Def. 59): let `Dˢ = αi(C) \ C`. Delete the darts of `C`; for each
`d ∈ Dˢ`, set `αi'(d) = (αi ∘ α(i+1))^k ∘ αi(d)` with `k` minimal such that the
result lands in `Dˢ`. Every other `αj` is untouched. Alg. 51 does this in place
in time linear in `|C|`.

`k > 1` really happens here — loops (Fig. 6.16) and boundaries (Fig. 6.17) — so
the implementation must follow the path, not assume `k = 1`.

## 3. Layering

The repo already separates *mechanism* (`src/builders/`, works on
`&mut TopologyEdit`) from *policy*. Healing keeps that split:

| Layer | Where | Responsibility |
|---|---|---|
| Mechanism | `src/builders/removal.rs` (new) | `is_removable` + `remove_cell_staged`: pure combinatorics + attribute reconciliation. Knows nothing about "redundant". |
| Predicates | `src/healing/predicates/` (new) | "do these two curves / surfaces describe the same support?" Pure geometry, no topology mutation. |
| Policy | `src/healing/passes/` (new) | "is this cell redundant?" — combines removability, predicates and structural guards, then calls the mechanism. |
| Entry | `src/healing/mod.rs` (new) | `heal` / `heal_staged`, options, report, errors. |

`src/builders/removal.rs` sits beside `builders/edges.rs::split_edge_staged`,
which it is the inverse of. Anything that wants to remove a cell (future
simplification, defeaturing, decimation) uses it without depending on healing.

## 4. Module layout

```
src/builders/
  removal.rs              # is_removable, remove_cell_staged, reseed_attributes

src/healing/
  mod.rs                  # heal, heal_staged, re-exports
  options.rs              # HealingOptions, HealingScope
  report.rs               # HealingReport, HealingSkip
  errors.rs               # HealingError (thiserror)
  predicates/
    mod.rs
    curve.rs              # curve_supports_match, join_curves
    surface.rs            # surface_supports_match
    pcurve.rs             # join_pcurves (2D counterpart, for FaceAttr::pcurves)
  passes/
    mod.rs                # fixed-point driver
    edges.rs              # 1-removal pass: fuse cosurfacial faces
    vertices.rs           # 0-removal pass: fuse cocurvilinear edges

tests/
  healing.rs              # module aggregator (mirrors tests/topology.rs)
  healing/
    predicates.rs
    vertex_removal.rs
    edge_removal.rs
    boolean_integration.rs
  builders/removal.rs     # mechanism-level tests
```

`src/lib.rs` gains `pub mod healing;`, `src/builders/mod.rs` gains
`pub mod removal;`, `tests/builders.rs` gains the `removal` module.

## 5. Mechanism — `builders::removal`

```rust
/// Def. 58. `dim` must be Zero, One or Two.
pub fn is_removable<P: Payload>(g: &GMap<P>, dart: Dart, dim: Dim) -> bool;

/// Def. 59 / Alg. 51, plus attribute reconciliation. One staged operation.
pub(crate) fn remove_cell_staged<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    dart: Dart,
    dim: Dim,
    merged: MergedGeometry,     // caller-supplied joined curve / kept surface
) -> Result<CellRemoval, CellRemovalError>;

pub fn remove_cell<P: Payload>(g: &mut GMap<P>, ...) -> Result<CellRemoval, CellRemovalError>;
```

`CellRemoval` reports the surviving `(i+1)`-cell key and the consumed one.

The mechanism is responsible for four things beyond the α surgery:

1. **Lineage.** The two merged `(i+1)`-cells are declared with
   `merge_edges_into` / `merge_faces_into` (and `merge_profiles_into` when two
   boundary loops fuse), so `EditPolicy` sees a genuine merge and
   `PreservePayload` keeps the survivor's payload. The removed `i`-cell's
   attribute is dropped with `remove_vertex` / `remove_edge`.
2. **Survivor choice.** Deterministic: the survivor is the cell whose reference
   dart is *not* deleted; if both qualify, the smaller key. Never
   nondeterministic — healing must be reproducible.
3. **Attribute reseeding.** Every `*Attr::dart`, `FaceAttr::outer_loop` and
   `FaceAttr::inner_loops` entry pointing at a deleted dart must be repointed to
   a surviving dart *of the same cell with the same orientation* (the `FaceAttr`
   contract: replace `d` by a dart in the same orbit, never by an arbitrary
   canonical representative). `TopologyEdit::remove_faces`
   (`src/topology/edit.rs:588`) already does this ad hoc; factor that loop out
   into a shared `reseed_attributes` helper and have both call it.
4. **Pcurve maintenance.** `FaceAttr::pcurves` is keyed by oriented dart.
   Deleted darts' entries are dropped. For a 0-removal, the two pcurve entries
   on each side of the vanishing vertex must be **joined in 2D** exactly the way
   the 3D curves are joined, and re-keyed onto the surviving dart. This is the
   part most likely to be got wrong; it gets its own predicate module and its
   own tests.

The whole thing runs inside the caller's transaction, so any failure rolls the
map back wholesale — the existing `edit.md` contract carries all the safety.

## 6. Predicates — when is a cell redundant?

### 6.1 Redundant vertex (0-removal)

`V` is redundant iff **all** hold:

1. `is_removable(V, Dim::Zero)` — i.e. exactly two edges meet there, in the
   `⟨α1, α2, α3⟩` sense, uniformly around the vertex;
2. the two incident edges are distinct cells;
3. `curve_supports_match(c1, c2, tol)` returns a match, and the two parameter
   intervals are **adjacent at the vertex point** — so their union is a single
   interval;
4. tangent continuity at the vertex within `ANGULAR_TOLERANCE` (implied by (3)
   for a shared analytic support, but checked explicitly so a NURBS/NURBS match
   with a kink is rejected);
5. `join_curves` succeeds.

**Guards.** Never remove the last vertex of a closed edge (a circle needs a seam
vertex). Never remove a vertex whose removal would collapse a one-edge profile
loop to nothing — a two-edge loop collapsing into a single closed edge is legal
and desirable, a one-edge loop vanishing is not.

### 6.2 Redundant edge (1-removal)

`E` is redundant iff **all** hold:

1. `is_removable(E, Dim::One)`;
2. `surface_supports_match(s1, s2, tol)` for the two incident faces, **including
   normal agreement** — two faces on the same plane with opposite orientation
   must not be fused;
3. the two incident faces belong to the same shell (never fuse across α3);
4. merging the two loops yields a valid loop set: closed, non-self-touching, and
   with a well-defined outer/inner partition after the merge;
5. the merged face's pcurve set stays consistent on the surviving surface
   parameterization.

**Guards.**

- *Seam edges.* A cylinder's lateral face has its seam edge with the **same**
  face on both sides and, trivially, the same surface. Removing it destroys the
  parameterization. Rule: when the two incident face darts resolve to the same
  `FaceKey`, refuse unless the surface is non-periodic in the relevant direction
  *and* the resulting loops are still closed and disjoint. This is the Fig. 6.16
  loop case and the Fig. 6.15 dangling-edge case; both are legal GMap-wise and
  both are wrong for us at this stage.
- *Inner/outer merge.* Removing an edge that joins an outer loop to an inner
  loop is legal and correct (it fuses a hole into the outer boundary) but must
  reclassify the resulting single loop as the outer loop. Handle it explicitly
  rather than as a fallthrough.

### 6.3 Support-matching predicates

```rust
pub enum SupportMatch { Same, Reversed }

pub fn curve_supports_match(a: &Curve, b: &Curve, tol: f64) -> Option<SupportMatch>;
pub fn surface_supports_match(a: &Surface, b: &Surface, tol: f64) -> Option<SupportMatch>;
```

Implementation follows the repo's NURBS-first policy with cheap analytic fast
paths first:

1. **Structural fast path** — both `Curve::Bounded(inner, interval)` with the
   same `inner` variant and matching analytic parameters (same `Line` axis, same
   `Circle` plane + radius): exact, allocation-free, and precisely the case
   produced by `split_edge_staged`, which keeps the parent curve and only
   narrows the interval. Same idea for `Plane` / `Cylinder` / `Revolution`
   surfaces.
2. **NURBS path** — `to_nurbs()` both sides and compare after normalization
   (degree, knot vector up to affine reparameterization, control points within
   `tol`).
3. **Sampled fallback** — evaluate both over the overlap and compare points and
   tangents / normals at a fixed sample count. Used only to *reject*; a sampled
   match alone never authorizes a merge when `strict` is set.

`join_curves(a, b, tol)` mirrors this: interval widening in case (1) — exact and
free — knot-join in case (2), refusal otherwise. `join_pcurves` is the `Curve2`
counterpart, and must handle the case where the two edges carry opposite
`Orientation` relative to the shared vertex: reverse one side before
concatenating, then set the survivor's default orientation from its reference
dart.

## 7. On storing `RefCell<Curve>` in topology

Recommendation: **don't**, at least not now.

- `GMap` derives `Serialize` / `Deserialize` and crosses the wasm and pyo3
  boundaries. `RefCell` (or `Rc<RefCell<_>>`) breaks `Sync`, complicates serde,
  and forecloses any future parallel traversal.
- It would silently break the transaction contract. `GMap::transaction`
  snapshots by cloning; a shared `Rc<RefCell<Curve>>` would be *shared with* the
  snapshot, so an in-place geometry mutation would survive a rollback. That is a
  correctness regression in the core invariant documented in
  `src/topology/edit.md`, not an ergonomic trade-off.
- It does not actually answer the question. Shared curve identity tells you two
  edges came from one split; it does not tell you the intervals are adjacent,
  that the orientations agree, or that the join is continuous. You still need
  the predicate. Pointer identity would be an *accelerator*, not a decision
  procedure.
- Most importantly, **it would not help the motivating case.** In
  `block_fused_with_cylinder_tangent_to_block_faces` the redundant topology
  comes from two *different operands* whose faces happen to be cosurfacial.
  Those cells never shared a curve or surface object, so provenance is blind to
  them. The geometric predicate is mandatory regardless.

If profiling later shows the predicate is hot, the snapshot-safe version of the
same idea is a `Copy`, serde-friendly provenance tag rather than interior
mutability:

```rust
pub struct GeometrySupport { pub id: u64, pub domain: Interval }
// EdgeAttr { dart, curve, support: Option<GeometrySupport>, data }
```

`split_edge_staged` propagates the parent's `id` with a narrowed `domain`;
matching becomes an `id` comparison plus interval adjacency, with the geometric
predicate as the fallback when the tag is absent (imported or boolean-generated
geometry). Deferred to milestone 5 — the predicates must exist first either way.

## 8. Entry points, driver and integration

```rust
pub enum HealingScope {
    Solid(SolidKey),
    Cells { vertices: Vec<VertexKey>, edges: Vec<EdgeKey> },
    WholeMap,
}

pub struct HealingOptions {
    pub scope: HealingScope,
    pub remove_redundant_vertices: bool,   // default true
    pub remove_redundant_edges: bool,      // default true
    pub linear_tolerance: f64,
    pub angular_tolerance: f64,
    pub max_iterations: usize,             // default 8
    pub strict: bool,                      // refuse on sampled-only matches
}

pub fn heal<P: Payload>(g: &mut GMap<P>, options: HealingOptions)
    -> Result<HealingReport, HealingError>;

pub(crate) fn heal_staged<P: Payload>(g: &mut TopologyEdit<'_, P>, options: HealingOptions)
    -> Result<HealingReport, HealingError>;
```

**Driver order** (`passes/mod.rs`): edges first, then vertices, iterated to a
fixed point.

1. 1-removal pass — fuses cosurfacial faces. Removing an edge can turn a
   degree-3 vertex into a degree-2 vertex, so it must run first.
2. 0-removal pass — fuses cocurvilinear edges.
3. Repeat while the previous round removed anything, bounded by
   `max_iterations`; exceeding the bound is a `HealingError::NoConvergence` (it
   indicates a predicate that flip-flops, which is a bug worth surfacing).

Two faces separated by *two* edges need both removals; after the first, the
second edge has the merged face on both sides, which is the same-face case
guarded in §6.2 — the guard must allow the legitimate shape of it, and the
fixed-point loop is what makes the two-step resolution work.

**Scope matters for the boolean integration.** `assemble::run` already knows
exactly which cells it created (`BooleanResultLineage`, `span_edges`), so it
passes `HealingScope::Cells { .. }` and healing stays O(new cells) rather than
O(model). Wire it as a new `BooleanOptions::healing: Option<HealingOptions>`,
called at the end of `assemble::run` inside the boolean's existing transaction.

Default **off** through milestones 1–3, so the current boolean tests keep
asserting raw post-imprint topology and healing lands without churning them;
flip to on-by-default in milestone 4 together with the updated expectations.

**`HealingReport`** carries removed / merged key pairs, the iteration count, and
— importantly for debugging — a `skipped: Vec<HealingSkip { cell, reason }>`
list. When a case still shows junk, that list is the first thing to read.

## 9. Invariants and validation

Commit already runs `validate_gmap`, `validate_all_solid_manifolds` and
`validate_all_solid_orientations`. Healing adds its own invariants, checked in
debug builds and asserted in tests:

- **Euler characteristic is preserved.** A 1-removal between two distinct faces
  drops one edge and one face; a 0-removal of a degree-2 vertex drops one vertex
  and one edge. `V − E + F` is unchanged in both. Any pass that changes it has
  removed something it should not have.
- **Cell counts are monotone non-increasing**, and the fixed point is reached.
- **Shape is preserved**: sampled points on the surviving faces still lie on the
  original surfaces within `linear_tolerance`.

## 10. Test plan

Tests live in `tests/`, mirroring `src/`, integration-style, named after the
stable invariant (per `skills/test-first-workflow`). Write them red first.

`tests/builders/removal.rs` — mechanism, on hand-built maps:

- `removing_a_split_vertex_restores_the_original_dart_count`
- `removal_follows_multi_step_alpha_paths_on_a_loop_edge` (Fig. 6.16)
- `removal_reseeds_attributes_that_referenced_deleted_darts`
- `removing_a_non_removable_cell_is_rejected`

`tests/healing/predicates.rs`:

- `bounded_curves_on_one_support_with_adjacent_intervals_match`
- `curves_on_one_support_with_a_gap_do_not_match`
- `coplanar_planes_with_opposite_normals_do_not_match`

`tests/healing/vertex_removal.rs`:

- `splitting_an_edge_then_healing_restores_a_single_edge`
- `a_corner_vertex_between_two_directions_is_preserved`
- `the_seam_vertex_of_a_closed_edge_is_preserved`

`tests/healing/edge_removal.rs`:

- `coplanar_faces_sharing_an_edge_fuse_into_one_face`
- `two_edges_between_the_same_face_pair_both_disappear`
- `a_cylinder_seam_edge_is_preserved`
- `healing_preserves_shell_euler_characteristic`

`tests/healing/boolean_integration.rs`:

- `block_fused_with_cylinder_tangent_to_block_faces_has_no_redundant_topology`
  — the motivating case, asserting final face / edge / vertex counts.

## 11. Milestones

| # | Deliverable | Status |
|---|---|---|
| 0 | Module skeleton, `lib.rs` + `tests/healing.rs` wiring. | **Done** |
| 1 | `builders::removal` — `is_removable`, `remove_cell_staged` for `Dim::Zero` / `Dim::One`, identity merges, attribute reseeding. | **Done** — `tests/builders/removal.rs` |
| 2 | Predicates: support fitting, `join_curves`, `surfaces_match`, `boundary_pcurve`. | **Done** — `tests/healing/predicates.rs` |
| 3 | The two passes + fixed-point driver + report + guards (seam, closed edge, outer loop, shared-edge count). | **Done** — `tests/healing/{vertex,edge}_removal.rs` |
| 4 | Boolean integration: `BooleanOptions::heal`, call site in `assemble::run`, lineage rewritten onto the surviving identities. | **Done, opt-in** — `tests/healing/boolean_integration.rs`; see §12 |
| 5 | Loop-reshaping 1-removal (same face on both sides), up to a single rejoined loop. | **Done** — see §12 |
| 6 | Splitting a rejoined boundary into two loops (annulus, cylinder seam), which needs outer/inner classification in parameter space; contraction (Defs. 63–64) for degenerate cells; optional `GeometrySupport` provenance tag; flipping `BooleanOptions::heal` on by default. | Outstanding |

## 12. What shipped

Milestones 0–4 are implemented. Four things differ from the design above, all
found while building it.

**The structural fast path of §6.3 does not exist.** `Curve::trimmed` converts
to NURBS, so splitting a `Bounded(Line, interval)` yields two degree-1 NURBS
curves rather than two narrowed `Bounded` views of the parent. Comparing
representations therefore misses the exact case healing exists for. The
predicates in `src/healing/predicates/curve.rs` work on sampled geometry
instead: both curves are sampled, one analytic support is fitted through
`start`, `through` and `end`, and it is accepted only when every sample lies on
it. Lines and circles are supported; a free-form pair is reported as
`SkipReason::CurvesNotJoinable` rather than approximated.

**Face fusion is restricted to planes and cylinders.** `Surface` has no
`PartialEq`, and a sampled comparison can produce a false "identical" — the
dangerous direction, because parameter curves would then be carried onto the
wrong parameterization. `surfaces_match` therefore compares defining data for
`Plane` and `Cylinder` only, and reports every other pair as
`SkipReason::SurfacesNotJoinable`. Coplanar planes with different frames are
handled by rebuilding the whole fused face's parameter curves through
`curve_pcurve`; a shared surface value carries its curves over unchanged.

**The loop-reshaping 1-removal is implemented, up to a single loop.** §6.2
planned to refuse an edge the same face bounds on both sides. That refusal was
too strong: two faces sharing *two* edges — which is what a tangency imprint
produces — fuse across the first edge into a face with a slit, and closing that
slit is exactly this case. `MergePlan::Loops` handles it: it counts the
boundary components the removal would leave, using the replacement links
computed for Def. 59 so the answer is known before anything is mutated. One
component is rejoined, and the new seed is chosen from the surviving darts in
the loop's own orientation class. Two components — a cylinder's seam, an
annulus closing up — are refused with `CellRemovalError::LoopWouldSplit`,
because which of them then bounds the face from outside is not a combinatorial
question. A seam on a periodic surface is refused before that, on
`Surface::periodicity`.

Two consequences fell out of it. The Def. 59 path does not always leave the
removed cell: at the vertex where a slit's two edges met, once both are gone,
it loops forever. `replacement_seeds` now returns `Option`, and a vertex or
edge with no replacement is removed rather than reseeded — which is what
deletes the isolated corner. And a cell can carry two identities part-way
through an operation, when a fusion earlier in the same transaction has left
the consumed key in place until commit; removing that cell now drops both, and
commit treats the merge that named them as spent
(`edit.rs::is_spent_merge`) instead of failing on a dangling survivor.

**The tangent union came out corrupt for a reason unrelated to healing.**
Splitting a *closed* boundary edge — a cylinder's rim — handed its two halves'
3D curves to the wrong pieces whenever the face traversed the edge from the
`alpha0` partner of the edge's own reference dart. Both endpoints of a closed
edge are the same point, so only direction distinguishes the halves, and
`closed_boundary_curve_reversed` measured that direction from the face's
boundary dart while the split assigns from the edge's reference dart. It now
composes the two with `edge_orientation_at_dart`. This is what drew an arc
across the block's bottom face in the viewer, and it reproduces with
`split_face_edge` alone, no Boolean involved.

**`BooleanOptions::heal` defaults to `false`.** Milestone 4 called for flipping
it on. Four tests assert raw post-split counts and would need updating, so the
flip is its own change rather than a side effect of this one.
