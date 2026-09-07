# Analytic-first intersections, with fast NURBS booleans

## Context

`ngk.fuse(ngk.block(10,20,30), ngk.sphere(6))` never returns. That repro is the
symptom; the cause is structural.

The kernel has a written **NURBS-first policy** (`CLAUDE.md:89-91`,
`plan/analytical_geometry.md:40-45`, `plan/nurbs_surface_surface_intersection.md:11`):
analytic types are stored, but every algorithm converts to NURBS and works there.
So `Surface::Plane` × `Surface::Sphere` — a pair whose section is *a circle*, in
closed form, in one line of algebra — is instead answered by: converting both to
rational Bézier patches, running a Bernstein-sign subdivision search over a 4096-node
budget to find seeds, marching along the contour with a 4-variable damped Newton
corrector for up to 4096 steps, fitting a NURBS curve to the traced states, and
re-tracing up to 4 more times at halved step if the fit misses. Then discarding
most of that when the result gets recognized, post-hoc, as a circle again
(`surface_surface/simplification.rs:13`).

That is paid once per face pair (6 here), and again per classification ray:
`classify.rs:168 curved_ray` runs a **complete curve/surface intersection** per ray,
per curved face, per fragment, up to 16 rays each.

There are **no benchmarks in the repository** — no `benches/`, no criterion, no timing
test. Performance is tracked only as unchecked plan items
(`plan/boolean_evaluation.md:983-989`). Nothing today can say where the time goes.

Two decisions frame the work, both confirmed:

- **Analytic dispatch in front of the NURBS solver**, not a rewrite. The NURBS path
  stays as the fallback for unrecognized supports *and* as the differential-test
  oracle. Every milestone below is independently shippable.
- **Make the repro actually work**, not just fail fast. `block ∪ sphere` is
  plausibly both slow *and* incorrect — see "Two separate problems" below.

## Two separate problems, do not conflate them

**Speed.** Everything above. Fixable by dispatch + caching.

**Correctness — the parameter seam.** `plan/boolean_evaluation.md:1354-1374`
documents this precisely: a face whose seam sits at `u = 0` gets pcurves that leave
`[0, 2π]`, so `FaceTrimDomain` classifies part of the loop as outside and drops it;
and `FaceImprintCut` only recognizes a chord between two boundary corners of a single
traversal. The repo's single `#[ignore]`d test is exactly this
(`tests/builders/boolean.rs:1294`). Separately, `classify.rs:55` only certifies a
ray/trim predicate for `Surface::Plane`; everything else is flagged `curved`.

`ngk.sphere(6)` is a *worse* instance than that ignored test: **one** face carrying
`Surface::Sphere`, closed by a single self-α2-sewn seam meridian edge, with two
degenerate pole vertices and pcurves at `u = 0` and `u = 2π`
(`src/builders/solids.rs:44-65`). There is no sphere boolean test anywhere in `tests/`.

The plan addresses both, but says which milestone does which. Speed work alone would
leave the repro failing — quickly, with a good diagnostic, but failing.

## What analytic dispatch can and cannot deliver

Worth being blunt up front, because it shapes the design.

The **3D section curve is exact** for the pairs in scope: plane×sphere is a
`Curve::Circle`, plane×cylinder is a `Curve::Ellipse` (or circle, or a line pair),
sphere×sphere is a `Curve::Circle`. Zero seeding, zero marching, zero Newton, zero
refit. This is the bulk of the win.

The **pcurve is generally not exact**. A circle on a sphere, expressed in the sphere's
own (longitude, latitude) parameters, is transcendental — not a `Line2`, `Circle2` or
`Ellipse2`. Likewise a plane section of a cylinder is `h = c + m·cos(θ − φ)` in
(θ, h). Exact pcurves exist only in the aligned cases (plane ⟂ axis → constant-`v`
`Line2`; plane containing the axis → meridian `Line2`).

So the analytic layer returns an **exact 3D curve plus a pcurve that is exact where a
closed form exists and otherwise a certified NURBS fit** — sampled from the known
exact 3D curve and inverted through the surface's closed-form `closest_parameter`
(`surfaces.rs:318` plane, `:403` cylinder, `:539` sphere, `:660` cone). That fit is a
cheap, well-conditioned 1D interpolation of a curve we already know, not a search. It
must carry its measured deviation so callers can tell the two apart. This also
neatly sidesteps the `to_nurbs` reparameterization mismatch documented at
`plan/analytical_geometry.md:56-80` — we never rely on NURBS parameter correspondence.

---

## Milestone 0 — Measure, and make hangs impossible

Nothing else should start before this. We do not currently know which stage hangs.

- **`benches/booleans.rs`** (criterion, new `[dev-dependencies]` in `Cargo.toml`):
  `block ∪ block` (planar baseline), `block ∪ cylinder` (the certified curved case,
  `plan/boolean_evaluation.md:1344-1352`), `block ∪ sphere` (the repro),
  `sphere ∪ sphere`, two orthogonal cylinders, and one genuinely free-form NURBS pair.
  Every case gets a wall-clock ceiling so a regression fails rather than hangs.
- **Node budgets on the two unbounded searches.** `curve_curve.rs:95-154
  intersect_pieces` and `dim2/intersections.rs:211-269 intersect_pieces` have *no node
  budget* — only `max_subdivision_depth = 32` per side, i.e. a worst case depth-64
  binary tree. Add a decrementing budget in the same shape as
  `curve_surface.rs:30 SEARCH_NODE_BUDGET`, reporting
  `IntersectionIncompleteReason::SubdivisionBudgetExhausted`. This matters most for
  the 2D one: `trim.rs:190` and `contacts.rs:611` call it with
  `linear_tolerance = parameter_tolerance`, far tighter than the default, making
  leaves much harder to reach.
- **Work counters.** Extend `BooleanDiagnostics` (`diagnostics.rs:8-25`, which already
  carries `candidate_pairs_tested`/`pruned` etc.) with per-stage elapsed time and
  solver call counts: `ssi_calls`, `curve_surface_calls`, `curve_curve_calls`,
  `trace_steps`, `newton_iterations`, `trim_domains_built`, `prepared_surfaces_built`.
  Cheap counters, always on.
- **Delete the debug spam** in the working tree: `assemble.rs:30-41` (the six-line
  `eprintln!` block is duplicated verbatim, and does a linear `Vec::contains` inside a
  loop over all fragments purely to print), `:277-279` (two of the three lines are
  identical, each allocating two `Vec<FaceKey>`), and `:297`.

**Exit:** `cargo bench` prints a stage-attributed profile of `block ∪ sphere`, and the
repro terminates — with a typed error if it is incorrect, rather than hanging. Whatever
that profile says re-ranks Milestone 5; if it contradicts the analysis here, say so
before continuing.

---

## Milestone 1 — Canonicalize analytic supports at construction

An analytic pair table keyed on `Surface::Cylinder` would **silently miss the kernel's
own cylinders**: `cylinder_at` (`src/modeling/solids.rs:60`) goes through the generic
extrusion, and `lateral_face_surface` (`src/builders/solids.rs:377-393`) emits
`Surface::Ruled(RuledSurface)` wrapping a `Circle` for anything non-linear.

- `lateral_face_surface`: extruding a `Circle`/`Bounded(Circle)` whose plane normal is
  parallel to the extrusion direction now yields `Surface::Cylinder`.
- The revolution builders (`src/builders/revolve.rs`) get the same treatment: a `Line`
  parallel to the axis → `Surface::Cylinder`, meeting the axis at an angle →
  `Surface::Cone`, perpendicular → `Surface::Plane`; a `Circle` centred on the axis →
  `Surface::Sphere`. `add_sphere` already passes an explicit `Surface::Sphere`
  (`solids.rs:57`) — this generalizes that special case.
- Keep a dispatch-time `Surface::canonicalized(&self) -> Option<Surface>` recognizer as
  a safety net for geometry built before this change or imported.

**The risk here is pcurves, not surfaces.** `lateral_face_uv` (`solids.rs:395`)
computes uv per surface variant; a `Ruled` face is parametrized by (curve parameter,
height) while a `Cylinder` is (θ ∈ [0,2π], height). Changing the variant changes the
parametrization of every pcurve on every cylindrical face, which touches
`broad_phase.rs:130` (`bbox_over`), `healing/predicates/surface.rs:38-47`,
`tessellate/face.rs:56-62` and `builders/faces.rs:964-985`
(`periodic_boundary_curve`). Do this milestone on its own, with the existing suite
green, before any analytic-table work lands on top of it.

**Exit:** `cargo test --all-targets --all-features` green with
`tests/modeling/solids.rs` updated to assert the canonical variants; benches unchanged
or better.

---

## Milestone 2 — The analytic surface/surface table

New module `src/geometry/dim3/intersections/analytic/` (`mod.rs`, `surface_surface.rs`).

```rust
/// How faithfully a section's pcurve represents it in a support's own parameters.
pub enum PcurveFidelity {
    /// The pcurve is the section, in closed form.
    Exact,
    /// Interpolated from the exact 3D curve; deviation is measured, not assumed.
    Fitted { deviation: f64 },
}

/// One section computed in closed form from the supports' own parametrizations.
pub struct AnalyticSection {
    pub curve: Curve,
    pub pcurve_a: Curve2,
    pub pcurve_b: Curve2,
    pub fidelity: PcurveFidelity,
}

pub enum AnalyticSurfaceIntersection {
    /// Proven disjoint — not "nothing found".
    Empty,
    Sections(Vec<AnalyticSection>),
    Coincident,
}

/// `None` means the pair is not in the table; the caller falls back to NURBS.
pub fn intersect_analytic_surfaces(
    a: &Surface,
    b: &Surface,
    options: IntersectionOptions,
) -> Option<Result<AnalyticSurfaceIntersection, IntersectionError>>;
```

`None` versus `Ok(Empty)` is the whole contract: the first declines, the second
certifies. A pair in the table that hits a case it cannot represent — plane×cone
yielding a parabola or hyperbola, which the `Curve` enum has no variant for
(`plan/analytical_geometry.md:195` lists them as unimplemented) — returns `None` and
falls back. Do not fabricate a NURBS `Curve` and call it analytic.

**Table, in implementation order:**

| Pair | Cases |
|---|---|
| plane × plane | line / coincident / empty — **move the existing closed-form path down from `contacts.rs:974-1038`** so it stops being boolean-local |
| plane × sphere | circle / tangent point / empty |
| plane × cylinder | ellipse / circle / two parallel lines / one tangent line / empty |
| plane × cone | circle / ellipse / line pair / apex point / empty; **decline** parabola and hyperbola |
| sphere × sphere | circle / tangent point / empty / coincident |

**Dispatch points.** `intersect_surfaces_with_options` (`surface_surface.rs:30`) tries
the table before `to_nurbs()`. `intersect_prepared_surfaces` (`:45`) can dispatch on
`a.source()`/`b.source()`, but preparing is itself the expensive part — so the real fix
is in the boolean: `intersect_general_face_pair` (`contacts.rs:1041`) tries the
analytic table on `face.surface()` **before** building either `FaceTrimDomain` or
`PreparedSurface` (`contacts.rs:1050-1053`).

**Pcurves.** Exact in the aligned cases (constant-`v` latitude circle → `Curve2::Line`;
constant-height cylinder section → `Curve2::Line`; meridian → `Curve2::Line`).
Otherwise sample the exact 3D curve, invert through the support's closed-form
`closest_parameter`, and interpolate with the existing
`NurbsCurve2::interpolate_with_parameters` (`dim2/nurbs.rs:133`) — passing the 3D
curve's own natural parameters, so the pcurve and the section share one
parametrization by construction rather than by luck. Then **measure** the deviation and
record it as `PcurveFidelity::Fitted`. No new fitter.

**Seam splitting starts here.** When a fitted pcurve would cross the support's periodic
seam, split the section at the seam and emit two pcurve pieces, each inside `[0, 2π]`.
This is the geometry-side half of the documented blocker; Milestone 4 is the
topology-side half.

**Exit:** `tests/geometry/intersections/analytic.rs` — for every table entry, the
analytic section agrees point-set-wise with `intersect_surfaces_with_options` within
`fit_tolerance`, and each pcurve evaluates back onto the 3D curve within linear
tolerance. Bench: `block ∪ sphere` face-pair time drops by orders of magnitude.

---

## Milestone 3 — Analytic curve/surface and curve/curve

Same shape, same decline-to-`None` contract, in
`analytic/curve_surface.rs` and `analytic/curve_curve.rs`.

- curve/surface: line × {plane, sphere, cylinder, cone}, circle × {plane, sphere,
  cylinder}. All closed-form root solves.
- curve/curve: line × line, line × circle, circle × circle (coplanar and skew).

Dispatch in `intersect_curve_surface_with_options`, `intersect_curves_with_options`,
their 2D counterparts, and — importantly — at the *boolean* call sites so the
`PreparedCurve`/`PreparedSurface` construction is skipped, not just the search:
`contacts.rs:167-224` (`compute_edge_contacts`), `contacts.rs:364`
(`intersect_edge_face`), and `classify.rs:188`.

This turns `classify.rs:168 curved_ray` — today a full NURBS curve/surface search per
ray, per curved face, per fragment — into a quadratic root solve.

**Exit:** differential tests against the NURBS routines; `classification_rays` cost per
fragment drops to roughly the planar case.

---

## Milestone 4 — Periodic trim domains and certified curved classification

This is the correctness milestone. It is topology-layer work, and
`plan/boolean_evaluation.md:1372-1374` already names it as the next Milestone 7 item.

- **`FaceTrimDomain` learns periodicity** (`trim.rs:32-71`). It takes the face's
  `SurfacePeriodicity` and classifies in the periodic quotient, so a query at
  `u = 2π − ε` and a pcurve at `u = 0` are recognized as the same seam. Fixes the first
  documented failure: loops straddling `u = 0` are no longer classified outside and
  dropped.
- **`FaceImprintCut` accepts a seam-to-seam chord** — a cut running from the `u = 0`
  boundary to the `u = 2π` boundary, which today it does not recognize. Fixes the
  second documented failure.
- **Exact containment for analytic loops.** Keep the flattened-polygon winding path
  (`trim.rs:98-145`, `TRIM_CHORD_RATIO = 1e-4`) as the general fallback, but when every
  pcurve in a loop is `Line2`/`Circle2`/`Ellipse2`, use exact segment/arc winding — no
  flattening, no few-hundred-segment linear scan per `contains` call.
- **Certify quadric ray predicates** in `classify.rs:55`, using Milestone 3's
  closed-form line×quadric, so `BooleanError::UncertifiedClassificationSurface` stops
  firing for spheres, cylinders and cones.

**Exit:** `tests/builders/boolean.rs:1294` un-`#[ignore]`d and passing; new tests for
`block ∪ sphere`, `block − sphere`, `block ∩ sphere`, `sphere ∪ sphere`; the Python
repro returns a correct solid.

---

## Milestone 5 — Make NURBS booleans fast

Independent of everything above; this is what keeps genuinely free-form booleans quick.
Ranked by expected payoff, **re-rank against the Milestone 0 profile before starting.**

1. **One `BooleanGeometryCache` per boolean**, keyed by `FaceKey`/`EdgeKey`, holding
   broad-phase bounds, `FaceTrimDomain`, lazily-built `PreparedSurface` and
   `PreparedCurve`, and the tessellation `classify::probe` needs. Threaded through all
   four contact passes, `clip.rs`, `graph.rs` and `classify.rs`.
   Today: `PreparedGeometry` (`contacts.rs:262-303`) is local to
   `compute_edge_face_contacts` and dropped on return, so `compute_face_contacts`
   re-prepares everything (`contacts.rs:1052-1053`); `FaceTrimDomain` is never cached
   at all and is rebuilt at `contacts.rs:141, 351, 1050, 1139, 1202`, `classify.rs:63`
   and `classify.rs:275`; and `contacts.rs:362` *clones* a whole prepared Bézier
   decomposition per pair to dodge the borrow checker. A face touching *k* partners
   pays *k* × conversion + decomposition instead of 1.
2. **Hoist the recomputed decompositions in seeding.** `seeds.rs:766-770` recomputes
   `b.bezier_spans()?` — a full clone-and-knot-insert — inside the loop over `a`'s
   spans. `seeds.rs:1035` (and `:633`) builds
   `PreparedSurface::new(&Surface::Nurbs(other_surface.clone()))` per cone-disjoint
   leaf, up to the 4096-node budget.
3. **Broad phase for edge×edge.** `contacts.rs:167-224` is a full O(E₁·E₂) product with
   no culling, each pair paying `to_nurbs() + bezier_spans()` on both curves
   (`curve_curve.rs:72-83`). Reuse the BVH in `broad_phase.rs:179-205` with an edge
   index. Vertex×vertex and vertex×edge (`contacts.rs:20-55`) are also unculled.
4. **Spatial index in classification.** `classify.rs:122 ray` scans every face of the
   opposite solid; `classify.rs:269 probe` builds a fresh `FaceTrimDomain` *and* a full
   `tessellate_face_key` mesh per fragment.
5. **Spatial hash for network canonicalization.** `graph.rs:203 record_event` and
   `:262 record_span` are linear scans (O(n²)), and `graph.rs:511 node_spans` is
   O(spans × events) inside an 8-pass fixed point.
6. **Incremental anchors.** `contacts.rs:1063-1070` and `:353-360` rebuild the anchor
   list by rescanning all accumulated contacts, per pair.
7. **Memoize broad-phase bounds.** `face_bounds` runs a full `bezier_spans()` and is
   called twice per face per boolean with nothing memoized across the two entry points.

Each item's acceptance criterion is a measured bench delta, not a code review.

---

## Milestone 6 — Tests and written policy

- `tests/geometry/intersections/analytic.rs` — differential tests, the NURBS solver as
  oracle, parametrized over placements *including* the degenerate ones: tangency,
  coincident axes, pole-crossing sections, sections lying on the seam.
- `tests/builders/boolean.rs` — the sphere cases and the un-ignored seam test.
- Repo conventions: tests in `tests/` mirroring `src/`, integration style, no inline
  `mod tests`; name tests after the invariant, not the bug.
- **Policy documents to rewrite**, all of which currently assert NURBS-first:
  `CLAUDE.md:89-91` → analytic-first dispatch with a certified NURBS fallback;
  `plan/analytical_geometry.md:40-45` → the pair table is filled, with its scope and
  its decline cases; `plan/nurbs_surface_surface_intersection.md:11,89`;
  `docs/boole_paper_ngk_integration.md:349`.

---

## Verification

```bash
cargo fmt && cargo clippy --all-targets --all-features && cargo test --all-targets --all-features
```

```bash
cargo bench --bench booleans
```

End-to-end, the original repro — rebuild the Python bindings with `.\build.ps1`, then:

```bash
python bindings/python/examples/show_block.py
```

It must return a solid, in well under a second, with `BooleanDiagnostics` showing zero
`ssi_calls` for the block/sphere face pairs (every one answered analytically).

## Risks

- **Milestone 1 changes pcurve parametrization** on every cylindrical face. It is the
  most likely source of unrelated breakage in the whole plan, which is why it ships
  alone.
- **Milestone 0 may contradict this ranking.** If the profile says the time goes
  somewhere unlisted — the 2D trim intersections, or network noding — Milestone 5 gets
  re-ordered and that should be reported rather than quietly absorbed.
- **`block ∪ sphere` may still fail after Milestones 0-3** because of the seam, and
  that is expected: Milestone 4 is the one that makes it correct. The plan is not
  finished at the point the boolean gets fast.
- The **analytic pcurve is a fit** in the non-aligned cases. It is a far better fit than
  a traced one, and its deviation is measured rather than assumed, but "analytic" does
  not mean "exact in parameter space", and `PcurveFidelity` exists so no caller can
  forget that.