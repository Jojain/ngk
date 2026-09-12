# STEP interop architecture (ISO 10303-21 / AP203 / AP214 / AP242)

> **Scope.** An *architectural* plan: the layer split, the decisions and their
> rationale, the extension points, and what lives where. Not line-level
> implementation. Staging is §10.
>
> **Where this plan ends.** Stage 6 (NURBS) is the last stage in scope. Stage 7
> — vendor leniency, the document type, AP242 — is **parked**, and assemblies
> in particular are deferred to `Model` rather than solved here; see §12. Do not
> start any of it without Romain saying so explicitly.
>
> **Status: In progress.** Stages 1–5 are complete. `part21/` reads and writes
> ISO 10303-21 on `winnow` with zero kernel references; the AP entity model §1
> always called for landed in stage 3 (D15), so every entity both directions
> touch states its attribute order once; and the round trip now closes on
> curved supports, seams, faces with no boundary at all, and cavities. Every
> analytic primitive NGK builds — block, cylinder, sphere, torus — survives a
> round trip, and OpenCascade reads each of them back at the right volume;
> OpenCascade's own box, cylinder, cone, sphere and torus import into sewn,
> correctly oriented maps that it then reads back unchanged.
> Stage 6 (NURBS) is the next entry point, and the last one. When it lands this
> plan is finished; what is left over is §12.

## Context

NGK has no file I/O of any kind. `std::fs` appears nowhere in the crate; the only
serialization is `serde_json` over a TCP socket to the debug viewers. A kernel that
cannot read or write STEP cannot be used with anything else, cannot be differentially
tested against a reference kernel, and cannot receive a bug report as a file.

`src/exchange/step/` already exists as an empty directory, unregistered in
`src/lib.rs`. This plan fills it.

The requirement driving the design is that the mapping be **complete, bidirectional,
and able to absorb geometry NGK does not have yet** — hyperbola, parabola, offset and
intersection curves, bounded and offset surfaces — without reopening the dispatch,
and to round-trip such geometry correctly meanwhile. `Surface::Torus` landing in
`518654b` mid-design is the proof case: it collapsed one whole decision (§2, D4) and
touched nothing else.

Three properties of the kernel shape everything below.

1. **NGK is NURBS-first by policy** (`skills/ngk-project/SKILL.md`), and every
   `Curve`/`Surface` has `to_nurbs()`. Every STEP curve and surface type is
   representable as NURBS. That makes the mapping a *total* function with a universal
   fallback rather than a partial one that fails on an unknown entity.
2. **NGK stores periodic faces seamlessly; STEP writes them cut open.** A cylinder
   wall is one ring face with two `LoopKind::Wrapping` loops and no seam edge; a
   sphere or torus is one *boundaryless* face with zero loops, edges and vertices,
   rooted by `ShellRoot::Face`. This is the largest piece of real work and it is
   asymmetric between the directions.
3. **NGK does not store a face's sense; it derives it.** `Face::normal_at`
   (`src/topology/face.rs:404-423`) flips the surface normal iff the boundary's signed
   area in the unwrapped domain is negative. This makes STEP's `same_sense` flag
   *redundant on import* — see D5, the most useful consequence in this document.

---

## 1. The central split: four layers, three seams

The failure mode for STEP code is one 3000-line module that lexes, resolves,
interprets units, builds geometry and sews topology in a single pass. The
architecture is the refusal to do that.

| # | Layer | Knows about | Does **not** know about |
|---|---|---|---|
| **L1** | **Part 21 syntax** — tokens, instance table, header | ISO 10303-**21** only | Any AP, any entity name, any NGK type |
| **L2** | **AP entity model** — typed records, reference resolution, units | Entity names, attribute order, AP203/214/242 differences | NGK types |
| **L3** | **Geometry mapping** — `CYLINDRICAL_SURFACE`↔`Surface::Cylinder`, and the parameter maps | `geometry::` | `topology::`, GMap, darts |
| **L4** | **Topology mapping** — shells, faces, loops, edge stitching, seams | `topology::`, `builders::`, `healing::` | Part 21 text |

**Why these seams:**

- **L1/L2** is the dependency boundary. L1 is small, frozen by the standard, and
  identical for every AP and even for IFC — the only part a third-party crate can
  supply (§9). Putting the boundary here makes the parser choice *reversible*.
- **L2/L3** makes AP differences someone else's problem. AP203/214/242 differ in
  product-structure boilerplate and a few entity spellings, almost none of it
  geometric. L3 never learns which AP it came from.
- **L3/L4** mirrors the crate's own layering (`geometry` below `topology`). L3 is pure
  math, exhaustively testable with no GMap at all — which matters, because that is
  exactly where the parameterization bugs live (§4).

L3 and L4 are each **bidirectional**: one table drives read and write. Keeping the
directions adjacent is what stops them drifting into a pair of mappings that
disagree — the classic round-trip failure.

---

## 2. Decision log

### D1 — Module at `src/exchange/step/`, layered above `healing`

`src/exchange/` is already staked out. Add `pub mod exchange;` to `src/lib.rs` — one
line; there is no registry machinery. In the `CLAUDE.md` layering table `exchange`
sits **above `healing`**: import must call healing to canonicalize a seamed file.
Nothing in the kernel may depend on `exchange`.

Named `exchange`, not `io`/`step`, because IGES, STL, glTF and a native `.ngk` format
belong beside it later.

It must live **in-crate**, not as a sibling crate: `FaceAttr.loops` is `pub(crate)`.

### D2 — Analytic-first dispatch with a certified NURBS fallback, reusing the crate's own convention

**This is the answer to "geometry we don't have yet".**

`CLAUDE.md` already describes this pattern for intersections:

> `intersect_analytic_*` return `Option<Result<..>>`: `None` declines the pair (fall
> back)… A pair in the table that reaches a case the `Curve` types cannot carry also
> declines.

L3 uses the **same convention** in both directions: an ordered table of readers and
writers per kind, each returning `Option<Result<..>>`, terminated by a fallback that
never declines.

- *Import*: `read_as_nurbs` handles `B_SPLINE_*` directly and converts anything else
  — exactly where a closed form exists (a `HYPERBOLA` arc is a rational quadratic, a
  `PARABOLA` arc a plain quadratic), by tolerance-driven approximation otherwise
  (`OFFSET_CURVE_3D`, `INTERSECTION_CURVE`).
- *Export*: `write_as_nurbs` via `Curve::to_nurbs()` / `Surface::to_nurbs_over(u, v)`
  — the `_over` form matters, since a `Plane`'s domain is unbounded and the box has
  to come from the face's unwrapped-domain bounds.

**What this buys.** Adding `Curve::Hyperbola` later is: write one reader that stops
declining, write one writer, delete nothing. No call site changes, and until then the
file already round-trips correctly — just as NURBS. `Surface::Torus` arriving during
this design is the worked example: it turned a two-paragraph workaround into one
table row. A `match` over entity names instead forces every future geometry addition
to touch the dispatch, and fails silently on the entity it has not met.

**The guard this decision must carry.** `src/geometry/traits.rs:14-21`: `to_nurbs`
reproduces a support *as a point set*, **not its parameterization** — a circle's angle
is not a linear function of the rational quadratic's parameter. So **any fallback
that also carries an interval must convert it through `Curve::nurbs_param_map()` /
`Surface::param_map_over()`** (`src/geometry/reparam.rs`). A fallback that copies the
analytic interval onto the converted NURBS produces a file that looks right and is
wrong. This belongs in rustdoc on the fallback itself.

### D3 — Round-trip preserves geometry to tolerance, not entity type

A documented contract, or it gets reported as a bug:

> A STEP round trip preserves the **point set** of every curve and surface to within
> the document tolerance, the topology exactly, and the analytic *type* only where
> NGK has a matching representation.

`CIRCLE` → `Curve::Circle` → `CIRCLE` is stable; `HYPERBOLA` → `Curve::Nurbs` →
`B_SPLINE_CURVE_WITH_KNOTS` is exact-but-retyped. Every demotion is recorded in the
import report (§7) rather than being silent.

### D4 — All parameter-space differences go through one `UvMap` value; no sign is reasoned about by hand

Two of NGK's surfaces do not share STEP's parameterization, and both differences are
orientation- or scale-bearing, so getting either wrong inverts the face normal and
surfaces much later as `SolidFaceNormalNotOutward`.

- **`SurfaceOfRevolution` is transposed.** `src/geometry/dim3/surfaces.rs:904` carries
  the inline comment *"u walks the profile curve, v is the angle"*; `domain()` returns
  `(curve.domain(), [0, TAU])` and `degenerate_rows` answers only for `Axis2::U`. ISO
  10303-42's `surface_of_revolution` puts **u = the angle of revolution**. A
  transposition is orientation-reversing in 2D, so it flips the boundary's signed area
  *and* the surface normal together.
- **`Cone` measures `v` along the generatrix**, STEP's `CONICAL_SURFACE` along the
  **axis**, with `frame.origin` on the `v = 0` reference circle rather than at the apex
  (`surfaces.rs:673-720`). Substituting `v_ngk = v_step / cos α` reproduces STEP's
  `radius = R + v·tan α` and `height = v` exactly.

Every such mapping is affine and axis-aligned, so **one value covers all of them**:

```
UvMap { swap: bool, scale: Vector2, offset: Vector2 }
  apply / inverse
  reverses_orientation() = swap XOR (scale.x * scale.y < 0)
  map_pcurve(&TrimmedCurve2) -> TrimmedCurve2
```

and the orientation question stops being a case analysis and becomes the sign of a
determinant.

| surface | swap | scale |
|---|---|---|
| plane, cylinder, sphere, **torus**, linear extrusion, B-spline | no | (1, 1) |
| **cone(α)** | no | **(1, 1/cos α)** |
| **surface of revolution** | **yes** | (1, 1) |

**`Surface::Torus` is an exact identity map** — verified term-for-term against ISO
10303-42's `toroidal_surface`:
`σ(u,v) = C + (R + r·cos v)·(cos u·x + sin u·y) + r·sin v·z`, with u the major
angle and v the tube angle, against `Torus::point_at` at `surfaces.rs:1457-1462`.
`Frame::from_xz(location, ref_direction, axis)` is the placement, and
`add_torus` already stores `Surface::Torus` directly (`src/builders/solids.rs:91`).
This is the decision `518654b` deleted: before the variant existed, a torus had to
round-trip through `SurfaceOfRevolution` and inherit its transposition *plus* a
profile-frame handedness mismatch against NGK's own builder. That is all gone.

So the `UvMap` now carries exactly two non-identity cases. It still earns its place —
the remaining two are the dangerous ones, it keeps the sign reasoning mechanical, and
it is where a future `OFFSET_SURFACE` or reparameterized variant lands without
touching anything else.

**One consequence worth stating up front:** `map_pcurve` is exact for a `Line2` under
every map and for `Circle2`/`Ellipse2` only when `|scale.x| == |scale.y|` — so a
circular pcurve on a *cone* must demote to `Curve2::Nurbs` under D3, since a
non-uniform scale turns a circle into an ellipse. In practice cone pcurves are almost
always lines, so this rarely fires.

Pinned by a test comparing `Surface::point_at(map.apply(p))` against the ISO formula
at a dozen points per surface type, with no GMap involved.

### D5 — Import does not store `same_sense`; it uses it as a checksum

Because NGK *derives* the face normal from the boundary winding (property 3 above),
an importer that (a) walks each bound in the STEP-composed traversal direction and
(b) writes each pcurve in the NGK chart through the `UvMap` gets the correct face
normal **automatically**. In the transposing case the two sign flips cancel: the
transposition negates both the normal and the signed area, and NGK multiplies them.

So `same_sense` is not state to carry — it is a **free consistency check**: compute
the mapped outer bound's signed area, compare against the file's flag, and on
disagreement either fail (strict mode) or record it. It costs one polyline and is the
cheapest possible early warning that a `UvMap` entry or a reconstructed pcurve is
wrong — catching it at the face that caused it rather than at
`validate_all_solid_orientations` much later.

**It earned that claim in stage 4.** The area has to be read from parameter curves
*placed on one branch*, not from the ones inversion returns: a rebuilt pcurve comes
back folded into a single period, so the two sides of a seam land on top of each other
and the rectangle the file described shoelaces as a triangle. A cylinder survives that
by luck; a cone does not, and the one thing that said so was this check reporting a
single face of an otherwise perfectly valid oriented solid.

**Stage 5 found the one case where it is a datum rather than a check, and the
exception is narrow enough to state.** A boundary that encloses no area has no
winding to compare against, so there is nothing for the flag to check and nothing
else for the sense to come from. That happens where a cut is walked out and straight
back along one parameter line: a whole sphere's two meridian walks invert to the same
longitude, because inversion answers within one period and a pole names every
longitude at once. The return walk is placed one period along the transverse axis to
make the boundary the rectangle it really is, and `same_sense` picks which of the two
directions that period runs in — a rectangle and its mirror describe the same sphere
and opposite normals. Everywhere else the rule above stands unchanged.

This is the "one conversion point per direction" rule that STEP round trips need, and
the reason it is affordable.

**Export takes the same idea the other way, and it is just as cheap.** Writing
`same_sense` needs no new machinery either: `Face::normal_at` *is* the support
normal flipped by the winding, so the flag is the sign of
`face.normal_at(u,v) · surface.normal_at(u,v)` — STEP's definition of it, asked
directly. Nothing is stored and no case analysis is written. The worry that a curved
support would need a `(u, v)` inside the trimmed region turned out to be unfounded:
`Face::normal_at` applies the winding as a *sign*, and the winding belongs to the
face rather than to a parameter — so any `(u, v)` where the support has a normal at
all gives the same answer, and one where it has none is what the refusal is for.

The dart-composition rule is separate and equally narrow:
`forward = (FACE_BOUND.orientation == ORIENTED_EDGE.orientation)` — both flags mean
*agrees*, so the walk runs forward when they agree and backwards when either one
alone is `.F.`. A reversed bound also reverses the *order* of its oriented edges,
not just each one's direction.
`ADVANCED_FACE.same_sense` does **not** enter it — it describes the normal, not the
walk.

`EDGE_CURVE.same_sense` is a separate composition again, and stage 4 corrected what
this document first said about it. On a line the corners do re-derive everything the
flag states, which is why planar import could ignore it. On a **closed** support they
cannot: two complementary arcs of one circle share both corners, and NGK's derived
span is always the one running *forward* along the support — so which arc an edge is
depends entirely on which corner its reference dart leaves from. The walk's direction
along the curve is therefore `forward == same_sense`, and that composed answer is what
roots the `EdgeAttr` and what the α2 pairing compares. Read on the way in; written
back out from the edge's own default span.

### D6 — Import is faithful; canonicalization is a separate, explicit stage

The importer builds the **seamed** form exactly as the file states it, then — as a
distinct step the caller can disable — runs
`healing::remove_redundant_cells(&mut g, HealingOptions::seams_only())`.

This is not new design; it is pre-built. `HealingOptions::seams_only()`
(`src/healing/options.rs:69-76`) carries the rustdoc *"This is what an importer
runs."* `tests/fixtures/seamed.rs` is a hand-built model of importer output and says
so in its module doc. The seam pass is milestone 7 of
`plan/seamless_periodic_faces.md`, status **Complete**.

Separating the stages means a file that heals badly still imports, the healing report
surfaces separately, and the importer is testable against the existing fixtures with
healing out of the picture.

### D7 — Export synthesizes seams in its own output; it never mutates the map

The tempting design — "un-heal" the map by inserting seam edges, then export normally
— is wrong twice: it mutates the user's model in order to serialize it, and it
reintroduces exactly the arbitrary, rotation-dependent seam that
`plan/seamless_periodic_faces.md` spent seven milestones removing.

Instead export derives a **seamed view**. This is mostly built already:
`UnwrappedFaceDomain::of_face` cuts a periodic face's domain open and returns closed
UV polygons, with `cut(axis)` reporting where the cut fell and
`UnwrappedFaceDomainCurve::corners()` — whose rustdoc is precisely the seam
description: *"the unwrapped domain's own cut, joining two wrapping loops"*, or a
degenerate row. Crucially, `place_loop` pushes **exactly one curve per
`loop_.edges()` entry, in order**, so real edges keep their identity positionally and
only the gaps are synthesized. `UnwrappedFaceDomain` is derived-on-demand by design
(*"Nothing here is stored on the map"*).

The abstraction to define — consumed by the STEP writer, which never learns which
kind of face it came from:

```
SeamedFace = closed UV polygons
           + per boundary: Real(EdgeKey, Orientation) | Synthetic(TrimmedCurve2)
```

Two producers implement it:

- **Loop-bearing faces** — walk the unwrapped loop; a real pcurve emits an
  `ORIENTED_EDGE` over the edge's existing `EDGE_CURVE`, a corner-gap emits a
  synthesized `EDGE_CURVE` + `VERTEX_POINT` from `surface.point_at(u, v)`. A gap whose
  two endpoints share a 3D image (a pole) emits **no edge** and reuses one vertex.
  Seam curves are isoparametric and analytic in every case: a line on a cylinder or
  cone, a great meridian circle on a sphere, a circle on a torus, the rotated profile
  or a circle on a revolution.
- **Boundaryless faces** (sphere, torus) — zero loops, so nothing to walk. The
  boundary comes from `Surface::domain()`, `periodicity()` and `degenerate_rows(axis)`
  (`surfaces.rs:48/61/95`) instead. Sense comes from `ShellRoot::Face{sense}`, which is
  also what makes a spherical *cavity* come out right.
  - A **sphere** is `UPeriodic` with degenerate rows at both poles: it cuts to one
    meridian traversed twice plus two pole vertices.
  - A **torus** is `UVPeriodic(TAU, TAU)` with `is_degenerate_at` always false and no
    degenerate rows (`surfaces.rs:1551-1573`) — **no poles at all**, so it is the
    *simpler* of the two: the standard `a·b·a⁻¹·b⁻¹` unfolding, four oriented edges
    over two closed edges sharing one vertex, with no degeneracy special case.

A `Surface::Nurbs` face is never periodic in NGK, so there is no seam to synthesize on
one. That is not a gap.

**The self-check that makes this trustworthy:** the loop-bearing outputs are exactly
the existing fixtures read backwards. A cylinder wall yields bottom circle, seam up,
top circle, seam down — `seamed_cylinder_wall`'s 8 darts. A spherical cap yields
latitude circle plus meridian twice — `seamed_spherical_cap`'s 6 darts. A sphere
yields `seamed_revolved_sphere`. **The torus has no such fixture and is the one
unproven link** — see D11.

### D8 — Pcurve reconstruction is lifting, and the capability already exists to be copied from

STEP does not require pcurves: `SURFACE_CURVE.associated_geometry` may carry a
`PCURVE` or may not, and OpenCascade-written files usually omit them on analytic
surfaces. `FaceAttr` **requires** a `TrimmedCurve2` per boundary dart. So the importer
must reconstruct them — and the capability is nearly all present already, in a form
that is merely private and mis-shaped.

**What exists.** Three things go by this name, and only the third is a gap:

1. **Projection onto a plane** — `builders::profiles::curve_pcurve`
   (`src/builders/profiles.rs:179`, `pub(crate)`). Lines stay `Line2`; every other
   variant converts to NURBS and has its homogeneous control polygon projected into
   plane coordinates, which is exact because the projection is affine. Plane-only.
2. **Synchronized fitting from intersection samples** —
   `intersections::surface_surface::fitting::fit_branch`. `pub(super)`, and it takes
   `Vec<TraceState>` from the tracer, so it cannot be called with a curve and a
   surface.
3. **Lifting an arbitrary 3D curve onto a curved support** — what import needs.

**And (3) substantially exists as `SectionTrace`**
(`src/geometry/dim3/intersections/analytic/surface_surface.rs:406`), whose
constructor `build(curve, interval, surface, domain, options)` is close to the exact
signature an importer wants. It already carries every step this plan previously
described as new work:

| step | `SectionTrace` member |
|---|---|
| closed-form pcurve when the support admits one | `exact`, `plane_pcurve` |
| recognize a UV line from samples, accept on deviation | `exact_piece` (samples at 25% / 75%, tolerance-checked) |
| NURBS interpolation fallback with refinement | `fitted_over` |
| unwrap the periodic branch into a continuous chain | `unwrap_periodic`, `shifted_into_period` |
| report exact vs approximated | `PcurveFidelity` |

So the work is **extraction and generalization, not construction**: lift
`SectionTrace` out of the analytic-intersection subtree into a public `geometry`
facility keyed on `(Curve, Interval, Surface)`. It is already *in* `geometry` — this
decision is about where it is reachable from, not about adding a layer. That also
pays down `plan/curved_support_pcurve_rebuild.md`, which needs the same primitive.

**The one real gap is narrow.** Period handling is u-only:

```rust
let period = match surface.periodicity() {
    UPeriodic(p) | UVPeriodic(p, _) => Some(p),   // the v period is discarded
    _ => None,                                     // VPeriodic gets nothing
};
```

and `unwrap_periodic` unwraps only `.x`. A `Torus` is `UVPeriodic(TAU, TAU)` and a
`SurfaceOfRevolution` is `VPeriodic`, so both are mishandled. **This is not a live
bug:** the analytic table only reaches plane, sphere and cylinder, all u-periodic or
aperiodic, so both arms are dead code today. STEP import is simply the first caller
that exercises them, and generalizing `period` to `[Option<f64>; 2]` with a
two-axis unwrap is the substantive change.

**It is also not blocked on `plan/curved_support_pcurve_rebuild.md`** (status
Proposed). That plan is about healing *fusing* two pcurves on a curved support — a
harder, different problem. Import needs *lifting*, which is one-directional and has
`Surface::closest_parameter` implemented for every variant, `Torus` included.

**Sequencing: this is a stage-4 need, not a prerequisite.** Planar import (stage 3)
is served by `curve_pcurve`, and export never lifts at all — it reads pcurves that
already exist on the face. Building this before stage 4 would be speculative.

**Stage 4 kept the analysis and rejected the conclusion.** The list above is what
lifting needs and `SectionTrace` does have all of it; what it *also* has is a great
deal that belongs to intersection and nothing to import — splitting a section at
degeneracy crossings, `IntersectionOptions`, combining fidelity across two supports —
and an extraction that carried those along would have moved the coupling rather than
removed it. So `convert/pcurve.rs` states the five steps directly, in one direction,
in about a third of the code. The two-axis unwrap this decision identified is the part
that mattered and it is written there, at the first caller that exercises it, rather
than in the subtree where both arms are still dead.

Two things were learned in the doing. A **plane is not a lift at all** but a
projection — an isometry onto its own coordinates, so every support keeps its
parameterization and no sampling happens; and `builders::profiles::curve_pcurve` could
not serve for it, because it ignores the endpoints for anything but a line and returns
the whole support, which is right for a builder whose curve already *is* the section
and wrong for an arc read out of a file. And the **collapsed-row handling is not
optional**: inversion at a pole returns an arbitrary longitude, so a sample there takes
the parameter that pins the row and the other from its neighbour, which is the
direction the curve was travelling when it arrived.

### D9 — Import is best-effort with a report, which constrains transaction ordering

Real files contain faces that do not close, loops with duplicated edges, and
references into nothing. Aborting the file on the first bad face is useless in
practice. This collides with the transaction rule (`src/topology/edit.md`): a
transaction is atomic and any failure restores the snapshot. The resolution is an
**ordering constraint**, which is why it is architectural:

> Build and validate all geometry **outside** the transaction. Enter
> `GMap::transaction` only with a set of faces already known to be constructible.
> One transaction per solid, so one bad solid does not lose the file.

Rejected faces are recorded with a reason, in the same shape as
`HealingReport.skipped: Vec<HealingSkip>`. Entities NGK cannot represent at all —
`POLY_LOOP`, an edge with more than two uses (non-manifold) — are **rejected by
name**, never silently skipped. `VERTEX_LOOP` is not one of them: it names a
point on a face that covers its whole support, which is the face with no loops
NGK already stores, so it is read rather than refused.

### D10 — Units and tolerance are per-document, normalized at the L2/L3 boundary

STEP carries units (`SI_UNIT`, `CONVERSION_BASED_UNIT` for inches, and **degrees for
plane angle**, which reaches `CONICAL_SURFACE.semi_angle`) plus an
`UNCERTAINTY_MEASURE_WITH_UNIT`, typically `1e-6`–`1e-7`. NGK's `LINEAR_TOLERANCE` is
a compile-time `1e-9` — three orders tighter than the files it will read.

L2 resolves the unit block and normalizes before L3 sees anything, so no NGK type ever
holds an inch or a degree. The document uncertainty is carried in the import options
and used for the decisions the exchange layer owns — vertex merging, edge stitching,
pcurve fit residuals — and passed to `HealingOptions.linear_tolerance`, which is
already a per-run field rather than the global const.

What this does *not* solve: constructors and `PointCoincidence` inside `geometry::`
still use the global const. Threading tolerance through the kernel is a much larger
change, explicitly out of scope; the boundary is documented rather than blurred.

### D11 — The torus round trip is proven by a fixture *before* the exporter depends on it

D7's torus unfolding needs two successive seam removals to converge, and there is no
`seamed_torus` fixture. **Hard prerequisite on the boundaryless stage:** add
`seamed_torus(major, minor)` to `tests/fixtures/seamed.rs` and prove
`remove_redundant_cells(&mut g, HealingOptions::seams_only())` reduces it to a
boundaryless face first. If healing cannot, that is a healing bug to fix in its own
right — `src/healing/mod.rs:16-21` claims exactly this capability — and must not be
smuggled into the exporter. No `#[ignore]`: if the prerequisite fails it fails loudly
and the stage does not start.

*It failed, and it was a healing bug — two of them, in `builders::removal`, both
older than the exporter that would have depended on them. Writing the fixture
first is what made them findable at all: one of the two showed up only about
half the time, because the shell seed it turned on came out of a `HashSet`.*

### D12 — Provenance in the report by default; `Payload` is the escape hatch

STEP entity ids, `PRODUCT` names, colours and layers have no home in a `GMap`.
`Payload` (`src/topology/payload.rs`) is architecturally the right place for durable
provenance — it rides inside the attributes and survives edits via `EditPolicy`. But
the importer is generic over `P: Payload` and can only produce `P::F: Default`, so it
cannot populate an arbitrary payload; and forcing a bespoke payload would make
imported shapes incompatible with `modeling::fuse`, which is `StandardPayload`.

Decision: importer generic with `P = StandardPayload` defaulted, provenance in the
report (matching the `HealingReport` idiom); a caller needing provenance to survive
later edits supplies a payload-filling hook.

### D13 — A STEP file is a document, not a shape — *superseded: assemblies wait for `Model`*

A STEP file holds several products in an assembly with placements and names.
`Shape<SolidTag, P>` holds one solid and no transform. `Model<P>` is a 17-line embryo
with no `insert`, and `docs/model_api.md` is unimplemented target design.

The decision was for `exchange` to define its **own neutral document type** — named
nodes with placements, leaves carrying `Shape<SolidTag, P>` — rather than block on
`Model` or flatten assemblies into a `Vec<Shape>` and lose structure the file
contained.

**That is reversed, and deliberately.** A second document type in `exchange` would be
a shape hierarchy NGK maintains beside the one `Model` is meant to be, and the two
would have to be kept in step for as long as both existed — for a structure no caller
is asking for yet. Assemblies therefore wait for `Model` and arrive through it, not
around it. Until then a file's product structure is read for its geometry and dropped,
which is what `read_step` returning `Vec<Shape>` already says.

What survives of D13 is the part that was never about the document type: **build one
`GMap` per B-Rep**, so each `Shape` owns its map. Geometry caching still works across
solids because `Curve`/`Surface` are values. The exporter takes solids one at a time
for the same reason.

The cost is stated rather than hidden: an assembly imports as loose solids in world
coordinates with their names and nesting gone, and NGK cannot write one at all. §12
carries it.

### D14 — `no_std`-shaped core, `std::fs` only at the rim

The crate builds as a `cdylib` for wasm, where there is no filesystem. L1–L4 operate
on `&str` and `impl Write`; `read_step_file`/`write_step_file` are three functions
behind `#[cfg(not(target_arch = "wasm32"))]` and the only place `std::fs` appears.
This also makes every test operate on string literals rather than fixture files.

### D15 — One Rust type per STEP entity, stating its attribute order once

§1 called L2 "typed records" and §3 listed `schema/entities.rs`; stage 3 shipped
without it, and the gap was not cosmetic. **A Part 21 file is positional and carries
no field names** — `#21 = EDGE_CURVE('',#22,#24,#26,.T.);` says nothing about which
reference is the start vertex — so something must turn a position into a meaning, and
with only an untyped record accessor that something was *every caller*. 47 sites
outside the product-structure boilerplate each knew an attribute order independently,
and the read and write sides knew it in two unrelated notations: `EDGE_CURVE`'s layout
was asserted in `topology/import.rs` and asserted again in `topology/export.rs`. They
agreed, but nothing made them.

`schema/entities.rs` holds one struct per entity, each with `read` and `record` a few
lines apart, walking the same attributes in the same order. **The indices are gone
entirely**: `Attributes` is a cursor, consumed in sequence, so the struct's field order
*is* the schema rather than being restated as integers that can disagree with it. Three
conventions carry that:

- **Skips are named, never counted** — `a.name()?` for the decorative name every
  geometric entity carries, `a.derived()?` for an `ORIENTED_EDGE`'s two `*` vertices.
  A counted skip shifts everything after it when it is wrong.
- **Bind with `let` before building `Self`.** Rust evaluates struct-literal fields in
  the order *written*, so filling them inline would make the literal's source order
  silently load-bearing — a hazard the indexed form does not have and the one real
  cost of going sequential.
- **The decorative name is not a field**, since NGK writes `''` and reads nothing from
  it. It appears only where it identifies the entity, as `CONVERSION_BASED_UNIT`'s
  `'INCH'` does.

**What makes it trustworthy is a property, not the types.** `read(x.record()) == x`,
once per entity, is a total check that the two directions agree — it catches a
transposed pair, an attribute read from the wrong position, a miscounted skip, and a
literal whose fields were reordered. Neither design prevents those by construction;
the test is what proves them absent. `tests/exchange/step_entities.rs`.

**Scope: every entity both directions touch, and no others.** Nineteen today —
geometry, topology, and the four unit entities, `SI_UNIT` most of all, since its order
is written in `schema/product.rs` and read in `schema/units.rs` and a mistake there
rescales the whole model silently. The ~29 write-only product-structure records stay as
raw `Record::new`: nothing reads them, so there is no second encoding to drift from,
and each already sits adjacent to its own argument list. They earn types if a document
reader ever gives them a second direction, which is parked (§12).

`part21` is untouched and stays untyped. Its ignorance is what keeps the parser choice
reversible, and the declining dispatch still needs an untyped view to ask "is this a
`PLANE` or a `CYLINDRICAL_SURFACE`?" before it knows what to decode into — which
`Attributes::decode` answers in the crate's own `Option<Result<..>>` convention:
`None` declines the keyword, `Some(Err(..))` is a real failure.

A **sequential cursor was chosen over indices** deliberately. The usual argument for
indices — that inserting an attribute mid-list forces a renumber — assumes a schema
that changes, and ISO 10303 entity attribute lists are frozen. The usual argument
against a cursor — that complex instances have no single attribute stream — does not
hold either: a cursor is *per record*, so a rational B-spline surface asks its instance
for two or three of them by keyword and walks each. What settled it is that the write
side was already sequential: `Record::new("EDGE_CURVE", vec![..])` has never had an
index in it, so the integers were an artefact of only the reader having been written.

---

## 3. Module layout

```
src/exchange/
  mod.rs          // pub mod step;   — no document type: assemblies wait for
                  //                   Model rather than for a second one (D13, §12)
  step/
    mod.rs        // read_step / write_step / *_file, options, reports
    error.rs      // one thiserror enum per layer, nested
    options.rs
    report.rs
    fs.rs         // the crate's only std::fs, cfg-gated       (D14)

    part21/       // L1 — zero knowledge of the kernel
      value.rs    // Value / Record / Instance
      parse.rs    // winnow parsers                            (§9)
      write.rs    // ours regardless; incl. the real-formatting guard

    schema/       // L2
      resolver.rs     // Attributes cursor, Origin, Located, SchemaError
      entities.rs     // one type per entity, both directions     (D15)
      units.rs  product.rs

    convert/      // L3 — pure math, bidirectional, no GMap
      uv_map.rs       // the load-bearing type                 (D4)
      iso_curve.rs    // the curve a cut runs along            (D7)
      pcurve.rs       // projecting onto a plane, lifting onto a quadric (D8)
      curves.rs  surfaces.rs  nurbs.rs  placement.rs

    topology/     // L4
      import.rs  export.rs  seam.rs  ids.rs
                  // stitch.rs is still inside import.rs — §10, stage 3
```

`part21`, `schema` and `convert` are **public**: tests are integration-only by project
convention, and `convert::uv_map` is the highest-risk unit in the feature. `import`
and `export` are private, their entry points re-exported from `step/mod.rs`.

---

## 4. The three conversions that corrupt silently

Isolated deliberately, because none of them crashes — they produce files that look
correct.

1. **The revolution transposition and the cone scale** (D4). Symptom appears as
   `SolidFaceNormalNotOutward` far from the cause. Mitigated by making every parameter
   go through one value, and by D5's checksum firing at the offending face.
2. **NURBS parameterization under `to_nurbs`** (D2). Mitigated by making
   `nurbs_param_map` mandatory on the fallback path and by testing the point set
   rather than the parameters.
3. **Real formatting on write.** `format!("{}", 1.0f64)` yields `"1"`, which is an
   *integer* in Part 21 and changes the parsed type. `{:?}` gives shortest round-trip;
   append `"."` when the result contains no `.` or exponent. One helper, one test table
   of awkward values.

The NURBS entity conversions sit just below these in risk — knot run-length coding
against NGK's fully-expanded `KnotVector`; homogeneous `HPoint` (`x·w, y·w, z·w, w`)
against STEP's Cartesian-points-plus-weights; **control-net transposition** (NGK is
flat row-major with u fastest, STEP is `[i][j]` with i along u — only a test with
`nu != nv` catches it); and periodic→clamped conversion, since NGK's NURBS types have
no periodic flag and report `Periodicity::None` unconditionally.

One hazard is *narrower* than it first appears: `KnotVector::multiplicity` compares
with exact `==`, but the importer *expands* from the file's explicit
`knot_multiplicities` rather than discovering multiplicity by comparison, so the values
are bit-identical by construction.

---

## 5. Entity coverage

| STEP | NGK | Note |
|---|---|---|
| `CARTESIAN_POINT`, `DIRECTION`, `VECTOR` | `Point3`/`Point2`, `UnitVector3` | dedup points on export — dominates file size |
| `AXIS2_PLACEMENT_3D` | `Frame::from_xz(loc, ref_direction, axis)` | exact constructor match; `$` ref_direction → any ⟂ |
| `AXIS1_PLACEMENT` | `Axis3` | |
| `LINE(p, VECTOR(d,m))` | `Line::through(p, p + m·d̂)` | affine param matches; `Line::with_axis` is private |
| `CIRCLE`, `ELLIPSE` | `Circle`, `Ellipse` | angle param, exact both ways |
| `HYPERBOLA`, `PARABOLA` | → NURBS | exact conic; **D3 demotion** |
| `OFFSET_CURVE_3D`, `INTERSECTION_CURVE` | → NURBS | approximated; D3 demotion |
| `TRIMMED_CURVE` | `TrimmedCurve` | interval direction carries `SENSE_AGREEMENT` |
| `PLANE`, `CYLINDRICAL_SURFACE`, `SPHERICAL_SURFACE` | identity `UvMap` | |
| **`TOROIDAL_SURFACE`** | **`Surface::Torus`** | **identity, both ways** — verified against ISO 10303-42 |
| `CONICAL_SURFACE` | `Cone` | **scale (1, 1/cos α)** |
| `SURFACE_OF_REVOLUTION` | `SurfaceOfRevolution` | **swap** |
| `SURFACE_OF_LINEAR_EXTRUSION` | `RuledSurface` | identity; STEP's magnitude baked into `direction` |
| `B_SPLINE_CURVE/SURFACE_WITH_KNOTS` + rational complex forms | `Nurbs` | §4 |
| `OFFSET_SURFACE`, `*_BOUNDED_SURFACE` | → NURBS | D3 demotion |
| `MANIFOLD_SOLID_BREP` / `CLOSED_SHELL` | `SolidAttr` / `SheetAttr` | |
| `BREP_WITH_VOIDS`, `ORIENTED_CLOSED_SHELL` | `SolidAttr.inner_shells` | both ways; a void faces into itself either side of the file, so the oriented shell is written `.T.` and a `.F.` one is turned on the way in |
| `ADVANCED_FACE` | `FaceAttr::with_loops` + pcurves | `same_sense` is a checksum (D5) |
| `FACE_OUTER_BOUND` / `FACE_BOUND` | `LoopDefinition::Outer` / `Inner` | never `Wrapping`/`Capping` — those are healing's output |
| `EDGE_LOOP`, `ORIENTED_EDGE` | `Profile` + loop darts | `ProfileAttr` must be registered or commit fails |
| `EDGE_CURVE` | `EdgeAttr` | no interval stored; derived from the corners, so `same_sense` picks which arc (D5). A closed edge names one corner twice |
| `VERTEX_POINT` | `VertexAttr` | one key per instance id; never merged by position |
| `PCURVE`, `SURFACE_CURVE` | `FaceAttr.pcurves` | reconstructed when absent (D8) |
| `SEAM_CURVE` | — | unwrapped to its 3D curve on in, then consumed by healing (D6). Not written: a plain `EDGE_CURVE` the loop walks twice is what identifies a seam |
| `VERTEX_LOOP` | a face with no loops | not a rejection: it is how a writer spells a face covering its whole support, so the bound is dropped and the face read as boundaryless. `same_sense` then states the sense, which nothing else can |
| `POLY_LOOP` | — | rejected by name (D9) |

Plus the unavoidable AP203 product-structure boilerplate — `APPLICATION_PROTOCOL_DEFINITION`,
`PRODUCT`/`_DEFINITION_FORMATION`/`_DEFINITION`/`_DEFINITION_SHAPE`,
`SHAPE_DEFINITION_REPRESENTATION`, `ADVANCED_BREP_SHAPE_REPRESENTATION`, the unit
block, and the `(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT
GLOBAL_UNIT_ASSIGNED_CONTEXT REPRESENTATION_CONTEXT)` complex entity — all confined to
`schema/product.rs`. The **read** side is deliberately lenient: find every
`MANIFOLD_SOLID_BREP` directly rather than walking down from
`SHAPE_DEFINITION_REPRESENTATION`, because product structure is where vendor files
diverge most and we need none of it.

---

## 6. Stitching: the import step with no existing counterpart

STEP gives faces whose loops reference shared `EDGE_CURVE`s by `#N`; NGK needs an
α2-sewn 3-GMap. A distinct component, not a detail of import:

1. Each `ORIENTED_EDGE` occurrence → two α0-linked darts; consecutive occurrences in a
   loop → α1 sew.
2. Build an **edge-use table** `EDGE_CURVE id → [(face, dart, direction)]`.
3. Two uses → α2 sew, paired so both darts start at the same vertex — exactly the
   invariant `validate_oriented_shell_volume` checks. **One** use → open shell
   (report). **More than two** → non-manifold, which NGK cannot represent — reject
   that solid by name (D9).
4. Register a `ProfileAttr` per loop and a `SheetAttr` per shell, or the commit is
   rejected (`MissingProfileRegistration` / `MissingSheetRegistration`).
5. α3 stays free on a solid's boundary darts.

A seam edge's two uses are on the *same* face; the rule handles it unchanged. Step 3
is where real files fail, and the report must name the entity id.

The edge-use table is **per shell**, not per solid: a void is disjoint from the
material's outside, so an `EDGE_CURVE` naming both would be an edge with four uses
rather than a join. A shell that is one face covering a closed support has no step 1
to do at all — it has no edge, so its `SheetAttr` is rooted at the face rather than
at a dart.

---

## 7. Errors and reports

`thiserror` throughout, **one enum per layer, nested** (`StepError::{Syntax, Schema,
Geometry, Topology}`), so a message says which layer failed and a caller can match on
the layer rather than on sixty variants. `NurbsError` and `TopologyEditError` wrapped
with `#[from]`.

**Every error that names a position carries the entity id and the source line.** A
STEP error without `#1234` in it is not actionable — this is a requirement on L1 and a
major input to §9.

Reports mirror `HealingReport`: counts plus `skipped: Vec<Skip>` with a reason enum.
D3 demotions, D5 sense mismatches, D8 reconstructed-and-approximated pcurves, and D9
rejected faces all land here.

---

## 8. Verification

No snapshot framework exists in the crate and adding one would be a first, so **round
trips are checked by invariant, never by text diff**. The precedent is
`tests/topology/serialization.rs`, which round-trips a `GMap` and compares
structurally.

Tests mirror `src/` per project convention: `tests/exchange.rs` as the harness root
with `#[path]` mod declarations, including the shared
`#[path = "fixtures/seamed.rs"] mod seamed;` that `tests/builders.rs` and
`tests/healing.rs` already use.

- **L1** — string literals: reals vs integers vs `.T.`, `''` escaping, `\X2\`,
  `/* */`, `$`, `*`, complex instances, dangling refs, duplicate ids. Malformed input
  must produce an error naming the line. Plus the real-formatting table (§4.3).
- **L3** — pure geometry, no GMap: `UvMap` against the ISO formula at a dozen points
  per surface; the D2 fallback asserted to preserve the point set under
  `coincides(.., LINEAR_TOLERANCE)`; a NURBS surface with `nu != nv` for the
  transposition; the cone v-parameter identity.
- **D5** — the sense checksum, as a pure unit test with a deliberately inverted file.
- **Import** — reproduce `seamed_cylinder_wall`, `seamed_spherical_cap` and
  `seamed_revolved_sphere` from equivalent STEP text *before* healing (using `seam_of`
  to find the doubly-walked edge), then assert the post-healing shape: a ring, a
  `Capping` loop, a boundaryless face.
- **Round trip** — one shared helper over each primitive: cell counts, then
  `validate_gmap` + `validate_all_solid_manifolds` + `validate_all_solid_orientations`,
  then vertex coincidence, then surface-kind identity, then an empty sense-mismatch
  list. The orientation validator does most of the work — it checks per-edge winding
  agreement *and* the global signed-volume sign, exactly the pair a broken sense
  mapping violates.
- **External conformance** — the check that actually matters: read a file written by
  another kernel, and have another kernel read ours. Prefer generating fixtures from
  NGK primitives plus a few small hand-written files; large real STEP files are often
  encumbered and bloat the repo.
- **Visual** — a `src/scripts/` scene loading a file, per the script registry
  convention. Not a test (the project excludes `src/scripts/` from coverage).

Project loop after each change: `cargo fmt`, `cargo clippy --all-targets
--all-features`, `cargo test --all-targets --all-features`.

---

## 9. The parser — **decided: `winnow` for L1 reading, our own writer**

**It is a smaller decision than it looks.** The boundary is L1/L2; L1 is ~10% of the
work, and **no crate supplies Part 21 *writing*** — truck, the closest precedent,
wrote its own output layer despite depending on `ruststep` for input. The writer is
ours either way, and the reader is reversible behind one seam.

| Option | Status | Assessment |
|---|---|---|
| **`winnow` 1.0.4** | **MIT** · July 2026 · 888M downloads · [crates.io](https://crates.io/crates/winnow) | **Chosen.** License matches NGK's MIT exactly. Every runtime dependency is *optional*, gated behind the `debug`/`simd` features — the default `std` feature pulls **nothing** transitively, so the wasm `cdylib` stays clean. 1.0 stable with six releases in 2026 (nom's last was Jan 2025). Decisively: it ships `ContextError` + `StrContext` and `LocatingSlice` **in-crate**, so §7's "every error names an entity id and a line" is close to free — with nom that needs `nom-supreme` + `nom-locate`, i.e. three dependencies to match one. |
| **`nom` 8.0** | MIT · Jan 2025 · 714M downloads | The incumbent, and fine. But error context and span tracking are external crates, which is precisely the part of L1 that matters here. `winnow` is a fork of nom by the `toml_edit`/cargo maintainer, so the combinator style ports either way. |
| **`ruststep` 0.4.0** | Apache-2.0 · Sep 2024 · [crates.io](https://crates.io/crates/ruststep) | The STEP-specific option. `ruststep::ast` is a schema-agnostic untyped AST — `Record`, `Parameter`, and `SubSuperRecord` for complex instances. Rejected on cost: pulls `nom` 7, `Inflector` (**unmaintained**), `derive_more` 0.99, `itertools` 0.10, and `thiserror` **1.0** against NGK's 2.0. Apache-2.0 into MIT adds a notice obligation. Read-only, and two years since release. **Its AST shape is borrowed wholesale anyway** — see below. |
| **`iso-10303`** | [J-F-Liu/iso-10303](https://github.com/J-F-Liu/iso-10303) · ~38★ | EXPRESS→Rust codegen, read-only, early-stage. Heavier buy-in than `ruststep` for less. |
| **`truck-stepio` 0.3.0** | Apache-2.0 · [crates.io](https://crates.io/crates/truck-stepio) | Not a dependency candidate — bound to truck's own geometry types. Valuable as **precedent**: a pure-Rust kernel that took `ruststep` for input and wrote its own output. Its README concedes shapes from set operations cannot be output yet — worth knowing about the difficulty of the export side. |
| **OCCT bindings** | — | Would solve STEP completely and contradict the point of the project. |

**Why a combinator library rather than hand-rolling**, given the grammar is only ~15
productions: the grammar was never the cost. The cost is *error quality on malformed
vendor files*, and that is exactly what `winnow` supplies in-crate. It also removes
the hand-written lexer as a maintenance surface, which was the one piece of this plan
with no test oracle other than files we do not have yet.

**The one design point borrowed wholesale from `ruststep`**, because it is what makes
complex entities a non-event rather than a special case:

```
Value    ::= Integer | Real | Text | Enum | Ref | Null | Derived | List | Typed
Record   ::= { keyword, params: Vec<Value> }
Instance ::= { id, records: Vec<Record> }   // len 1 = simple, len N = complex
```

`#5 = A(..);` and `#5 = (A(..) B(..) C(..));` are then the *same type*, and the whole
complex-instance API is `Instance::record(keyword)` / `Instance::is(keyword)` — which
covers both places AP203 requires one (the rational B-spline forms and the
`GEOMETRIC_REPRESENTATION_CONTEXT` unit block).

Two grammar rules carry most of the awkwardness and are worth stating now: a Part 21
**real always contains `.` and an integer never does**, and `.T.` must not parse as a
real — dispatch on `.` followed by an ASCII letter. String decoding (`''`,
`\X2\`/`\X4\`/`\X\`/`\S\`, `\\`) should **pass unrecognized escapes through
literally** rather than erroring: a malformed `\P?\` in a vendor file must not sink an
otherwise-good import.

Adding `winnow` makes it NGK's first parsing dependency; `Cargo.toml` gains one line
under `[dependencies]`. It pulls **nothing** transitively, as predicted, and the wasm
target still builds.

**Two things stage 1 settled that the design above did not anticipate:**

- **`cut_err` at the commit points is what makes §7's "every error names a line" true.**
  Without it a malformed instance merely backtracks, the enclosing `repeat` swallows
  the failure, and the message degrades to *"expected ENDSEC"* at column 1 of the bad
  instance — naming neither the defect nor its column. Three places commit: past `#N =`
  it can only be an instance, past `(` it can only be a parameter list, and past `'` it
  can only be a string. Only the innermost context is rendered; the outer ones are the
  enclosing constructs and listing them buries the useful one.
- **`decode_text` and `encode_text` are exact inverses, and `''` belongs to *both*.**
  Resolving the doubled quote in the parser instead — while it scans for the closing
  quote — looks natural and silently breaks that property, since the writer doubles
  quotes that the decoder then never collapses. The parser finds the literal's extent
  and decodes nothing.

---

## 10. Staging

Each stage leaves the tree green. **Export precedes import deliberately**: it needs no
pcurve reconstruction, and it gives the importer a generator of known-good input.

| # | Stage | Delivers | Explicitly not yet |
|---|---|---|---|
| **1** | ~~L1 Part 21 read/write on `winnow`~~ — **done** | text → table → text; `pub mod exchange` | any kernel reference at all — `part21` must compile knowing nothing of NGK |
| **2** | ~~Export, planar~~ — **done** | `block(1,2,3)` opens in another CAD system; AP214 boilerplate | import, curved supports, seams, NURBS |
| **3** | ~~Import, planar — the round trip closes~~ — **done** | units, uncertainty, stitching (§6), pcurve path on planes | curved supports, seams |
| **4** | ~~Analytic curved supports, both ways, with seams~~ — **done** | full `UvMap`; cylinder/cone/sphere/torus surfaces; circle/ellipse; the unwrapped-domain seam walk; `seams_only` healing on read; pcurve rebuilding on two periodic axes (D8) | boundaryless faces, NURBS |
| **5** | ~~Boundaryless faces and voids~~ — **done** | sphere and torus both ways, `VERTEX_LOOP` read as the boundaryless face it spells, `BREP_WITH_VOIDS` | NURBS |
| **6** | NURBS — **the last stage in scope** | both B-spline entities, rational complex forms, knot RLE, net transposition, periodic→clamped, `SURFACE_OF_REVOLUTION`. Adds `NurbsSurface::is_rational()` — one new public method in `geometry` | everything in §12 |
| ~~**7**~~ | ~~Robustness and assemblies~~ — **parked, see §12** | — | — |

Stages 1–3 prove the architecture against a real external kernel before anything
depends on it.

**What stage 2 settled.**

- **The file is written as AP214 (`AUTOMOTIVE_DESIGN`), not AP203.** The entity
  list is the one §5 names either way — only `FILE_SCHEMA` and the
  application-context string differ — and AP214 is what OpenCascade writes by
  default, so it is the spelling most readers are known to accept. `schema/`
  still holds it all in one module, so the choice stays one file's worth of
  change.
- **Pcurves are omitted, and this is not a gap.** `SURFACE_CURVE` with a
  `PCURVE` is optional (D8), so an `EDGE_CURVE` names its 3D curve directly.
  A block is 123 instances against OpenCascade's 350 for the same shape, and
  OCCT reads it back as a valid solid of volume 6000 — the pcurves it wrote for
  its own file were not load-bearing.
- **Boolean output is not yet exportable, for a reason that belongs to stage 6.**
  Cutting one block with another yields 11 NURBS edges against 7 lines, so
  `modeling::cut` results hit the `UnsupportedCurve` arm even though every
  support involved is planar. The exportable set today is therefore primitives
  and extrusions. Nothing about the walk changes when stage 6 lands the NURBS
  writer; this is purely the L3 table being short.
- **Instance sharing is exact, and its key must be uniquely decodable.**
  `add_shared` collapses equal records into one instance — this is what stops a
  file being dominated by repeated points (§5) — and the two ways it can go
  wrong are not symmetric. Comparison is **bit-exact, never within a
  tolerance**, so arithmetic error can only cost file size; a tolerance here
  would merge entities that differ, which is the direction that corrupts. The
  share table is keyed on a rendering of the record, and that rendering has to
  be *injective*: with a plain separator, a string containing it spells what two
  parameters spell, so `K('x\u{1},ty')` and `K('x','y')` collapse into one
  instance. Length-prefixing every keyword, string and enumeration makes that
  impossible rather than unlikely. Sharing is also confined to value-like
  entities; anything with identity — `VERTEX_POINT`, `EDGE_CURVE`, the faces,
  and a representation's own placement — is written under a name of its own, so
  topological identity never depends on the share table at all.
- **`fs.rs` and the read API landed early, on request.** `write_step_file` works;
  `read_exchange_file` works and is genuinely useful already, since L1 is
  complete — a vendor file can be opened and walked entity by entity long
  before it can be imported. `read_step`/`read_step_file` are pinned but return
  `StepError::NotImplemented { stage: 3 }`, having *first* run the Part 21
  parse, so a malformed file is still diagnosed with a line number today rather
  than hidden behind the missing stage. `StepReadOptions`, `StepImport` and
  `ImportReport` are shaped from D6, D9, D10 and D12 so that code written
  against the reader now keeps compiling when stage 3 fills the body in.
  `fs.rs` is `#[cfg(not(target_arch = "wasm32"))]` and compiled out rather than
  stubbed, so the wasm `cdylib` still builds.
- **Writing is exposed through the Python binding**, so a shape can be handed
  to another kernel from a REPL: `ngk.write_step(solid)` returns the path it
  wrote (a temp file when none is named) and `ngk.step_to_string(solid)` returns
  the text. Refusals arrive as exceptions — `ValueError` for geometry a stage
  cannot yet write, `OSError` for the filesystem — so the stage boundaries stay
  visible from Python rather than turning into a corrupt file.
  `bindings/python/tests/test_step.py` round-trips through build123d directly.
- **An external oracle is wired in and repeatable.** `cargo run --example
  step_export_fixtures` writes the shapes that
  `tests/fixtures/step/validate_ngk_export.py` reads back through OpenCascade,
  asserting volume, area, bbox and cell counts. `cargo test` does not depend on
  Python; the two are run together deliberately, because a file can satisfy
  every structural invariant the Rust tests assert and still be refused by a
  real reader.

**What stage 3 settled.**

- **D5's composition rule is written the wrong way round, and it matters.**
  The document says `forward = FACE_BOUND.orientation XOR
  ORIENTED_EDGE.orientation`; both flags mean *agrees*, so the walk is forward
  when they are **equal**, not when they differ. Inverting it fails nine of
  the fourteen import tests rather than one, which is the reassuring part —
  a reversed loop does not close, so most faces are refused outright instead
  of arriving silently inside out.
- **`EDGE_CURVE.same_sense` is read in neither direction, and D5 overstates
  what it does.** It is not "which vertex the stored dart starts at" — that
  is fixed by `edge_start`, an attribute rather than the flag. The flag
  relates the *support* to that pair of vertices, and since `EdgeAttr` stores
  no interval, NGK re-derives exactly that from the vertices. What the
  importer does need is to root the `EdgeAttr` on a dart running the way the
  `EDGE_CURVE` does, so the default orientation NGK derives is the one the
  file declared and export writes the same flag back out.
- **`FACE_OUTER_BOUND` is not optional in theory only: OpenCascade writes
  none at all.** Risk #7's fallback is therefore a stage-3 requirement, not a
  stage-7 leniency item — a pierced face arrives as two indistinguishable
  `FACE_BOUND`s, and taking the wrong one as the outer boundary is a mistake
  no validator catches, since both windings are legal. The rule implemented
  is: a declared outer bound wins, a lone bound is outer, and otherwise the
  one enclosing the most area is, reported as `GuessedOuterBound`.
  `tests/fixtures/step/holed_slab.step` is the fixture that exercises it.
- **The α2 pairing rule is one line, and it is the same statement the
  orientation validator makes.** Two loops walking a shared edge in opposite
  directions — which an orientable shell always does, seams included — sew
  one's *start* against the other's *end*. That is precisely what makes
  `α0 ∘ α2` land back in the boundary walk, which is what
  `validate_oriented_shell_volume` tests, so the invariant and the
  construction are the same fact written twice.
- **The entity model landed with it, and §1 had been right to ask for it.**
  Reading through a bare positional accessor left 47 sites each knowing an
  attribute order, and the two directions knowing it separately — see D15 for
  what replaced it and why the cursor won over indices.
- **Stitching did not become its own module.** §3 lists `topology/stitch.rs`;
  it stayed inside `import.rs`, because the edge-use table is built from the
  same planned faces the sewing consumes and splitting them would mean
  publishing `PlannedFace` between two private modules for no reader's
  benefit. Worth revisiting if stage 4's seam handling makes it grow.
- **D10 is two lines of code and one asymmetry.** Scales are resolved once
  when the `Resolver` is built; the length scale is then applied in
  `read_point` and in a `VECTOR`'s magnitude, and *nowhere else*. A
  `DIRECTION` is a ratio and must not be scaled — that is the whole of it,
  and it is the one place a wrong answer would be uniform enough to look
  right.
- **`StepError::NotImplemented` is gone.** No direction of the mapping is
  absent any more, and what remains are geometry *kinds*, which the
  `Unsupported*` and `Unreadable*` variants already name. Leaving a public
  variant nothing can construct would have been a match arm that never runs.
- **The external oracle now runs both ways.** `step_export_fixtures` also
  re-exports the two committed OpenCascade fixtures, so
  `validate_ngk_export.py` checks the volume and area of what NGK's
  *importer* produced — the one thing the Rust tests cannot say, since cell
  counts and a valid orientation are satisfied by a map that is the right
  shape's worth of wrong geometry. Reading is exposed through the Python
  binding too (`ngk.read_step`, `ngk.step_from_string`, returning a
  `StepImport` with `solids` and `skipped`), so the build123d loop closes
  from a REPL.

**What stage 4 settled.**

- **The seam abstraction D7 asked for is `SeamedFace`, and it is thin,
  because `UnwrappedFaceDomain` was already the hard half.** The domain
  cuts the periodic parameter space open, joins the wrapping loops across
  the cut and reports the corners the boundary turns through; what it does
  *not* carry is identity, and a STEP shell is unusable without it. So
  `topology/seam.rs` walks the face's loops in the same order the domain
  placed them and pairs the two off positionally — one placed curve per
  loop edge, as `place_loop` guarantees — leaving the gaps between them as
  the synthesized part. The count is checked rather than trusted: a
  mismatch means a pcurve would be attached to the wrong edge, which no
  validator downstream catches.
- **A cut always runs along a parameter line, which is what makes its curve
  analytic.** `convert/iso_curve.rs` is the table: a line along a cylinder's
  or a cone's axis, a latitude circle, a sphere's meridian, a torus's tube
  circle. Each is anchored so its own parameter *is* the surface parameter
  that varies, because an edge derives its span from its corners — a circle
  anchored a quarter turn away would still pass through both ends and take
  the wrong way round between them.
- **A full circle is one `EDGE_CURVE` naming one `VERTEX_POINT` twice, and
  the export had to learn that before any of the rest mattered.** A
  cylinder's rims are closed edges, so `ClosedEdge` was being raised on the
  planar *caps* long before the wall's seam came up. With one vertex at both
  ends, the direction a loop walks such an edge is no longer a question the
  corners can answer, so it is asked combinatorially —
  `edge_orientation_at_dart` — for every edge rather than only for closed
  ones.
- **`SEAM_CURVE` is not written, and §5 was wrong to call it required.**
  Writing one means writing the two `PCURVE`s it is defined by, which is the
  machinery stage 2 deliberately skipped. What actually identifies a seam to
  a reader is that one loop walks the same `EDGE_CURVE` twice, and that is
  true of a plain one. OpenCascade reads NGK's cylinder back as a valid
  three-face solid of the right volume.
- **D8's extraction did not happen, and the reason is worth recording.**
  `SectionTrace` carries a great deal that belongs to intersection — piece
  splitting at degeneracy crossings, `IntersectionOptions`, fidelity
  combination across two supports — and import needs none of it. What import
  needs is one direction of one of its steps, so `convert/pcurve.rs` states
  that directly: invert the curve at samples, resolve the collapsed rows,
  unwrap **both** periodic axes, try a parameter line, fall back to a fit
  measured in model units. The two-axis unwrap is the generalization D8
  identified; it is now written where the first caller needed it rather than
  in the intersection subtree, where both arms are still dead code.
  `plan/curved_support_pcurve_rebuild.md` is unaffected either way.
- **A plane is a projection, not a lift, and the projection keeps the
  parameterization.** Plane parameters are Cartesian coordinates in the
  plane, so the map is an isometry: a circle stays a circle at the same
  angle, a B-spline keeps its knots, and the interval the corners bound
  means the same thing on both sides. `builders::profiles::curve_pcurve`
  could not be reused for this — it ignores the endpoints for anything but a
  line and returns the whole support, which is right for a builder whose
  curve is already the section and wrong for an arc read out of a file.
- **`EDGE_CURVE.same_sense` *is* read, and stage 3's note that it is not no
  longer holds.** The claim was that the corners re-derive everything the
  flag says. On a line they do. On a circle they cannot: two complementary
  arcs share both corners, and NGK's derived span is always the one running
  forward along the support — so which arc an edge *is* depends on which
  corner its reference dart leaves from, and only `same_sense` says which
  that should be. The composition is one line —
  `along_curve = orientation_agrees == same_sense` — and it feeds both the
  reference dart and the α2 pairing.
- **The winding must be read from *placed* parameter curves, and D5's
  checksum is what found that.** Rebuilding a pcurve inverts its curve onto
  the support, and inversion answers within one period — so the two sides of
  a seam come back at the same parameter rather than a period apart, and the
  rectangle the file described shoelaces as a triangle. The cylinder
  survived that by luck; the cone did not, and reported one
  `SenseMismatch` against an otherwise valid solid. Shifting each pcurve by
  whole periods so it continues from the one before — the same rule
  `place_after` uses — is the fix. This is precisely the early warning D5
  was for: it named the face, not the solid, and not a failed validation
  three stages later.
- **No builder in the kernel makes a cone,** so the only way one reaches NGK
  is through a file: `revolve` refuses any profile edge touching the axis
  (`ApexRevolveUnsupported`). `tests/fixtures/step/frustum.step` is
  therefore both the read fixture and, re-exported, the only cone the
  external oracle can check — which it does, at the exact frustum volume.
- **Two shapes are known not to survive, both of them stage 5's business.**
  An OpenCascade sphere bounds its face with a `VERTEX_LOOP`, which D9
  refuses by name; and an OpenCascade torus imports, sews and heals only one
  of its two seams, arriving as a one-edge face with an inverted normal.
  That second one is D11's predicted failure, observed rather than
  anticipated — the fixture prerequisite stays, and it now has a concrete
  symptom to aim at. *Both are fixed in stage 5; the refusal turned out to be
  the wrong reading of `VERTEX_LOOP` rather than a missing capability.*
- **A revolved annulus is an open shell in NGK, and that is not an exchange
  bug.** Revolving a rectangle offset from the axis yields a tube whose caps
  carry two distinct radial edges each, α2-free — `validate_all_solid_manifolds`
  says so on the map itself, before any file is written. The exporter writes
  what the map states and the importer reports `OpenShell`, which is the
  honest behaviour on both sides.

**What stage 5 settled.**

- **D11's prerequisite found two healing bugs, not one, and both were
  latent long before the exporter needed them.** `seamed_torus` is in
  `tests/fixtures/seamed.rs` and the seams-only pass reduces it to a
  boundaryless face, but only after `MergePlan::loops` stopped gating the
  *unbounded* outcome on a single affected loop — a torus's second cut is
  walked by both of the loops the first removal left — and after
  `rerooted_shell` stopped reading a re-rooted shell's sense off the face's
  first loop seed, which for a ring is an orbit the shell's dart need not be
  in. The second showed up as a coin-flip: the seeds come out of a `HashSet`,
  so the test failed on about half its runs.
- **A boundaryless face's sense has exactly one home, and it is the shell
  root.** A face states which way it points by the winding of its boundary,
  and a face with no boundary states nothing — so `ShellRoot::Face { sense }`
  is not a convenience for a face with no dart, it is the only place the
  answer lives. Three places were reading past it: `Face::normal_at` returned
  the support normal whatever the view said, `validate_oriented_shell_volume`
  re-read each face by key and dropped the shell's own reading of it, and the
  removal computed the sense *after* dropping the edge attribute the winding
  is read through, so it always came back `Same`. The sense is now taken in
  `Preflight`, before anything is touched.
- **`VERTEX_LOOP` is not an entity NGK cannot represent; it is how STEP
  spells the face NGK already had.** §5 listed it with `POLY_LOOP` as refused
  by name under D9, and that was wrong: OpenCascade writes a whole sphere as
  one `ADVANCED_FACE` with no cut at all, bounded by a `VERTEX_LOOP` naming a
  point *on* the face. Dropping such a bound and reading the face as
  boundaryless is the faithful mapping, and it is the only route by which a
  sphere arrives from the most common kernel. D9's rule stands for what is
  genuinely unrepresentable; this was not.
- **`same_sense` is read rather than checked in exactly one case, and D5
  needs that exception written down.** A cut walked out and straight back
  along one parameter line encloses no area: a sphere's two meridian walks
  invert to the same longitude, because inversion answers within one period
  and a pole names every longitude at once. The boundary then shoelaces to
  zero and states no winding at all. `unfold_cut_walk` puts the return walk
  one period along the transverse axis, and which of the two directions that
  period runs in is the one thing left for the file to say — a rectangle and
  its mirror describe the same sphere and opposite normals.
- **The cut's corners are shared by parameter position, and only within one
  face.** A torus's domain rectangle has four corners that are one point of
  the surface; writing four `VERTEX_POINT`s there leaves a file no reader can
  sew. They are matched modulo the support's periods, which is precisely when
  two corners are one point — and it is a statement about the cut, not about
  the model, so real vertices are still shared by key and never by position.
  For the same reason a cut's two stretches are matched by *direction* along
  their parameter line rather than by which corner they leave: on a torus both
  leave the same one.
- **A void needs no flip in either direction.** Every shell bounds the
  material from outside it — an outer shell faces away from the solid, a
  cavity faces into itself — so NGK's inner shells are already oriented the
  way `BREP_WITH_VOIDS` wants and the `ORIENTED_CLOSED_SHELL` is written
  `.T.`. A file that says `.F.` is turned on the way in, by reversing each
  planned face's walk, rather than by carrying a flag no later reader would
  consult. OpenCascade reads a hollow sphere back at exactly the difference of
  the two volumes.
- **Nothing in the tree builds a hollow solid**, so `tests/fixtures/hollow.rs`
  registers the cavity's shell by hand, the same way `seamed.rs` does for
  seams. It is two concentric spheres: both shells are one boundaryless face,
  so the fixture states the shell orientation and nothing else.

---

## 11. Risks, ranked

1. **The revolution transposition and the cone scale** (D4). Everything downstream
   inverts together — pcurve direction, boundary winding, `same_sense`, the face
   normal, and the symptom (`SolidFaceNormalNotOutward`) appears far from the cause.
   Mitigated by routing every parameter through `UvMap`, never reasoning about signs
   by hand, and pinning it with a no-GMap test against the ISO formula. *`Surface::Torus`
   removed the most dangerous instance — a headline primitive — by making it an
   identity map.*
   *Stage 4 discharged the cone half: a `CONICAL_SURFACE` OpenCascade wrote imports
   with the right taper and comes back out at the exact frustum volume. The
   transposing case is untouched, because `SURFACE_OF_REVOLUTION` is stage 6.*
2. **The torus boundaryless round trip** (D7/D11). *Discharged in stage 5.* It was a
   healing bug, exactly as D11 said it would have to be if the fixture failed: the
   1-removal refused to leave a face unbounded when more than one of its loops was
   walked on the cut, which is every torus. The fixture is
   `tests/fixtures/seamed.rs::seamed_torus`, and OpenCascade reads NGK's torus back at
   the right volume in both directions.
3. **Vendor spellings of a degenerate bound.** *Discharged in stage 5, by changing the
   reading rather than by adding a capability.* A `VERTEX_LOOP` is not a loop with no
   edges: it names a point on a face that has no boundary, which is a face NGK already
   stores. It is dropped and the face read as boundaryless, so an OpenCascade sphere
   imports. What has no answer yet is a *vendor* degenerate edge — a zero-length
   `EDGE_CURVE` standing in for a pole — which is parked with the rest of the
   leniency work (§12).
4. **Silent NURBS parameterization corruption** (D2).
5. **Tolerance mismatch** (D10). Expect foreign files whose vertices do not coincide by
   NGK's `1e-9`, and expect stitching (§6.3) to be where that surfaces.
6. **Cone pcurves lose analytic identity** (D4). Correct but lossy; rarely fires.
7. **Vendor-file deviations.** Three of the five are discharged: a missing
   `FACE_OUTER_BOUND` falls back to the bound enclosing the most area and reports
   `GuessedOuterBound` (stage 3); plane angles in degrees and lengths in inches are
   normalized by the unit block (D10); a `$` ref_direction takes any perpendicular.
   Unknown entities and malformed string escapes are not, and are parked (§12). The
   mitigation stands where it is implemented: leniency by default, report rather than
   fail, strictness opt-in.
8. **Non-manifold and open-shell input.** NGK is a 3-GMap; STEP files contain surface
   models and non-manifold solids that cannot be represented. Rejecting by name is the
   design, not a failure.
9. **Test-corpus licensing and repo size.**

---

## 12. Parked — not started without an explicit go-ahead

Everything below was in scope when this plan was written and is not any more. It is
recorded rather than deleted because each item is a *known* gap with a known shape:
a future reader deserves to find out from here that NGK drops assembly structure on
purpose, rather than discover it from a file that came back flattened.

**None of this is to be picked up on a passing judgement that it looks small.** The
plan is finished at stage 6; resuming any of it is Romain's call, said explicitly.

### Assemblies and the document type (was D13, was stage 7)

The largest one, and the one with a decided home. A STEP file holds several products
with names, nesting and placements; NGK reads the solids out of it and drops the rest,
and cannot write an assembly at all. The answer is `Model` — `src/model.rs` is a
17-line embryo with no `insert`, and `docs/model_api.md` is its unimplemented target
design — not a second hierarchy inside `exchange`. So this waits on `Model` landing
first, and then becomes a thin lowering rather than a design problem.

Concretely, when it resumes: read the product structure that `read_solids` currently
sweeps past (`SHAPE_DEFINITION_REPRESENTATION` down through `PRODUCT_DEFINITION` and
`REPRESENTATION_RELATIONSHIP`), carry `AXIS2_PLACEMENT_3D` transforms, and have the
exporter consume a `Model` rather than one solid at a time. `schema/product.rs`
already writes the boilerplate; it is the *reading* half and the transform plumbing
that do not exist.

### Vendor leniency (was stage 7)

Three of risk #7's five items landed on the way through stages 3–5. What remains:

- **Unknown entities.** L1 parses any record, so the gap is L3: an unrecognized
  keyword in a position the walk needs is an error today rather than a reported skip.
- **Malformed string escapes.** `decode_text` and `encode_text` are exact inverses
  (stage 1); a vendor escape neither of them knows should pass through rather than
  fail the file.
- **A vendor degenerate edge** — a zero-length `EDGE_CURVE` standing in for a pole,
  which is the other spelling of what `VERTEX_LOOP` says and which stage 5 now has
  the boundaryless reading to absorb.

### AP242 (was stage 7)

Only `FILE_SCHEMA` and the application-context string differ from the AP214 NGK
writes, both confined to `schema/product.rs`. Nothing reads or asserts either, so
"AP242 support" today means "AP214 that AP242 readers accept".

### Untested rather than unbuilt

Two things exist but have never been exercised, which is worth knowing before anyone
trusts them:

- **Tolerance mismatch** (risk #5, D10). The document uncertainty is threaded into the
  import options and `HealingOptions`, but no fixture has vertices that fail NGK's
  `1e-9` while passing the file's `1e-6` — so the stitching path meant to absorb that
  has never actually run.
- **No foreign-kernel corpus** (risk #9). Every fixture is OpenCascade by way of
  build123d. A file from another kernel would be the first real test of the leniency
  above, which is part of why the leniency is parked rather than guessed at.

### Deliberately not done, and not a gap

`PCURVE` and `DEFINITIONAL_REPRESENTATION` are neither read nor written. D8 rebuilds
parameter curves from the 3D curve instead, on the grounds that a file need not carry
them at all. That is a decision with a stated reason, not an item waiting here.
