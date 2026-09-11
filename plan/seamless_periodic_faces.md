# Seamless periodic faces

Status: **In progress** — milestones 0 (`src/topology/attributes.rs`),
1 (`src/topology/face.rs`), 2 (`src/topology/unwrapped_face_domain.rs`), 3, 4 and 5
are implemented. Milestone 6 is nearly complete: a boundaryless face and its
key-rooted shell are in (§11.5), and so is a direction closed by a *degeneracy* —
a spherical cap (§11.6). What is left is the Boolean path onto a boundaryless
face. Milestone 7 is designed but not built.

Nothing in the tree builds a seam any more: a swept or revolved wall comes out a
ring, a sphere comes out one face with no boundary at all, and an intersection
crossing a closed direction stays one section. A seam now arrives only from
outside, and healing takes it apart. §§11.1–11.3 and §11.5 record what landed;
§11.4 and §11.6 record what was deliberately left open.

**Sections 1 and 8 describe the tree as it was before this work and are kept as
the rationale, not as a report of the current state.** Where they say "today",
read "before milestone 3".

This document says what "no seam edge" means for NGK, what it costs, and in what
order it can be built.

## 1. What a seam is today

A seam is a topological edge that exists only because the kernel insists a face
boundary be a closed loop in the face's *planar* parameter domain. A cylinder's
lateral surface is closed in `u`; to bound it with a loop, the builder cuts it
open along `u = 0` and walks the cut twice.

Measured on today's primitives (`ngk::modeling::solids`):

| Primitive | darts | vertices | edges | faces |
|---|---|---|---|---|
| `cylinder(1, 2)` | 12 | 2 | 3 | 3 |
| `sphere(1)` | 4 | 2 | 1 | 1 |

- **Cylinder.** `builders/solids.rs::sew_extruded_loop` builds one quad lateral
  face per bottom edge. The base is a single closed circular edge, so there is
  exactly one lateral face, and the loop-closing `sew(Dim::Two,
  last.end_vertical, first.start_vertical)` sews that face's two vertical dart
  pairs *to each other*. That self-sew is the seam edge. The third edge and the
  second vertex in the table above exist for no other reason.
- **Sphere.** `builders/solids.rs::add_sphere` revolves a meridian arc a full
  turn and `builders/revolve.rs::sew_full_revolved_seam` sews the swept copy of
  the arc back onto the arc itself. The result is one face whose single loop is
  the meridian traversed twice, plus two pole vertices — four darts for a shape
  with no natural boundary at all.
- **Torus / full surface of revolution.** Same mechanism, closed in both
  directions, so two seams.

The representation then propagates:

- `FaceAttr::pcurves` is keyed by *boundary dart*, so a seam stores two distinct
  UV curves for one edge — `builders/faces.rs:2312` says so outright, and
  `incident_face_pcurves` collects `(face, dart)` occurrences rather than faces
  because of it.
- `builders/faces.rs:640-1000` carries a whole parallel imprint path
  (`periodic_seam_edge`, `split_periodic_face_by_imprints`,
  `merge_faces_across_edge`, `prepare_periodic_boundary_merge`) whose only job is
  to cut a periodic face open at its seam, imprint it, and sew it back.
- `geometry/dim3/intersections/analytic/surface_surface.rs::sections_for` splits
  every intersection section at `SectionTrace::seam_crossings()` — one physical
  circle becomes two edges — with the stated reason that "a face trim classifies
  in `[0, 2pi]` and a loop that leaves it is dropped".
- `healing` refuses to remove a seam (`SkipReason::PeriodicSurface`,
  `passes/edges.rs:107`) and refuses the vertex removal that would leave a
  vertexless edge (`SkipReason::WouldCloseEdge`).
- Boolean classification carries `FaceTrimDomain::periods` and its `images`
  fan-out purely so a loop written at one end of the period is still recognized
  at the other (`builders/boolean/trim.rs:41`).

So the seam is not a local wart: it is a premise five subsystems are written
against.

## 2. The one idea

> **A seam is a property of an unwrapped domain, not of a shape.**

Cutting a closed parameter direction open is a legitimate and sometimes
necessary *algorithmic* step — to run a planar winding test, to emit a NURBS
patch, to write STEP. It is not a fact about the solid. The proposal is to stop
storing the cut and to synthesize one on demand, in one place, for the
algorithms that genuinely need a simply-connected domain.

Everything below follows from that sentence.

## 3. What the representation becomes

### 3.1 Darts are lost only where there is nothing to be incident to

Worth stating up front, because it is the most common misreading of this plan:
**edges never lose their darts.** An edge cell is the orbit `<a0, a2, a3>`
(`GMap::orbit_indices` is "every index except mine"). For a cylinder's bottom
circle, the cap face contributes two darts `c1, c2` — alpha0- and alpha1-linked,
the closed one-edge profile — the lateral face contributes `l1, l2`, and alpha2
sews `c1<->l1`, `c2<->l2`. That four-dart orbit is the edge.

Removing the seam deletes only the *seam edge's* darts: the lateral quad's two
vertical dart pairs. 12 darts becomes 8 — 4 in the lateral face (two circle
occurrences), 2 in each cap. The circle edge's orbit is untouched, so
`Edge::faces()`, which is `incident_cells(dart, Dim::One, Dim::Two)`, still
returns the cylindrical face and the cap. **That API does not change.**

The same walk gives a clean characterization of the new edge kind. On a seamless
cylinder the vertex orbit `<a1, a2, a3>` from `c1` is `c2` (alpha1), `l1`
(alpha2), `l2` (alpha1) — the *same four darts* as the edge orbit. So:

> An edge is **closed** exactly when its vertex orbit coincides with its edge
> orbit — when its two ends are the same vertex.

That is derivable from the combinatorics. Nothing about it needs storing, and it
is false today only because the seam edge also meets that vertex and makes the
vertex orbit strictly larger.

Darts disappear in exactly one situation: a face with **no boundary at all**
(§3.4), which has no incidences for any structure to record.

### 3.2 Level 1 — ring faces

A cylinder's lateral face keeps its two circular edges and loses the seam:

| | darts | vertices | edges | faces |
|---|---|---|---|---|
| cylinder today | 12 | 2 | 3 | 3 |
| cylinder, level 1 | 8 | 2 | 2 | 3 |

The lateral face now has **two boundary loops, neither an outer loop nor a
hole**. In UV it occupies `u` over the whole period and `v` over `[0, height]`;
each loop is a *horizontal line exactly one period long*, not a closed polygon.

The invariants that die here:

- **A loop's pcurve chain is no longer a closed curve in UV.** It closes on the
  quotient. `TrimmedCurve2::is_closed` (`geometry/dim2/trimmed.rs:156`) answers
  `false` for it, and is right to.
- **Signed area of a loop is meaningless.** `Face::normal_at` decides the face
  sense from `outer_loop_signed_area()`; `tessellate/face.rs` decides triangle
  winding the same way. Both need another source of truth.
- **Winding-number point classification is meaningless in the closed
  direction.** `FaceTrimDomain::polygons` is exactly that test.

The replacement rule is simple and total: **a closed parameter direction is
covered in full unless a loop bounds it; a loop that runs one whole period bounds
the direction it is transverse to, and its travel direction says which side holds
material.** That is the same information the loop already carried, read
per-direction instead of by winding.

### 3.3 A direction can also be closed by a degeneracy

Cut a seamless sphere with a plane and you get two caps. A cap's boundary is
*one* latitude circle — period-spanning in `u`, a horizontal line in UV. It is
bounded in `v` on one side by that loop, and on the other side **by the pole,
which is a parametric degeneracy, not a loop**.

So a parameter direction is closed in one of three ways: bounded by a loop,
closed by periodicity, or closed by a degeneracy. OCCT spells the third with a
zero-length "degenerated edge" at the pole; the seamless answer is that the
`v`-range simply runs to the pole and the surface says so.

This is the most common periodic face in real models — every sphere-plane cut
produces one — so it belongs in the design from the start, not as an afterthought.

### 3.4 Level 3 — boundaryless faces

A sphere with no seam and no poles-as-vertices, or a full torus, has **no
boundary at all**. Its face has zero loops, therefore zero darts:

| | darts | vertices | edges | faces |
|---|---|---|---|---|
| sphere today | 4 | 2 | 1 | 1 |
| sphere, level 3 | 0 | 0 | 0 | 1 |

This is where the design stops being an extension and becomes a change of
foundation (§5).

The poles do not disappear as geometry — they stay parametric singularities of
`Sphere`, where `normal_at` is degenerate and `param_at` ill-conditioned. They
stop being *topology*.

## 4. The two enums

### 4.1 `LoopKind` — per loop, not per face

The boundary kind belongs on **each loop**, not on the face's boundary as a
whole. A whole-boundary enum — `{ Disk, Ring, Closed }` — multiplies: a cylinder
side with a hole punched in it is a ring *and* carries an inner loop; a sphere
with a disk removed is closed *and* carries an inner loop. Every variant ends up
repeating an `inner: Vec<Dart>` field, which is the usual sign the cut is in the
wrong place.

```rust
enum LoopKind {
    Outer,                     // closed in UV, bounds the region from outside
    Inner,                     // closed in UV, a hole
    Wrapping { axis: Axis2 },  // spans exactly one period; bounds the other axis
}
```

Per loop, every configuration falls out of one `Vec` with no special cases:

| Face | Loops |
|---|---|
| block face | `[Outer]` |
| block face with a hole | `[Outer, Inner]` |
| cylinder side | `[Wrapping{U}, Wrapping{U}]` |
| cylinder side with a hole | `[Wrapping{U}, Wrapping{U}, Inner]` |
| spherical cap | `[Wrapping{U}]` — the pole side closed by degeneracy (§3.3) |
| sphere minus a disk | `[Inner]` — no outer loop, and no special case needed |
| sphere, torus | `[]` |

The face-level question — is this a disk, a ring, a closed surface? — becomes a
*derived query* over the vec rather than stored state, so it cannot drift out of
sync with the loops themselves.

### 4.2 `Edge` — an enum, and no parallel `Option` API

```rust
enum Edge<'a, P: Payload> {
    Bounded(BoundedEdge<'a, P>),
    Closed(ClosedEdge<'a, P>),
}
```

`Edge` **is** the enum — not a struct with a `kind()` beside it, which would be
the same `Option` API wearing a hat. `start()` and `end()` come off it entirely
and live on `BoundedEdge`, where they are *total*: holding that type is the proof
the two ends exist and differ. Leaving `Option`-returning endpoints on `Edge` is
the failure mode: callers take the `is_none()` path and the type safety buys
nothing. The only way to an endpoint is through the narrowing.

What stays **total**, because it never needed endpoints in the first place:
`trimmed_curve()`, `parameter_interval()`, `length()`. `Bounded` derives its span
from its vertices; `Closed` takes the whole period; the method resolves that
internally. Call sites that reach for `start().point()` only to rebuild a span —
`healing/passes/edges.rs:345`, `builders/faces.rs:1554` — should move onto
`trimmed_curve()` because that is what they meant.

This keeps the `CLAUDE.md` contract intact. *"The span is derived only on an
edge; `EdgeAttr` stores no interval"* still holds, with one added rule:

> **A closed edge spans its support's whole period.** A closed edge *is* its
> support.

The span is still derived, just from a different fact. Storing an `Interval` on
`EdgeAttr` for this one case would be exactly the loose pair the codebase avoids.

The two enums are the same distinction one dimension apart, and both are
*computed from the combinatorics* (§3.1), never stored.

## 5. Identity when there is no dart

### 5.1 The problem

A boundaryless face has no incidences. Every incidence structure — GMap,
half-edge, winged-edge, radial-edge — encodes a cell by its relationships, and
this cell has none. There is no clever traversal; the membership must be
recorded.

One fact makes this cheap: **a boundaryless face is always alone in its sheet.**
It has no boundary edges, so nothing can be alpha2-sewn to it. There is no mixed
case.

### 5.2 `ShellRoot`, with the dart preferred

```rust
enum ShellRoot {
    Dart(Dart),      // derive everything, exactly as today
    Face(FaceKey),   // this sheet is exactly this one boundaryless face
}
```

Used by both `SheetAttr` and `SolidAttr`'s shells, under one invariant:

> **A root is a dart whenever any dart exists in the cell. It is a key only when
> there is no dart to point at.**

This deliberately does *not* promote `SolidAttr` to store `SheetKey`s: that would
cost the lazy dart-derived resolution in the normal case for no benefit. Solids
stay dart-derived and degenerate to a key only when nothing else exists.

### 5.3 Why the dangling-key worry does not materialize

The scenario to beat: a sphere shell, split by a plane, the new loop extruded,
the original spherical face deleted — the sheet must survive without referencing
a dead `FaceKey`.

1. **Sphere shell.** `SheetAttr.root = Face(F_s)`. The only configuration in
   which the key variant is used at all.
2. **Split with a plane.** `F_s` becomes two faces, both with darts. At commit
   the invariant fires: darts are available, so the sheet re-roots to a dart. The
   key reference is gone before anything can dangle. The variant *changes kind*
   here — that is the case to write the first test against.
3. **Extrude and delete the original face.** The sheet is dart-rooted by now, so
   this is the case already handled today, by the same re-pointing
   `removal.rs:967` performs.

The key variant is self-eliminating: the moment a sheet gains topology it stops
being key-rooted. The only edit that can invalidate a key root is deleting the
boundaryless face itself — and a sheet with zero faces is not a sheet, so the
commit deletes it or rejects.

Enforcement goes where the referential checks already live:
`TopologyEditError::MissingProfileRegistration` (`edit.rs:787`) and
`MissingSheetRegistration` (`edit.rs:797`) already reject a commit whose face or
solid references an unregistered component. Add: every stored root must resolve,
and must be a dart if one is available. Lineage (`add_*_split_from`,
`merge_*_into`) already answers "what succeeded this cell", so the re-root follows
existing merge chains rather than needing new tracking.

### 5.4 Stored darts are not maintenance-free today either

The lazy-orbit property — an attribute keyed by a representative dart survives
edits that spare that dart — is real but narrower than it looks. When the dart
does not survive, the fix-up is manual and already written:
`removal.rs:959-968` rewrites `outer_shell` and every `inner_shells` dart,
`chamfer.rs:1103` does the same, `gmap.rs:723` remaps face loop darts on dart
compaction.

On stability the comparison runs the other way: a `FaceKey` survives dart
renumbering, a dart does not. slotmap's generational keys mean a stale key
resolves to `None` rather than silently aliasing a reused slot, so dangling is
detectable rather than corrupting.

**Scoping.** Nothing here touches profiles, edges, vertices or dart-backed faces.
"Remove an edge and the profile identity survives via the orbit" is untouched —
it is a property of cells that *have* orbits, and every such cell keeps deriving
from its orbit. The change is confined to two things: a face with no darts, and
the roots of sheets and solids.

### 5.5 `Face`'s dart is only an orientation

Every use of `Face`'s dart field (`face.rs` lines 36, 73, 81, 98, 113, 266) feeds
`face_orientation_at_dart` or is alpha0-flipped by `reversed()`. It does no
locating. Replacing it with `sense: Orientation`, computed once in
`Face::from_dart`, is a simplification of today's code that also removes the last
obstacle to a face view over a dartless face.

## 6. Is the GMap still the right substrate?

Yes — but it should stop being described as the single source of truth.

**What a switch would buy: nothing.** Half-edge and radial-edge have the
identical boundaryless problem (a sphere has no half-edges either) and are
strictly weaker on orientation bookkeeping and non-manifold configurations.
OCCT's `TopoDS` "solves" it by not being a topological structure at all — a
container that checks nothing, and keeps seams anyway. The only representation
handling `d = 0` natively is a chain complex / incidence-matrix CW-complex, paid
for by losing cheap ordered local traversal, which is most of why darts were
chosen.

**The departure is older and deeper than seams.** In both a GMap and a
CW-complex a 2-cell is a disk. A trimmed surface with holes is not a disk, but it
is unarguably one face to the user. **A CAD face is not a cell.** Every B-Rep
kernel makes this same departure; NGK did not bend the GMap for holes out of
expedience.

**So name the two layers**, because the split already exists and is merely
implicit — `logical_sheet_darts` (`gmap.rs:506`) exists precisely because
alpha-connectivity does not span a multi-loop face, and `build_derived_indexes`
already registers one `FaceKey` across several orbits:

- the **GMap is the connectivity substrate** — darts, alpha involutions, sewing,
  orientation propagation, local traversal. It knows about disks.
- **keys and attributes are the B-Rep cell complex** — regions, which may be
  non-disks, and which in exactly two cases (closed face, closed shell) have no
  connectivity to derive from.

Cells with incidences stay dart-backed; cells without are not. This is the same
refactor whether or not seams are dropped — dropping them forces the issue rather
than letting it keep leaking.

## 7. Invariant changes, by layer

| Layer | Today | After |
|---|---|---|
| `FaceAttr` | `outer_loop: Dart` + `inner_loops: Vec<Dart>` | `FaceBoundary`: a `Vec` of `(dart, LoopKind)`; may be empty |
| Face identity | `cell_key::<Cell2>(outer_loop)` | key-first; dart lookup is a convenience that can fail |
| `Face` view | carries a dart | carries a `sense: Orientation` (§5.5) |
| Face trimming | closed UV polygon(s) | per-direction: whole period unless a loop or a degeneracy bounds it |
| Loop | closed profile, closed in UV | closed profile; in UV closed *or* period-spanning |
| Edge endpoints | `start()`/`end()` on `Edge`, infallible | `Edge::{Bounded, Closed}`; `start()`/`end()` only on `BoundedEdge` |
| Edge span | from bounding vertices | from bounding vertices, or the whole period when closed |
| `SheetAttr` / `SolidAttr` | rooted at a `Dart` | `ShellRoot`, dart preferred |
| Shell validity | no alpha2-free dart | that, plus geometric closedness for a key-rooted shell |
| Seam | a stored `EdgeKey` | an unwrapped-domain cut computed on demand, stored nowhere |

## 8. Work breakdown

### 8.1 `topology`

- `attributes.rs` — `LoopKind`, `BoundaryLoop`, `FaceBoundary` replacing
  `outer_loop`/`inner_loops`; `ShellRoot` on `SheetAttr` and `SolidAttr`.
- `face.rs` — `Face::new`, `outer_loop_dart`, `outer_loop`, `inner_loops`,
  `loops`, `reversed`, `normal_at` (signed area), `signed_volume_contribution`.
  `pcurve()` gets *simpler*: its alpha0/alpha2 candidate probing exists largely to
  find the right seam occurrence.
- `edge.rs` — `Edge` as an enum; `parameter_interval` and `length` stay total.
- `gmap.rs` — `build_derived_indexes` (face/sheet/solid registration),
  `logical_sheet_darts`, `face_orientation_at_dart`, `cell_orientation_from_seed`
  (no seed exists for a boundaryless face), dart remapping at `gmap.rs:723` and
  `gmap.rs:1320`, `sheet_key`/`solid_key` from a dart.
- `edit.rs` — root re-rooting and referential checks at commit (§5.3); identity
  reconciliation over face representatives (`edit.rs:785`, `edit.rs:1224`), where
  a boundaryless face has no representative dart.
  `tests/builders/face_lineage.rs` is the pressure test.
- `validation.rs` — `validate_shell` builds a `Sheet` from a dart;
  `validate_oriented_shell_volume` walks `face.loops()`, needs a point from every
  loop, and checks outwardness via `directed.contains(alpha0(alpha2(dart)))`,
  vacuously true with no darts. A key-rooted shell needs its closedness and
  outwardness from the surface and the stored sense.
- `sheet.rs` / `solid.rs` — `Sheet::faces()` walks darts; `Solid::faces()`,
  `edges()`, `vertices()` likewise.

### 8.2 `geometry`

Mostly ready — `TrimmedCurve2` already carries native parameters, so a pcurve
running `u0 -> u0 + 2pi` is directly expressible, and `native_parameter_at`
(`geometry/dim2/trimmed.rs:167`) already shifts a query onto the span's branch.
Two changes:

- `analytic/surface_surface.rs::sections_for` — stop splitting sections at
  `seam_crossings()`; keep the split at `degeneracy_crossings()`. The comment
  justifying the seam split ("a loop that leaves `[0, 2pi]` is dropped") is
  exactly the premise being removed.
- ~~`surface_surface/tracer.rs:393-430` — the "move a state that reached a closed
  parameter boundary to the far seam edge" step can instead let `u` run past the
  period, with the branch closing when it returns to the seed in the quotient.~~
  **Wrong, and unnecessary — see §11.2.** `NurbsSurface::point_at` clamps to its
  domain, so a parameter running past the period would silently evaluate the wrong
  point; and `fitting.rs::unwrap_parameter` already unwraps the finished trace, so
  what reaches the fitter is continuous regardless.

### 8.3 `builders`

- `solids.rs::sew_extruded_loop` — when the swept loop is a single closed edge and
  the sweep direction is a closed direction of the resulting surface, build the
  lateral face with two `Wrapping` loops and no vertical edges instead of
  self-sewing.
- `revolve.rs::add_full_revolved_face`, `sew_full_revolved_seam`,
  `close_revolved_apex` — a full turn should not create the swept copy that is
  then sewn back onto its source.
- `solids.rs::add_sphere` — a boundaryless face and a key-rooted shell.
- `faces.rs` — delete `periodic_seam_edge`, `split_periodic_face_by_imprints`,
  `merge_faces_across_edge`, `prepare_periodic_boundary_merge` and their error
  variants (`FaceCreationError::{PeriodicMerge, PeriodicEdit}`). Roughly 350
  lines go. In exchange `split_face_by_imprints` gains:
  - a period-spanning imprint on a ring face splits it into two ring faces,
    creating no vertex;
  - a period-spanning imprint on a boundaryless face turns it into a ring face;
  - a UV-closed imprint on a boundaryless face splits off a disk and leaves
    `[Inner]` with no outer loop.
- `removal.rs` — 1-removal of a seam edge becomes *supported*, producing a ring
  or boundaryless face; 0-removal of a closed edge's vertex produces a vertexless
  edge instead of `WouldCloseEdge`.

### 8.4 `healing`

`SkipReason::PeriodicSurface` and `SkipReason::WouldCloseEdge` stop being guards
and become supported operations. Healing is then the canonicalizer: seam edge
removed, then the orphaned vertex. Worth a named pass (`seam_removal`) rather
than folding into the generic edge pass, because it is what any future STEP
import will run.

### 8.5 `tessellate`

`tessellate_face` starts from `outer_loop` and bounds everything by the UV bbox
of its samples. It must start from `Surface::domain()` and clip only the
directions a loop actually bounds. Two requirements the seam used to satisfy for
free:

- the grid must **wrap**: the `u = 0` and `u = 2pi` columns are the same points
  and must share mesh indices, or every cylinder gets a visible crack;
- **pole handling**: a `v` extreme that collapses to a point needs a triangle
  fan, not degenerate quads.

### 8.6 `builders/boolean`

- `trim.rs::FaceTrimDomain` — `polygons` presumes closed planar polygons. Keep the
  winding test, but build the polygon **in the universal cover by synthesizing a
  unwrapped domain cut** — reconstruct a virtual seam at query time, at a `u` away from the
  loops. That confines the change to one constructor, keeps `images` intact, and
  is the concrete instance of §2. A boundaryless face answers `Inside`
  unconditionally.
- `classify.rs` — `domain_center` / `domain_bounds` read `polygons.first()`; they
  should read the surface domain when a direction is unbounded.
- `imprint.rs` / `assemble.rs` — result faces must be emittable with `Wrapping`
  loops and with no loops at all.
- `graph.rs` — an intersection circle that used to arrive as two seam-split edges
  now arrives as one closed edge, changing the network's degree structure.

### 8.7 `viz`, `scripts`, bindings

`viz/gmap.rs` dart overlays have fewer darts and, for a sphere, none — the debug
viewer needs a fallback anchor (sample the surface domain). `FaceAttr` is
`Serialize`/`Deserialize`, so stored fixtures are invalidated.

## 9. What this buys

- **~400 lines of periodic special-casing deleted** from `builders/faces.rs`
  alone, plus the seam splitting in the analytic intersection path.
- **Rotational symmetry becomes exact.** Today a cylinder rotated about its own
  axis is a *different* model — the seam lands elsewhere — and Boolean results
  depend on where it lands.
- **Intersection results stop being fragmented.** A plane cutting a cylinder
  yields one circular edge, not two arcs meeting at an arbitrary seam point.
- **Healing has less to undo**, and the per-dart double bookkeeping of seam
  pcurves disappears.
- Fewer cells: a cylinder drops from 12 darts to 8, a sphere from 4 to 0.

## 10. Costs and risks

1. **Closedness of a key-rooted shell becomes a geometry question.**
   `Closed::new(sheet)` checks "no free alpha2 dart", vacuously true for a
   boundaryless face. Proving a sphere shell closed means asking the surface
   whether it is closed in every direction — a real weakening of
   `validate_solid_manifold` for that one case.
2. **`validate_gmap` alone stops being sufficient.** The domain layer needs its
   own referential-integrity pass. Better explicit than implicit.
3. **Orientation loses its dart carrier** for boundaryless faces, so the triple in
   `docs/topology_orientation_refactor.md` becomes inconsistent across face kinds.
   Write that into the orientation doc rather than discovering it later.
4. **Every "the boundary is a closed UV polygon" assumption must be re-derived**,
   including ones not listed here. Grep targets: `signed_area`, `outer_loop`,
   `loops()`, `is_closed`.
5. **Tessellation regressions are visual, not assertive.** A wrap seam in the mesh
   will not fail a test. Add a watertightness check — every mesh edge used by
   exactly two triangles — as part of milestone 2.
6. **Serde round-tripping needs verifying.** `slotmap` has the `serde` feature, so
   keys should survive, but a stored key reference across a round trip is a new
   dependency on that behaviour. Test it on a sphere before relying on it.
7. **Interop debt.** STEP AP242 advanced B-Rep represents periodic faces *with*
   seams, so a future exporter must synthesize one. §2 makes that supported rather
   than a hack, but it is real work today's representation does not need.

## 11. Milestones

Each leaves the tree green.

| # | Milestone | Scope |
|---|---|---|
| **0** | **`FaceBoundary` / `LoopKind`** | Replace `outer_loop` + `inner_loops` with an ordered `Vec` of kinded loops. `Outer`/`Inner` only; today's semantics exactly. No behaviour change. **Done.** |
| **1** | **`Face` sense** | Replace the `Face` view's dart with `sense: Orientation` (§5.5). Pure simplification; unblocks milestone 5. **Done.** |
| **2** | **Unwrapped face domain synthesis** | One function: given a face and its loops, produce a cut position and a simply-connected unwrapped UV domain. Route `FaceTrimDomain::new` and `tessellate_face` through it, still on seamed input. Proves the abstraction before anything depends on it. **Done** — `topology::unwrapped_face_domain::{UnwrappedFaceDomain, UnwrappedFaceDomainLoop, UnwrappedFaceDomainCurve, Axis2}`. |
| 3 | Ring faces | `LoopKind::Wrapping`, cylinder and full-revolve builders, per-direction trimming, tessellation wrap + watertightness test, seam 1-removal in `removal.rs`, Boolean trim on rings. Sphere still seamed. **Done** — see §11.1. |
| 4 | Seamless intersections | Drop `seam_crossings()` splitting; period-spanning pcurves end to end through imprint and assembly. **Done** — see §11.2. |
| 5 | `Edge` as an enum + closed edges | The enum and the whole-period span rule land together. 0-removal of the vertex between two arcs that close on each other. **Done** — see §11.3. |
| 6 | Boundaryless faces | `ShellRoot` and the dart-preferred invariant, degeneracy-closed directions, validation by surface for key-rooted shells, sphere and torus builders, pole-aware tessellation. **Nearly done** — §11.5 and §11.6 record what landed; the Boolean path onto a boundaryless face is what is left. |
| 7 | Healing canonicalizer | `seam_removal` pass: a seamed model in, a seamless model out. |

Milestones 0–4 deliver most of the practical benefit — Booleans on cylinders stop
being seam-sensitive — without touching dart-rooted identity. That is now in, as
is 5. Milestone 6 is separable and can be judged on its own; it is the one that
changes the foundation, since a boundaryless face has no dart to root anything
at (§5). Milestone 7 is partly delivered already: healing removes an imported
seam (§11.1), and what remains is to separate that out as a named
`seam_removal` pass — worth doing when there is a STEP importer to run it.

### 11.1 Milestone 3, as it stands

Complete, with the whole suite green on it.

Built first:

- `LoopKind::Wrapping { axis: Axis2 }` (`Axis2` now lives in `geometry::dim2`),
  plus `FaceBoundary::{seed, wrapping, is_ring, retain_mapped}`. `darts()` now
  walks every loop rather than outer-plus-inner.
- `Face` no longer assumes an outer loop: `outer_loop()` returns `Option`,
  `dart()` reads the boundary's seed, `loops()` covers every kind, and
  `boundary()` exposes the stored kinds. `normal_at` reads its winding off the
  unwrapped domain rather than off the outer loop, which is what makes it answer for a
  ring at all.
- `UnwrappedFaceDomain` fuses wrapping loops into one closed unwrapped domain boundary across a
  synthesized cut. The cut is the same mechanism as a pole corner — a point the
  loop travels through carrying no pcurve — so `FaceTrimDomain`'s winding test
  and `tessellate_face` work on rings unchanged.
- `builders/solids.rs::sew_wrapping_lateral_face`: sweeping one closed edge over
  a whole period of the swept surface builds a face with two wrapping loops and
  no vertical edges. `cylinder(1, 2)` is now 8 darts, 2 edges, 3 faces — the
  numbers §3.2 predicts — with the wall carrying no outer loop.
- `builders/faces.rs::split_ring_face_by_wrapping_chains`: imprints that join
  into a chain spanning a whole period cut a ring into two rings, each keeping
  one original wrapping loop. Which copy of the chain bounds which half follows
  from travel direction alone.
- `reverse_face_winding` reverses every loop whatever its kind; `GMap::merge`
  carries kinded loops across; healing's `fuses_outer_loop` counts a wrapping
  loop as bounding from outside.

Built since, closing the milestone:

1. **A chord across a ring.** The tangent-contact case turned out not to be the
   loop-to-loop chord this section first guessed at: a cylinder tangent to two of
   a block's faces imprints a *staple* — up one tangent line, across the cap
   plane, back down the other — whose two ends land on the **same** wrapping
   loop. A chord between two corners of one loop is what the existing splitter
   already does, so the fix was to stop assuming that loop is the outer one:
   `split_one_face_by_imprints` now tries each bounding loop in turn, and
   `apply_outer_face_chord_split` became `apply_face_chord_split`, taking the
   chorded loop and its kind. The two halves are told apart by travel — one still
   runs a whole period and stays `Wrapping`, the other closes and becomes
   `Outer` — so a ring chorded once yields a ring and a disk, with no tolerance to
   tune since one half travels a period and the other travels nothing. Every
   other wrapping loop goes to the half that still wraps, which needs no sampling:
   a period-spanning loop cannot sit inside a half bounded in that axis.
2. **The double-pcurve test moved to a sphere**
   (`faces::splitting_a_sphere_seam_preserves_both_face_pcurves`). The invariant
   it guards — an edge a loop walks twice carries a different pcurve each time —
   is still live for any seamed periodic face, and a sphere is still one.
3. **Tessellation wrap** (§8.5). `grid_bounds` takes a wrapped axis's range from
   the unwrapped domain's cut and period rather than from sampled points, so the two ends of
   the period are the same parameter to the last bit; `tessellate_surface_patch`
   then drops the closing row of samples and indexes the quads that would reach it
   back to index 0. `modeling::a_cylinder_wall_tessellates_into_a_closed_tube` is
   the watertightness check §10.5 asks for — no two mesh vertices at one point,
   and the only single-use edges are the tube's two rims. It was confirmed to fail
   with the wrap disabled.
4. **Seam 1-removal.** `MergePlan::Ring` reads a two-way boundary split as a ring
   when both components wrap the same axis, keeping one profile identity and
   splitting off a second — the face genuinely has two boundaries where it had
   one. `SkipReason::PeriodicSurface` stopped being a blanket guard in healing and
   became `CellRemovalError::WouldLeaveWrappingLoop`, raised by the removal
   itself; see §11.2 for why the precise refusal is still needed.
5. **`revolve.rs`.** `add_full_revolved_ring_face` builds a band of a whole turn
   from the two circles its endpoints sweep, instead of building the swept copy of
   the source edge and sewing it back. `RevolvedFace::{bottom_edge, top_edge}` are
   `Option`, `None` on a ring band — which has no copy of the source edge on it at
   all — and `Some` only on a partial turn, where the caps still need them.
   `sew_full_revolved_seam` is **deleted**: `validate_revolvable_radii` already
   refuses any edge touching the axis, so every band of a whole turn is now a ring
   and the function had nothing left to sew. The band offers its two
   side darts asymmetrically (`start_side` is the far dart of its loop, `end_side`
   the seed) so that `alpha0(alpha2(seed))` reaches the neighbour's seed, which is
   what `validate_oriented_shell_volume` means by a consistently oriented shell;
   `sew_wrapping_lateral_face` does the same thing for the same reason.
   `add_full_revolved_open_edge_face` was already seamless in topology and now
   kinds its two loops `Wrapping` as well. A triangle revolved a full turn is
   3 vertices, 3 edges, 3 faces — `V - E + sum(chi) = 0`, a torus.

### 11.2 Milestone 4, as built

**The seam split is gone.** `sections_for` no longer cuts a section at
`seam_crossings()`; only `degeneracy_crossings()` still cuts one, because there
the support genuinely stops. A plane cutting a sphere now yields *one* circular
section whose pcurve runs continuously past longitude zero, rather than two arcs
meeting wherever the parameterization happened to be opened.
`seam_crossings`, `refined_crossing` and `within_one_period` went with it.

**The tracer needed nothing.** §8.2's second bullet proposed letting the tracer's
`u` run past the period instead of moving a state to the far seam edge. That
would be a mistake as written: `NurbsSurface::point_at` *clamps* to its domain, so
parameters running past it would silently evaluate the wrong point. It is also
unnecessary — `fitting.rs::unwrap_parameter` already walks the finished trace and
shifts each state by whole periods, so the branch handed to the fitter is
continuous whatever the stepping did. The fold is an internal stepping detail, and
the output was never the problem.

**The periodic imprint path is deleted** — `periodic_seam_edge`,
`split_periodic_face_by_imprints`, `merge_faces_across_edge`,
`merge_periodic_boundary_edge`, `periodic_boundary_curve`,
`rebuild_periodic_boundary_curves`, `unwrap_periodic_face_pcurves`,
`periodic_offset`, `periodic_u_period`, `is_constant_u_imprint`, the orphaned
`face_edge_dart_for_imprint`, and the three `Periodic*` error variants. 450 lines
of `builders/faces.rs`. It was still reachable before this milestone: its trigger
was *two constant-`u` imprints*, which is exactly what a seam-split section
produced. Dropping the split removed its only supply.

### 11.3 Milestone 5, as built

**`Edge`** (`src/topology/edge.rs`) *is* the enum — `Bounded(BoundedEdge)` or
`Closed(ClosedEdge)` — decided by the combinatorics, two ends that are the same
vertex or none, and never stored. `Edge::start`/`Edge::end` are gone from `Edge`;
they live on `BoundedEdge` and are **total** there, because a value of that type
is the proof its two ends exist and differ. `Closeable for Edge` is now a
`matches!` on the variant rather than a comparison of two points within a
tolerance, so two *distinct* vertices that happen to coincide are a degenerate
model rather than a circle.

A first attempt kept `Edge` a struct with a `kind()` method beside it. That is the
same `Option` API §4.2 warns about, one call deeper, and it was rejected.

**Narrowing, and how the ~90 bounded-by-construction call sites read.**
`Edge::bounded()` is the total form and `Edge::bounded_unchecked()` asserts — the
same convention as `FaceBoundary::outer_unchecked` beside `outer()`. The point of
returning a *type* rather than a tuple is that the check happens **once**: a
function narrows at its top, and everything below holds a `BoundedEdge` and never
asks again. The further step this enables, and the one to reach for when a
function gains a second endpoint access, is to take `BoundedEdge` as the parameter
type — pushing the decision out to the one caller that can actually do something
about a closed edge. Everything that can genuinely meet one — healing, chamfer,
revolve, the Boolean over imported geometry — matches on the two variants.

**How the ~11 shape-independent methods stay written once.** `key`, `dart`,
`darts`, `vertices`, `faces`, `sheets`, `curve`, `parameter_interval`,
`trimmed_curve`, `length` and the orientation flip do not care what bounds an
edge, and duplicating them across `Edge`, `BoundedEdge` and `ClosedEdge` would be
three copies of each. They live on a single `EdgeCore` that all three `Deref` to,
so they read identically on every one and narrowing costs nothing. `EdgeCore` is
never named at a call site. `Clone`/`Copy` are written out rather than derived:
`derive` would demand `P: Copy`, and a view never touches the payload.

`parameter_interval` stays total without endpoints: a bounded edge derives its
span from its vertices as before, and a closed one takes the support's whole
domain. Call sites that only wanted a span moved onto `trimmed_curve()` /
`parameter_interval()`, which is what they meant — that accounts for most of the
migration and removed a good deal of hand-rolled `interval_between` at the call
site.

Three call sites wanted neither: an alpha2 merge of two closed circles needs *the
vertex at this dart*, which for a closed edge is the same vertex twice.
That is a dart-level question, so those read `Vertex::from_dart` directly rather
than asking the edge about endpoints it does not have. `ClosedEdge::vertex()` is
the view-level form of the same question, for a caller that has already narrowed.

**0-removal of the vertex between two arcs that close on each other** is now
supported; `SkipReason::WouldCloseEdge` is deleted. Lifting that guard exposed a
second one underneath, in geometry rather than topology: `join_on_circle` refused
a full sweep, and with the two ends coincident there were only two distinct points
— which determine no circle. It now borrows a third from the samples (the one
farthest from both, where the three-point fit is best conditioned) and returns the
whole circle. A disc split across its rim heals back to the single closed edge it
started with.

### 11.4 What milestones 3–5 deliberately left alone

**A lone wrapping loop is not a ring.** Removing a seam can leave *one*
period-spanning component rather than two: that is a face bounded by a loop on one
side and by a parametric degeneracy on the other — a spherical cap (§3.3), which
every sphere-plane cut produces. Its loop is neither outer nor inner, and no
`LoopKind` can say what it is until milestone 6. Lifting healing's periodicity
guard without noticing this silently relabelled such caps `Outer`, which put a
winding test on a loop with no inside and broke point membership on
`boolean_union_of_a_block_and_a_sphere_closes_a_single_solid`. The guard is
therefore not gone but sharpened: `WouldLeaveWrappingLoop` refuses exactly that
configuration, and the decision lives in `remove_cell_staged` — where
`can_remove_cell` can report it before anything is touched — rather than in
healing.

For the same reason `add_revolved_edge_face` builds a ring band only when neither
endpoint sits on the axis. A band with an end on the axis sweeps no circle there
and is closed by that degeneracy, so it keeps its seam until milestone 6, and the
sphere keeps hers.

**Seamed input is now import-only.** No builder makes a seam, so
`tests/builders/removal.rs::seamed_cylinder_wall` constructs one by hand — one
quad face whose two vertical sides are the same edge, sewn to itself. That is the
honest fixture: STEP AP242 and every other interchange format writes periodic
faces cut open (§10.7), so the canonicalizer's subject is an import, and the test
should say so rather than lean on a builder that correctly refuses to produce one.

### 11.5 Milestone 6, boundaryless faces

**A sphere is one face and nothing else.** `add_sphere` no longer revolves a
meridian: it stores a `FaceAttr` carrying `Surface::Sphere` with an empty loop
vec, and roots the sheet and the solid at that face. `sphere(2.0)` is 0 darts,
0 vertices, 0 edges, 1 face — the numbers §3.4 predicts. The poles stay
parametric singularities of `Sphere`; they are not topology.
`add_full_revolved_edge_staged_with_surface`, which existed only to build the
seamed sphere, is **deleted**; the apex-to-apex path it wrapped is still reached
from the general revolve.

**`ShellRoot`** (`src/topology/attributes.rs`) is `Dart(Dart)` or
`Face { face, sense }`, on `SheetAttr::root` and on both of `SolidAttr`'s shell
fields. It carries a `sense` that §5.2 did not foresee: a dart root spells its
shell's direction in the dart, and a face root has neither a dart to `alpha0`
nor a loop seed to reverse, so a spherical *cavity* — an inner shell facing
inward — could not otherwise be written at all. Reversing a boundaryless face is
therefore a property of the root, not of the face.

`validate_shell_roots` at commit enforces §5.3's two rules: a face root must
resolve, and the face it names must be boundaryless
(`TopologyEditError::{DanglingShellRoot, ShellRootNotAtDart}`). Nothing else
stores a key: profiles, edges, vertices and dart-backed faces are untouched.

**Closedness became a geometry question**, as §10.1 warned. `Closeable for Sheet`
answers a boundaryless sheet from `Surface::is_closed`, which asks whether each
parameter direction closes either by periodicity or by collapsing to a point at
both ends of its domain — a sphere both ways, a torus twice periodic, a cylinder
neither. `validate_shell` routes a face root there and raises
`SolidShellSurfaceOpen`. Outwardness keeps the volume sign only: a boundaryless
face has no neighbour across an edge to agree with, and
`Face::boundaryless_signed_volume` integrates over the surface's own domain
rather than fanning from a boundary, applying the view's sense explicitly since
there are no pcurves to carry it.

**Views answer `None` rather than panicking.** `Face::dart`, `Sheet::dart` and
`Solid::dart` return `Option<Dart>`, each with a `_unchecked` sibling — the same
convention as `FaceBoundary::outer_unchecked`. `Sheet` carries an anchor rather
than a dart field, so a boundaryless sheet is still readable in either
orientation; `Sheet::faces` returns its one face, and every dart traversal is
empty.

**Merging needed a handle that is not a dart.** `GMap::merge` returns
`MergeHandle::{Dart, Face}` and `TopologyMerge::with_faces` names the
boundaryless faces to copy, since a merge is otherwise defined by the darts it
copies and such a face has none. Callers that only wanted the copied solid's key
now ask `GMap::solid_key_at`, which is what they meant.

**Tessellation.** A face with no loops meshes over `Surface::domain()` directly.
The pole handling §8.5 asks for was half there — `tessellate_surface_patch`
already dropped the collapsed triangle of a quad on a degenerate row — and is now
complete: a row that collapses across its whole length emits *one* vertex, so the
fan around a pole is stitched from a single apex instead of a row of coincident
copies. `modeling::a_sphere_tessellates_into_a_closed_ball` is the
watertightness check: every mesh edge used by exactly two triangles, at the poles
and across the cut alike.

**Tests that named the old sphere moved rather than died.** The seam a sphere
used to carry is still a real shape — a full revolution of an arc with both ends
on the axis — so `unwrapped_face_domain::a_revolved_meridian_unwraps_the_poles_its_loop_turns_through`
keeps the pole-corner property on that. The double-pcurve invariant moved to
`tests/builders/removal.rs::seamed_cylinder_wall`, the hand-built import fixture
of §11.4, which is now the honest subject for anything about seams.
### 11.6 Milestone 6, the cap

**`LoopKind::Capping { axis, side }`** is in. A lone period-spanning loop says
outright which side of itself the closing degeneracy lies on, because travel
direction cannot: reversing a face has to be a normal flip, and if direction also
chose the side, reversing a cap would move it to the opposite pole.
`UnwrappedFaceDomain::of_face` closes such a loop against the degenerate row the
way it already closes a pair of wrapping loops against a synthesized cut — out to
the row, along it for the period, and back — which is why
`UnwrappedFaceDomainCurve` now carries a *list* of corners rather than one: a cut
takes one turning point, crossing a collapsed row takes two.

**The loop stores a side; the surface says where.** `DomainSide::{Low, High}`
(`geometry::dim2`, beside `Axis2`) is a direction along an axis, not a position,
and `SurfaceGeometry::degenerate_rows(axis)` answers with the parameters at which
a whole row collapses:

| support | rows |
|---|---|
| `Sphere` | its two poles, the ends of the `v` domain |
| `Cone` | its apex, from `apex_parameter` — *inside* an unbounded `v` domain |
| `SurfaceOfRevolution` | where the profile meets the axis, by intersecting the two |
| plane, cylinder, ruled, NURBS | none |

This is what the first attempt got wrong. It spelled the bound as a *domain end*,
which is exact for a sphere and useless for everything else: a cone's `v` domain
is unbounded with the apex somewhere inside it, and a surface of revolution's
profile direction is the profile *support's* domain, not the swept arc's span.
Nor can a caller find the row by searching, because `is_degenerate_at` is a
predicate and an unbounded domain gives no bracket to bisect. Only the surface
knows, so the surface is asked — and nothing is stored that could drift from it.

**A cap is a disk in space.** `revolve_edge_full_turn_with_an_end_on_the_axis_has_one_loop`
sweeps a line that meets the axis at right angles, which is flat — and it is
still a cap, because `revolved_support` cannot build a plane's frame from a
profile starting on the axis and sweeps a surface of revolution instead. What
decides the kind is the parameterization, not the shape: the rim runs a whole
period and the centre is a collapsed row. Only a support that closes the loop in
its own parameters gives a genuine `Outer`.

**Still to build.** `FaceTrimDomain` must answer `Inside` unconditionally for a
face with no loops at all (§8.6), and `split_face_by_imprints` must take a
period-spanning imprint on a boundaryless face and produce two caps (§8.3). Those
two are what `boolean_{union,difference}_of_a_block_and_a_sphere_*` fail on — the
two remaining red tests, with the block-and-cylinder cases beside them passing.
