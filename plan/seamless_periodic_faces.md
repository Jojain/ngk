# Seamless periodic faces

Status: **In progress** — milestones 0 (`src/topology/attributes.rs`),
1 (`src/topology/face.rs`) and 2 (`src/topology/chart.rs`) are implemented.
Milestone 3 is **partly built**: see its row in §11 for what stands and what is
left. Everything from milestone 4 on is designed but not built.

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

> **A seam is a property of a chart, not of a shape.**

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

### 4.2 `EdgeKind` — an enum, and no parallel `Option` API

```rust
enum EdgeKind<'a, P: Payload> {
    Bounded { start: Vertex<'a, P>, end: Vertex<'a, P> },
    Closed,
}
```

`Edge::start()`, `Edge::end()` and `Edge::vertices()` come **off** `Edge`
entirely. Leaving them as `Option`-returning methods beside the enum is the
failure mode: callers take the `is_none()` path and the type safety buys nothing.
The only way to an endpoint is through the match.

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
| Edge endpoints | `start()`/`end()`, infallible | `EdgeKind::{Bounded, Closed}`, no `Option` API |
| Edge span | from bounding vertices | from bounding vertices, or the whole period when closed |
| `SheetAttr` / `SolidAttr` | rooted at a `Dart` | `ShellRoot`, dart preferred |
| Shell validity | no alpha2-free dart | that, plus geometric closedness for a key-rooted shell |
| Seam | a stored `EdgeKey` | a chart cut computed on demand, stored nowhere |

## 8. Work breakdown

### 8.1 `topology`

- `attributes.rs` — `LoopKind`, `BoundaryLoop`, `FaceBoundary` replacing
  `outer_loop`/`inner_loops`; `ShellRoot` on `SheetAttr` and `SolidAttr`.
- `face.rs` — `Face::new`, `outer_loop_dart`, `outer_loop`, `inner_loops`,
  `loops`, `reversed`, `normal_at` (signed area), `signed_volume_contribution`.
  `pcurve()` gets *simpler*: its alpha0/alpha2 candidate probing exists largely to
  find the right seam occurrence.
- `edge.rs` — `EdgeKind`; `parameter_interval` and `length` stay total.
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
- `surface_surface/tracer.rs:393-430` — the "move a state that reached a closed
  parameter boundary to the far seam edge" step can instead let `u` run past the
  period, with the branch closing when it returns to the seed in the quotient.

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
  chart cut** — reconstruct a virtual seam at query time, at a `u` away from the
  loops. That confines the change to one constructor, keeps `images` intact, and
  is the concrete instance of §2. A boundaryless face answers `Inside`
  unconditionally.
- `classify.rs` — `chart_center` / `chart_bounds` read `polygons.first()`; they
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
| **2** | **Chart synthesis** | One function: given a face and its loops, produce a cut position and a simply-connected UV chart. Route `FaceTrimDomain::new` and `tessellate_face` through it, still on seamed input. Proves the abstraction before anything depends on it. **Done** — `topology::chart::{Chart, ChartLoop, ChartCurve, Axis2}`. |
| 3 | Ring faces | `LoopKind::Wrapping`, cylinder and full-revolve builders, per-direction trimming, tessellation wrap + watertightness test, seam 1-removal in `removal.rs`, Boolean trim on rings. Sphere still seamed. **Partly done** — see §11.1. |
| 4 | Seamless intersections | Drop `seam_crossings()` splitting; period-spanning pcurves end to end through imprint and assembly. |
| 5 | `EdgeKind` + vertexless closed edges | The enum and the whole-period span rule land together — today's circle edges carry a vertex, so the `Closed` variant cannot be honest before they lose it. 0-removal of the orphan vertex. |
| 6 | Boundaryless faces | `ShellRoot` and the dart-preferred invariant, degeneracy-closed directions, validation by surface for key-rooted shells, sphere and torus builders, pole-aware tessellation. |
| 7 | Healing canonicalizer | `seam_removal` pass: a seamed model in, a seamless model out. |

Milestones 0–4 deliver most of the practical benefit — Booleans on cylinders stop
being seam-sensitive — without touching dart-rooted identity. Milestone 6 is
separable and can be judged on its own once 0–5 are in.

### 11.1 Milestone 3, as it stands

Built, with the tree's cylinder tests green on it:

- `LoopKind::Wrapping { axis: Axis2 }` (`Axis2` now lives in `geometry::dim2`),
  plus `FaceBoundary::{seed, wrapping, is_ring, retain_mapped}`. `darts()` now
  walks every loop rather than outer-plus-inner.
- `Face` no longer assumes an outer loop: `outer_loop()` returns `Option`,
  `dart()` reads the boundary's seed, `loops()` covers every kind, and
  `boundary()` exposes the stored kinds. `normal_at` reads its winding off the
  chart rather than off the outer loop, which is what makes it answer for a
  ring at all.
- `Chart` fuses wrapping loops into one closed chart boundary across a
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

Left to do, with the tests that currently fail on each:

1. **A chord across a ring** — an imprint running from one wrapping loop to the
   other, which a tangent contact produces. One chord opens the ring into a
   disk; two cut it into two disks. Neither is expressible by
   `apply_outer_face_chord_split`, which needs an outer loop to chord across.
   The shape of the fix is to open the ring at the first chord — splitting both
   wrapping loops at its endpoints and walking the chord twice — and let the
   existing chord machinery apply the rest. Fails:
   `boolean::block_fused_with_cylinder_tangent_to_block_faces`,
   `removal::redundant_faces_of_boolean_fuse_are_deleted`,
   `boolean_integration::{a_block_intersected_with_a_tangent_cylinder_keeps_no_redundant_topology,
   boolean_results_are_healed_by_default,
   healing_a_tangent_union_fuses_the_fragments_the_imprint_created}`.
2. **`faces::splitting_a_cylinder_seam_preserves_both_face_pcurves`** tests the
   double pcurve bookkeeping a seam needs. A seamless cylinder has no seam to
   split, so the test states an invariant this milestone removes; it should be
   rewritten against a still-seamed periodic face (a sphere) or dropped with
   milestone 4.
3. **Tessellation wrap and the watertightness check** (§8.5) — `tessellate_face`
   runs on rings through the chart, but nothing yet asserts the `u = 0` and
   `u = 2pi` columns share mesh indices.
4. **Seam 1-removal in `removal.rs`** and `SkipReason::PeriodicSurface`
   (§8.4) — still a guard, so a seamed import cannot yet be canonicalized into
   a ring.
5. **`revolve.rs`** — a full revolution still sews its swept copy back onto its
   source, so surfaces of revolution keep their seam.
