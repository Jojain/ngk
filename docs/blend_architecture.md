# Blend Architecture

Chamfer and fillet are one engine, `src/builders/blend/`, behind two thin entry
points:

```rust
chamfer(&mut model, target, distance) -> Result<TargetChamfer, BlendError>
fillet(&mut model, target, radius) -> Result<TargetFillet, BlendError>
```

A target is any mix of vertices, edges, profiles and faces. One call is one
transaction: every refusal names the entity it could not blend and leaves the
model unchanged.

## Pipeline

```text
resolve    BlendTarget  -> Resolution { planar corners, solid edges, corner cuts }
capture    Resolution   -> BlendNetwork                      network.rs
section    network edge -> EdgeSection                       section/
treat      network vertex + sections -> VertexBlend          vertex/
plan       planar corner -> trim points + new edge           corner.rs
assemble   all of the above -> Surgery                       solid.rs
execute    Surgery      -> the model                         execute.rs
check      no face crosses itself, every solid stays closed  check.rs
```

Everything before `execute` reads the model as the call found it. That is what
makes one call order-independent: the resolution is a set, and nothing is
planned against a model an earlier piece of the same call already changed.
Separate calls are sequential, and there order is the caller's to choose.

Only the section solvers and the planar corner solver read the law
(`BlendLaw::{Chamfer, Fillet}`). A chamfer and a fillet therefore differ in
exactly those two places.

## Resolution (`target.rs`)

Incidence decides whether a selection is 2D or 3D.

| selection | wire, or free planar face | solid |
|---|---|---|
| vertex | its corner | chamfer: corner cut; fillet: refused |
| edge | refused | the edge |
| profile | every corner, excluding the ends of an open profile | every edge |
| face | every corner of every loop | every edge of every loop |

When a profile or face is expanded, flat corners and flat edges are skipped.
Named explicitly, they are refused. An edge is flat where its two faces
continue each other all along it: two faces of one plane, or a round and the
face it runs tangent into.

A solid edge brings its **tangent chain**: every solid edge it runs on into
without a corner, found by walking from each end to the one edge that leaves
along the tangent it arrives on. A blend cannot stop part way along a smooth
crease, so naming one arc of a slot's rim blends the whole rim. Where two
edges would continue a chain, it does not grow there.

The three sets are ordered by key.

## Network (`network.rs`)

For each selected solid edge the network records:

- its two faces;
- each face's dart on the edge at the edge's start;
- the network vertices at its ends, or none for a closed edge.

A **closed edge** — an unmarked one, a rim nothing meets — has no end for any
treatment to close. A marked closed edge, whose one corner other edges meet,
is refused.

For each vertex it records the **ring**: slots of edge, face, and that face's
two darts at the vertex, in rotation order. The ring is walked with `turn`, so
bridges and cavity scaffold are crossed. Cutting `k` selected edges splits a
ring into `k` fans, and that tells a treatment what the vertex falls apart into.

## Sections (`section/`)

An `EdgeSection` holds:

- the blend face's surface;
- one rail support per side (the curve that face now ends at);
- the edge's convexity;
- a `SectionForm` recording the closed form, or how the general row built it.

The table, closed form first:

| faces | edge | chamfer | fillet | file |
|---|---|---|---|---|
| plane / plane | straight, whatever curve carries it | `Strip` | `Cylinder` | `planar.rs` |
| plane / extruded wall | translate of the wall's base | `Translated` | — | `translated.rs` |
| surfaces of revolution about the edge's axis | a circle, whatever curve carries it | `Revolved`: cone or cylinder | `Revolved`: torus | `revolved.rs` |
| anything else | anything | `Swept`: skinned bevel | `Swept`: skinned round | `swept.rs` |

Each face's direction into itself is read off the way its stored loop runs
along the edge. Convexity is where that direction points against the other
face's outward normal; the general rows read both at several points along the
edge and refuse one that is flat anywhere, or turns from convex to concave.

**The cross-section solvers** (`contact.rs`) work on a `Crease`
(`crease.rs`): the two faces as oriented surfaces, asked only where a point
lands on each and the outward normal there. A section is pinned to the plane
square to the edge at the point asked for.

- A fillet's ball has its centre `radius` behind both faces on a convex crease
  and in front of both on a concave one. Newton on the centre: each face
  contributes its signed distance, whose gradient is its normal, and the
  section plane the third equation.
- A chamfer's rail on each face is where the face meets the circle of radius
  `distance` about the edge point, in that plane: a true setback, which is
  what the planar strip already is.

**Revolved.** A circular edge between two surfaces turning about its axis —
plane square to it, coaxial cylinder, cone or torus, sphere centred on it — is
one meridian section turned about the axis. One section is solved at the
edge's start and revolved: the round is the torus the ball's centre sweeps,
the bevel a cone, or a cylinder where both rails share a radius. Everything
shares one frame whose `x` points at the edge's start and whose `z` turns the
edge positively, so each rail is the blend surface's isoline at its level,
with the surface's `u` as its own parameter.

**Swept.** Everywhere else sections are solved at evenly spread fractions of
the edge and skinned: `u` along the edge over `[0, 1]`, `v` across from side 0
to side 1. A fillet's section is the exact rational quadratic arc. Each row of
control points is joined along the edge by quintic Hermite spans matching the
row's value, slope and curvature, differenced over sections solved just
either side of each sample; matching curvature is what keeps a skin round a
whole turn to a handful of spans. Sampling doubles until the section solved
half way between each pair of samples lies on the skin within a quarter of
the fitting tolerance. A closed edge gives a closed skin. The rails are the
skin's `v = 0` and `v = 1` isolines.

- **Fillet planar.** The ball's centre is `r` behind both faces on a convex
  edge, and `r` in front of both on a concave one. The cylinder's seam is put
  opposite the arc, so no blend face straddles it.
- **Translated chamfer.** It moves the edge down the wall's rulings and across
  its own chord. That is a true setback only where the edge runs parallel to
  its chord.

## Treatments (`vertex/`)

A treatment writes a `VertexBlend`: local surgery in a fixed vocabulary.

- **corners:** points;
- **joints:** curves between two corners. A joint is inserted into an existing
  face corner, or shared by two new faces;
- **ends:** for each blend ending at the vertex, the corner each rail stops at,
  and the joints that close it;
- **patches:** new faces bounded by joints.

`treat.rs` chooses the treatment from how many selected edges end at the vertex
and how many faces meet there.

| treatment | when | chamfer | fillet |
|---|---|---|---|
| `run_out` | 1 selected, 3 faces | segment on the end face | circle or ellipse on the end face |
| `smooth` | 2 selected running on into each other, 3 faces | their shared segment | their shared arc |
| `mitre` | 2 selected meeting at a corner, 3 faces | segment | ellipse in the plane mirroring the two axes |
| `trihedral` | 3 selected, 3 faces | three mitres to the planes' common point | spherical triangle |
| `corner_cut` | a solid vertex target | triangle | refused |

In a run-out, each rail carries on until it lands on the unselected edge beside
it, strictly inside that edge.

A **smooth** join is where one selected edge leaves along the tangent the other
arrives on: a slot's straight rim running into its arc. Both blends are cut by
the plane square to that tangent in the same section, whatever solved them —
a cylinder and a torus, two skins — and that section is the joint between the
two blend faces, running from the rails' meeting point on the face the edges
share to where the rails across meet on the unselected edge. Sections that do
not agree there are refused.

A mitre needs both blends to land on the same point of the unselected edge
between them. That holds when the two faces across that edge make the same
angle with the face the selected edges share.

A treatment refuses, naming the vertex, anything it has no closed form for:

- another valence;
- mixed convexity;
- different radii meeting;
- blends that land apart;
- a revolved or swept blend ending at a corner.

## Surgery (`surgery.rs`) and execution (`execute.rs`)

`Surgery` is geometry-complete. It holds:

- corners: a point and lineage sources for each;
- consumed vertices;
- cuts: a selected edge, and on each side its rail and pcurve. A rail runs
  between two corners, or is closed (`RailEnds::Closed`) where the cut edge
  was;
- joints: a curve, an optional insertion, and lineage;
- new faces: a surface, lineage, and a boundary (`NewBoundary`): a walk of
  rails and joints, each with its pcurve, or a band between two closed rails.

`assemble` in `solid.rs` builds one blend face per selected edge. A bounded
edge's face walks

```text
rail 0 (start -> end), the chain at the end, rail 1 reversed, the chain at the start reversed
```

A closed edge's face is a **band**: its two whole rails, the first with its
rail and the second against it.

The executor computes no geometry. It:

1. unsews α2 along every cut, so each side becomes its own edge;
2. splices each inserted joint in after the incoming edge's dart. A bridge
   attached at the corner therefore stays on the corner's far side;
3. lays down each new face's walk and sews it to its partners, which it finds
   by id and never by coordinates. A band is two one-edge loops;
4. orients each new face so that it crosses every shared edge opposite its
   neighbour, starting from the existing faces;
5. registers lineage:
   - rails and blend faces derive from their edge;
   - corners, joints and patches derive from their vertex;
   - the selected edges and blended vertices are consumed;
   - a closed rail owns the 0-cell where it closes, as the cut edge did;
6. registers a band with two `Wrapping` loops along `u`, and joins them by a
   scaffold cut (`cut_between_loops`) so it occupies one 2-cell, as a lofted
   band does;
7. writes the planned pcurves, and cuts down the pcurves of every surviving
   edge whose end moved.

No dart is ever removed, so nothing that points into the map needs repair.
Faces keep their keys, and a face whose boundary moves outward (a concave
blend) is handled exactly like one whose boundary moves inward. Each planned
corner must end up as exactly one vertex, and every piece of a consumed vertex
must land on a planned corner. A treatment that forgets a piece fails before
commit.

## Pcurves (`pcurve.rs`)

A plane's pcurves are exact (`curve_pcurve`). Every other surface goes through
`pcurve_on_surface`, the trace the analytic intersections use. It is exact on
an isoline (a cylinder's rulings and cross-sections) and a measured fit
otherwise (an ellipse on a cylinder, a great circle on a sphere). A fit beyond
the fitting tolerance is refused.

A rail that is an isoline of its blend surface — every revolved and swept
rail — has its pcurve there written exactly. A closed rail on an existing
curved face is followed sample by sample, each lifted onto the branch of the
one before and the first onto the branch the face's old rim started on, so a
wall's rim stays one period long and its two rims keep bounding one band.

## Checks (`check.rs`)

After execution, every face the surgery touched is sampled in its parameter
space. A face fails if its loops cross, or if its outer boundary's winding
changed sign. Windings are recorded for every face a cut runs along as well as
every face at a consumed vertex, so a rail pushed past the far rim of a wall
— which crosses nothing — is caught turning the wall inside out. A wrapping
loop runs one period and is sampled open. Every solid touched is then
validated as a manifold, outward shell. A failure rolls the call back.

## Code map

- `src/builders/blend/`: the engine; `section/` holds the section table and
  its solvers, `vertex/` the treatments.
- `src/builders/chamfer/`, `src/builders/fillet/`: entry points and results.
- `tests/builders/chamfer.rs`, `tests/builders/fillet.rs`: behaviour, including
  closed-form volumes; `tests/support/blend_shapes.rs`: the shapes they blend.
- `src/scripts/chamfered_block.rs`, `chamfered_rectangle.rs`,
  `chamfered_wavy_edge.rs`, `filleted_block.rs`: playground scenes.
- `plan/blend.md`: what comes next and why the abstractions are shaped as they
  are.
