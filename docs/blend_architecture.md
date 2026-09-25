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
section    network edge -> EdgeSection                       section.rs
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
Named explicitly, they are refused. The three sets are ordered by key.

## Network (`network.rs`)

For each selected solid edge the network records:

- its two faces;
- each face's dart on the edge at the edge's start;
- the network vertices at its ends.

For each vertex it records the **ring**: slots of edge, face, and that face's
two darts at the vertex, in rotation order. The ring is walked with `turn`, so
bridges and cavity scaffold are crossed. Cutting `k` selected edges splits a
ring into `k` fans, and that tells a treatment what the vertex falls apart into.

## Sections (`section.rs`)

An `EdgeSection` holds:

- the blend face's surface;
- one rail support per side (the curve that face now ends at);
- the edge's convexity;
- a `SectionForm` recording the closed form.

The table:

| faces | edge | chamfer | fillet |
|---|---|---|---|
| plane / plane | straight, whatever curve carries it | `Strip` | `Cylinder` |
| plane / extruded wall | translate of the wall's base | `Translated` | refused |

Each face's direction into itself is read off the way its stored loop runs
along the edge. Convexity is where that direction points against the other
face's outward normal.

- **Fillet.** The ball's centre is `r` behind both faces on a convex edge, and
  `r` in front of both on a concave one. The cylinder's seam is put opposite
  the arc, so no blend face straddles it.
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
| `mitre` | 2 selected, 3 faces | segment | ellipse in the plane mirroring the two axes |
| `trihedral` | 3 selected, 3 faces | three mitres to the planes' common point | spherical triangle |
| `corner_cut` | a solid vertex target | triangle | refused |

In a run-out, each rail carries on until it lands on the unselected edge beside
it, strictly inside that edge.

A mitre needs both blends to land on the same point of the unselected edge
between them. That holds when the two faces across that edge make the same
angle with the face the selected edges share.

A treatment refuses, naming the vertex, anything it has no closed form for:

- another valence;
- mixed convexity;
- different radii meeting;
- blends that land apart.

## Surgery (`surgery.rs`) and execution (`execute.rs`)

`Surgery` is geometry-complete. It holds:

- corners: a point and lineage sources for each;
- consumed vertices;
- cuts: a selected edge, and on each side its rail and pcurve;
- joints: a curve, an optional insertion, and lineage;
- new faces: a surface, lineage, and a boundary of rails and joints, each with
  its pcurve.

`assemble` in `solid.rs` builds one blend face per selected edge, walking:

```text
rail 0 (start -> end), the chain at the end, rail 1 reversed, the chain at the start reversed
```

The executor computes no geometry. It:

1. unsews α2 along every cut, so each side becomes its own edge;
2. splices each inserted joint in after the incoming edge's dart. A bridge
   attached at the corner therefore stays on the corner's far side;
3. lays down each new face's walk and sews it to its partners, which it finds
   by id and never by coordinates;
4. orients each new face so that it crosses every shared edge opposite its
   neighbour, starting from the existing faces;
5. registers lineage:
   - rails and blend faces derive from their edge;
   - corners, joints and patches derive from their vertex;
   - the selected edges and blended vertices are consumed;
6. writes the planned pcurves, and cuts down the pcurves of every surviving
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

## Checks (`check.rs`)

After execution, every face the surgery touched is sampled in its parameter
space. A face fails if its loops cross, or if its outer boundary's winding
changed sign. Every solid touched is then validated as a manifold, outward
shell. A failure rolls the call back.

## Code map

- `src/builders/blend/`: the engine.
- `src/builders/chamfer/`, `src/builders/fillet/`: entry points and results.
- `tests/builders/chamfer.rs`, `tests/builders/fillet.rs`: behaviour, including
  closed-form volumes.
- `src/scripts/chamfered_block.rs`, `chamfered_rectangle.rs`,
  `chamfered_wavy_edge.rs`, `filleted_block.rs`: playground scenes.
- `plan/blend.md`: what comes next and why the abstractions are shaped as they
  are.
