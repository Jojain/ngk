---
status: accepted
date: 2026-11-07
---

# A constraint system draws loose regions

## Context

`add_profiles_from_system` draws a solved system as contours and refuses curves
that cross or touch. A sketch is expected to be drawn loosely: two rectangles
crossing, a line overshooting a rectangle to cut it in two. What the user wants
from it is the closed areas the curves enclose, as in other CAD sketchers.

An area is not a profile. It has one outer boundary and any number of holes,
whereas a profile is a connected set of edges: a rectangle with a circle inside
it is two profiles and two areas, but the areas are a ring and a disc. A cross is
five areas, and if they shared edges it would be one profile.

Those areas could be returned in three ways: as profiles for the caller to fill,
as loose faces, or sewn into a sheet.

## Decision

`add_sketch_from_system` returns one loose face per region. Each region owns its
copy of every edge it runs, and all are on the sketch plane with its normal.

Curves meeting form a node whether or not a coincidence joins them, and what
bounds no area is dropped without a refusal. `add_profiles_from_system` stays for
contours and open chains and keeps its refusals, because a profile cannot branch.

## Consequences

- Each region can be used alone. This is the main reason for loose faces:
  `add_extruded_face` fails on a face already sewn to another, and a caller may
  want only some of the regions.
- Sewing and fusing stay available and need nothing new: `add_sheet_from_faces`
  matches the shared stretches by geometry, which is reliable because both copies
  come from one cut, and `remove_redundant_cells` fuses a sewn sheet into one face.
  A later option on the builder can do this for the caller.
- A sketch is not one sheet. Disjoint regions cannot be sewn, and
  `add_sheet_from_faces` refuses them.
- A curve can draw several edges, and a stretch two regions share is drawn twice,
  so the result ties edges to curves per region rather than one edge per curve.
- Drawing loosely has a cost: a line that was meant to touch a rectangle and
  stops short is dropped without complaint rather than reported.
- Regions have no stable identity across edits, since their number depends on the
  system's dimensions.
