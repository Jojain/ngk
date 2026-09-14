# Seamless periodic faces — done

Status: **Complete.**

This plan removed parameterization seams from NGK's logical topology. The
original implementation journal described several intermediate designs,
including dartless faces and `ShellRoot`; those were later superseded by the
finite scaffold and one-cell invariant in
[one_logical_cell_one_raw_cell.done.md](one_logical_cell_one_raw_cell.done.md).
This file records only the final result.

## Contract

A seam that exists only to cut open a periodic parameter domain is not a
logical edge. It may exist as an embedded raw edge inside a face, or be
synthesized temporarily in an unwrapped domain or exchange file, but ordinary
face traversal, selection, and rendering do not expose it.

Periodic topology has three principal forms:

- a ring face has wrapping boundary loops and no longitudinal seam edge;
- a whole sphere or torus is one boundaryless face with no logical edge or
  vertex;
- an intersection or pcurve may cross a period without being split at the
  parameterization origin.

## Final representation

### Closed edges

Logical edge kind follows distinct corner count rather than curve type:

| Kind | Corners | Typical use |
|---|---:|---|
| bounded | 2 | segment or arc |
| marked | 1 | deliberately cut closed curve |
| unmarked | 0 | seamless whole circle |

An unmarked edge still has a raw closure 0-cell, embedded inside the edge.
Cutting it materializes one logical corner and produces a marked edge; cutting
a marked edge produces two bounded edges.

Curves remain unbounded supports. An edge's native interval is derived from
its corners and traversal orientation; closed unmarked edges span one complete
period. Reversal keeps the same physical interval with opposite direction.

### Face boundaries

`FaceBoundary` stores either typed loop definitions or a closed-face anchor.
`LoopDefinition::{Outer, Inner, Wrapping}` states the domain role. The logical
boundary walk turns over face-owned raw bridge and seam edges instead of
emitting them.

`UnwrappedFaceDomain` cuts periodic parameter space on demand for trimming,
tessellation, Boolean work, and STEP export. It preserves full-period spans and
avoids jumps across the chosen branch.

### Boundaryless faces

A boundaryless face is never dartless. It occupies one finite raw 2-cell whose
lower-dimensional cells are embedded in the face:

- a sphere uses a bigon with its two sides identified;
- a torus uses a square with both pairs of opposite sides identified.

The face's oriented anchor carries its sense. Sheet and solid roots are bare
darts, and all public dart accessors are total.

## Construction and exchange

- Cylinder and full-turn revolve builders create ring faces directly.
- Sphere and torus builders create one boundaryless logical face over their
  analytic support.
- Full-turn intersections retain continuous period-spanning sections.
- STEP export synthesizes the seams and closure points required by the file
  format without mutating the model.
- STEP import rebuilds topology, then its seams-only healing pass removes
  synthetic periodic edges before any solid cavity cut is attached.
- A `VERTEX_LOOP` on a closed support imports as a boundaryless face.

## Healing

The seam pass uses the GMap removal operation and recognizes the shape left
behind:

- two remaining wrapping loops: ring;
- one loop closed by a degenerate row: cap;
- no remaining loop on a closed support: boundaryless face.

It refuses an actually open support instead of inventing closure. Demotion from
a bounded to a boundaryless face preserves the old winding in the new anchor.

## Milestones delivered

1. Typed face loop roles and atomic boundary storage.
2. Unwrapped periodic-domain synthesis.
3. Ring faces in cylinder and revolution construction.
4. Seam-crossing imprint and intersection spans.
5. Bounded, marked, and unmarked edge views.
6. Finite boundaryless sphere and torus cells.
7. STEP seam synthesis and import canonicalization.

## Deferred independent work

Rebuilding a fused pcurve on a curved support is not part of seamless topology;
it remains in
[curved_support_pcurve_rebuild.md](curved_support_pcurve_rebuild.md). The
topological representation already accepts the intended healed result, while
that plan improves the geometric reconstruction needed to reach it.
