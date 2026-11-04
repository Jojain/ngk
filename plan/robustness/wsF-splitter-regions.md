# WS F: face splitting on periodic faces, at poles, across loops, around holes

**Owns:** `src/builders/faces/imprints.rs` and the splitter around it,
`src/topology/unwrapped_face_domain.rs` (shared with WS A for
`polyline_on` only), `src/topology/trim_domain.rs`, the coincident-region
code of `src/builders/boolean/graph.rs`. **Decisions:** D3, D4.
**Bench:** about 9 cases, and most real parts. Builds on the audit in
`docs/architecture/reviews/2026-11-02-boolean-bug-classes.md`, whose classes
A, B1 and E this workstream closes.

## F1. Several wrapping loops on one periodic face (class E)

**Symptoms**, all `imprint 0 of face … was not realized by any section`:

- a ball cut by two parallel planes (a slab through it), or bored through
  along its pole axis (`napkin-ring`): the sphere face gets two latitude
  circles;
- the golf ball: two of the 62 dimples sit on the poles;
- a box intersected with a ball that reaches all six faces;
- a groove turned into a shaft: cylinder less a ring, `result shell … is open`;
- a cone frustum cut by a plane tilted 2° or 10° (a cylinder cut the same way
  works);
- a torus cut by a slightly tilted plane, or by a cylinder or ball sitting on
  its seam (the same tool off the seam works).

The same cuts with one wrapping loop, or with loops that do not wrap, work.
The sphere cut along a meridian through both poles fails too
(`ball-halved-through-its-poles`), whatever the pole axis's direction in the
plane; cut off the poles it works.

**Root cause.** The splitter forms regions in the unwrapped domain from loops
that are closed there. A loop that wraps a period is closed only on the
cylinder (or torus) the domain covers. One such loop on a boundaryless face
is handled; a second, or one added to a face that already has wrapping rims
(a cone or torus wall), leaves a band whose two boundaries both wrap, and
the region builder has no rule for it. On a torus, a loop crossing the seam
splits into pieces a period apart that are not rejoined. At a pole the
section's pcurve runs into a row of the domain that is one point in 3D, and
the meridian's two halves do not connect through it.

**Approach (D3, decided: no seam entity, the seam is map scaffold only).**

- **Reason on the quotient.** Classify each loop by its
  winding numbers about the periodic directions (0 for an ordinary loop,
  ±1 for a band boundary, (p, q) on a torus). Regions are formed on the
  cylinder or torus: two loops with winding +1 and −1 bound a band; an
  ordinary loop bounds a disc; a pole is a point of the domain where a
  section can enter and leave along any meridian. The unwrapped domain stays
  the storage; the splitter stops assuming a loop closes in it.
- For poles: the splitter joins section pieces through a degenerate row.
  (Re-parameterising a face's sphere so no section passes through its pole
  was considered and set aside: it only moves the problem to the next
  section.)

**Family.** Sphere with 1, 2, 3 latitude cuts; with two off-axis circles each
around a pole; with a meridian through both poles; with a circle through one
pole. Cylinder wall with 1, 2, 3 bands removed, with a tilted ellipse, with a
helical band. Cone wall with a tilted ellipse and with a band. Torus with a
plane at 0°, 1°, 10°; with a ball and a cylinder on its seam, beside it and
straddling it; with a (1, 1) loop. All four operations.

## F2. An imprint bridging two loops of a face (class B1)

**Symptoms.** A stadium prism or a torus slid past a rod, ending past both
rod ends; a wedge on a hub with the same height; a hole drilled through a
tube's wall parallel to its axis: the sides cut the cylinder wall along
lines from one rim to the other. The audit's mechanism B1 holds: the splitter
tries one bounding loop at a time, and a chain from one loop to another
matches none.

**Fix, in the current architecture.** A chain whose ends sit on two
different loops is a bridge: the first bridge merges the two loops into one
(the face count is unchanged), and every further chord then splits that loop
as usual. The audit lists the family in full.

## F3. Coincident regions with holes (class A)

**Symptoms.** `coincident region 0 is not bounded by one closed oriented
cycle` for a tube standing on a face, a hollow square standing on a face, a
block standing on a face drilled under it, and the tray's standoffs.
A solid boss on a face works.

**Root cause.** The coincident overlap of two planar faces is taken to be
bounded by one cycle; where either face has a hole inside the overlap, the
overlap is a region with holes, bounded by several cycles.

**Fix, in the current architecture.** The overlap region is a set of cycles
(one outer, any number of holes, possibly several components), oriented and
nested; each becomes an imprint like any other.

**Family.** On a block's top: a tube, a square tube, a ring of two tubes,
a plate with two holes; a block on a drilled face, a block on a face with
the hole half under it; the same with the coincident faces' orientations
reversed (cut, intersection).

## F4. Non-manifold answers (D4)

Where the regularised answer is not a manifold solid (a solid pinched at a
point or along a line), the splitter and assembly must recognise it and
refuse by a named error, until D4 decides to represent it. Today they fail
earlier with an unrelated error, or (`tangent/cone-tip-on-face`, WS G) read
one solid as two.
