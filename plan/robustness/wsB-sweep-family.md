# WS B: extrude, revolve, sweep and loft correctness

**Owns:** `src/builders/revolve/`, `src/builders/sweep/`, `src/builders/loft/`,
the extrusion in `src/builders/solids/` and `src/builders/sheets/extrude.rs`.
**Depends on:** C for the panic-to-error safety net. **Decision:** D6 (loft).
**Bench:** about 25 cases.

Four independent classes, each its own sub-task.

## B1. Unmarked edges (closed, cornerless) in the sweep family

**Symptoms.**

- `revolve_face` of a disc (`faces::circle`) fails with
  `MissingVertexPoint { dart: Dart(0) }` for 90° and for 360°: a torus cannot
  be made by revolution, nor a ball-swept groove.
- `sweep_face` of a disc panics, even along a straight line:
  `ConflictingOwnership { dimension: Zero, held: Vertex, claimed: Edge }` in
  `sew_cap`.
- A loft between two circles works only if their circles start at the same
  angle; otherwise `CapEdgeMissing { column: 0, slot: 1 }`.
- The Boolean on a revolved triangle (`parts/v-belt-pulley`) panics the same
  way in `split_face_by_imprints`.

**Root cause.** An unmarked edge has no corner; its 0-cell is scaffold owned
by the edge itself. Each of these builders needs a corner there: the seam of
the lateral face starts at it. They read the 0-cell as a vertex, or register a
vertex on it, without first *cutting* the edge (CONTEXT: cutting an unmarked
edge marks it), so the labelling says both "vertex" and "edge scaffold" for
one 0-cell. `solids::cylinder`, `sphere` and `torus` work only because they
are built by hand rather than through these builders.

**Fix.** One shared step at the start of each builder: every unmarked edge of
the section (and of the spine) is cut at its natural start, and the builder
works with the marked edge; healing may unmark it again at the end where the
result needs no corner there (a torus from a disc has none). Loft cuts both
circles at corresponding points rather than demanding they already match.

**Family.** Disc, annulus, ellipse face and a face whose outer loop is a
closed NURBS edge, each revolved (90°, 360°), swept along a line, an arc, a
helix and a polyline (both transitions), extruded, and lofted to another
circle turned by 0°, 37° and 90°, and to a square.

## B2. Edge sense ignored when an edge becomes a lateral face

**Symptom.** A square less a quarter disc, its arc edge stored running against
the loop: the face's area is right (0.8037), but extruding it gives 1.589,
the complement arc swept instead of the quarter. Revolve gives a different
(wrong) volume for the same face depending on how the arc is stored. Sweep
and loft are right.

**Root cause.** Extrusion and revolution build each lateral face from the
edge's stored span and ignore which way the loop runs it, so an arc stored
backwards is swept the other way round its circle.

**Fix.** Read each section edge as the loop runs it (span reversed where the
loop runs it backwards) in extrusion and revolution, as sweep and loft
already do. Audit every builder that turns a loop's edges into faces
(`add_extruded_profile`, the revolve family, chamfer and fillet rails) for the
same read.

**Family.** For lines, arcs, ellipse arcs and NURBS edges, each stored along
and against the loop, convex and concave: extrude, revolve, sweep, loft, and
check volume against the closed form.

## B3. Revolve: meridians that meet the axis, negative angles

**Symptoms.**

- An edge with both ends on the axis (a semicircle, half an ellipse, a
  rational meridian, a lens's arc) is refused, `ApexRevolveUnsupported`:
  no spheroid, ellipsoid of revolution, lemon, apple, NURBS sphere.
- An edge with one end on the axis builds, but a curved one (Bézier dome,
  hyperbola, NURBS cone) comes out with "shell 0 has the wrong orientation".
- A circle tangent to the axis (horn torus) fails with `MissingVertexPoint`.
- A negative angle commits a solid of zero signed volume and inward faces
  instead of refusing or building the mirror.

**Fix.** `add_axis_revolved_face` handles a meridian meeting the axis at both
ends (two poles, as `add_sphere` already builds), at an angle or square to it
(apex or smooth pole), and tangent to it (a pole that is also a degenerate
seam). Orientation of the result is read from the geometry, not assumed from
the section's winding. A negative angle revolves the mirror way, or is
normalised to a positive angle about the reversed axis.

**Family.** Half disc, half ellipse, rational semicircle, Bézier dome,
hyperbola branch, parabola, lemon segment, apple segment, horn-torus circle,
each at 90°, 360° and −90°, both windings.

## B4. Sweep frames, and the helix's hidden start

- `plan_wall_groups` builds `Frame::from_xz` from a transported frame that is
  no longer orthogonal along a NURBS spine, and `from_xz` panics. Transport
  must re-orthogonalise at each step (or compute the rotation-minimising frame
  directly), and library code must call `try_from_xz` and return an error.
  Family: pipes and square bars along planar and twisted cubics, a closed
  NURBS spine, a spine with an inflection.
- `edges::helix(axis, …, start_angle, …)` measures its angle from an x
  direction `Axis3` chooses itself (for +z it is −x), so a user cannot say
  where a helix starts. Give `helix` a `Frame`, or document and fix the
  reference direction.

## B5. Loft correspondence (D6, decided: expose the choice)

`loft/square-to-octagon`, `loft/square-to-round` and `sweep/twisted-duct`
build a valid solid that is not the one described, silently: the loft aligns
seams by proximity and ignores the start vertex it is given, and pairs
unequal corner counts by arc-length fraction.

`LoftOptions` gains the pairing as an explicit choice:

- `AsGiven`: each section's start vertex and direction as the caller built
  them;
- `Aligned`: the current automatic alignment (least twist), named as such;
- `Correspondence`: an explicit list, vertex to vertex, or vertex to a whole
  edge (the face between them a triangle), for sections with unequal corner
  counts.

The default is chosen and documented once; whichever it is, the loft never
changes a pairing the caller asked for. Audit the other builders for the
same rule (sweep transitions, chamfer corners, blend corner treatments):
where more than one valid result exists, the choice is an option.
