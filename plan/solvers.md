# Constraint solvers

Status: discussion completed and initial Rust implementation delivered in
`src/solver/`. The packaging choice is a module inside NGK, with focused
responsibilities to keep future extraction feasible. These notes preserve the
discussion; the current contract is in [solver.md](../docs/architecture/solver.md).

The concrete proposal following this discussion is in
[solver-architecture.md](solver-architecture.md). Its implementation status
records the delivered scope and remaining numerical extensions.

## Starting brief

These are the directions expressed so far; they remain open to revision during
the architecture discussion.

- Focus this scope on the 2D solver only. Rigid-motion solving is outside the
  active design and implementation discussion.
- Sketch geometry is 2D. Spatial sketch constraints are outside this scope.
- These are NGK's constraint solvers, implemented as a module in the existing
  crate. They receive geometric data and constraints and return a result.
  Reuse NGK's geometry and mathematical types where useful. Future extraction
  is an option; standalone packaging and general integration abstractions are
  not initial requirements. The exact public types remain open.
- Repeated solving must support interactive movement, without GUI concepts or
  interaction handling inside the solvers.
- The user favors a struct that manages the editable constraint system and
  remembers its last successful configuration. The caller can modify the
  system and solve repeatedly. The exact public ownership boundaries and
  reusable computation state remain open.
- The desired movement is reasonably continuous, avoiding unnecessary jumps
  between solutions. The exact guarantee and branch-selection behavior are
  still open.
- Rust only for this work. Python and WASM bindings are deferred.
- Use "2D solver" as a provisional discussion name. "Sketch solver" is also
  acceptable to the user; naming is not the current architecture question.

## Packaging discussion

Use a module inside the existing NGK crate. The user prioritizes simplicity
over separating packages now. A dedicated solver crate or shared geometry
crate is not part of the current work.

The proposed separation is by responsibility: solver data and numerical
computation use geometric and mathematical types, while application to a
Model and its topology stays outside the numerical computation. Solver-owned
references, parameters, constraints, and configurations remain necessary for
solving; they do not require duplicating the existing geometry library.

A later extraction may follow a concrete reuse need. Architectural boundaries
should keep that possibility reasonable, without adding interfaces or geometry
conversion layers solely to anticipate it.

The D-Cubed precedent concerns an embeddable component integrated with host
geometry. The FreeCAD precedent concerns host-geometry adaptation to solver
parameters; the survey does not establish that FreeCAD's entire integration
is a ready-made standalone package. These are distinct architectural lessons.

The geometric object catalog and the caller's editing workflow are agreed
below. The representation of numerical unknowns remains open. A solver's
segment referencing two variable points is a different role from an evaluated
curve whose coordinates are already known.

## Geometric input and references

The user describes a Nales UI that currently draws lines and circles by
creating NGK objects. The exact Nales representations have not been inspected.
The intended input is a batch of identified geometric primitives plus a list
of constraints referring to them. Primitives may initially be drawn apart,
then joined and dimensioned by constraints to obtain a closed profile.
The initial geometric catalog is points, segments, circles, and circular arcs.
Ellipses and B-splines are deferred from the first delivery.
Reusing NGK geometry and retaining solver-specific variables are compatible.

The user proposes pools with direct references through specific identifiers,
and mentions structure of arrays. Referenced pools and the exact memory layout
are separate design choices; no SoA implementation has been selected.

The user accepts this caller workflow for a stateful system object:

1. Add supported primitives, including from existing evaluated NGK geometric
   values, either progressively or as an initial batch.
2. Receive stable solver-owned identifiers for the primitives.
3. Address characteristic points through those references: segment start/end,
   circle center, and arc center/start/end. Independent points are also part
   of the geometric catalog.
4. Add constraints referring to the geometry and characteristic points.
5. Solve and retrieve evaluated geometry using the same identifiers.

Numerical parameter arrays remain internal. Exact Rust signatures, reference
types, and ownership of imported values remain open. Direct binding to Model
entities is not implied by this workflow; any such integration also needs
model-local key correspondence and coherent geometric updates in a transaction.

Concrete cases that distinguish the representations:

- Two geometric segments constructed using equal endpoint coordinates do not
  retain an identified shared point. A solver must explicitly represent one
  shared variable or two variables joined by a coincidence constraint.
- Two separate endpoints joined by a coincidence constraint permit detachment
  by removing that constraint. A single shared point requires a separate
  operation to split its identity. The user requires the first editing
  behavior: independently drawn primitives can be joined by adding a
  coincidence constraint and detached by removing it. Internal parameter
  sharing or substitution must preserve that editable relation.
- A circle's center is a geometric parameter, not a topological vertex. Solver
  references must be able to address its center and radius independently of
  any model entity key.
- Changing solved vertex coordinates alone does not update the corresponding
  curve supports. Materializing a solved sketch must preserve their agreement.

The public stateful object manages editable geometry and constraints and
retains the last successful configuration. Internal separation between problem
data and numerical computation does not require multiple public objects.

### Circular arcs

The user accepts arcs with positive radius and a sweep magnitude strictly
between zero and one turn: 0 < magnitude < 2*pi. Major arcs such as 270 degrees
are supported. Full circles use the circle primitive; zero-length and
multi-turn arcs are outside the initial solver domain. Arc direction and
initial sweep must survive import and guide repeated solving. The internal
representation of orientation and angles remains open.

## Initial constraint catalog

The user accepts the following catalog as the target for the first delivery,
with implementation progressing through subsets:

- Geometric relations: coincidence, point on curve, horizontal, vertical,
  parallel, perpendicular, tangent, and concentric.
- Dimensions: point-to-point distance, point-to-line distance, horizontal and
  vertical offsets, segment length, radius or diameter, and angle.
- Other relations: equal lengths, equal radii, midpoint, symmetry, and fixing
  a point or a complete primitive.

For the first delivery, point-on-curve constraints address the geometric
support: an infinite line for a segment and the supporting circle for an arc.
A solution may lie outside the segment or arc span. Coincidences can separately
constrain points to identified endpoints. Restricting a point to the interior
of a bounded span is deferred, rather than being silently imposed by the
support relation. The user accepts this distinction and initial scope.

Circle-to-circle tangency supports explicit internal and external modes,
including the circle supports of arcs. The user also accepts an automatic
mode that chooses from the initial configuration and retains that choice
during repeated solving. Rules for ambiguous initial configurations, choosing
the containing circle for internal tangency, and resetting the automatic
choice remain to be specified.

The user accepts two angle variants: unsigned by default, allowing either
side with the initial configuration guiding the choice, and oriented when
the caller wants to impose a side. Angles are in radians. The exact convention
for segment reversal and other signed measurements is specified as a proposal
in the architecture document, not inferred from this agreement alone.

## Result and movement discussion

- The user requests a Result-style return with an error for conflicting
  constraints and references identifying the constraints involved, so the
  caller can repair the problem afterward.
- Exact error types and available diagnostic guarantees remain open. In
  particular, numerical non-convergence must not automatically be labeled a
  proven conflict.
- Underconstrained systems are valid solve inputs. A configuration satisfying
  the supplied constraints counts as success even when degrees of freedom
  remain. Full constraint is not a prerequisite for solving. Closure of an
  output profile is a separate property. The precise DOF diagnostic API is
  still open.
- Temporary movement targets guide the solution toward the user's requested
  position while respecting permanent constraints. Reaching the target exactly
  is not required; an unreachable target alone is not a constraint conflict.
  This is algorithmic input, with GUI interaction handled by the caller. The
  objective, weighting of multiple targets, and solution-selection details
  remain open; no global nearest-solution guarantee has been selected.
- Redundant but compatible constraints are acceptable and produce a warning,
  not a solve error. The user distinguishes underconstrained, fully
  constrained, compatibly overconstrained, and incompatibly overconstrained
  cases. Incompatible constraints produce an error with identifying
  references.
- The stateful object should remember its last successful configuration when
  a solve fails. Detailed editing semantics, especially added or removed
  geometry, remain to be designed. A previous solution must remain associated
  with the problem version it solved, not be presented as satisfying the
  newly edited problem.
- The user provisionally accepts local solving guided by the current
  configuration. Failure to converge locally returns a convergence error and
  preserves the last successful configuration; the ordinary solve does not
  automatically restart from unrelated configurations that could substantially
  change the sketch. A broader search would be explicitly requested by the
  caller. This preference is to be evaluated in use, not treated as a proven
  guarantee of geometric continuity or global infeasibility detection.

Diagnostic representation proposal, not yet an accepted API: keep remaining
degrees of freedom, redundancy warnings, and solve outcome separate. These
properties can coexist: a segment constrained twice to length 10 still has
translation and rotation freedom. A single exclusive status would lose that
information. The exact local rank/DOF interpretation at singular configurations
also remains open.

## Scale and reference research

Typical problems are small sketches, with a few hundred primitives sufficient
as the initial design target. A few thousand may occur in rare cases; this is
not a requested hard size limit. No measured latency target or numerical method
has been selected yet.

The reference investigation requested during the discussion is recorded in the
exploratory
[architecture survey](../docs/architecture/reviews/2026-10-22-constraint-solvers-survey.md).
It records observable precedents and distinguishes them from possible NGK
designs. It is a dated research snapshot; the accepted workflow and later
architecture proposal are recorded in these plan documents.

Local catalog check: `Curve2` currently includes analytical line, circle, and
ellipse variants, as well as NURBS. `TrimmedCurve2` represents segments and
arcs. The absence of a public hyperbola variant and the presence of an ellipse
variant must not be conflated with what the future solver will support.

## Existing NGK boundaries

- The current kernel exposes geometric primitives and topological profiles,
  but no parametric sketch or persistent assembly object.
- [Geometry](../docs/architecture/geometry.md) is independent of topology and
  payload, but is part of the NGK crate.
- [Rigid](../src/geometry/transform.rs) represents rigid transformations.
  [Model transformation](../src/builders/transform.rs) rewrites model geometry
  inside a transaction; it does not store separate instance placements.
- Solver object references must not be confused with model-local entity keys
  or short-lived darts.

## Open design tree

This tree tracks topics to explore. It does not require the user to settle
each topic as soon as it is discussed, and it carries no implicit commitments.

- Module API and ownership
  - Allowed dependencies within NGK.
  - Exact public types for the accepted author/edit/solve/read workflow.
  - Reference lifetime and deletion behavior for objects and constraints.
  - Storage and versioning of the last successful configuration.
  - Application of solved geometry to NGK models.
- Sketch domain
  - Internal parameterization and preservation of arc orientation and sweep.
  - Constraint catalog and the meaning of each measurement.
  - Fixed and external reference geometry.
  - Degenerate geometry and validity requirements.
- Repeated solving and result contract
  - Movement-target representation, priorities, and multiple targets.
  - Remaining-DOF diagnostics and selection between multiple solutions.
  - Continuity expectations and singular configurations.
  - Diagnostic guarantees for conflicts, redundancy, and non-convergence.
  - Diagnostic detail, tolerances, units, and determinism.
- Numerical strategy and completion criteria
  - Representative problem sizes and acceptable repeated-solve latency.
  - Existing numerical dependencies versus a new implementation.
  - Numerical method for the 2D solver.
  - Representative successful, underconstrained, and failing scenarios.
  - Delivery sequence and confirmation of the complete scope.
