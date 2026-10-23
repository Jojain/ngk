# 2D geometric constraint solver

`src/solver/` exposes a stateful Rust `System2d`. It owns a geometric constraint
problem, an editable initial configuration and its last successful solution.
It imports and returns NGK geometry values without depending on `Model`, GMap,
builders, Nales, or a GUI. The caller materializes topology separately.

## Public workflow

```rust
use ngk::geometry::dim2::{trimmed::TrimmedCurve2, utils::Point2};
use ngk::solver::{Constraint2d, System2d, Target2d};

let mut system = System2d::default();
let segment = system.add_segment(TrimmedCurve2::segment(
    Point2::new(0.0, 0.0), Point2::new(6.0, 2.0),
))?;
system.add_constraint(Constraint2d::length(segment, 10.0))?;
let solution = system.solve()?;
let moved = system.solve_toward(&[
    Target2d::point(segment.end(), Point2::new(15.0, 8.0)),
])?;
let geometry = moved.segment(segment)?;
```

`solve` and `solve_toward` return `Result<Solution2d, SolveError>`. Inputs are
evaluated values: importing a curve does not establish a mutable binding to
the source curve or a model entity. Constructors validate finite data,
nonzero segments, positive radii and circular arcs strictly shorter than one
turn. Ellipses, splines, spatial constraints and bindings are outside this API.

## Visualization playground

The visualization app registers `solver-2d` (sidebar: **Contraintes / Solveur 2D**,
URL fragment `#/solver-2d`). It uses a DOM/SVG panel surface instead of the shared
3D scene. Existing scene experiments keep their scene shell and Leva controls.

`bindings/wasm/visualization/solver.rs` exposes `SolverPlayground`, a visualization
adapter with a persistent `System2d`. Its JSON commands import primitive values,
add/update/remove relations, solve, and return geometry and diagnostics. Its
session-local numeric IDs map to typed solver keys. The adapter is separate from
the modeling bindings; it is not a second constraint implementation or a Python
API. The browser releases each session when replacing an example or unmounting.

The panel offers rectangle, point-on-circle and tangent-arc examples. Points use
one drawing click; segments and circles use two; arcs use center, start and end
direction, with an optional clockwise sense. A grid option snaps drawing and
movement targets to whole units. Angles in the panel are degrees; the adapter
converts them to radians. Point dragging sends temporary targets, coalesced once
per animation frame, followed by a final target on pointer release.

Constraints can be added through typed reference selectors, edited by clicking
their list entries, or removed. Removing a primitive also removes its dependent
constraints. Numerical failure preserves the problem for repair and highlights
the reported constraint group. Compatible redundancy remains a warning. Reports
show degrees of freedom, solve time and distance to the requested target.

The generated WASM package must include `SolverPlayground`; TypeScript/Vite
validation alone does not rebuild it. The normal `visualization` development and
build commands regenerate WASM. With an already generated package, run
`npm exec vite` from `visualization/` and open `#/solver-2d`.

## Identity and state

Geometry and constraints have separate slotmap stores. Typed public keys carry
their solver owner, slot generation and an issuance serial. The serial is
allocated independently of editable state, so a key leaked from an aborted
batch cannot accidentally identify a later insertion. A point reference is
an independent point or a primitive's stable start, end or center role.

Each primitive owns its minimal parameter vector. A solve compacts these
vectors into component-local numerical arrays; the public API never exposes
their offsets. There is no permanent merged-point identity: removing a
coincidence removes its equation, allowing the two original points to separate.

| Primitive | Parameters |
|---|---|
| Point | x, y |
| Segment | x/y of both endpoints |
| Circle | center x/y, radius |
| Arc | center x/y, radius, start angle, positive sweep |

Code names each parameter with `primitives::Param` rather than its position.
A parameter sits at the same position in every kind that has it, so the name
resolves to a constant; debug builds check that the kind has it.

Arc direction is discrete metadata; its sweep satisfies `0 < sweep < 2*pi`.
Endpoints are evaluated from these five parameters and differentiated through
them. Major and clockwise arcs retain their direction and sweep. A full circle
retains its imported parameter frame without adding an artificial rotation DOF.

Edits update the seed and problem revision. Fixing a point or primitive captures
its values at constraint creation; updating geometry does not silently change
that fixed target. Replacing the fixing constraint explicitly recaptures it.
Fixing a primitive also captures its discrete direction and circle parameter frame.

`edit` applies a batch on staged state and commits it atomically as one revision.
An invalid edit or failed batch leaves the original state intact. Ordinary
primitive removal refuses referenced geometry and identifies its dependent
constraints; `remove_cascade` removes them explicitly and reports their keys.

Successful solving publishes the entire candidate configuration and an owned,
immutable `Solution2d` together. If any component fails, none is published.
The last success remains available and earlier accepted edits remain editable.
New primitives use their new seeds, even when absent from the last solution.

`solution()` is available only for the current revision. `last_solution()`
also exposes a historical result, tagged with its revision and solve sequence.
A solution owns its geometry behind an `Arc`; it can still read a primitive
deleted from the live system. The caller can retain snapshots without blocking
further edits. Positional targets do not change the permanent problem revision.

## Constraint semantics

The catalog includes coincidence, point on line/circle, horizontal/vertical,
point distance, distance to a line, signed x/y offsets, segment length, radius
or diameter, equal segment lengths/radii, parallelism, perpendicularity,
unsigned/oriented angles, concentricity, midpoint, point symmetry about a line
or point, fixing, and line/circle or circle/circle tangency.

Curve relations use supports. A point on a segment's supporting line may lie
beyond its endpoints; a point on an arc's supporting circle may lie outside
the arc span. A tangent endpoint connection combines coincidence and tangency.

Unsigned angles lie in `[0, pi]` and allow either sign. Oriented angles are
modulo one turn. Segment direction is start to end, so reversing endpoints
changes angle semantics. Horizontal, vertical, parallel and perpendicular
relations use locally unwrapped angle residuals to avoid stalling on an
initially orthogonal direction. Zero point distance emits two coincidence
residuals instead of a squared norm with a vanishing derivative.

Unsigned relation sides are selected from the editable seed for each solve.
Repeated solves start from the previous success; an explicit geometry update
can select the other side. Circle/circle tangency is different: an automatic
choice is captured when adding/replacing the constraint and retained until
explicit replacement. It compares internal and external residuals; ties choose
external contact. For internal contact the initially larger circle contains
the other, with the first argument breaking equal-radius ties. Callers can
also specify which circle contains the other. Coincident circles are excluded
as an internal tangency because they have no isolated contact point.

## Numerical computation

Preparation builds independent components from geometric constraint references.
Each component is recentered and scaled using its initial extent and primitive
sizes. Positions, radii and dimensions share that length scale; angles remain
radians. Tolerances are checked in physical units.

With `SolveOptions::linear_reduction` enabled (the default), weighted coordinate
aliases eliminate affine equalities and captured constants before numerical
factorization. These include direct-point coincidences, alignments, signed
offsets, fixed parameters, radii, and lengths of aligned segments. Multiplicity
weights preserve the original minimum-motion metric. Public keys and original
equations remain separate; removing a constraint rebuilds the reduction and
restores its freedom. If projecting the seed would create invalid geometry,
that component uses its full parameter space instead.

Curve primitives are evaluated through one `Curve` interface: `point_at(t)` and
`derivative_at(t)`, differentiable through both `t` and the curve's shape. Arc
endpoints, segment directions and curve targets all use it. Circle relations
keep their closed forms (distance to centre, radius sums), which need no curve
parameter.

Geometric expressions carry sparse forward derivatives. Their arithmetic and
trigonometry use the chain rule; no finite differencing is used in resolution.
The independent centered-difference check lives in
`tests/solver/derivatives.rs`. Local derivatives are assembled into a dense
Jacobian per component.

Feasibility uses damped, minimum-norm SVD corrections, adaptive step bounds and
rejection of invalid trial geometry. SVD handles both underdetermined and
rank-deficient systems. The solver does not form normal equations or randomly
restart. It starts from the complete editable seed, including new and updated
objects. Linearizations and factorizations are rebuilt for each call.

Temporary objectives project onto the permanent Jacobian's null space. Each
trial then restores feasibility and is accepted only if the target error
improves. Permanent constraints and targets are not combined into one weighted
compromise. Positive target weights express priorities among positional targets.
Targets can reference characteristic points, fixed fractions of segments/arcs,
or native angles of circles. Their parameters are fixed for that call.

An unreachable target returns the best admissible candidate found, with its
remaining distance. Exhausting the target iteration budget is reported and
does not invalidate a feasible result. All permanent residuals are checked
again before publication, including a separate evaluation on the actual
denormalized world-coordinate values. This catches loss of precision at large
offsets rather than publishing a false success.

## Diagnostics and limits

`SolveReport` keeps local degrees of freedom, residuals, redundancy diagnostics,
target outcomes, chosen circle tangencies and iteration counts separate.
Temporary targets do not consume permanent DOFs. Rank comes from the scaled
Jacobian; it is a local linearized estimate, not a proof of finite mobility at
singular configurations.

Identical constraints with identical captured values produce a `Duplicate`
warning. Rows consumed by affine reduction are counted as independent relations.
Other row dependencies produce local or partial-local warnings, not
advice to delete a nonlinear relation. Every original constraint is retained
and rechecked. Compatible redundancy and underconstraint are successful cases.

`Conflict` establishes directly contradictory dimensions, fixed positions or
fixed curve orientations.
Other unsolved components return `NonConvergence`, identifying their constraints
and residuals. Non-convergence is not a global infeasibility proof and the
identified component is not claimed to be a minimal conflicting set. Invalid
references and input data use structured `EditError`/`SolveError` values.

Dense per-component factorizations favor a verifiable numerical reference.
Costs depend on connectivity, not just primitive count. The connected-chain
scenario in `tests/solver/scale.rs` exercises this distinction. Large connected
systems and badly conditioned cases require measuring the actual workload;
the API provides iteration budgets and separate linear/angular/rank tolerances.

## Validation

The integration scenarios in `tests/solver/` cover the constraint catalog,
underconstraint and compatible redundancy, target priority, independent endpoint
identity, atomic edits/publication, stale keys, captured fixed frames, repair after
failure, scale changes, and output rounding. The private derivative test compares
the geometric Jacobians to centered differences independently of the evaluator.

On this Windows development machine, one optimized run on 2026-10-22 of
`cargo test --release --test solver scale:: -- --nocapture` measured:

| Connected horizontal chain | Solve | Move one endpoint |
|---|---:|---:|
| 100 length-constrained segments | 1.15 ms | 1.17 ms |
| 300 length-constrained segments | 4.63 ms | 4.88 ms |

The scenario checks all lengths and the two remaining translation DOFs. These
highly reducible chains measure affine reduction, not arbitrary nonlinear CAD
workloads. Measurements are observations, not timing assertions in the tests.

## Code boundaries

- `system.rs`: edits, captured choices, revisions and atomic publication.
- `keys.rs`, `primitives.rs`, `constraints.rs`: identity and problem representation.
- `solution.rs`, `error.rs`, `options.rs`: outputs and numerical contract.
- `numerical/prepare.rs`: components and scaled parameter layouts.
- `numerical/reduce.rs`: weighted affine aliases and constant elimination.
- `numerical/differentiate.rs`, `evaluate.rs`: sparse derivatives and residuals.
- `numerical/curve.rs`: segments, circles and arcs as differentiable maps of their own parameter.
- `numerical/solve.rs`: feasibility, targets, rank and diagnostics.

The module reuses NGK geometry types and existing dependencies. Its parametric
constraint representation keeps segment length, circle radius and arc sweep as
explicit variables; it does not replace NGK's curve types with a NURBS copy.
