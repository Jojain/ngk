# NURBS surface/surface intersection implementation plan

- Status: **In progress**
- Owner: intersection refactor branch
- Target subsystem: `geometry::dim3::intersections`
- Primary consumer: `builders::boolean`

## 1. Objective

Replace ngk's current tessellation-based surface/surface intersection with an
adaptive NURBS-first solver that constructs complete intersection branches
directly against the analytic surfaces.

For two surfaces

\[
A(u,v), \qquad B(s,t),
\]

the solver must trace the solution set of

\[
A(u,v)-B(s,t)=0
\]

and return synchronized representations of every branch:

```text
3D curve: C(lambda)
A pcurve: (u(lambda), v(lambda))
B pcurve: (s(lambda), t(lambda))
```

The output must be suitable for building the canonical Boolean intersection
network. A sampled polyline may assist search and tracing, but it must not be
treated as the geometric or topological source of truth.

## 2. Why this work is required

The current implementation in
`src/geometry/dim3/intersections/surface_surface.rs`:

1. samples each NURBS surface onto a fixed grid;
2. triangulates both grids;
3. intersects every triangle pair;
4. deduplicates the resulting 3D points; and
5. sorts all points along the coordinate with the greatest range.

That implementation is useful as a prototype and possible seed generator, but
it cannot provide the guarantees needed by a CAD Boolean:

- small branches and interior closed loops can be missed;
- separate branches can be combined;
- folded or non-monotone curves can be ordered incorrectly;
- returned points are not necessarily refined onto both analytic surfaces;
- tangencies and singularities are not reliably distinguished;
- fixed sampling has no geometric error certificate;
- surface coincidence is inferred from a limited set of samples.

## 3. Scope

### Included

- conversion of all supported analytic surfaces to NURBS;
- decomposition of NURBS surfaces into rational Bézier spans;
- conservative candidate pruning from control hulls;
- adaptive parameter-domain subdivision;
- transverse branch seed isolation and refinement;
- predictor-corrector tracing in the four-dimensional parameter space;
- synchronized 3D curve and two pcurves;
- adaptive step control and residual validation;
- detection and representation of closed loops;
- boundary events between neighbouring Bézier spans;
- explicit results for isolated points, tangent/singular contacts, and
  two-dimensional overlap candidates;
- diagnostics sufficient to explain incomplete or uncertain results;
- integration with Boolean trimmed-face clipping and `IntersectionNetwork`.

### Initially excluded

- a complete symbolic resultant implementation like BOOLE;
- guaranteed exact rational answers for general intersections;
- full coincident curved-face region construction in the first milestone;
- topology mutation inside the geometry solver;
- parallel execution before deterministic correctness;
- special analytical plane/cylinder/sphere optimizations.

Direct analytical algorithms may be added later, but the first production path
must remain NURBS-first.

## 4. Required guarantees

The production solver must satisfy the following contracts.

### 4.1 Geometric validity

Every accepted branch sample must satisfy:

\[
\|A(u,v)-B(s,t)\| \leq \varepsilon_{residual}.
\]

The reported 3D point must be derived consistently from the two corrected
surface evaluations, rather than from an unrelated triangle intersection.

### 4.2 Synchronized parameterization

For a shared branch parameter `lambda`, all three representations refer to the
same location:

```text
curve_3d(lambda)
surface_a(pcurve_a(lambda))
surface_b(pcurve_b(lambda))
```

Their pairwise distances must remain within the declared tolerance.

### 4.3 Branch identity

Distinct connected branches remain distinct. A branch owns an ordered sequence
of corrected parameter-space states; global sorting by a Cartesian coordinate
is forbidden.

### 4.4 Completeness reporting

The solver must never silently present an uncertified partial trace as a
complete intersection. It must return either:

- all branches certified within the implemented case contract; or
- an explicit incomplete/ambiguous diagnostic identifying the unresolved
  Bézier-span pair and reason.

### 4.5 Determinism

Identical input and options must produce the same branches, orientations,
anchors, and diagnostics. Parallel scheduling must not affect canonical output.

### 4.6 No topology mutation

The geometry layer returns immutable observations. Boolean network
canonicalization, trimming, imprinting, and GMap mutation remain in
`builders::boolean`.

## 5. Proposed module architecture

Keep the public façade at the current location and split the implementation by
responsibility:

```text
src/geometry/dim3/intersections/
    surface_surface.rs             public orchestration
    surface_surface/
        bezier_spans.rs            knot-span decomposition and local domains
        bounds.rs                  conservative control-hull bounds
        candidates.rs              adaptive span-pair subdivision
        seeds.rs                   boundary/interior seed isolation
        corrector.rs               constrained Newton/SVD correction
        tracer.rs                  predictor-corrector branch tracing
        loops.rs                   loop closure and interior-loop search
        singularities.rs           rank loss and tangent classification
        fitting.rs                 synchronized 3D/2D curve approximation
        validation.rs              residual and coverage certification
        diagnostics.rs             internal trace diagnostics
```

Do not force this exact file split on day one. Introduce a submodule when a
responsibility has a stable contract and enough implementation to justify it.
Avoid a new monolithic `surface_surface.rs`.

Boolean-specific clipping belongs in:

```text
src/builders/boolean/contacts.rs
```

or in a later focused helper under `builders/boolean`, because trimming loops
belong to faces, not to unbounded geometric surfaces.

## 6. Proposed public data model

The current `SurfaceSurfaceIntersection::Curve { points: Vec<Point3> }` is not
sufficient. The replacement should preserve parameter-space states and solver
quality.

Conceptual API:

```rust
pub struct SurfaceIntersectionPoint {
    pub point: Point3,
    pub uv_a: Point2,
    pub uv_b: Point2,
    pub kind: SurfaceIntersectionPointKind,
    pub residual: f64,
}

pub struct SurfaceIntersectionBranch {
    pub curve_3d: Curve,
    pub pcurve_a: Curve2,
    pub pcurve_b: Curve2,
    pub start: SurfaceIntersectionPoint,
    pub end: SurfaceIntersectionPoint,
    pub closed: bool,
    pub kind: SurfaceIntersectionBranchKind,
    pub quality: IntersectionQuality,
}

pub enum SurfaceIntersectionBranchKind {
    Transverse,
    Tangent,
    Singular,
}

pub struct IntersectionQuality {
    pub max_residual: f64,
    pub max_fit_error: f64,
    pub certified: bool,
}

pub enum SurfaceSurfaceIntersection {
    Point(SurfaceIntersectionPoint),
    Branch(SurfaceIntersectionBranch),
    OverlapCandidate(SurfaceOverlapCandidate),
}
```

The exact Rust shape should be settled during milestone 0. In particular, a
closed branch should not be forced to pretend it has two geometrically distinct
endpoints. The Boolean graph can create a deterministic anchor event when it
turns the closed branch into spans.

## 7. Numerical model

Use one operation-scoped tolerance configuration rather than unrelated magic
constants.

Suggested categories:

```rust
pub struct SurfaceIntersectionOptions {
    pub linear_tolerance: f64,
    pub residual_tolerance: f64,
    pub parameter_tolerance: f64,
    pub angular_tolerance: f64,
    pub min_step: f64,
    pub max_step: f64,
    pub max_subdivision_depth: usize,
    pub max_corrector_iterations: usize,
    pub max_trace_steps: usize,
}
```

Requirements:

- validate every option before computation;
- derive sensible defaults from model scale where possible;
- do not compare UV distances directly with world-space tolerances;
- record which limit stopped subdivision or tracing;
- distinguish convergence failure from proven absence of intersection.

## 8. Mathematical algorithm

### 8.1 Normalize to NURBS

Convert both `Surface` values with the existing NURBS conversion path. Preserve
a mapping to the original surface domains for diagnostics and downstream
pcurves.

Reject invalid NURBS data before beginning:

- non-finite control points or weights;
- invalid knot vectors;
- empty parameter domains;
- unsupported weights for convex-hull pruning.

### 8.2 Decompose into rational Bézier spans

Insert knots until each non-empty knot interval is represented as a rational
Bézier patch. Each local span records:

- its control net;
- local `[0,1] x [0,1]` domain;
- mapping to the parent NURBS `(u,v)` domain;
- derivative evaluation;
- conservative 3D bound;
- stable span identity.

This decomposition supports convex-hull pruning and local subdivision without
changing the public surface representation.

### 8.3 Generate candidate span pairs

For every Bézier span pair:

1. compare conservative 3D bounds;
2. reject disjoint bounds;
3. optionally apply a stricter convex-hull separating-axis test;
4. estimate whether the pair is regular, tangent-like, coincident-like, or too
   broad to decide;
5. subdivide the less well-conditioned/larger patch when necessary.

Candidate generation must be conservative: false positives are acceptable;
false negatives are not.

### 8.4 Isolate seeds

Find possible intersections on the boundaries of each candidate pair by
solving curve/surface intersections on all Bézier edges. Merge only compatible
seed incidences.

Also search for branches with no boundary hit:

- retain unresolved interior boxes during subdivision;
- inspect critical points of a projection or distance function;
- use interval bounds or another conservative exclusion test;
- refine any interior box that cannot be proven empty.

The initial implementation may support boundary-seeded transverse branches
first, but it must report interior-loop coverage as unsupported until loop
search is implemented. It must not silently claim completeness.

### 8.5 Correct a seed on both surfaces

Represent a state as:

```text
x = (u, v, s, t)
F(x) = A(u,v) - B(s,t)
```

The Jacobian is the `3 x 4` matrix:

\[
J = [A_u\; A_v\; -B_s\; -B_t].
\]

Because the regular solution is a curve, add a fourth constraint fixing the
correction to a plane normal to the predicted parameter-space tangent. Solve
the resulting constrained system with QR or SVD.

The corrector must:

- keep parameters inside their active domains;
- use damping or line search when a full Newton step increases residual;
- return iteration count and conditioning information;
- reject convergence to a different isolated branch;
- stop with an explicit reason when rank or progress is insufficient.

### 8.6 Compute the branch tangent

At a regular state, find a unit vector `d` in the null space of `J`:

```text
d = (du, dv, ds, dt)
J d = 0
```

Orient it consistently with the preceding step. The corresponding 3D tangent
can be checked from either surface and against:

\[
N_A \times N_B.
\]

If the surface normals are nearly parallel or the Jacobian loses rank, route
the state to singular/tangent handling rather than continuing an ordinary
transverse trace.

### 8.7 Trace with an adaptive predictor-corrector

From a corrected seed, trace in both directions:

1. predict `x_next = x + h d`;
2. correct `x_next` onto `F(x)=0` with the normal-plane constraint;
3. evaluate residual and parameter-domain membership;
4. append the corrected synchronized state;
5. recompute and orient the tangent;
6. adapt `h` from curvature, correction size, residual, and conditioning;
7. stop at a classified terminal event.

Decrease the step when:

- Newton needs more iterations;
- the tangent changes rapidly;
- the residual approaches its limit;
- a Bézier boundary is close;
- rank conditioning worsens;
- another known event is close.

Increase it conservatively on well-conditioned, nearly straight sections.

Terminal events include:

- parameter-domain boundary;
- neighbouring Bézier-span transition;
- return to the starting seed with matching tangent;
- encounter with an already traced branch;
- tangent/singular state;
- minimum step reached;
- iteration or trace budget exhausted.

Budget exhaustion produces an incomplete diagnostic, never a valid truncated
branch.

### 8.8 Cross Bézier-span boundaries

When a trace reaches a knot-span boundary:

- snap/refine the event on the shared boundary;
- map it into the adjacent span's local domain;
- continue from the same parent-surface parameters;
- preserve one branch identity;
- prevent duplicate tracing from the neighbouring span pair.

The knot boundary is an implementation partition, not a topological break in
the final intersection curve unless surface continuity or contact type changes.

### 8.9 Detect closed loops

For a boundary-seeded trace, closure occurs when the corrected state returns to
the seed within parameter and spatial tolerances with a compatible tangent and
after sufficient travelled length.

Completeness also requires discovering loops without boundary seeds. Implement
this through conservative unresolved-box coverage rather than random sampling.

Assign every closed loop a deterministic anchor, for example the
lexicographically smallest corrected parent-domain state after canonical
orientation.

### 8.10 Handle tangencies and singularities

Detect suspect states using:

- small `|N_A x N_B|`;
- small singular values of `J`;
- unstable/null-space dimension;
- corrector stagnation;
- multiple traces converging to one state.

Initially return a localized `Tangent` or `Singular` observation and stop
ordinary tracing. Later milestones can add:

- tangent-curve continuation;
- isolated touch-point classification;
- branch splitting at multi-valence singularities;
- local sector analysis for Boolean propagation.

### 8.11 Detect overlap candidates

Do not classify complete surface coincidence from a few samples. Use
subdivision and derivative/normal agreement to identify a candidate
two-dimensional common region.

The first implementation may return `OverlapCandidate` with supporting domain
boxes and a diagnostic requiring Boolean overlap resolution. Full overlap
boundaries can be implemented after transverse branches are reliable.

### 8.12 Fit synchronized curves

Tracing produces ordered corrected states:

```text
(point_3d, uv_a, uv_b)
```

Fit or interpolate the three curves using one shared normalized parameter set,
preferably chord length from the 3D samples. Fit adaptively until all of these
are below tolerance:

- 3D fit error against corrected samples;
- pcurve fit error in each parameter domain;
- evaluated distance from the fitted 3D curve to each surface evaluated at its
  fitted pcurve;
- tangent discrepancy where continuity is required.

Retain corrected trace samples internally when needed to validate or refit a
branch. A visually smooth spline is not sufficient evidence of correctness.

After fitting, an optional simplification pass (enabled by default) attempts to
replace the fitted curves with supported analytical representations. Each curve
dimension uses an ordered recognizer pipeline so new analytical types can be
added without changing the fitting algorithm. The initial recognizers support
3D lines and circles and two-dimensional lines and circles.

The recognized 3D curve defines the common normalized parameterization (for
example projected distance for a line or unwrapped angle for a circle). A
pcurve is simplified only when its analytical representation evaluates to its
corrected UV samples using that same parameter. The complete proposed 3D/pcurve
triple is then checked with the normal adaptive fit and surface-consistency
validation. Failed recognition or validation is non-fatal: the solver retains
the corresponding synchronized NURBS representation, and a failed triple-level
validation restores the original synchronized NURBS triple atomically.

### 8.13 Validate coverage and output

Before returning:

- ensure all candidate boxes were rejected, assigned to a branch, or reported
  unresolved;
- deduplicate branches by parameter-domain coverage, not only 3D proximity;
- validate every branch endpoint and closed-loop anchor;
- validate synchronized curve residual adaptively;
- orient branches deterministically;
- sort results by stable domain-based keys;
- include maximum residual, fit error, and certification state.

## 9. Boolean integration

The unbounded surface solver returns complete supporting-surface branches. The
Boolean layer then clips each branch against both trimmed faces.

Required clipping sequence:

1. intersect `pcurve_a` with every outer and inner loop of face A;
2. intersect `pcurve_b` with every outer and inner loop of face B;
3. refine every crossing against the synchronized surface branch;
4. express all crossings using the common branch parameter;
5. sort and deduplicate crossings with incidence-aware rules;
6. split the branch at every crossing;
7. classify the midpoint of each interval in both trimmed domains;
8. retain intervals valid in both faces;
9. submit the retained intervals, pcurves, and endpoint incidences to
   `IntersectionNetworkBuilder`.

Never filter trace samples and reconnect the survivors. Leaving and re-entering
a trimmed region must produce separate spans.

The finalized `IntersectionNetwork` should validate:

- both face uses exist for every transverse face/face span;
- both pcurves agree with the 3D curve;
- spans are split at every trim crossing, singularity, and support transition;
- ordinary transverse solid/solid branches form closed networks;
- overlap regions have explicit oriented boundaries;
- no unresolved geometry is passed to topology mutation.

## 10. Milestones and tracking checklist

### Milestone 0 — Contracts and API

- [x] Replace the point-list curve result with a branch-oriented API.
- [x] Define point, branch, contact-kind, quality, and diagnostic types.
- [x] Separate search, residual, fit, and UV tolerances.
- [ ] Define deterministic branch orientation and closed-loop anchoring.
- [x] Decide how incomplete coverage is represented in `Result`/errors.
- [x] Update existing callers and tests in the same API-breaking change.

Exit criterion: the API can describe correct future results without losing UV,
branch, contact-type, or quality information.

### Milestone 1 — Bézier decomposition and conservative pruning

- [x] Implement surface knot insertion/decomposition into rational Bézier
      spans.
- [x] Preserve exact mappings between local and parent parameter domains.
- [x] Implement conservative bounds from homogeneous/control-net data.
- [ ] Add recursive span-pair subdivision.
- [ ] Prove candidate pruning cannot discard a valid transverse intersection
      for supported weight configurations.
- [x] Add diagnostics for unsupported/non-convex weight cases.

Exit criterion: test surfaces produce a deterministic conservative set of
candidate parameter boxes.

### Milestone 2 — Seed refinement and transverse tracing

- [x] Implement surface derivatives required by `J`.
- [ ] Implement SVD/QR-based null-space tangent calculation.
- [x] Implement constrained Newton correction with damping.
- [x] Implement boundary seed generation.
- [x] Trace regular transverse branches in both directions.
- [ ] Cross neighbouring Bézier spans without duplicating branches.
- [x] Add adaptive step control and termination diagnostics.

Exit criterion: simple plane/plane, plane/cylinder, and transverse curved
examples return ordered synchronized states with bounded residual.

### Milestone 3 — Curve fitting and certified branch output

- [x] Fit synchronized 3D and two-dimensional curves with one parameterization.
- [x] Recognize and validate supported analytical 3D curves and pcurves, with
      synchronized NURBS fallback.
- [x] Measure adaptive fit and surface-consistency errors.
- [ ] Refine trace samples or fitting knots when validation fails.
- [ ] Canonically orient, anchor, and sort branches.
- [ ] Deduplicate by parameter-domain coverage.

Exit criterion: consumers no longer depend on raw unordered point lists.

### Milestone 4 — Loop completeness

- [ ] Detect closure of a traced branch robustly.
- [ ] Track coverage of candidate subdivision boxes.
- [ ] Implement conservative interior-loop seed search.
- [ ] Add tests containing small loops fully inside both patch domains.
- [ ] Fail explicitly if any candidate box remains unresolved.

Exit criterion: the supported regular case has no silent missed-loop path.

### Milestone 5 — Trimmed-face clipping

- [ ] Compute pcurve/trim-curve intersections for outer and inner loops.
- [ ] Refine trim crossings and unify their branch parameters.
- [ ] Split branches into maximal valid intervals.
- [ ] Preserve separate exit/re-entry intervals.
- [ ] Feed two-sided spans and events to `IntersectionNetworkBuilder`.
- [ ] Strengthen network residual, incidence, valence, and closure validation.

Exit criterion: Boolean preparation imprints the same canonical curve sections
on both operands without sample filtering.

### Milestone 6 — Tangencies, singularities, and overlap candidates

- [ ] Detect rank loss and near-parallel normals.
- [ ] Return isolated tangent points without inventing a branch.
- [ ] Split branches at singular/multi-valence events.
- [ ] Continue supported tangent curves or return a precise unsupported-case
      diagnostic.
- [ ] Replace sample-only coincidence detection with conservative overlap
      candidate isolation.
- [ ] Connect contact types to Boolean neighbourhood rules.

Exit criterion: degeneracies are either correctly represented or explicitly
rejected before imprinting.

### Milestone 7 — Performance and cleanup

- [ ] Add a hierarchy/BVH over conservative span bounds.
- [ ] Profile candidate generation, correction, tracing, and fitting.
- [ ] Parallelize independent span-pair searches into immutable observations.
- [ ] Preserve deterministic merge and branch ordering.
- [ ] Retain the old tessellation implementation only as an explicitly named
      approximate/debug seed strategy, or remove it once no longer useful.
- [ ] Document algorithm limitations and tuning guidance.

Exit criterion: production behavior is deterministic and topology-safe, with
performance measured on representative Boolean models.

## 11. Test plan

Tests must mirror the source tree under `tests/geometry/dim3/intersections/`.
Boolean integration tests remain under `tests/builders/`.

### 11.1 Bézier decomposition and bounds

- [ ] knot-span domain mappings round-trip;
- [ ] reconstructed span evaluations match the parent NURBS;
- [x] bounds contain dense independent evaluations;
- [x] disjoint control hulls are rejected;
- [ ] touching bounds remain candidates;
- [ ] unsupported weights produce explicit diagnostics.

### 11.2 Corrector

- [ ] converges from nearby seeds;
- [ ] reduces residual monotonically with damping;
- [ ] respects active domains;
- [ ] reports rank deficiency;
- [ ] rejects seeds outside convergence/branch safeguards;
- [ ] remains stable under model scaling.

### 11.3 Tracer

- [x] plane/plane straight line;
- [x] plane/cylinder closed branch;
- [ ] cylinder/cylinder multiple branches;
- [ ] non-monotone branch which cannot be sorted by one axis;
- [ ] branch crossing several Bézier spans;
- [ ] reverse tracing joins without duplicate seed points;
- [ ] adaptive step shrinks in high curvature;
- [ ] trace-budget exhaustion returns incomplete status.

### 11.4 Completeness and degeneracies

- [ ] small interior closed loop;
- [ ] two disjoint loops in one surface pair;
- [ ] isolated tangent point;
- [ ] tangent curve;
- [ ] branch point/singularity;
- [ ] coincident planar surfaces;
- [ ] coincident curved-surface candidate;
- [ ] near-coincident but distinct surfaces.

### 11.5 Synchronized curve fitting

- [x] fitted 3D and pcurves share parameterization;
- [x] adaptive residual is measured between trace samples;
- [x] closed branches remain closed after fitting;
- [x] analytical simplification is enabled by default and can be disabled;
- [x] perpendicular planes simplify to a 3D line and linear pcurves;
- [x] a plane/cylinder loop simplifies to a 3D circle and circular planar
      pcurve while retaining the nonlinear cylindrical NURBS pcurve;
- [ ] reversed input order preserves geometry and swaps pcurves;
- [ ] reparameterized but geometrically identical surfaces produce equivalent
      branches.

### 11.6 Boolean integration

- [ ] branch exits and re-enters a face trim without false reconnection;
- [ ] inner trimming loop creates the correct retained intervals;
- [ ] paired face spans use the same canonical event IDs;
- [ ] ordinary solid/solid transverse networks close;
- [ ] failed validation rolls back every topology edit;
- [ ] box/cylinder and cylinder/cylinder preparation produce matching splits
      on both operands.

### 11.7 Invariance/property tests

- [ ] rigid transformations preserve branch topology;
- [ ] uniform scaling with scaled tolerances preserves topology;
- [ ] swapping A and B preserves 3D branches and swaps pcurves;
- [ ] reversing a surface parameter direction preserves geometry;
- [ ] increasing search resolution does not delete certified branches;
- [ ] all accepted samples satisfy the residual contract.

## 12. Diagnostics required for development

Every failure should identify enough context to reproduce it:

- source surface IDs when supplied by the caller;
- Bézier span IDs and parent parameter boxes;
- candidate subdivision depth;
- seed origin and initial residual;
- corrector iterations, final residual, and smallest singular value;
- trace step index, size, and termination reason;
- unresolved candidate boxes;
- maximum fitting and synchronized-surface errors;
- contact classification and confidence/certification state.

Add optional debug extraction of:

- candidate boxes;
- seeds;
- corrected trace samples;
- tangent vectors;
- rejected/merged branches;
- final 3D curve and both pcurves.

Debug visualization must consume these observations; it must not influence the
solver's result.

## 13. Key risks and mitigations

### Missing interior loops

Risk: boundary-only seeding never sees a closed interior component.

Mitigation: conservative candidate-box coverage plus explicit unresolved state
until an interior-loop isolation method is implemented.

### Newton branch jumping

Risk: the corrector converges to a neighbouring branch.

Mitigation: small adaptive steps, normal-plane constraint, domain boxes,
distance-to-prediction bounds, and continuity checks on the tangent.

### Tangency mistaken for transverse intersection

Risk: an unstable tangent produces arbitrary topology.

Mitigation: monitor `N_A x N_B` and singular values; route rank-deficient states
to explicit degeneracy handling.

### Good-looking fit with bad surface residual

Risk: interpolation smooths the samples but leaves the true intersection.

Mitigation: validate fitted curves by re-evaluating both surfaces at their
pcurves between samples, then refine or reject.

### Tolerance coupling

Risk: world-space and UV comparisons use the same numeric tolerance.

Mitigation: separate tolerance dimensions and convert through local derivative
bounds where a relation is required.

### Premature topology mutation

Risk: incomplete branches partially split the operands.

Mitigation: certify and finalize the complete network before entering the GMap
transaction; preserve rollback for every downstream failure.

## 14. Definition of done

This plan is complete only when all of the following are true:

- [ ] the production surface/surface path does not derive topology from a fixed
      triangle grid;
- [ ] every returned branch includes synchronized 3D, A-UV, and B-UV curves;
- [ ] every returned branch has measured residual and fitting errors;
- [ ] multiple and non-monotone branches retain independent order;
- [ ] closed interior loops are found for the supported regular case;
- [ ] unresolved coverage is an error, not an omitted branch;
- [ ] trimmed clipping splits at crossings instead of filtering samples;
- [ ] tangent, singular, and overlap cases are typed explicitly;
- [ ] Boolean network validation rejects incomplete or one-sided branches;
- [ ] all geometry, network, rollback, and end-to-end tests pass;
- [ ] `cargo fmt`, `cargo clippy --all-targets --all-features`, and
      `cargo test --all-targets --all-features` pass, except for separately
      documented pre-existing environment or feature failures;
- [ ] algorithm limitations and diagnostics are documented for callers.

## 15. Recommended first implementation slice

Start with the smallest slice that establishes the correct architecture:

1. define the new branch-oriented result API;
2. implement Bézier decomposition and conservative bounds;
3. support regular transverse branches that reach candidate boundaries;
4. implement constrained correction and predictor-corrector tracing;
5. return synchronized trace samples with residuals;
6. fit synchronized curves;
7. recognize and validate supported analytical curves, falling back to the
   synchronized NURBS curves when simplification is not valid;
8. integrate them into the network without trimmed clipping changes yet;
9. keep an explicit `coverage_incomplete` result until interior-loop search is
   implemented.

This slice intentionally does not claim a production-complete Boolean. It
establishes the core solver and data contracts without hiding the loop,
trimming, or degeneracy work that remains.
