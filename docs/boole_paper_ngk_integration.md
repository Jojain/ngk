# BOOLE and ngk: paper summary and integration roadmap

## Scope

This note summarizes **“BOOLE: A Boundary Evaluation System for Boolean
Combinations of Sculptured Solids”** by S. Krishnan, D. Manocha, M. Gopi,
T. Culver, and J. Keyser, then compares its design with the Boolean work that
currently exists in ngk.

The paper is treated only as a technical source. Its implementation choices are
not instructions for this repository. The comparison below is based on the
current ngk worktree, including the in-progress `src/builders/boolean` module.

## Executive summary

BOOLE converts a CSG tree into a manifold boundary representation made of
trimmed rational Bézier patches. For every binary Boolean operation, it:

1. rejects most non-intersecting surface-patch pairs;
2. computes and trims all surface/surface intersection curves;
3. joins curve pieces into complete spatial intersection loops;
4. partitions both input boundaries along those loops;
5. groups the resulting surface fragments into connected components whose
   inside/outside status is uniform;
6. classifies one component by ray shooting and propagates the result through
   a component-adjacency graph;
7. selects fragments according to union, intersection, or difference; and
8. rebuilds the topology of the result.

Its most valuable lesson for ngk is architectural, not symbolic: **a Boolean is
not complete after computing an intersection curve or even after imprinting
that curve**. The algorithm needs two different connectivity structures:

- the **common intersection network**, which says where the two operands meet;
- a **boundary-fragment adjacency graph**, built after imprinting, which says
  how the newly created face fragments are connected and how classification
  crosses the intersection.

ngk now has a transactional solid Boolean pipeline with a separate fragment
graph, deterministic planar classification, operation selection, and canonical
span sewing. Transverse box operations and nested cavities are covered end to
end. The public result contains one solid; empty and disconnected outcomes are
errors with rollback. Curved classification, complete coplanar arrangements,
and fully certified network finalization remain unfinished. Surface/surface
coverage and quality diagnostics prevent incomplete solver results from being
used as a solid Boolean.

For targeted reading of the source paper, the essential sequence is PDF pages
13–21 (representation and the six algorithm stages), pages 22–24 (software
layers), pages 25–29 (robustness and degeneracies), and page 39 (conclusion).
The parallel implementation and historical timings on pages 29–38 are useful
only after the geometric and topological contracts are understood.

## 1. What BOOLE represents

### 1.1 Input and output

The input is a CSG tree. Leaves are primitive solids and internal nodes are
regularized union, intersection, or difference operations. BOOLE evaluates the
tree bottom-up, producing a B-rep after every binary operation so that the
result can become an operand of the next one.

Each solid boundary is represented by trimmed tensor-product rational Bézier
patches. NURBS inputs can be decomposed into Bézier patches by knot insertion.
For each face, the system stores:

- the parametric surface;
- oriented trimming curves in its parameter domain;
- the corresponding spatial curves;
- adjacent faces for each edge; and
- the cyclic order of incident faces around vertices.

This is a conventional face/edge/vertex adjacency representation. ngk's GMap
is richer: its darts and involutions encode these incidences and orientations
directly rather than maintaining a separate ad-hoc adjacency graph.

### 1.2 Intersection-curve representation

In general, the intersection of two rational parametric surfaces is not itself
rationally parameterizable. BOOLE therefore uses more than one representation:

- an analytic implicit representation, the singular set of a bivariate matrix
  polynomial, for accurate tests and refinement;
- a traced piecewise-linear approximation for efficient traversal and initial
  guesses;
- two parameter-space representations, one on each surface; and
- a 3D spatial representation used to verify that merged curves close and for
  display.

That redundancy is deliberate. The polyline is an accelerator, not the source
of truth for topology. Endpoints found approximately are refined against the
analytic surfaces/curve before they are used to merge pieces.

## 2. The BOOLE algorithm

### Stage 1 — Candidate patch pairs

BOOLE first constructs an axis-aligned bounding box from each Bézier patch's
control points. Because a positive-weight rational Bézier patch lies in the
convex hull of its control points, disjoint boxes prove that the patches do not
intersect.

Pairs surviving that inexpensive test receive a stricter convex-hull
separation test formulated as a linear program. Only the survivors enter the
expensive narrow phase.

### Stage 2 — Complete surface/surface intersection

For each remaining pair, the paper's solver eliminates variables with Dixon
resultants and represents the projected intersection as a bivariate matrix
polynomial. Numerical linear algebra is then used to evaluate its singular
set.

The solver attempts to find every connected component:

- find starting points using curve/surface intersection and loop detection;
- decompose the surface domain so each subdomain contains at most one branch;
- isolate singularities by local optimization when branches are not separable;
- trace every branch to a user-specified tolerance.

The loop-detection method follows complex continuations of the algebraic curve
to find real closed components which do not touch a patch boundary. This is a
paper-specific symbolic-numeric solution to a fundamental problem: a marcher
seeded only from patch boundaries can silently miss an interior loop.

### Stage 3 — Trim and merge the curve pieces

The untrimmed intersection is clipped to the valid trimming regions of both
faces. BOOLE first intersects the traced approximations with trimming curves,
then refines the resulting points using the analytic representation and local
minimization.

The retained pieces are joined first within a patch and then across adjacent
patches. Under the paper's assumptions—closed, C0-continuous solids and a
regularized operation—the solid/solid intersection should form closed spatial
curves. BOOLE checks closure in 3D and retries or reports an error if it fails.

This closure check is a crucial checkpoint: it prevents incomplete numerical
curves from being mistaken for valid topology.

### Stage 4 — Partition boundaries and build components

The merged curves partition each operand boundary. BOOLE builds an undirected
graph from:

- the partitions produced inside each face;
- the original face/edge/vertex connectivity; and
- barriers and adjacency introduced by the intersection curves.

Connected components of that graph are maximal boundary pieces that lie
entirely inside or entirely outside the other solid. The paper calls them
**orientation-invariant components**.

It then builds another adjacency graph whose vertices are these components.
Crossing a suitable intersection boundary relates the classifications of two
neighbouring components. This permits classification propagation.

### Stage 5 — Classify components

A representative point is chosen on one component. A semi-infinite ray is
intersected with every trimmed patch of the other closed, non-self-intersecting
solid. Odd parity means inside; even parity means outside.

The implementation:

- rejects ray/surface hits outside a face's trimming region using a
  triangulation and planar point location;
- merges duplicate 3D hits where the ray crosses a shared patch boundary;
- shoots multiple random rays to reduce the risk of tangencies and numerical
  ambiguity; and
- propagates the resulting state through the component graph, avoiding one
  expensive ray test per component.

The paper reports one ray-shooting classification per solid per operation in
the ordinary case, followed by graph propagation.

### Stage 6 — Select and rebuild

The operation and classification decide which components survive:

| Operation | Keep from A | Keep from B |
|---|---|---|
| `A union B` | outside B | outside A |
| `A intersection B` | inside B | inside A |
| `A difference B` | outside B | inside A, with reversed boundary orientation |

The final B-rep connectivity is reconstructed from the original topology and
the intersection curves. The result can then be used in the next CSG-tree
operation.

Coincident faces and lower-dimensional contacts need additional policies; the
simple table alone is not sufficient for those cases.

## 3. Robustness and degeneracies in the paper

BOOLE combines symbolic formulations with floating-point numerical routines.
It uses QR, LU, SVD, inverse iteration, local minimization, triangulation,
point location, bounding boxes, and linear programming. Condition estimates
from numerical linear algebra are used to detect suspect results.

The paper identifies two recurring failure mechanisms:

1. inaccurate inversion/refinement of points used to trim and merge curves;
2. incorrect orientation or inside/outside predicates near a boundary.

For near-boundary classification, it evaluates the sign of a determinant
associated with the matrix-polynomial curve. SVD supplies an error bound; when
the bound contains zero, floating point cannot certify the answer and exact
rational arithmetic is required.

The authors identify determinant signs, point-versus-curve orientation, and
component classification as good filtered-predicate boundaries: use fast
floating point first and exact arithmetic only when the result is uncertain.
At the time of the paper, that exact module was not yet integrated into BOOLE.

The discussed degeneracies include:

- surfaces touching at an isolated point;
- tangential intersection along a curve;
- coincident surfaces with a two-dimensional overlap;
- a face touching an edge;
- vertex/face and edge/edge coincidences;
- four or more surfaces meeting at one point.

Some are detected, some change propagation rules, and some still rely on
tolerance-based equality. The paper should therefore not be read as a complete
modern solution to every degenerate Boolean.

### Important limitations to avoid copying

- Its published implementation still uses global/manual tolerances in places.
- Its suggested tracing step (`0.03` in parameter space) is unrelated to model
  scale or geometric error.
- Multiple random rays reduce probability of failure but do not certify a
  classification.
- The exact-arithmetic fallback was planned rather than integrated.
- The algorithm assumes manifold, closed, consistently oriented,
  non-self-intersecting input solids for its main path.
- Its shared-memory scheduling details are historically interesting, but Rust
  task parallelism should be introduced only after deterministic geometry and
  topology are established.

## 4. Current ngk state

### What already aligns well

The current code has a promising separation of concerns:

- operand resolution/import in [`operand.rs`](../src/builders/boolean/operand.rs);
- a broad-phase boundary in [`broad_phase.rs`](../src/builders/boolean/broad_phase.rs);
- narrow-phase observations in [`contacts.rs`](../src/builders/boolean/contacts.rs);
- a canonical common network in [`graph.rs`](../src/builders/boolean/graph.rs);
- network-driven subdivision input in [`imprint.rs`](../src/builders/boolean/imprint.rs);
- public preparation and lineage types in [`result.rs`](../src/builders/boolean/result.rs);
- atomic mutation through a GMap transaction in
  [`mod.rs`](../src/builders/boolean/mod.rs#L240).

The `IntersectionNetwork` already distinguishes:

- canonical **events**: remarkable points with operand-local incidences;
- canonical **spans**: curve sections with a 3D curve and edge/face-local uses;
- **regions**: two-dimensional overlaps.

This is conceptually stronger than BOOLE's loosely described collection of
curve pieces. A span can carry the paired 3D curve and pcurves needed to imprint
both operands consistently. The GMap also provides transactions, orbit-based
cells, formal sewability, and manifold/orientation validators. These are strong
foundations for the reconstruction stage.

The preparation pipeline is real rather than a stub: it computes contacts,
normalizes face-imprint chains, builds the network, splits edges and faces from
that network, and records source-to-fragment lineage. Existing tests cover
isolated contacts, perpendicular planar faces, overlapping solids, coplanar
partial overlap, external-tool import, and rollback.

### What is incomplete or unsafe for a CAD Boolean

#### 4.1 Broad phase is exhaustive

[`candidate_face_pairs`](../src/builders/boolean/broad_phase.rs#L10) currently
returns the Cartesian product of the two face sets. This is correct as a
placeholder but gives no pruning and becomes prohibitive for real solids.

#### 4.2 General surface intersection is a mesh approximation

[`surface_surface.rs`](../src/geometry/dim3/intersections/surface_surface.rs#L26)
converts both surfaces to NURBS, samples each into a fixed grid, tests every
triangle from one grid against every triangle from the other, deduplicates the
hit points, then sorts all points along the coordinate axis with the greatest
range.

Consequences:

- small branches and closed loops can be missed;
- distinct branches can be merged into one point list;
- a folded or non-monotone branch can be ordered incorrectly;
- singular and tangent intersections are not reliably characterized;
- the reported curve is not refined to lie on both analytic surfaces;
- cost is quadratic in the number of sampled triangles per face pair;
- the method cannot provide a geometric error certificate.

The coincident-surface test is also sample-based. It can confuse local
coincidence with complete supporting-surface coincidence.

This is the largest blocker to a correct curved-solid Boolean.

#### 4.3 Trimming currently filters samples instead of splitting branches

For general face pairs, [`contacts.rs`](../src/builders/boolean/contacts.rs#L579)
projects each sampled 3D point to both faces and keeps only the samples inside
both trim regions. The surviving samples are then interpolated as one curve.

If a branch leaves a trimmed face and later re-enters it, filtering removes the
outside samples but can reconnect the two valid pieces across the invalid gap.
Correct clipping must compute ordered intersections with every trim boundary
and split the branch into intervals before applying inside/outside tests.

#### 4.4 The common network is only structurally validated

[`IntersectionNetwork::validate`](../src/builders/boolean/graph.rs#L131) checks
that events and spans have uses, endpoint IDs exist, and the 3D curve meets its
endpoints. It does not yet check:

- that every transverse face/face span has a use on both operands;
- that its two pcurves evaluate to the same 3D locus;
- that spans are split at every interior crossing, trim crossing, singularity,
  or change of supporting cells;
- that branch valences are valid for the recorded contact kind;
- that ordinary solid/solid transverse intersections form closed loops;
- that duplicate or partially overlapping spans have been resolved;
- that every overlap region has a valid oriented boundary.

In fact, region boundaries are currently created empty in
[`mod.rs`](../src/builders/boolean/mod.rs#L179).

#### 4.5 Imprinting is preparation, not Boolean evaluation

`BooleanPreparation` remains available for split-only callers. The additive
`boolean` entry point returns `BooleanResult` for union, intersection, and
difference. The `neighborhood`, `classify`, `select`, and `assemble` modules
provide fragment adjacency, planar polygon classification, operation selection,
tool-face reversal for subtraction, deletion, identity-based sewing, and
validated result registration. General coincident/tangent arrangements and
curved certification remain pending.
### Comparison matrix

| Concern | BOOLE | ngk now | Required direction |
|---|---|---|---|
| Surface representation | Trimmed rational Bézier patches | Analytic surfaces normalized to NURBS, faces with pcurves | Stay NURBS-first; decompose into Bézier spans internally where convex-hull bounds are useful |
| Topology | Explicit face/edge/vertex adjacency graph | GMap cells, darts, involutions, typed views | Use GMap as the B-rep authority; do not duplicate its adjacency permanently |
| Broad phase | Control-point AABB, then convex-hull LP | Deterministic BVH over planar trim and native NURBS Bézier hulls; unbounded fallback | Conservative per-face/per-span bounds and a BVH; optional hull-separation refinement |
| Surface intersection | Complete symbolic-numeric tracing with loop/singularity detection | Synchronized branches with explicit certification and coverage status | Adaptive NURBS subdivision, seed isolation, analytic refinement, branch tracing, loop and singularity handling |
| Trimmed intersection | Curve/trim intersection followed by analytic refinement | Exact pcurve crossings and synchronized interval trimming; adaptive winding for holes | Split at exact/refined trim crossings; retain ordered valid intervals |
| Common curve | 3D + both parameter domains + analytic/polyline forms | Canonical event/span network with 3D curve and pcurve uses | Preserve this model; strengthen finalization and validation |
| 2D overlap | Detected as a two-dimensional intersection | Planar convex overlay; general overlap candidates retained as unresolved diagnostics | Add oriented region boundaries and general overlap policy |
| Boundary partition | Patch-domain partitions and connectivity graph | Transactional splitting; original imprint indices, directed intervals, and face-produced span/edge lineage | Complete arrangements and guarantee identical subdivision on both operands |
| Component graph | Explicit orientation-invariant components and adjacency | Separate post-imprint adjacency with known span barriers | Add a post-imprint fragment graph derived from GMap incidence plus network barriers |
| Classification | Ray shooting plus propagation | Deterministic planar polygon ray classification with two agreeing rays; explicit ambiguity; no propagation yet | Certified point-on-solid classifier, local sector rules, propagation with ambiguity states |
| Selection | Operation table over classified components | Union/intersection/difference table including on-boundary states | Add explicit regularized Boolean policy including `OnBoundary` cases |
| Assembly | Rebuild adjacency from source topology and common curves | Transactional deletion, span-identity sewing, shell discovery and validated single-solid registration | Sew by canonical network identity, then validate closure, GMap axioms, manifoldness, and orientation |
| Robustness | Stable numerics, checkpoints, planned exact fallbacks | Operation-scoped tolerances, coverage rejection, rollback and structural validation | Operation-scoped tolerance context, residual/error checks, filtered predicates, deterministic diagnostics |

## 5. Recommended ngk architecture

The existing directory split is appropriate. Complete it as follows:

```text
src/builders/boolean/
    mod.rs              orchestration and public entry points only
    operand.rs          import, validation, normalization, tolerance scale
    broad_phase.rs      conservative face/span bounds and candidate index
    contacts.rs         narrow phase and canonical geometric observations
    graph.rs            finalized common intersection network
    imprint.rs          planned and transactional subdivisions
    neighborhood.rs     post-imprint sectors and fragment adjacency
    classify.rs         point/solid classification and propagation
    select.rs           regularized union/intersection/difference rules
    assemble.rs         deletion/copying, reversal, sewing, shell registration
    result.rs           public result, lineage, diagnostics
    errors.rs           stage-specific failures with geometric context
```

The geometry solvers should remain under `geometry::dim3::intersections`; the
Boolean layer should orchestrate and interpret them, not own generic
curve/surface mathematics.

### 5.1 Keep two separate graphs

#### `IntersectionNetwork` — geometry shared by the operands

Its job is to answer: **where and how do A and B meet?**

An event should occur at every place where continuation changes:

- branch endpoint at a trim boundary;
- crossing or branch point;
- tangency or singular point;
- entry/exit of an overlap region;
- transition from one source face/edge to another.

A span must be indivisible between two events and contain:

- one canonical 3D curve interval;
- one synchronized pcurve interval on every supporting face;
- orientation and contact type;
- residual/error information;
- incidences to both operand sides when applicable.

A closed loop with no natural endpoint still needs representable topology. The
simplest policy is to insert one deterministic anchor event and represent the
loop as a cyclic span sequence, rather than dropping it because its endpoints
coincide.

#### `FragmentGraph` — classified pieces after imprinting

Its job is to answer: **which pieces of each boundary belong to the result?**

Suggested conceptual types:

```rust
struct BoundaryFragment {
    side: BooleanSide,
    face: FaceKey,
    source_face: FaceKey,
}

enum FragmentRelation {
    SameSide,      // ordinary source-topology adjacency
    CrossesSpan,   // classification may flip across a transverse span
    Tangent,
    Coincident,
}

enum RelativeLocation {
    Inside,
    Outside,
    OnBoundarySame,
    OnBoundaryOpposite,
    Ambiguous,
}
```

This graph should be temporary algorithm state derived from GMap incidence and
the finalized network. The permanent result topology remains the GMap.

### 5.2 Restrict the first public solid Boolean

The current preparation API accepts vertices, edges, profiles, faces, sheets,
and solids. That is useful as a general intersection/imprint facility, but the
first complete regularized Boolean should accept **two validated closed solids**
only. Lower-dimensional Boolean semantics can be designed later instead of
being accidentally inferred from solid rules.

A possible public shape is:

```rust
pub enum BooleanOperation {
    Union,
    Intersection,
    Difference,
}

pub fn boolean<P: Payload>(
    target_map: &mut GMap<P>,
    first: SolidKey,
    second: SolidKey,
    operation: BooleanOperation,
    options: BooleanOptions,
) -> Result<BooleanResult, BooleanError>;
```

The whole operation should remain one transaction, including import, imprint,
selection, removal, sewing, registration, and validation.

## 6. Prioritized implementation roadmap

### Phase 0 — Contracts and observability

Before adding more geometry, define invariants and diagnostics for every stage.

- Validate both operands as closed, manifold, consistently oriented solids.
- Introduce an operation-scoped tolerance context derived from model scale,
  with separate linear, angular, parameter, and residual tolerances.
- Record face-pair IDs, solver status, residual bounds, branch IDs, and reasons
  for ambiguous classification in diagnostics.
- Make deterministic ordering part of the contract so failures reproduce.

**Exit condition:** invalid operands and uncertain predicates fail explicitly;
no topology has been modified.

### Phase 1 — Make the intersection network complete

- Add a finalization pass which clusters events using geometry **and incidence**.
- Split spans at every event lying in their interior.
- Resolve duplicate, reversed, and partially overlapping spans.
- Preserve separate connected branches instead of sorting all hits globally.
- Support anchored closed loops.
- Populate and validate boundaries of overlap regions.
- Verify both pcurves against the canonical 3D curve at adaptive samples and
  with endpoint residuals.
- Add solid/solid loop and valence checks, with explicit exceptions for
  tangencies and overlaps.

**Exit condition:** the network is a valid noded arrangement shared by both
operands and can be inspected without mutating topology.

### Phase 2 — Replace sampled surface/surface intersection

An incremental implementation suited to ngk is preferable to immediately
recreating BOOLE's Dixon-resultant machinery:

1. decompose NURBS domains into Bézier spans at knots;
2. build conservative control-hull bounds and recursively reject disjoint
   span pairs;
3. isolate seed boxes for transverse branches;
4. refine seeds by solving `S_A(u,v) - S_B(s,t) = 0` with a constrained
   Newton/least-squares corrector;
5. trace each branch with adaptive predictor-corrector steps in synchronized
   `(xyz, uv_A, uv_B)` coordinates;
6. use geometric residual and curvature to control step size;
7. deduplicate complete branches by parameter-domain coverage;
8. add boundary seeding, closed-loop detection, tangent handling, and
   singular-point isolation.

The symbolic BOOLE representation remains a useful research reference, but
adaptive subdivision plus analytic correction fits ngk's existing NURBS
implementation and can be delivered in verifiable stages.

**Exit condition:** every returned point satisfies both surfaces within the
declared tolerance, distinct branches stay distinct, and adversarial tests show
that interior loops are not silently missed.

### Phase 3 — Correct trimmed clipping and imprinting

- Intersect each traced pcurve with all outer and inner trim loops.
- Refine and order every trim crossing in the curve parameter.
- Split into maximal intervals and classify interval midpoints in both trimmed
  domains.
- Insert the same canonical events and spans into both operands.
- Build complete 2D face arrangements, including multiple imprints, loops,
  holes, and crossings—not only one boundary-to-boundary chain.
- Apply all subdivisions transactionally and update network-to-fragment
  lineage.

**Exit condition:** after imprinting, each canonical span corresponds to
isomorphic boundary subdivisions on A and B. No filtered samples can bridge an
invalid trim interval.

### Phase 4 — Neighborhoods and classification

- Enumerate all post-imprint face fragments from lineage/GMap cells.
- Build `FragmentGraph` from GMap adjacency, with network spans acting as typed
  boundaries.
- Classify one safe representative per connected class using a reusable
  point-on-solid query.
- Make ray hits topologically unique: a hit on a shared edge or vertex counts
  once according to a deterministic ownership rule.
- Reject tangent rays and retry deterministic directions; expose
  `Ambiguous` when no direction certifies the answer.
- Propagate only across relations whose local sector analysis proves whether
  the state stays the same or flips. Do not blindly flip across tangent or
  coincident spans.

**Exit condition:** every fragment has a certified relative location or the
operation stops before selection with a localized diagnostic.

### Phase 5 — Selection and assembly

- Implement the operation table for ordinary `Inside`/`Outside` fragments.
- Add explicit same-orientation/opposite-orientation policies for coincident
  faces.
- Reverse selected B-side faces for `A - B`.
- Remove rejected fragments while preserving lineage.
- Pair corresponding boundary edges by `IntersectionSpanId`, not by a fresh
  geometric proximity search.
- Verify isomorphic sewing orbits and sew with the existing GMap operation.
- Discover connected shells, determine outer versus inner shells, register the
  result solid, and remove regularized lower-dimensional remnants.
- Run GMap, closed-shell, manifold, and orientation validation before commit.

The GMap theory matters here: an i-sew identifies matching lower-dimensional
cells only when their sewing orbits are isomorphic. Therefore imprinting must
produce matching subdivisions; assembly must not try to compensate for
different edge segmentations with tolerance-based endpoint matching.

**Exit condition:** the transaction commits one valid solid and the public
result includes full lineage and diagnostics.

### Phase 6 — Degeneracies and filtered predicates

Add support in an explicit order:

1. disjoint and strict containment without intersections;
2. ordinary transverse closed curves;
3. isolated vertex/face and edge/edge contacts;
4. tangent curves which do not change ownership;
5. coplanar planar overlap with holes and non-convex boundaries;
6. coincident curved faces;
7. multi-branch singular events.

Centralize uncertain predicates. A predicate should return a sign plus an error
bound, and invoke a higher-precision/exact fallback only when zero lies inside
that bound. This is the modern form of BOOLE's recommended checkpointing.

### Phase 7 — Performance and parallelism

Only after deterministic correctness:

- cache bounds per Bézier span;
- use a BVH or sweep-and-prune for face candidates;
- parallelize independent candidate-pair narrow phases;
- merge observations deterministically before canonicalization;
- parallelize independent ray/surface queries;
- profile before adding convex-hull LP separation.

Do not mutate the GMap from parallel workers. Geometry workers should emit
immutable observations; network finalization and topology mutation should stay
ordered and transactional.

## 7. Required test strategy

Each phase needs tests at three levels.

### Geometry-level

- residual on both surfaces for every curve sample;
- paired pcurve/3D consistency;
- closed loops entirely inside two patch domains;
- multiple branches and non-monotone branches;
- near-tangent and singular intersections;
- trim exit/re-entry without false reconnection;
- model-scale variations and reparameterized surfaces.

### Network/imprint-level

- all crossings are noded;
- canonical events merge only with compatible incidences;
- reversed observations produce one span with aligned uses;
- every ordinary face/face span has both operand uses;
- loop closure and valence invariants;
- bounded overlap regions;
- identical split counts on paired boundaries;
- rollback on any failed split or sew.

### End-to-end solid-level

For union, intersection, and both differences:

- disjoint boxes;
- one box strictly inside another;
- overlapping boxes;
- shared face, shared edge, and shared vertex;
- box/cylinder and cylinder/cylinder transverse cases;
- tangent cylinders/spheres;
- through-hole subtraction;
- nested shells/cavities;
- repeated operations forming a small CSG tree.

Every successful result should satisfy:

- GMap axioms;
- closed shell(s);
- manifold incidence;
- consistent orientation;
- no unintended duplicate boundary faces;
- operation-specific point membership checks;
- stable topology under harmless scale and parameterization changes.

## 8. Recommended immediate next milestone

The next milestone should **not** be `select.rs`. Selection built on the current
sampled intersection would make incorrect topology look complete.

The most useful vertical slice is:

1. closed, oriented solid operands only;
2. conservative face-pair bounding boxes;
3. planar/planar intersections using the existing exact line construction;
4. finalized, two-sided intersection loops;
5. complete planar face arrangements;
6. fragment graph and deterministic point classification;
7. union/intersection/difference selection;
8. GMap sewing and result validation.

This produces a correct polyhedral/planar solid Boolean through the same public
pipeline intended for curved surfaces. The sampled curved solver can remain
available for visualization or diagnostics, but should return an explicit
“approximate/not suitable for topology” quality until Phase 2 replaces it.

After that vertical slice is green, curved NURBS intersection can be integrated
without redesigning classification and assembly.

## Conclusion

BOOLE validates the overall architecture already emerging in ngk: isolate
candidate pairs, compute a common curve in both surface domains, canonicalize
it globally, imprint both operands, classify connected boundary components,
select, and assemble.

ngk's GMap and transaction system give it a better topological foundation than
the paper's explicit adjacency structures. The current `IntersectionNetwork`
is also the correct place to make the two operands agree before mutation.

The decisive work still required is:

1. a complete and residual-controlled NURBS surface-intersection engine;
2. exact branch clipping against trimmed domains;
3. stronger network finalization and invariants;
4. complete sectors and barriers in the fragment graph;
5. certified classification and propagation;
6. complete coincident/tangent assembly and original-edge span realization;
7. filtered predicates and explicit degeneracy policies.

If those stages remain separate and every transition has a checkable contract,
ngk can adopt the durable ideas from BOOLE without inheriting its dependence on
fixed parametric steps, random classification as proof, or globally tuned
tolerances.
