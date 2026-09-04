# Boolean evaluation implementation plan

- Status: **In progress — transverse planar solid evaluation implemented**
- Owner: `boolean` branch
- Target subsystem: `builders::boolean`
- Depends on: [NURBS surface/surface intersection](nurbs_surface_surface_intersection.md)
- Design references: `docs/boole_paper_ngk_integration.md`,
  `docs/boolean_algorithm_guide_fr.md`, `src/topology/edit.md`

## Implementation status (2026-09-04)

The concrete planar slice is implemented. See the checkpoint at the end for
verified cases and remaining requirements. The architectural inventory below
records the starting point of this plan.

## 1. Objective

Turn the existing Boolean *preparation* pipeline (contacts → canonical network →
two-sided imprinting) into a complete regularized solid Boolean that returns one
validated `SolidKey` for union, intersection, and difference.

The end state is one public entry point:

```rust
pub fn boolean<P: Payload>(
    map: &mut GMap<P>,
    first: SolidKey,
    second: SolidKey,
    operation: BooleanOperation,
    options: BooleanOptions,
) -> Result<BooleanResult, BooleanError>;
```

executed inside **one** `GMap::transaction`, so any failure at any stage leaves
the map exactly as it was.

This plan is explicit about the data structures each stage owns, where they
live, what they borrow from the GMap, and which of them are permanent.

## 2. Where the implementation stands

Implemented today (`src/builders/boolean/`, 2165 lines):

| File | Role | State |
|---|---|---|
| `mod.rs` | orchestration, raw observation enum, network build, split application | preparation only |
| `operand.rs` | operand → `OperandCells`, geometry presence checks, external import | usable, no solid validation |
| `broad_phase.rs` | `candidate_face_pairs` | Cartesian product placeholder |
| `contacts.rs` | vertex/edge/face narrow phase, planar overlap, chain normalization | sample-filtered trimming |
| `graph.rs` | `IntersectionNetwork` events/spans/regions + builder | structural validation only |
| `imprint.rs` | network → `edge_points` / `face_imprints` | loses span identity |
| `result.rs` | `BooleanPreparation`, `BooleanLineage` | no operation result |
| `errors.rs` | `BooleanError` | needs per-stage variants |

Missing entirely: `neighborhood.rs`, `classify.rs`, `select.rs`, `assemble.rs`,
`tolerance.rs`, `trim.rs`, `diagnostics.rs`.

Two facts about the ngk topology model shape everything below:

1. **A solid is a set of closed 2-sheets, not an α3-paired volume.**
   `SolidAttr { data, outer_shell: Dart, inner_shells: Option<Vec<Dart>> }`,
   and `validate_solid_manifold` only requires each shell to be `Closed`.
   Assembly therefore means *α2-sewing face fragments into closed shells*,
   never α3 work.
2. **`TopologyEdit::remove_isolated_darts` returns a `HashMap<Dart, Dart>`
   remap.** Every cached `Dart` becomes stale after a deletion pass. All
   algorithm state below is keyed by `FaceKey` / `EdgeKey` / `VertexKey` and
   resolves darts lazily; the only exception is the assembly stage, which
   re-resolves darts after each deletion batch.

## 3. Scope

### Included

- operand admission: two registered, closed, manifold, consistently oriented solids;
- an operation-scoped tolerance context replacing ad-hoc `LINEAR_TOLERANCE` use;
- a bounding-volume broad phase over face bounds;
- exact trimmed clipping of intersection branches (no sample filtering);
- network finalization: noding, span splitting, loop closure, region boundaries;
- span-attributed imprint lineage on both operands;
- a post-imprint `FragmentGraph` with intersection spans as barriers;
- certified point-in-solid classification by ray casting with deterministic
  hit ownership;
- regularized selection including coincident-face policies;
- deletion, orientation reversal, span-paired α2 sewing, shell discovery,
  solid registration, and full validation before commit;
- a `BooleanResult` carrying lineage and diagnostics.

### Excluded from this plan

- CSG trees, history, or a `Model` façade (see `docs/model_api.md`);
- Boolean semantics for vertices, edges, profiles, faces, or open sheets
  (`prepare_boolean` remains the general imprint facility);
- exact/rational predicate arithmetic (interface reserved, no implementation);
- parallel execution;
- self-intersecting or non-manifold operand repair;
- the surface/surface solver itself, which is the other plan's subject.

## 4. Stage contracts

Every stage has one checkable exit contract. A stage that cannot satisfy its
contract returns a `BooleanError` with geometric context; it never degrades
silently.

| Stage | Input | Output | Contract |
|---|---|---|---|
| Admission | 2 `SolidKey` | `BooleanContext` | both solids valid, closed, oriented; tolerances fixed |
| Broad phase | `BooleanContext` | `CandidateSet` | no intersecting face pair is discarded |
| Narrow phase | `CandidateSet` | `Vec<RawContact>` | every observation carries residual + contact kind |
| Clipping | branches + faces | `Vec<ClippedSpan>` | every retained interval is inside both trim domains; exit/re-entry never bridged |
| Finalization | raw contacts | `IntersectionNetwork` | noded, two-sided, closed loops, bounded regions |
| Imprint | network | `ImprintLineage` | isomorphic subdivision on both operands, span→edges recorded |
| Neighborhood | lineage + GMap | `FragmentGraph` | every post-imprint face is in exactly one component |
| Classification | `FragmentGraph` | `FragmentClassification` | every component certified or explicitly `Ambiguous` |
| Selection | classification | `SelectionPlan` | every fragment is kept, kept-reversed, or dropped |
| Assembly | `SelectionPlan` | `SolidKey` | closed shells, GMap axioms, manifold, outward orientation |

## 5. Module architecture

```text
src/builders/boolean/
    mod.rs            orchestration + public entry points only
    tolerance.rs      BooleanTolerances, model-scale derivation      [new]
    operand.rs        admission, validation, import, OperandCells
    broad_phase.rs    FaceBounds, FaceBvh, CandidateSet              [rewrite]
    contacts.rs       narrow phase → RawContact only                 [shrink]
    trim.rs           FaceTrimDomain, TrimLocation                   [new]
    clip.rs           branch → ClippedSpan intervals                 [new]
    graph.rs          IntersectionNetwork + finalization/validation
    imprint.rs        ImprintPlan, ImprintLineage                    [extend]
    neighborhood.rs   BoundaryFragment, FragmentGraph                [new]
    classify.rs       SolidRayCaster, FragmentClassification         [new]
    select.rs         SelectionPlan, operation tables                [new]
    assemble.rs       deletion, reversal, sewing, shells, validation [new]
    result.rs         BooleanResult, lineage
    diagnostics.rs    BooleanDiagnostics, stage records              [new]
    errors.rs         per-stage error variants
```

`contacts.rs` is currently 962 lines and mixes narrow phase, trimming,
polygon clipping, and chain normalization. Split it: polygon/UV predicates move
to `trim.rs`, chain normalization is superseded by `graph.rs` finalization, and
what remains is pure observation.

## 6. Public API

```rust
/// Regularized Boolean operations on two closed solids.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanOperation {
    Union,
    Intersection,
    /// `first` minus `second`.
    Difference,
}

/// Tunables for one Boolean evaluation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BooleanOptions {
    pub intersections: IntersectionOptions,
    pub tolerances: BooleanTolerancePolicy,
    /// Number of deterministic ray directions tried before reporting `Ambiguous`.
    pub max_classification_rays: usize,
    /// Rejects the operation when any stage reports an unresolved degeneracy.
    pub strict: bool,
}

/// One committed Boolean evaluation.
pub struct BooleanResult {
    pub operation: BooleanOperation,
    pub solid: SolidKey,
    /// Source cell → surviving result cells, for both operands.
    pub lineage: BooleanResultLineage,
    pub diagnostics: BooleanDiagnostics,
}

pub struct BooleanResultLineage {
    pub first: BooleanLineage,
    pub second: BooleanLineage,
    /// Result edges created for each canonical span, per side, before sewing.
    pub span_edges: HashMap<IntersectionSpanId, [Vec<EdgeKey>; 2]>,
    /// Fragments removed by selection, retained for debugging and viz.
    pub discarded_faces: Vec<FaceKey>,
}
```

`prepare_boolean`, `compute_boolean_intersections`, and
`prepare_boolean_with_external_tool` stay as the general dimension-erased
imprint facility. `boolean` is solid-only and refuses anything else at
admission.

## 7. Data structures, stage by stage

### 7.1 Tolerance context — `tolerance.rs`

The current code reads the global `LINEAR_TOLERANCE` in at least four places
(`mod.rs::split_edge_at_points`, `graph.rs`, `contacts.rs`, `faces.rs`). One
operation-scoped context replaces all of them inside the Boolean layer.

```rust
/// How an operation derives its working tolerances.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BooleanTolerancePolicy {
    /// Scale the base tolerances by the diagonal of the combined operand bbox.
    ModelScaled { base_linear: f64 },
    /// Use the given tolerances verbatim.
    Fixed(BooleanTolerances),
}

/// Resolved tolerances for one Boolean evaluation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BooleanTolerances {
    /// World-space point coincidence.
    pub linear: f64,
    /// Distance a curve sample may sit off either supporting surface.
    pub residual: f64,
    /// Parameter-space coincidence on curves and pcurves.
    pub parameter: f64,
    /// Normal/tangent comparison.
    pub angular: f64,
    /// Padding added to every conservative bound in the broad phase.
    pub bbox: f64,
    /// Minimum distance a classification probe must keep from any span.
    pub probe_margin: f64,
    /// Model-space diagonal the tolerances were derived from.
    pub model_scale: f64,
}
```

`BooleanTolerances::from_operands` computes `model_scale` from the union of the
two operand `BBox`es and derives `probe_margin = linear * 100.0` (a probe must
be unambiguously interior, not merely non-coincident). Every stage receives
`&BooleanTolerances`; no stage reads a global constant.

`BooleanContext` is the immutable per-operation bundle threaded everywhere:

```rust
pub(crate) struct BooleanContext {
    pub(crate) operation: BooleanOperation,
    pub(crate) first: SolidKey,
    pub(crate) second: SolidKey,
    pub(crate) first_cells: OperandCells,
    pub(crate) second_cells: OperandCells,
    pub(crate) tolerances: BooleanTolerances,
    pub(crate) options: BooleanOptions,
}
```

`OperandCells` gains `side(&self, face: FaceKey) -> Option<BooleanSide>` backed
by `HashSet<FaceKey>` rather than the current `Vec` + linear `contains`.

### 7.2 Broad phase — `broad_phase.rs`

Replaces the Cartesian product. Bounds come from the face's *trimmed* geometry,
so a small trimmed patch on a huge surface does not inflate the box.

```rust
/// Conservative world bound of one trimmed face.
#[derive(Debug, Clone, Copy)]
pub(crate) struct FaceBounds {
    pub(crate) face: FaceKey,
    pub(crate) bbox: BBox,
}

/// Binary AABB tree over one operand's face bounds.
///
/// Built once per operand and reused by the narrow phase and by the ray caster
/// in `classify.rs`, so the same acceleration structure serves both stages.
pub(crate) struct FaceBvh {
    nodes: Vec<BvhNode>,
    /// Leaf payloads in node order; deterministic for identical input.
    faces: Vec<FaceBounds>,
}

struct BvhNode {
    bbox: BBox,
    /// `Leaf { first, count }` or `Inner { left, right }`.
    kind: BvhNodeKind,
}

/// Deterministically ordered candidate pairs plus the reason each side pruned.
pub(crate) struct CandidateSet {
    pub(crate) pairs: Vec<(FaceKey, FaceKey)>,
    pub(crate) tested: usize,
    pub(crate) pruned: usize,
}
```

Face bounds are computed from the surface evaluated over the trimmed UV
extent: take the face's pcurve loops, take their UV bounding rectangle, clamp
to the surface domain, and bound the corresponding NURBS control net over that
rectangle (`NurbsSurface::bezier_spans` already gives per-span `bbox()` — reuse
it and keep only spans whose UV box meets the trim rectangle). This is
conservative because a rational Bézier span with positive weights lies in its
control hull.

Ordering is by `(first_index, second_index)` in `OperandCells` order, never by
hash iteration, so failures reproduce.

### 7.3 Trim domains — `trim.rs`

Both clipping and classification need one shared, non-approximate answer to
"is this UV point inside this trimmed face?". Today `contacts.rs` has a private
`face_contains_uv` over polygons built from pcurve endpoints only.

```rust
/// Where a UV point sits relative to one face's trim loops.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum TrimLocation {
    Inside { margin: f64 },
    Outside { margin: f64 },
    OnBoundary { loop_index: usize, parameter: f64 },
}

/// The trimmed parameter domain of one face, cached for repeated queries.
pub(crate) struct FaceTrimDomain {
    face: FaceKey,
    /// Outer loop first, then inner loops, in `FaceAttr::inner_loops` order.
    loops: Vec<TrimLoop>,
    /// UV bound of the outer loop, used as a cheap rejection test.
    uv_bounds: (Interval, Interval),
}

struct TrimLoop {
    /// Directed boundary pcurves in loop order, from `FaceAttr::pcurves`.
    pcurves: Vec<Curve2>,
    /// Adaptive polyline approximation, refined until its sagitta is below
    /// the parameter tolerance; used for winding, never for final answers.
    polyline: Vec<Point2>,
}
```

`classify(uv)` computes a winding number against `polyline`, then, when the
point is within `parameter` tolerance of any segment, refines against the exact
`Curve2` with `Curve2::closest_parameter` and returns `OnBoundary`. `margin` is
the distance to the nearest loop, which lets classification probes demand a
minimum interior clearance.

### 7.4 Clipping — `clip.rs`

This is the fix for the "filter samples then reconnect" defect. A branch is cut
at every trim crossing on **both** faces and only whole intervals survive.

```rust
/// One crossing of a branch with a trim loop of one face.
#[derive(Debug, Clone, Copy)]
pub(crate) struct TrimCrossing {
    /// Normalized parameter on the branch curve.
    pub(crate) branch_parameter: f64,
    pub(crate) side: BooleanSide,
    pub(crate) face: FaceKey,
    pub(crate) loop_index: usize,
    /// Boundary edge hit, when the crossing lands on a registered edge.
    pub(crate) edge: Option<EdgeKey>,
    /// Parameter on that boundary edge's curve.
    pub(crate) edge_parameter: Option<f64>,
    pub(crate) residual: f64,
}

/// A maximal branch interval valid inside both trimmed faces.
pub(crate) struct ClippedSpan {
    pub(crate) first_face: FaceKey,
    pub(crate) second_face: FaceKey,
    /// Exact synchronized fragment of the solver branch.
    pub(crate) curve: Curve,
    pub(crate) pcurve_first: Curve2,
    pub(crate) pcurve_second: Curve2,
    pub(crate) kind: IntersectionSpanKind,
    pub(crate) contact: ContactKind,
    /// The crossings (or branch ends) bounding this interval.
    pub(crate) start: SpanBoundary,
    pub(crate) end: SpanBoundary,
    pub(crate) quality: IntersectionQuality,
}

/// What terminates a clipped interval.
#[derive(Debug, Clone, Copy)]
pub(crate) enum SpanBoundary {
    /// A crossing with a trim loop, possibly shared by both faces.
    Trim(TrimCrossing),
    /// A natural branch endpoint from the solver.
    BranchEnd,
    /// The deterministic anchor inserted into a closed loop.
    LoopAnchor,
}
```

Algorithm (per solver branch, per candidate face pair):

1. intersect `pcurve_a` with every `TrimLoop` pcurve of face A, and `pcurve_b`
   with every loop of face B, using `intersect_curves_with_options`;
2. refine each crossing onto the 3D branch (`Curve::closest_parameter`), and
   express it in the branch parameter;
3. merge crossings within `parameter` tolerance, keeping the union of their
   incidences — a crossing that is simultaneously on an A-edge and a B-edge is
   one boundary carrying two incidences;
4. sort by branch parameter; take consecutive pairs as intervals;
5. classify each interval **midpoint** with `FaceTrimDomain::classify` on both
   faces; keep only `Inside` on both;
6. build the interval's exact fragments with `Curve::trimmed` /
   `Curve2::trimmed`, never by re-interpolating samples.

For a closed branch (`SurfaceIntersectionBranch::closed`), insert a
`LoopAnchor` at the branch parameter of the lexicographically smallest 3D
sample before step 4, so the loop becomes a cyclic sequence of spans rather
than a discarded zero-length curve. This also removes the current
`record_span` behaviour of silently dropping spans whose endpoints coincide.

### 7.5 Network finalization — `graph.rs`

`IntersectionSpan` and `IntersectionEvent` gain the fields the later stages
need. These are additive; the existing shape is kept.

```rust
/// Local geometric relationship at a span or event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContactKind {
    /// The two boundaries cross; classification changes across it.
    Transverse,
    /// The boundaries touch without crossing.
    Tangent,
    /// The boundaries share a two-dimensional neighbourhood.
    Coincident,
}

pub struct IntersectionSpan {
    pub start: IntersectionEventId,
    pub end: IntersectionEventId,
    pub curve: Box<Curve>,
    pub kind: IntersectionSpanKind,
    pub contact: ContactKind,                    // new
    pub quality: IntersectionQuality,            // new
    pub uses: Vec<IntersectionSpanUse>,
}

pub struct IntersectionRegion {
    pub first_face: FaceKey,
    pub second_face: FaceKey,
    /// Oriented boundary, CCW in the first face's UV domain. Must be non-empty.
    pub boundary: Vec<(IntersectionSpanId, IntersectionOrientation)>,   // changed
    /// Whether the two supporting surfaces agree in orientation there.
    pub normals_agree: bool,                                            // new
}
```

`IntersectionNetworkBuilder::finish` grows a real finalization pass, in this
order:

1. **Node the arrangement.** Cluster events by point *and* incidence
   compatibility (already present), then split any span whose interior passes
   within `linear` of an event that is incidence-compatible with it. Repeat to a
   fixed point; a bounded iteration count guards against tolerance thrash.
2. **Deduplicate spans.** Two spans with the same endpoint pair and coincident
   geometry merge, unioning their uses (already present); additionally, spans
   that *partially* overlap are split at the overlap boundary before merging.
3. **Check two-sidedness.** Every `ContactKind::Transverse`,
   `IntersectionSpanKind::Transverse` span must carry at least one
   `IntersectionSpanUse::Face` for `BooleanSide::First` and one for `Second`.
   The current code records face imprints one side at a time and never checks
   this, which is why a one-sided imprint can reach the splitter.
4. **Check pcurve agreement.** For 8 adaptive parameters, assert
   `|surface_a(pcurve_a(t)) - curve(t)| <= residual` and likewise for B.
5. **Check loop closure.** Build the event→span incidence multigraph restricted
   to transverse spans. Every event must have even valence. Report
   `BooleanError::OpenIntersectionLoop { event, point }` otherwise. Tangent,
   coincident, and isolated-point events are excluded from the parity rule.
6. **Close regions.** For every `IntersectionRegion`, walk its overlap spans
   into an oriented cycle; an empty or open boundary is an error. This replaces
   `record_region(.., Vec::new())` in `mod.rs`.

`IntersectionNetworkValidationError` gains: `SpanNotTwoSided`,
`PcurveDisagreesWithCurve`, `OddEventValence`, `UnboundedRegion`,
`OverlappingSpans`.

### 7.6 Imprinting and lineage — `imprint.rs`

The blocking gap: `face_imprints` returns `HashMap<FaceKey, Vec<FaceImprint>>`,
so after splitting there is no way to know which `EdgeKey` came from which
span. Sewing then has no choice but to re-match edges geometrically, which is
exactly what `docs/boole_paper_ngk_integration.md` §5 forbids.

```rust
/// One canonical span expressed as an imprint on one operand face.
pub(crate) struct SpanImprint {
    pub(crate) span: IntersectionSpanId,
    pub(crate) side: BooleanSide,
    pub(crate) face: FaceKey,
    pub(crate) imprint: FaceImprint,
    pub(crate) orientation: IntersectionOrientation,
}

/// The complete, ordered subdivision program for both operands.
pub(crate) struct ImprintPlan {
    /// Event points to insert on each source edge, ordered by edge parameter.
    pub(crate) edge_points: BTreeMap<EdgeKey, Vec<(IntersectionEventId, f64, Point3)>>,
    /// Face imprints grouped by face, ordered by span id.
    pub(crate) face_imprints: BTreeMap<FaceKey, Vec<SpanImprint>>,
}

/// What the subdivision actually produced, recorded during the transaction.
pub(crate) struct ImprintLineage {
    pub(crate) vertices: HashMap<IntersectionEventId, [Option<VertexKey>; 2]>,
    /// Edges realizing each span, ordered along the span, per side.
    pub(crate) span_edges: HashMap<IntersectionSpanId, [Vec<EdgeKey>; 2]>,
    pub(crate) edge_fragments: HashMap<EdgeKey, Vec<EdgeKey>>,
    pub(crate) face_fragments: HashMap<FaceKey, Vec<FaceKey>>,
    /// Post-imprint face → its pre-imprint source and side.
    pub(crate) fragment_source: HashMap<FaceKey, (BooleanSide, FaceKey)>,
}
```

To populate `span_edges`, `builders::faces` must report which imprint produced
which section edge. `FaceImprintGraphEdge` already carries
`source_curve: usize` and `interval: Interval`, so the information exists inside
the splitter and only needs to be surfaced:

```rust
/// A section edge together with the imprint fragment that created it.
#[derive(Debug, Clone, PartialEq)]
pub struct FaceImprintSection {
    pub edge: EdgeKey,
    /// Index into the `imprints` slice passed to the splitter.
    pub imprint: usize,
    /// Parameter interval of that imprint realized by this edge.
    pub interval: Interval,
}

pub struct FaceImprintSplit {
    pub first: FaceKey,
    pub second: FaceKey,
    pub sections: Vec<FaceImprintSection>,   // replaces `section_edges`
}
```

This is an API break in `src/builders/faces.rs`; per project convention all
call sites migrate in the same change (`chamfer.rs` and the face tests are the
other consumers).

Determinism note: `mod.rs` currently sorts fragments with
`fragments.sort_by_key(|key| format!("{key:?}"))`. Replace with sorting by
`slotmap::Key::data().as_ffi()`, which is stable, allocation-free, and reflects
creation order.

### 7.7 Fragment graph — `neighborhood.rs`

BOOLE's "orientation-invariant components". The deliberate simplification:
**intersection spans are barriers and nothing propagates across them.** Each
component gets its own ray cast. Cross-span propagation with sector analysis is
a later optimization (Milestone 7), not a correctness requirement.

```rust
/// One post-imprint boundary piece of one operand.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct BoundaryFragment {
    pub(crate) face: FaceKey,
    pub(crate) side: BooleanSide,
    pub(crate) source_face: FaceKey,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub(crate) struct FragmentId(pub(crate) usize);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub(crate) struct ComponentId(pub(crate) usize);

/// Why two adjacent fragments are related.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum FragmentRelation {
    /// Shared edge that is not on the intersection network.
    SameSide,
    /// Shared edge realizing a transverse span: a classification barrier.
    CrossesSpan(IntersectionSpanId),
    /// Shared edge realizing a tangent span: a barrier, but no flip is implied.
    Tangent(IntersectionSpanId),
    /// Shared edge on the boundary of a coincident region.
    Coincident(IntersectionSpanId),
}

/// Temporary algorithm state; the GMap remains the topology authority.
pub(crate) struct FragmentGraph {
    fragments: Vec<BoundaryFragment>,
    index: HashMap<FaceKey, FragmentId>,
    /// Undirected adjacency, sorted by `(FragmentId, EdgeKey)`.
    adjacency: Vec<Vec<(FragmentId, EdgeKey, FragmentRelation)>>,
    /// Connected components under `SameSide` edges only.
    component_of: Vec<ComponentId>,
    components: Vec<Vec<FragmentId>>,
    /// Every edge that realizes a span, with the span it realizes.
    barrier_edges: HashMap<EdgeKey, (IntersectionSpanId, ContactKind)>,
}
```

Construction, entirely from typed views and `ImprintLineage`:

1. one `BoundaryFragment` per key in `ImprintLineage::fragment_source`;
2. `barrier_edges` is the inverse of `ImprintLineage::span_edges`;
3. for each fragment, for each `Edge` in `Face::edges()`, look at
   `Edge::faces()`; every other face on the *same side* is a neighbour, and the
   relation is `SameSide` unless the edge is in `barrier_edges`;
4. components are the connected components of the `SameSide` subgraph, found
   with an explicit stack (no recursion) in `FragmentId` order.

An edge appearing in `barrier_edges` but whose two incident faces are on
different sides indicates a sew that has not happened yet — at this stage that
is an invariant violation, since imprinting must not join the operands.

### 7.8 Classification — `classify.rs`

```rust
/// Where a fragment sits relative to the *other* solid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RelativeLocation {
    Inside,
    Outside,
    /// Coincident with the other boundary, normals agreeing.
    OnBoundarySame,
    /// Coincident with the other boundary, normals opposing.
    OnBoundaryOpposite,
    Ambiguous,
}

/// A validated interior sample used to classify one component.
pub(crate) struct FragmentProbe {
    pub(crate) fragment: FragmentId,
    pub(crate) uv: Point2,
    pub(crate) point: Point3,
    pub(crate) normal: UnitVector3<f64>,
    /// Distance from the nearest trim loop, must exceed `probe_margin`.
    pub(crate) margin: f64,
}

/// One counted crossing of a classification ray with the other solid.
struct RayHit {
    face: FaceKey,
    point: Point3,
    parameter: f64,
    uv: Point2,
    /// The cell that owns this hit for deduplication.
    owner: HitOwner,
    /// `n · d`; a magnitude below the angular tolerance rejects the ray.
    incidence: f64,
}

/// Deterministic ownership so a hit on a shared edge or vertex counts once.
#[derive(PartialEq, Eq, Hash)]
enum HitOwner {
    Face(FaceKey),
    Edge(EdgeKey),
    Vertex(VertexKey),
}

/// Ray casting against one operand's post-imprint boundary.
pub(crate) struct SolidRayCaster<'g, P: Payload> {
    map: &'g GMap<P>,
    bvh: &'g FaceBvh,
    trim: HashMap<FaceKey, FaceTrimDomain>,
    tolerances: BooleanTolerances,
}

pub(crate) struct FragmentClassification {
    /// One entry per `ComponentId`, in component order.
    pub(crate) components: Vec<RelativeLocation>,
    pub(crate) probes: Vec<FragmentProbe>,
    pub(crate) rays_used: usize,
}
```

Probe selection, per component: iterate fragments in `FragmentId` order,
tessellate the face (`tessellate_face_key`), take triangle centroids in
descending triangle area, invert to UV with `Surface::closest_parameter`, and
accept the first whose `FaceTrimDomain::classify` yields
`Inside { margin }` with `margin > probe_margin` and whose distance to every
barrier edge curve also exceeds `probe_margin`. A component with no acceptable
probe is `Ambiguous`.

Ray casting: build a `Curve::line(p, p + d * 4.0 * model_scale)`; gather
candidate faces with `FaceBvh`; use `intersect_curve_surface_with_options` per
face; keep hits whose UV is `Inside` or `OnBoundary`; assign `HitOwner` (an
`OnBoundary` hit is owned by the edge or vertex it lands on); deduplicate by
`HitOwner`; reject the ray if any surviving hit has `|incidence| <= angular`,
if any hit is `OnBoundary` on a vertex, or if the ray's own origin is within
`linear` of a hit. Parity of the surviving count gives `Inside` / `Outside`.

Directions are drawn deterministically from a fixed Fibonacci-sphere sequence
seeded by the operation, never from an RNG. Two independent directions must
agree; disagreement escalates to a third, and `max_classification_rays`
exhaustion yields `Ambiguous`.

`OnBoundarySame` / `OnBoundaryOpposite` do not use rays. A fragment is
on-boundary when its face lies inside an `IntersectionRegion` — detected by
probing the region's supporting face pair and comparing `Face::normal_at`
against the other face's normal at the corresponding UV, with
`IntersectionRegion::normals_agree` as the cross-check.

### 7.9 Selection — `select.rs`

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum FragmentDecision {
    Keep,
    /// Kept, with its boundary orientation reversed (tool side of a difference).
    KeepReversed,
    Drop,
}

pub(crate) struct SelectionPlan {
    /// One decision per fragment, indexed by `FragmentId`.
    pub(crate) decisions: Vec<FragmentDecision>,
    pub(crate) kept: Vec<FaceKey>,
    pub(crate) reversed: Vec<FaceKey>,
    pub(crate) dropped: Vec<FaceKey>,
    /// Spans whose two sides both survive and must be sewn.
    pub(crate) sew_spans: Vec<IntersectionSpanId>,
}
```

The decision table, `side × location × operation`. `A` is `First`, `B` is
`Second`, and `Difference` is `A - B`:

| Location of an A-fragment | Union | Intersection | Difference |
|---|---|---|---|
| `Outside` (of B) | Keep | Drop | Keep |
| `Inside` | Drop | Keep | Drop |
| `OnBoundarySame` | Keep | Keep | Drop |
| `OnBoundaryOpposite` | Drop | Drop | Keep |

| Location of a B-fragment | Union | Intersection | Difference |
|---|---|---|---|
| `Outside` (of A) | Keep | Drop | Drop |
| `Inside` | Drop | Keep | KeepReversed |
| `OnBoundarySame` | Drop (A's copy kept) | Drop (A's copy kept) | Drop |
| `OnBoundaryOpposite` | Drop | Drop | Drop |

Rationale for the coincident rows, each of which gets its own test:

- same-orientation coincidence is one shared piece of boundary, so exactly one
  copy survives a union or intersection and it is always A's — the result
  therefore never contains duplicate boundary faces;
- same-orientation coincidence in a difference means B locally covers A's
  boundary from the same side, so the material is removed and both copies go;
- opposite-orientation coincidence means the solids merely touch: the face is
  interior to a union and lower-dimensional in an intersection (dropped in
  both), and irrelevant to a difference, where A's copy survives.

`Ambiguous` is always an error for solids; there is no safe default. It is a
distinct `BooleanError` variant so diagnostics can name the fragment and the
directions tried.

`sew_spans` is computed after decisions: a span needs sewing when at least one
surviving fragment is incident to its A-side edges and at least one to its
B-side edges.

### 7.10 Assembly — `assemble.rs`

Order matters, because `remove_isolated_darts` remaps every dart.

```rust
/// A matched pair of result edges to be α2-sewn.
struct SewPair {
    span: IntersectionSpanId,
    first_edge: EdgeKey,
    second_edge: EdgeKey,
}

/// Shells discovered in the sewn result.
struct ResultShells {
    outer: Dart,
    inner: Vec<Dart>,
}
```

Steps, all inside the caller's `TopologyEdit`:

1. **Reverse** every face in `SelectionPlan::reversed` with
   `builders::faces::reverse_face_winding`, before any deletion, so its darts
   are still resolvable.
2. **Delete** dropped fragments: for each face, `unlink(Dim::Two, dart)` on
   every dart whose α2 partner belongs to a surviving face, `remove_face`, and
   `remove_profile` for its loops; then collect now-isolated darts and call
   `remove_isolated_darts` once. Apply the returned remap to any cached dart —
   or, preferably, cache nothing and re-resolve from keys afterwards.
   Dropped edges and vertices that became unreferenced are removed in the same
   pass.
3. **Pair** the surviving span edges. For each span in `sew_spans`, take
   `ImprintLineage::span_edges[span]`, which is ordered along the span on both
   sides. The two lists must have equal length — an inequality means imprinting
   produced non-isomorphic subdivisions and is a hard error
   (`BooleanError::NonIsomorphicSpanSubdivision`). Pair positionally, not by
   proximity.
4. **Sew.** For each `SewPair`, pick the free dart of each edge that belongs to
   a surviving face and call `sew(Dim::Two, first_dart, second_dart)`.
   `TopologyEdit::sew` already refuses non-isomorphic sewing orbits via
   `is_sewable`, so a mis-pairing fails loudly rather than corrupting the map.
   Sew order is by `(IntersectionSpanId, position)`.
5. **Merge identities.** For each sewn pair declare
   `merge_edges_into(first, second)` and, for the coincident endpoint vertices,
   `merge_vertices_into(...)`, so commit-time reconciliation keeps A-side keys
   and the result lineage stays meaningful.
6. **Discover shells.** Walk α0/α1/α2 orbits from each surviving face dart to
   partition faces into connected 2-sheets; each must be `Closed` (use
   `topology::closed::Closed::new(Sheet)`), otherwise
   `BooleanError::OpenResultShell`. Classify outer vs inner by signed volume
   computed from the face tessellation: the shell of largest positive volume is
   outer; negatively oriented shells are cavities.
7. **Register.** `add_sheet(SheetAttr::new(dart, data))` per shell,
   `add_solid(SolidAttr::new(data, outer, inner))`. Where the result derives
   from one operand's identity, prefer `add_solid_split_from` so lineage and
   `PreservePayload` behave.
8. **Validate before returning** — still inside the transaction, so a failure
   rolls everything back: `validate_gmap`, `validate_solid_manifold`,
   `validate_solid_orientation`, plus a Boolean-specific check that no two
   surviving faces are coincident duplicates.

Regularization cleanup belongs here too: an edge whose two incident faces are
now coplanar on the same surface, and which no result feature requires, is a
removal candidate. **Deferred** to Milestone 8 — it is not needed for a correct
first result.

### 7.11 Diagnostics — `diagnostics.rs`

```rust
#[derive(Debug, Clone, Default)]
pub struct BooleanDiagnostics {
    pub tolerances: BooleanTolerances,
    pub candidate_pairs_tested: usize,
    pub candidate_pairs_pruned: usize,
    pub branches_found: usize,
    pub branches_uncertified: usize,
    pub spans: usize,
    pub events: usize,
    pub regions: usize,
    pub fragments: usize,
    pub components: usize,
    pub classification_rays: usize,
    pub ambiguous_components: Vec<AmbiguousComponent>,
    pub coverage: Vec<SurfaceIntersectionIncompleteReason>,
}

#[derive(Debug, Clone)]
pub struct AmbiguousComponent {
    pub side: BooleanSide,
    pub representative_face: FaceKey,
    pub probe: Option<Point3>,
    pub directions_tried: usize,
}
```

Diagnostics are populated even on the error path, so a failed Boolean explains
itself. `BooleanError` variants carry the same context (`FaceKey`, `Point3`,
`IntersectionSpanId`) rather than a bare message.

## 8. Orchestration

```rust
pub fn boolean<P: Payload>(
    map: &mut GMap<P>,
    first: SolidKey,
    second: SolidKey,
    operation: BooleanOperation,
    options: BooleanOptions,
) -> Result<BooleanResult, BooleanError> {
    let context = BooleanContext::admit(map, first, second, operation, options)?;
    map.transaction(|edit| {
        let plan      = compute_boolean_intersections_in(edit, &context)?;  // stages 2-5
        let lineage   = imprint::apply(edit, &context, &plan)?;             // stage 6
        let graph     = neighborhood::build(edit, &context, &lineage)?;     // stage 7
        let classes   = classify::run(edit, &context, &graph, &plan.network)?;
        let selection = select::run(&context, &graph, &classes)?;
        assemble::run(edit, &context, &graph, &lineage, &selection)
    })
}
```

Admission runs *before* the transaction because it only reads. Everything that
mutates — including the tool-import path — stays inside the single transaction,
matching `src/topology/edit.md`.

## 9. Changes required in existing code

| File | Change |
|---|---|
| `src/builders/faces.rs` | `FaceImprintSplit.section_edges` → `sections: Vec<FaceImprintSection>` with imprint index and interval; migrate `chamfer.rs`, `boolean/mod.rs`, and the face tests |
| `src/builders/boolean/mod.rs` | move split application into `imprint.rs`; drop the `format!("{key:?}")` sort; thread `BooleanContext` instead of `BooleanOptions` |
| `src/builders/boolean/contacts.rs` | remove `normalize_face_imprint_chains` (superseded by finalization); move `face_contains_uv`, `face_uv_loops`, `point_in_polygon`, `clip_convex_polygon` into `trim.rs` with exact loop handling; emit `RawContact` with residual and contact kind |
| `src/builders/boolean/graph.rs` | add `ContactKind`, `IntersectionQuality`, oriented region boundaries; add the finalization pass and the five new validation errors; stop dropping degenerate closed spans |
| `src/builders/boolean/broad_phase.rs` | replace with `FaceBounds` / `FaceBvh` / `CandidateSet` |
| `src/builders/boolean/operand.rs` | add `admit_solid` running `validate_solid_manifold` + `validate_solid_orientation`; make `OperandCells` hash-set backed |
| `src/builders/boolean/errors.rs` | per-stage variants with geometric context |
| `src/geometry/dim3/intersections/options.rs` | derive defaults from a supplied model scale instead of bare `LINEAR_TOLERANCE` |
| `src/scripts/` | add `boolean_union`, `boolean_difference`, `boolean_intersection` scenes plus the matching `visualization/src/experiments/registry.ts` entries |

## 10. Milestones

Each milestone is independently testable and leaves the tree green.

### Milestone 0 — Admission, tolerances, diagnostics

- [ ] `BooleanTolerances` + `BooleanTolerancePolicy` in `tolerance.rs`.
- [ ] `BooleanContext` with `admit`, rejecting non-solid operands and any solid
      failing `validate_solid_manifold` / `validate_solid_orientation`.
- [ ] `BooleanDiagnostics` skeleton populated by the existing stages.
- [ ] Remove global-tolerance reads from the Boolean layer.

Exit: an invalid operand fails before any mutation, and every existing test
still passes with tolerances supplied by the context.

### Milestone 1 — Broad phase

- [ ] `FaceBounds` from trimmed UV extents over Bézier span bounds.
- [ ] `FaceBvh` build + query, deterministic leaf order.
- [ ] `CandidateSet` with tested/pruned counters wired into diagnostics.
- [ ] Property test: BVH candidate set contains every brute-force intersecting pair.

Exit: candidate generation is sublinear in practice and provably conservative.

### Milestone 2 — Trim domains and exact clipping

- [ ] `FaceTrimDomain` with adaptive polylines and exact boundary refinement.
- [ ] `TrimCrossing` computation on both faces, merged and ordered.
- [ ] `ClippedSpan` intervals built with `Curve::trimmed`.
- [ ] Closed-loop anchoring.
- [ ] Delete the sample-filtering path in `contacts.rs`.

Exit: a branch that leaves and re-enters a trimmed face yields two spans, and
no test can produce a span bridging an invalid interval.

### Milestone 3 — Network finalization

- [x] Fixed-point noding of events into span interiors.
- [x] Partial-overlap span splitting and deduplication.
- [x] Two-sidedness, pcurve-agreement, valence, and region-closure validation.
- [x] Oriented region boundaries replacing `record_region(.., Vec::new())`.

Exit: the network is a valid noded arrangement shared by both operands and is
rejected outright when it is not.

### Milestone 4 — Span-attributed imprinting

- [ ] `FaceImprintSection` in `builders::faces`, all call sites migrated.
- [ ] `ImprintPlan` / `ImprintLineage` with `span_edges` populated per side.
- [ ] Assert equal per-span edge counts on both sides immediately after
      imprinting.
- [ ] Deterministic fragment ordering.

Exit: every canonical span can name its realizing edges on A and on B.

### Milestone 5 — Fragment graph and classification

- [ ] `FragmentGraph` with barrier edges and `SameSide` components.
- [ ] `FragmentProbe` selection with margin guarantees.
- [ ] `SolidRayCaster` with `HitOwner` deduplication and ray rejection.
- [ ] Deterministic direction sequence and multi-ray agreement.
- [ ] `OnBoundarySame` / `OnBoundaryOpposite` from regions.

Exit: every component is certified or the operation stops with a localized
`AmbiguousComponent`.

### Milestone 6 — Selection and assembly

- [ ] `SelectionPlan` with the operation tables.
- [ ] Reversal, deletion, and dart-remap-safe re-resolution.
- [ ] Span-paired α2 sewing with identity merges.
- [ ] Shell discovery, outer/inner determination, solid registration.
- [ ] Full validation before commit; rollback proven by test.
- [ ] Public `boolean` entry point and `BooleanResult`.

Exit: the polyhedral vertical slice from
`docs/boole_paper_ngk_integration.md` §8 is green end to end.

### Milestone 7 — Curved solids

- [ ] Consume certified `SurfaceIntersectionBranch` output for curved pairs.
- [ ] Tangent-span policy in classification (barrier, no implied flip).
- [ ] Coincident curved regions.
- [ ] Cross-span propagation with sector analysis as a measured optimization,
      guarded by a debug assertion against the per-component ray result.

Exit: box/cylinder, cylinder/cylinder, and through-hole subtraction pass.

### Milestone 8 — Regularization, degeneracies, performance

- [ ] Remove redundant coplanar interface edges.
- [ ] Filtered predicates: sign + error bound, with a reserved exact fallback.
- [ ] Isolated vertex/face and edge/edge contacts.
- [ ] Parallel narrow phase emitting immutable observations only.
- [ ] Cached bounds and profiling on representative models.

## 11. Test plan

Tests live in `tests/builders/boolean.rs` and new siblings, integration style,
named after the invariant. The existing 12 preparation tests must keep passing.

### Focused (new files under `tests/builders/`)

- `boolean_broad_phase.rs` — BVH conservativeness, determinism, pruning counts.
- `boolean_trim.rs` — `FaceTrimDomain` inside/outside/on-boundary on faces with
  holes, curved loops, and periodic seams.
- `boolean_clip.rs` — exit/re-entry produces two spans; closed-loop anchoring;
  crossing merge when a point lies on both faces' boundaries.
- `boolean_network.rs` — noding at interior events; one-sided span rejected;
  odd valence rejected; region boundary closed and oriented.
- `boolean_classify.rs` — parity on a box for interior/exterior points; a ray
  through a shared edge counts once; a tangent ray is rejected and retried.

### End to end (`tests/builders/boolean.rs`)

For each of `Union`, `Intersection`, `Difference(A, B)`, `Difference(B, A)`:

- disjoint boxes;
- box strictly inside a box;
- partially overlapping boxes;
- boxes sharing a full face, an edge, and a vertex;
- box with a coplanar partial-overlap face;
- box minus a through cylinder (creates an inner loop on two faces);
- nested shells producing a cavity;
- two sequential operations, the second consuming the first's result.

Every successful result asserts:

- `validate_gmap`, `validate_solid_manifold`, `validate_solid_orientation`;
- each shell is `Closed`;
- expected face/edge/vertex counts for the polyhedral cases;
- Euler characteristic of each shell;
- membership spot checks: sample points known to be inside/outside the
  mathematical result are classified accordingly by `SolidRayCaster`;
- no two result faces share a surface and overlapping UV extent;
- lineage: every result face resolves to a source face on one operand.

Failure-path tests:

- an unclosed intersection loop rolls back with `OpenIntersectionLoop`;
- a non-isomorphic span subdivision rolls back and the map is unchanged;
- an operand failing orientation validation is rejected before mutation;
- `Ambiguous` classification names the offending face.

Invariance tests:

- uniform scaling by 1e-3 and 1e3 gives topologically identical results;
- swapping operand order in `Union` / `Intersection` gives isomorphic results;
- reparameterizing a face's surface does not change the result topology.

## 12. Risks and mitigations

**Sewing pairs that do not match.** The single largest failure mode. Mitigated
by pairing on `IntersectionSpanId` positional order, asserting equal counts
immediately after imprinting, and relying on `is_sewable` to refuse a bad pair.
Never fall back to proximity matching.

**Stale darts after deletion.** `remove_isolated_darts` remaps darts. Mitigated
by keying all algorithm state on slotmap keys and by re-resolving darts after
every deletion batch; a debug assertion checks that no `Dart` outlives a
deletion.

**A component with no safe probe.** Thin slivers may have no point clearing
`probe_margin`. Mitigated by falling back to a probe on the component's largest
fragment with a reduced margin and recording the reduction in diagnostics; if
that also fails the component is `Ambiguous`, not guessed.

**Tolerance-driven merging of distinct events.** Mitigated by requiring
incidence compatibility in addition to geometric coincidence (already the rule
in `record_event`) and by bounding the noding fixed-point iteration.

**Coincident faces double-counted.** Mitigated by resolving each coincident
pair once with A always the surviving copy, and by the "no duplicate boundary
faces" post-condition.

**Regressing the preparation API.** `prepare_boolean` and friends stay public
and tested; `boolean` is additive. Breaking changes are confined to
`FaceImprintSplit` and the internal Boolean modules.

## 13. Definition of done

- [ ] `boolean(map, a, b, op, options)` exists and runs in one transaction;
- [ ] both operands are validated as closed, manifold, oriented solids;
- [ ] no Boolean-layer code reads a global tolerance;
- [ ] the broad phase is bounding-volume based and provably conservative;
- [ ] trimmed clipping splits at crossings and never bridges invalid intervals;
- [ ] the network is finalized, two-sided, noded, closed, and region-bounded;
- [ ] imprinting records span → edge lineage on both sides with equal counts;
- [ ] every fragment component is certified or explicitly `Ambiguous`;
- [ ] selection implements the full table including coincident faces;
- [ ] assembly sews by span identity, discovers shells, and registers a solid;
- [ ] every result passes GMap, closure, manifold, and orientation validation
      before commit;
- [ ] every failure rolls back completely and names the geometry involved;
- [ ] the end-to-end test matrix in section 11 is green;
- [ ] `cargo fmt`, `cargo clippy --all-targets --all-features`, and
      `cargo test --all-targets --all-features` pass;
- [ ] `docs/boole_paper_ngk_integration.md`'s comparison matrix is updated to
      reflect the implemented state.

## 14. Recommended first slice

Milestones 0, 4, 5, and 6 restricted to **planar faces only**, reusing the
existing exact line/plane contact path. That produces a correct polyhedral
solid Boolean through the final public pipeline while Milestones 2 and 3 and
the curved solver land separately. Concretely: two axis-aligned boxes with a
partial overlap, through `boolean(..., Union, ...)`, returning one validated
solid with the expected face count.

Do not start with `select.rs`. Selection over an unfinalized network makes
incorrect topology look complete.

## Implementation checkpoint: planar solid evaluation

The additive `boolean` API now validates operands and evaluates union,
intersection, and difference in one transaction. It resolves tolerances once,
nodes observed spans at compatible events, retains parent imprint intervals,
realizes canonical subspan edges, classifies planar polygon fragments, selects
boundaries, deletes discarded topology, and sews by canonical span identity.
Result shells are registered and validated before commit. Empty or disconnected
results return errors and restore the map, as requested.

Verified cases include transverse overlapping boxes for all three operations,
disjoint result handling, nested and sequential subtraction with a cavity, and union topology
under operand swap and scaling by 1e-3 and 1e3. Planar shell orientation now
uses consistent registered face winding and signed volume, allowing concavity.

This completes the concrete transverse-box slice in section 14, not the whole
plan. Remaining requirements include fixed-point span/span noding, complete
oriented overlap-region boundaries, certified curved
classification/intersection, sector-aware propagation, and the remaining
end-to-end test matrix. Unsupported or ambiguous geometry must remain an error
rather than produce an unverified solid.

## Implementation checkpoint: boundary-coincident contacts (2026-09-04)

Contact sections that run along an operand's own trim loop are no longer
imprinted. `contacts::reroute_boundary_imprints` recognizes them with
`trim::boundary_edge_for` and records them as `RawIntersection::EdgeSection`,
so the network carries an `IntersectionSpanUse::Edge` for that side instead of
a face imprint that would split the face along its own boundary and leave a
degenerate fragment with no interior probe. `imprint::realize_edge_spans` then
resolves each such span to the fragment produced by the edge split pass, giving
assembly the second side it needs to sew by canonical span identity. This is
the "original-edge span realization for contact and coplanar cases" requirement.

Two supporting defects were fixed in the same slice:

- solid contacts are observed by several face pairs at once — a coplanar
  overlap and the transverse pairs bounding it report the same section — and the
  repeated imprint gave the chain graph a doubled edge, hiding the open chain the
  face splitter needs. `contacts::dedup_face_imprints` drops the repeats;
- `contacts.rs` iterated `face_imprints` in hash order while recording raw
  contacts, so canonical span identity varied between two runs on the same input.
  Both passes now iterate sorted keys.

Newly verified cases: union and difference of boxes sharing a full face; union,
difference in both operand orders, and empty intersection of boxes meeting on a
coplanar partial face; union and intersection of nested boxes and the empty
inner-minus-outer difference; difference of overlapping boxes in both operand
orders; rejection, with the map restored, of union and intersection for
operands meeting only on an edge or a vertex; and refusal of a block/cylinder
difference, with the map restored, because surface intersection coverage is
still incomplete for curved operands.

## Implementation checkpoint: network finalization (2026-09-04)

Milestone 3 is implemented in `graph.rs`.

`finalize_network` now runs noding to a fixed point: each pass re-nodes the
original spans against the previous pass's canonical events and stops when a
pass adds none, bounded by `MAX_NODING_PASSES`. One pass was not enough because
`record_event` merges incidences, so a point can only become compatible with a
span — and therefore only become a split parameter — after an earlier pass
merged the uses that made it compatible. `BooleanError::NodingDidNotConverge`
reports tolerance thrash instead of looping.

`close_regions` replaces `record_region(.., Vec::new())`. For every coincident
face pair it collects the spans that lie on both faces — recognizing a section
carried by a face's own boundary edge, not only face imprints, since the
boundary-coincident reroute records those as `IntersectionSpanUse::Edge` —
walks them into one closed cycle, and orients it counterclockwise in the first
face's parameter domain by signed area. `IntersectionRegion` therefore carries
`Vec<(IntersectionSpanId, IntersectionOrientation)>` plus `normals_agree`,
compared from the two oriented face normals at the overlap centroid. Spans that
form no single closed cycle raise `UnboundedRegion`.

`validate_solid_network` holds the contract that only a *solid* evaluation
needs: every span realized on both operands, every face pcurve evaluating onto
the canonical curve within the residual budget, even valence at every event,
and a bounded boundary for every coincident region. It runs from `boolean()`
only. The general `prepare_boolean` facility keeps admitting one-sided and open
contacts — imprinting a lone face against a solid is a legitimate use — so the
check cannot live in `finish`. Parity counts transverse spans only and skips
events touched by a coincident span: an edge/edge or face/face coincidence ends
where the boundaries stop sharing area, not on another crossing, which is why
two blocks meeting on one edge are still a legal difference.

New focused tests in `tests/builders/boolean_network.rs`: no event lies in a
canonical span interior; a coplanar overlap is bounded by a closed
counterclockwise cycle with opposing normals; a transverse box pair validates
and is two-sided everywhere; and two perpendicular faces, whose section ends on
a free boundary, are rejected with `OpenIntersectionLoop`.

## Test-matrix checkpoint and two blockers (2026-09-04)

### Focused tests added

- `tests/builders/boolean_broad_phase.rs` — the BVH keeps every face pair whose
  own vertex extents overlap, measured against a brute-force count on a sweep of
  block offsets, and every pair is either tested or pruned; candidate
  enumeration and event canonicalization repeat identically on a second run.
- `tests/builders/boolean_clip.rs` — a section that leaves and re-enters a
  U-shaped face is clipped into two spans and never bridges the notch. This
  exercises the planar interval path in `contacts.rs`; the `clip.rs` branch path
  is unreachable until curved pairs can be certified (see below).
- `tests/builders/boolean_classify.rs` — ray parity on a block, including points
  whose rays run straight at corners and edges and must be rejected and retried,
  and a point on a face reported as `AmbiguousClassification` rather than guessed.
- `tests/builders/boolean_network.rs` — see the finalization checkpoint above.

### End-to-end assertions added

`tests/builders/boolean.rs` now also asserts, for a transverse union: Euler
characteristic two per shell, every result face resolving to an operand source
face, and no two coplanar result faces with overlapping extents; membership spot
checks through the new public `solid_contains_point` for all three operations;
and identical result topology when one operand's faces are built on a rotated
parameter frame.

`solid_contains_point` exposes the certified ray classifier the selection stage
uses. It is the classifier §11 asks the membership assertions to use, and it is
useful on its own.

### Blocker 1 — curved operands wait on the surface/surface plan

`intersect_surfaces` pushes `InteriorLoopSearchNotImplemented` unconditionally
for every non-planar pair, so `boolean()` rejects any operand pair whose
candidate faces include a curved surface — before classification is ever
reached. Milestone 7 therefore cannot start here: it is gated on
`plan/nurbs_surface_surface_intersection.md`, exactly as this plan's dependency
line states. Generalizing `SolidRayCaster` beyond planes would be dead code
until that lands.

### Blocker 2 — a face's inner loop is invisible to shell traversal

`boolean_difference_of_a_through_slot_opens_an_inner_loop_on_both_caps` is
written and `#[ignore]`d. The Boolean itself is correct: the ten result faces
are built, both caps carry the shaft as an inner loop, every face's oriented
flux is right, and `signed_volume` returns the exact 24. It is rejected by
`validate_solid_orientation` because a face's inner loop is a *separate*
alpha0/alpha1 orbit, so the shell's alpha0/alpha1/alpha2 orbit from a cap dart
never reaches the shaft walls: `Sheet::faces()` reports six of the ten faces and
`Closed::new` passes on that partial shell. The same limitation makes
`add_extruded_face` return a six-faced solid for a profile with a hole.

This is a topology-layer representation gap — a multi-loop face needs its loops
connected in the sheet orbit — and fixing it there would also fix extrusion,
tessellation, and validation of any holed shell. Until then the "through
cylinder / inner loop on two faces" row of the matrix cannot pass, by either the
curved or the planar route.
