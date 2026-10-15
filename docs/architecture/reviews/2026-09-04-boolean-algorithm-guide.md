# Designing ngk's Boolean operations

> **Status: reference.** A study and design guide for Booleans drawn from Hoffmann ch. 3, translated from the French original. The "Proposed direction for ngk" passages are proposals, not decisions.

## Purpose of this document

This is a study and design guide for ngk's Boolean operations. It condenses the
ideas needed from chapter 3 of *Geometric and Solid Modeling: An Introduction*
by Christoph M. Hoffmann, then translates them into ngk's GMap and NURBS
architecture.

It is neither a translation of the chapter nor a specification the project has
adopted. The passages titled **Proposed direction for ngk** are architectural
conclusions drawn from the book and from the state of the code.

The aim is to answer one question precisely:

> How do we get from two B-rep solids that meet to a new, topologically
> consistent solid representing their union, intersection or difference?

The short answer:

```text
detect
    -> build canonical contacts
    -> split both boundaries in sync
    -> classify the fragments
    -> select the useful fragments
    -> assemble and sew them
    -> validate the result
```

Computing intersections is therefore only the start of the algorithm.

---

## 1. What to read before the chapter on Booleans

### Essential before chapter 3

In Hoffmann's book:

1. **Section 2.2.2, Regularized Boolean Operations**, around page 22.
   Explains why a solid kernel uses regularized Boolean operations.
2. **Section 2.3, Boundary Representations**, pages 36 to 46.
   Introduces the separation between topology and geometry, orientations,
   edges, faces and surface components.
3. **Section 2.4, Topological Validity of B-rep Solids**, pages 46 to 61.
   States what "being a valid solid" means and distinguishes topological
   validity from validity of the geometric embedding.

In ngk's documentation:

1. `docs/architecture/topology.md`, for stable identity and contextual
   orientation.
2. `docs/architecture/reviews/2026-05-28-model-api-direction.md`, for the
   separation between `Model`, operations, results and payload policies.
3. In the private GMap reference, the sections on cells, `sew`/`unsew`,
   embedding and geometric sewing.

Without these prerequisites, chapter 3 can give the impression that a Boolean
is essentially a sequence of intersections. In reality, most of the work is
keeping a boundary oriented and consistent while it is being transformed.

### Essential before implementing seriously

Then read **chapter 4, Robust and Error-Free Geometric Operations**, pages 111
to 151.

It can come after a first conceptual reading of chapter 3, but it must be read
before the algorithm is considered ready for general input. It explains in
particular that:

- a tolerance does not by itself make a predicate consistent;
- approximate coincidence is not transitive;
- two equivalent computations can lead to opposite topological decisions;
- recomputing the same event independently on two adjacent faces is dangerous;
- the precision needed depends on the conditioning of the problem.

### Needed for curved surfaces and NURBS

For ngk these are nearly as important as chapter 3:

- **Section 5.7, Edge Identification**, pages 193 to 203: identity of a piece
  of curve, branches, closed curves, singularities and orientation.
- **Chapter 6, Surface Intersections**, pages 205 to 254: tracing intersection
  curves, prediction-correction, Newton, SVD, singularities and intersection of
  parametric surfaces.

Chapter 3 mainly presents the polyhedral case. Its architecture still applies
to NURBS, but its geometric primitives are not enough as they stand.

### Optional for now

Chapter 7 on Gröbner bases is interesting for implicitization, singularities,
offsets and some algebraic systems. It is not needed to build ngk's first NURBS
Boolean architecture.

### Suggested reading order

```text
2.2.2 -> 2.3 -> 2.4 -> chapter 3
                            |
                            +-> chapter 4 before the robust implementation
                            +-> 5.7 and chapter 6 for NURBS
```

---

## 2. Minimal vocabulary

### B-rep solid

A B-rep solid is described by its boundary. That description has two layers:

- a **topology**, saying which cells are incident or adjacent;
- a **geometry**, embedding vertices, edges and faces in space.

In ngk, topology is carried by the `GMap`, and geometric attributes are
attached to the entities identified by `VertexKey`, `EdgeKey`, `FaceKey`, and
so on.

### Geometric support and bounded domain

A face is not a whole surface. It is a bounded region of a support surface.

An edge is not necessarily a whole curve. It is an oriented portion of a
support curve.

For a NURBS face, the boundary must be understood in two spaces:

- as a 3D curve on the solid;
- as a 2D pcurve in the surface's parameter domain.

### Shell

A shell is a connected component of the boundary. A solid may contain:

- one outer shell;
- inner shells bounding cavities;
- possibly several disjoint solid components, depending on the chosen model.

### Orientation

An oriented boundary tells inside from outside locally.

In ngk:

- the geometry has its own parametric orientation;
- an entity's attribute has a reference orientation;
- a topology view carries the contextual orientation given by its dart.

These three levels must not be confused.

### Regularized Boolean

A raw set operation can produce lower-dimensional pieces, for example an
isolated face or a lone edge where two volumes touch.

A regularized operation keeps the volumetric interior, then takes its closure.
Its result therefore stays within regular solids.

Practical consequence: a mere tangent contact, or two opposite faces lying
exactly on top of each other, may produce no boundary at all in the final
result, even though the raw set intersection is not empty.

---

## 3. The real problem a Boolean poses

Take two solids `A` and `B`.

To compute `A ∩ B`, knowing the curves where their surfaces cross is not
enough. A new boundary must be built, made of:

- the fragments of `A`'s faces lying inside `B`;
- the fragments of `B`'s faces lying inside `A`;
- the intersection edges joining those fragments;
- identical vertices and adjacencies on both sides of each contact;
- closed, correctly oriented shells.

For union and difference, the geometric computation of contacts is essentially
the same. What changes is mostly the rules for selecting and orienting
fragments.

This leads to three separate problems:

1. **Prepare a common subdivision** of the boundaries.
2. **Decide which fragments belong to the result.**
3. **Assemble those fragments into a new valid topology.**

This separation is fundamental to ngk's architecture.

---

## 4. The representation the algorithm needs

Hoffmann's chapter assumes a structure from which one can efficiently find:

- from a vertex: its incident edges and faces;
- from an edge: its vertices and its incident faces, in their order around the
  edge;
- from a face: its oriented boundary cycles;
- from a shell: its faces and its orientation;
- from a solid: its shells and their containment relations.

A GMap can express these relations through orbits and involutions. The ngk
translation should, however, keep those details away from the high-level
algorithm. The Boolean should use, as far as possible:

- `solid.sheets()` or its equivalent to walk the shells;
- `sheet.faces()`;
- `face.edges()` and its oriented loops;
- `edge.vertices()` and `edge.faces()`;
- oriented views built from the dart actually reached.

Splitting and sewing operations can then drop down to the edit capability.

### Irreducible decisions, not duplicated truths

The book recommends limiting redundant geometric data, because two separately
computed representations of the same object can contradict each other.

For ngk this does not necessarily mean storing the absolute minimum. Pcurves,
for example, are necessary. It means rather:

- one clearly defined source of authority for each piece of information;
- derived data identified as such;
- explicit validation of the relations between 3D curve, pcurves, vertices and
  support surface.

---

## 5. Overview of the proposed pipeline for ngk

```text
1. Resolve the operands
2. Build contact candidates
3. Compute and canonicalize contacts
4. Build the common intersection graph
5. Split both operands inside one transaction
6. Classify the fragments
7. Select according to the operation
8. Reconcile and sew the retained boundary
9. Rebuild shells and solids
10. Validate, then commit the transaction
```

Each phase should have an inspectable result. A Boolean should not be an opaque
function that mixes numerical computation, topological mutation and cell
deletion all at once.

---

## 6. Phase 1: resolve and freeze the operands

Both operands must be in the same `Model`, or in a transactional working map.

For each operand, collect:

- its vertices;
- its edges;
- its faces;
- its shells;
- the bounding boxes needed;
- a version, or some means of detecting that a plan has gone stale.

The code already has `BooleanOperand`, `OperandCells` and `StalePlan`
detection. That is a good base.

### Importing an external tool

When a tool solid comes from another map, its import must be part of the same
transaction as the preparation. Otherwise a failure can leave a partial copy in
the model.

`prepare_boolean_with_external_tool` already follows this idea.

---

## 7. Phase 2: reduce the candidate pairs

Testing every pair of faces costs quadratic time, even when the solids barely
touch.

The first pass should therefore build candidate pairs by intersecting bounding
boxes:

```text
faces of A = red
faces of B = blue
produce only the red-blue pairs whose boxes overlap
```

Boxes must be inflated by a broad-phase tolerance. A rejected pair is never
looked at again, so a few false positives are better than one false negative.

### Proposed direction for ngk

Start with a simple structure:

- an AABB per face;
- sort along the dominant axis, or a simple sweep;
- queries only between the two operands.

A BVH or a tree of NURBS patches can come later. There is no need to implement
straight away the exact interval and segment trees Hoffmann describes
historically.

The broad phase never decides that there is a contact. It only decides which
pairs deserve a narrow computation.

---

## 8. Phase 3: canonical geometric contacts

The narrow phase computes the real contacts. The useful families are already
close to `BooleanContact`:

- isolated point;
- intersection curve;
- edge overlap;
- region of coincident faces.

A contact curve between two curved faces should carry at least:

- a 3D representation or approximation;
- a pcurve on the first face;
- a pcurve on the second face;
- consistent parameters at its ends;
- whether it is open or closed;
- the precision achieved;
- special events: tangency, branching, singularity, boundary contact.

### The most important consistency rule

One geometric event must be built once, then shared by all its consumers.

Bad organization:

```text
face f1 computes its point on edge e
face f2 independently recomputes the same point on e
```

Good organization:

```text
one ContactId represents the event on e
f1, f2 and the other operand all reference that same event
```

This rule prevents a point from being considered on an edge from one face, and
slightly off it from the adjacent face.

### Common intersection graph

Contacts must be organized into a graph:

- nodes: canonical points, existing or new vertices;
- arcs: ordered portions of intersection curves;
- incidences: the source faces and edges touched;
- orientation: direction on each face and in each pcurve.

It must be possible to represent a temporarily incomplete graph. A curve found
on a first pair of faces may be subdivided later by another pair.

---

## 9. The local cases to understand

The chapter classifies point contacts by which cells meet. For 2D boundaries in
3D space there are six main families.

### 9.1 Face / face

Transversal case: the supports cross along a curve. A portion of that curve lies
within the bounded domains of both faces.

Actions:

- build the 3D curve and both pcurves;
- determine which portions actually lie inside both trims;
- orient the arc on each face;
- analyze its ends, which may land on edges or vertices.

Coincident case: the supports and the regions overlap.

Actions:

- intersect the trim domains;
- compare orientations;
- report a coincident region, not just a few sampled points.

### 9.2 Edge / face

Transversal case: a point crosses the interior of the edge and the interior of
the face.

Actions:

- subdivide the edge;
- place the point in the face's trim graph;
- determine which side of the edge enters the other solid;
- carry the incidence over to the faces adjacent to the edge.

Degenerate case: the edge lies in the face's support surface.

One must then determine which sub-segments belong to the face's domain, and
which incident faces locally surround useful volume.

### 9.3 Edge / edge

Transversal case: a point subdivides both edges.

Degenerate case: the edges share an interval.

The overlap must be synchronized: same ends, same sub-segments, and matching
orientation on both operands.

### 9.4 Vertex / face

The vertex must be inserted into the face's graph. The curves coming from the
faces adjacent to the vertex must be joined to that same canonical node.

### 9.5 Vertex / edge

The edge is subdivided at the existing vertex. Then the volumetric sectors or
corners around the vertex must be analyzed to find which branches enter the
other solid.

### 9.6 Vertex / vertex

The two complete neighborhoods meet. This is the richest point case: several
edges and faces may have to be identified, separated or rejected.

Merging coordinates is not enough. The local cones of material around the point
must be classified.

### Why these six cases still matter with NURBS

The polyhedral formulas change, but the topological dimension of the cells in
contact does not. This classification therefore remains a good dispatch
structure for ngk.

---

## 10. Local neighborhood analysis

Neighborhood analysis answers this question:

> Around a point or a portion of contact, which sectors belong to the interior
> of `A`, to the interior of `B`, and hence to the result?

For a simple transversal intersection of two faces, the normals determine the
four local sectors.

For a non-manifold edge or a vertex there may be more than four sectors. One
then needs the cyclic order of faces around the edge, or a local section built
around the vertex.

### Manifold MVP for ngk

For a first version restricted to closed manifold solids:

- a boundary edge normally has two incident faces;
- the neighborhood of a regular face point is an interior half-space;
- the neighborhood of an edge is a volumetric wedge;
- the neighborhood of a vertex can be analyzed through its oriented incident
  faces and edges.

This restriction cuts the number of cases sharply while keeping an architecture
that can extend to non-manifold.

### Non-manifold extension

The `GMap` can represent structures more general than manifolds. In that case
the Boolean must reason in terms of material sectors and incidences, not assume
"two faces per edge".

That extension should be explicit in the operation's options or its stated
capabilities.

---

## 11. Phase 4: synchronized splitting

Once the contact graph is built, it must be imprinted on both boundaries.

### Recommended order

1. Group and canonicalize all points destined for the same edge.
2. Order them in the edge's parameter domain.
3. Split each edge once, by the complete list.
4. Rewrite the face imprints with the new vertices and edges.
5. Build the complete face subdivisions.
6. Produce the source → fragments lineage.

Splitting immediately at each discovery forces a constant search for which
fragment still represents the source. A global plan followed by a
transactional application is simpler to reason about.

The code already partly follows this direction with `edge_points`,
`face_imprints`, `BooleanLineage` and `apply_boolean_splits`.

### Temporarily incomplete face

A single intersection arc does not necessarily divide a face into valid
regions straight away. Several pairs of faces may successively add arcs to the
same UV domain.

So one must distinguish:

- accumulating an imprint graph;
- the final polygonization, or extraction of cycles;
- creating the new `FaceKey`s.

### GMap sewing after subdivision

A dimension-3 sew identifies two faces and, as a consequence, their
corresponding edges and vertices. The GMap reference requires the two faces
being sewn to be isomorphic.

Therefore:

```text
intersect -> synchronize the subdivisions -> check the correspondence -> 3-sew
```

and not:

```text
intersect approximately -> try to sew faces that differ
```

During sewing, the identified attributes and embeddings must be reconciled by an
explicit policy. ngk's transaction is the right place to apply that policy,
after topological validation.

---

## 12. Phase 5: classifying the fragments

After subdivision, each face fragment must be given a relation to the other
solid.

A useful classification should not stop at `Inside` and `Outside`:

```rust
enum FragmentClass {
    Inside,
    Outside,
    OnSameOrientation,
    OnOppositeOrientation,
    Uncertain,
}
```

The two `On...` classes are needed for coincident faces. `Uncertain` must lead
to a resolution strategy or an explicit failure, not to an arbitrary selection.

### How to obtain the classification

There are two complementary sources.

#### Local classification

Around a contact curve or point, neighborhood analysis tells directly which
fragments lie inside the other operand.

#### Propagation through adjacency

Once a few fragments are classified, the classification can be propagated to
neighboring fragments that cross no boundary of the other solid.

This propagation is essential: some faces lying entirely inside touch no
intersection curve at all.

#### No boundary intersection

If no pair of faces crosses, several possibilities remain:

- `A` is inside `B`;
- `B` is inside `A`;
- they are disjoint;
- a multi-shell configuration combines several of these relations.

A containment test per shell or component is then needed, rather than
automatically concluding that the result is empty.

---

## 13. Phase 6: selection according to the operation

Once each fragment is classified, the Boolean operation becomes mainly a
selection policy.

For non-coincident fragments:

| Fragment source | Class relative to the other solid | Intersection | Union | Difference `A - B` |
|---|---:|---:|---:|---:|
| `A` | Inside | keep | drop | drop |
| `A` | Outside | drop | keep | keep |
| `B` | Inside | keep | drop | keep reversed |
| `B` | Outside | drop | keep | drop |

This table gives the main intuition. Coincident contacts need extra rules:

- same orientation: keep a single copy if it belongs to the result's boundary;
- opposite orientation: often cancel both copies for a regularized operation;
- in every case, avoid two geometrically identical but topologically competing
  faces.

### Separate selection from mutation

It is better to build a pure decision first:

```text
FragmentDecision {
    source,
    fragment,
    class,
    action: Keep | KeepReversed | Drop | MergeWith(...)
}
```

Then apply all decisions together. That makes possible:

- a preview mode;
- readable diagnostics;
- tests without mutation;
- atomic application.

---

## 14. Phase 7: assembling the boundary

The retained fragments must become one or more closed shells.

Assembly involves:

1. reusing or creating the canonical vertices;
2. identifying the edges that represent the same contact segment;
3. matching their orientations;
4. sewing compatible faces;
5. walking the resulting connected components;
6. determining their orientation and containment relations;
7. registering outer and inner shells;
8. creating the new `SolidKey`s.

### Identity and lineage

A Boolean destroys the naive idea that a source face always survives as the
same entity. A face can:

- disappear;
- survive unchanged;
- be split into several fragments;
- be merged with a coincident face;
- change shell;
- be kept with reversed orientation.

The result must therefore expose a rich lineage, for example:

```rust
struct SolidBoolean {
    solids: Vec<SolidKey>,
    kept_faces: Vec<FaceKey>,
    created_faces: Vec<FaceKey>,
    removed_faces: Vec<FaceKey>,
    source_to_faces: HashMap<FaceKey, Vec<FaceKey>>,
    contacts: Vec<BooleanContactId>,
}
```

The exact detail is still to be decided, but the result should not be just a
`SolidKey`.

### Payloads

When two cells become one, one must decide which payload survives or how the
two are merged. When a cell is split, one must decide how its payload is copied
or transformed.

That decision belongs to an `EditPolicy` or a Boolean policy, not to an
attribute `HashMap` manipulated directly from the builder.

---

## 15. Multiple shells and cavities

A cavity is represented by an inner shell whose orientation, relative to the
material, is opposite to that of the outer shell.

To handle several shells:

1. compute intersections between candidate shells;
2. assemble the components that touch;
3. classify the shells that touch nothing by containment;
4. rebuild the containment forest or hierarchy;
5. deduce which components are material and which are void.

### Proposed direction for ngk

Do not start with the complete general case. Build in steps:

1. two manifold solids, one outer shell each;
2. add disjoint solids and full containment;
3. add cavities;
4. add multi-component results;
5. extend to non-manifold if it is really needed.

The data architecture must, however, avoid making steps 2 to 4 impossible.

---

## 16. Robustness: design rules

### 16.1 One geometric question, one decision

If several cells depend on the same intersection, they must reference the same
canonical decision.

### 16.2 Separate measurement from decision

A routine may produce:

```text
estimated value + error bound + conditioning
```

A decision layer then turns that into:

```text
positive | negative | proven zero | uncertain
```

### 16.3 Do not use one universal tolerance

3D distances, curve parameters, UV parameters, angles and box sizes do not live
in the same space and need not share the same threshold.

### 16.4 Canonicalize before modifying

Before splitting:

- group events assumed to be identical;
- check their topological compatibility;
- choose canonical parameters;
- detect contradictions.

### 16.5 Fail cleanly on the uncertain

An `Uncertain` or `DegenerateUnsupported` result is better than a map that is
valid as far as the involutions go but represents the wrong boundary.

### 16.6 Validate several layers

At the end of the transaction:

1. GMap axioms;
2. indexes and attributes;
3. face cycles and pcurves;
4. shell closure;
5. orientation;
6. absence of unwanted self-intersection;
7. geometry–topology consistency.

---

## 17. NURBS surface intersection: the contract the Boolean needs

The Boolean algorithm should not depend on how an intersection curve is
computed internally. It should depend on a rich contract.

### Limit of the current result

The current surface/surface intersection approximates both surfaces with
triangles, collects points, then sorts them along the Cartesian axis of largest
extent.

That approach is useful for prototyping, but it cannot guarantee:

- the correct order on a closed loop;
- the separation of several branches;
- continuity in both UV spaces;
- crossing a periodic seam;
- detection of tangencies and singularities;
- a bounded geometric error between samples.

### Target contract

```rust
struct SurfaceIntersectionBranch {
    curve_3d: Curve,
    pcurve_a: Curve2,
    pcurve_b: Curve2,
    samples: Vec<IntersectionSample>,
    topology: BranchTopology,
    quality: IntersectionQuality,
}

struct IntersectionSample {
    point: Point3,
    uv_a: Point2,
    uv_b: Point2,
}
```

Chapter 6 suggests a natural direction for two parametric surfaces: trace the
curve in the `(u1, v1, u2, v2)` space defined by `S1(u1,v1) = S2(u2,v2)`. Both
pcurves are then produced together instead of being reconstructed afterwards.

---

## 18. Mapping to the code at the time

### What already exists and points the right way

- `BooleanOperand` erases the operand's dimension while keeping a typed handle
  in each variant.
- `BooleanContact` distinguishes points, curves, overlaps and regions.
- A contact curve already holds a 3D curve and two pcurves.
- `BooleanIntersectionPlan` separates the non-mutating computation from its
  application.
- `apply_boolean_splits` uses a transaction.
- `BooleanLineage` keeps the relation between source cells and fragments.
- The plan is revalidated before it is applied.
- Importing an external tool and splitting can be atomic.
- The tests already cover several contacts and rollback.

### What remains conceptually after preparation

The public code currently ends at preparation and splitting. For a complete
Boolean, what mainly remains is:

1. a canonical, oriented representation of the intersection graph;
2. systematic neighborhood analysis;
3. fragment classification;
4. rules for coincident regions;
5. union/intersection/difference selection;
6. assembling and sewing the retained fragments;
7. multi-shell reconstruction;
8. geometric validation of the result;
9. a rich public result and a payload policy.

This list is an observation about the state of the code, not a claim from the
book.

---

## 19. Proposed module split

The names are indicative.

```text
builders/boolean/
    mod.rs
    operand.rs          operand resolution and import
    broad_phase.rs      candidate face pairs
    contacts.rs         narrow phase and canonical contacts
    graph.rs            intersection graph
    imprint.rs          subdivision plan and application
    neighborhood.rs     local sector analysis
    classify.rs         classification and propagation
    select.rs           union/intersection/difference rules
    assemble.rs         reconciliation, sewing and shells
    result.rs           lineage, diagnostics and public result
    errors.rs
```

These files should not be created mechanically now. The split becomes useful
when the responsibilities actually exist. The important idea is to keep the
boundaries between phases.

---

## 20. Incremental implementation plan

### Step A: stabilize the preparation

- canonical contacts;
- globally ordered edge points;
- UV imprints forming valid chains and loops;
- complete lineage;
- diagnostics on ambiguities.

### Step B: intersection of two transversal blocks

- two manifold solids with one shell each;
- planar faces;
- no coincident contact;
- local classification and propagation;
- assembly of one closed shell.

This step proves the topological architecture without waiting for general NURBS
intersection.

### Step C: union and difference

- reuse the same contact plan and the same classifications;
- change only the selection and orientation policy;
- produce lineage and results that are uniform across operations.

### Step D: cases with no intersection curve

- disjoint solids;
- `A` contained in `B`;
- `B` contained in `A`;
- identical solids;
- point or tangent contact removed by regularization.

### Step E: polyhedral degenerate contacts

- vertex/face;
- vertex/edge;
- vertex/vertex;
- overlapping collinear edges;
- coplanar faces with the same orientation;
- coplanar faces with opposite orientation.

### Step F: NURBS curves and surfaces

- ordered intersection branches;
- synchronized pcurves;
- closed loops;
- periodic trims;
- tangencies;
- an explicit strategy for unsupported singularities.

### Step G: multiple shells

- cavities;
- several result components;
- containment forest;
- orientation of inner shells.

---

## 21. Minimal test matrix

### Relative position with no transversal intersection

- two disjoint blocks;
- identical blocks;
- `A` strictly inside `B`;
- `B` strictly inside `A`;
- vertex/vertex contact only;
- edge/edge contact only;
- tangent face/face contact only.

### Transversal intersections

- overlapping blocks;
- a cut crossing exactly one face, then several faces;
- an intersection producing a closed loop;
- an intersection producing several components.

### Coincidences

- coplanar faces with partial overlap;
- identical coplanar faces, same orientation;
- identical coplanar faces, opposite orientations;
- collinear edges with partial overlap;
- vertices that are nearly coincident in a non-transitive way.

### Topology of the result

For each operation:

- `validate_gmap` succeeds;
- the expected shells are closed;
- normals are correctly oriented;
- no unexpected orphan face or edge;
- each sew identifies isomorphic subdivisions;
- the lineage references no deleted key;
- a failure restores exactly the initial map.

### Useful algebraic invariants

When the inputs are valid:

- `A ∪ B` must be equivalent to `B ∪ A`;
- `A ∩ B` must be equivalent to `B ∩ A`;
- `A - A` must be empty;
- `A ∩ A` must be equivalent to `A`;
- `A ∪ A` must be equivalent to `A`;
- applying a stale plan twice must fail atomically.

Equivalence must be tested geometrically and topologically, not by equality of
keys.

---

## 22. API questions to settle for ngk

These deserve a decision before the final assembly is deeply implemented.

1. Does the Boolean always produce new `SolidKey`s, or can it keep an operand
   when nothing changes?
2. Does an operation modify both source solids, consume them, or build a
   separate result in the same `Model`?
3. Should a preview mode expose the contact graph and the classifications
   without mutation?
4. Which policy decides the surviving identity when cells are merged?
5. How are payloads merged or copied?
6. Does the MVP explicitly refuse non-manifold results?
7. Can an intersection return several solids?
8. How is an empty result represented?
9. How far does the public result expose lineage and diagnostics?
10. What guarantee is given when the numerical computation is uncertain?

### Recommended direction

- The operation works inside a transactional `Model`.
- It logically consumes the selected operands, unless explicitly told
  otherwise.
- It may produce zero, one or several `SolidKey`s.
- It returns a rich result structure.
- It exposes a non-mutating preparation/preview mode.
- The MVP clearly states "closed manifold".
- Unresolved numerical cases produce a structured error with diagnostics.

---

## 23. Operational summary

If only one page of this document is kept, keep this:

1. A B-rep Boolean transforms a boundary, not just a set of points.
2. The same intersection must be shared by both operands and every adjacent
   face.
3. Contacts form a temporary graph that may be incomplete while it is being
   built.
4. Edges are split after their parameters have been grouped globally.
5. Faces are split after their UV graphs are complete.
6. Local analysis classifies the fragments adjacent to contacts.
7. Classification is propagated to faces that touch no intersection.
8. Union, intersection and difference share one pipeline; only their selection
   policy changes.
9. The retained fragments must be reconciled, oriented, sewn and grouped into
   shells.
10. A GMap sew needs isomorphic cells and an embedding policy.
11. The transaction must commit only after topological and geometric checks.
12. A tolerance alone never guarantees consistent topological decisions.

---

## 24. Reading references

### Hoffmann

- Chapter 2, §2.2.2: regularized Boolean operations.
- Chapter 2, §2.3: boundary representation.
- Chapter 2, §2.4: topological validity of B-rep solids.
- Chapter 3, pp. 67-108: Boolean operations on B-reps.
- Chapter 4, pp. 111-151: robustness of geometric operations.
- Chapter 5, §5.7, pp. 193-203: identification of curved edges.
- Chapter 6, pp. 205-254: surface intersections.

### The project's GMap reference

- cells defined as orbits;
- involution axioms;
- `i-sew` and `i-unsew` operations;
- compatibility test before sewing;
- cell embedding;
- updating embeddings during merges and splits.

### ngk documentation

- `docs/architecture/topology.md`;
- `docs/architecture/reviews/2026-05-28-model-api-direction.md`;
- `src/builders/boolean.rs` and `tests/builders/boolean.rs` for the
  experimental state at the time.
