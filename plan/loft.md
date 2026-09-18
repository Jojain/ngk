# Loft: skinning a sequence of sections

Status: **Complete**

A loft takes an ordered sequence of `N` sections and builds the shape that
passes through all of them. It is the last of the three classical sweeps the
kernel owes — `builders::sheets::add_extruded_profile` sweeps along a vector,
`builders::revolve::add_revolved_profile` sweeps around an axis, and a loft
sweeps *between stated sections* with no generating motion at all.

That difference is the whole design problem. An extrusion and a revolution
derive every face from one source edge, so the correspondence between what is
swept and what is produced is free. A loft is handed `N` sections that were
authored independently, and it has to *establish* the correspondence before it
can build anything.

## The surface

Given sections `C_0 … C_{N-1}`, skinning (Piegl & Tiller §10.3) produces one
NURBS surface `S(u, v)` with

```
S(u, v_k) = C_k(u)    for every k
```

of degree `q = min(3, N-1)` in `v`. The surface interpolates every section, not
only the ends, and is `C^(q-1)` across the intermediate ones.

`q` is an option, not a constant. `q = 1` gives a surface that is linear in `v`
between consecutive sections — the multi-section generalization of a ruled
surface, and what a CAD user means by a "ruled" rather than "smooth" loft. The
two are the same construction with a different degree, so `LoftOptions` carries
`v_degree` and nothing branches on it.

That subsumes ruling between two arbitrary curves entirely, and there is no
analytic surface type for it: it is the `N = 2`, `q = 1` skin, and giving it its
own support would fork one construction in two against the NURBS-first rule.
`Surface::Ruled` is a different thing and is untouched — a curve swept along a
*vector* is an extrusion's support and has a closed form worth keeping.

## Correspondence is the problem

Sections do not agree on how many edges they have, where their vertices sit, or
which way round they run. Three obligations, in order.

### 1. Parametrize each section as one curve

Every section is reduced to a single parametrization over `[0, 1]`. This is what
makes sections with unlike edge counts comparable at all: a section's edge
structure stops being its interface and becomes an annotation on a shared
parameter line.

### 2. Split every section at the union of all breakpoints

Each section's vertices sit at parameters of its own `[0, 1]`. Take the union
across **all** `N` sections, merged within tolerance, and split every section at
every parameter in it. Each section then carries the same number of pieces,
corresponding by index. Those pieces are the loft's **columns**.

Two sections of three and five edges, whose vertices interleave, produce up to
eight columns and each section is cut into eight pieces. A section is split at
positions that came from other sections; that is the point.

### 3. Agree on direction and seam

Two failures here produce a valid map holding the wrong shape, which is the
class of bug this kernel takes most seriously.

- **Direction.** Sections that run opposite ways loft into a self-intersecting
  bowtie. Direction is settled once against section 0 and propagated along the
  chain: each section's plane normal is tested against the vector to the next
  section's origin, and a disagreeing section is traversed reversed.
- **Seam.** A closed section's `t = 0` is arbitrary. If two sections start at
  unrelated places, every column is skewed and the shape arrives twisted. The
  offset is chosen to minimize the summed distance between corresponding
  breakpoints, again against section 0 rather than against the previous section,
  so that a long chain cannot accumulate drift one pairing at a time.

Both are stated per-chain, not pairwise, because pairwise agreement does not
compose: `N` locally-consistent pairings can still spiral.

## `ProfileCurve` — the section adaptor

The kernel has no way to ask a section for a point at a parameter. It has
`Profile::edges`, and each edge answers over *its own* native span.

Joining a section's edges into one NURBS curve would answer that and throw away
the thing correspondence needs — which edge and which vertex a parameter belongs
to. The adaptor stays topology-aware.

AGENTS.md draws the line the type has to respect: *a profile says which edges, a
loop says how a face runs along them.* A section is a **directed traversal**: it
has a start, an order and a sense. So the adaptor is built over a walk, and both
kinds of walk the kernel has can supply one.

```rust
/// One directed traversal of a profile, parametrized over [0, 1].
pub struct ProfileCurve<'a, P: Payload> {
    spans: Vec<ProfileSpan<'a, P>>,
    closed: bool,
    length: f64,
}

pub struct ProfileSpan<'a, P: Payload> {
    edge: Edge<'a, P>,
    section: TrimmedCurve,        // the edge's span, already in traversal order
    extent: Interval<Normalized>, // this edge's slice of the traversal's [0, 1]
    starts_at_corner: bool,       // whether a vertex sits where this part begins
}

impl<'a, P: Payload> ProfileCurve<'a, P> {
    pub fn from_profile(profile: &Profile<'a, P>) -> Result<Self, ProfileCurveError>;
    pub fn from_loop(loop_: &Loop<'a, P>) -> Result<Self, ProfileCurveError>;

    pub fn point_at(&self, t: Fraction) -> Point3;
    pub fn locate(&self, t: Fraction) -> (usize, Fraction); // span index, fraction of that span
    pub fn breakpoints(&self) -> Vec<Fraction>;             // the vertex parameters
    pub fn subdivided(&self, at: &[Fraction]) -> Result<Vec<TrimmedCurve>, ProfileCurveError>;
    pub fn rotated_to(&self, t: Fraction) -> Result<Self, ProfileCurveError>;
    pub fn reversed(&self) -> Self;
    pub fn merge_tolerance(&self) -> f64;
    pub fn turning_normal(&self) -> Vector3<f64>;
}
```

Three departures from the sketch above, each forced by something the sketch did
not distinguish:

- **No `reversed` flag on a span.** `Edge::trimmed_curve` already answers in the
  view's own direction, so the section *is* the traversal's, and a second field
  restating that in the stored curve's frame would be the loose pair the
  codebase avoids.
- **`starts_at_corner` is per span, not per traversal.** A rotation cuts one
  edge in two, and the boundary it makes is a place nothing meets. Recording
  cornerness only for the traversal's start reports that cut as a breakpoint,
  which opens a column on a smooth section.
- **`subdivided` and `rotated_to` are fallible.** A piece crossing a corner has
  two supports and no single `TrimmedCurve` carries both; an open traversal has
  no seam to move. Both are refused by name.

A walk that emits one edge twice — a slit, a marked edge passed on both sides —
has no single-valued parametrization and is refused by name rather than
flattened into one.

### Span allocation is by arc length

Each edge gets a slice of `[0, 1]` proportional to its length;
`TrimmedCurve::length` already answers that.

The alternative, an equal slice per edge, makes a section's parametrization a
function of how finely it happens to be divided. Two sections describing the
same shape, one of them carrying an extra vertex from an earlier split, would
then correspond wrongly along their whole length. Arc length is a property of
the shape; edge count is a property of its history.

### Where it sits in the fraction rule

`plan/parameter_units.md` states the rule that governs this type:

> A bare `Curve` has no fractions, because it has no span to be a fraction of.
> Only `TrimmedCurve` and `TrimmedCurve2` produce or consume a `Fraction`, and
> they resolve it against the span they carry.

`ProfileCurve` carries an ordered sequence of spans, so it is entitled to the
brand and is the natural third member of that family. Its `Fraction` is a
fraction *of the traversal*, and `locate` is the conversion down to a fraction
of one span, as `Interval::at` is the conversion between `Normalized` and
`Native`.

## An intermediate section is not a boundary

The loft produces **one face per column**, spanning all `N` sections. An
intermediate section contributes no vertex, no edge and no face: a standard loft
is smooth where it crosses one, and an edge recorded there would assert a
discontinuity that the geometry does not have.

What an intermediate section does contribute is its **breakpoints**, and that is
why the union in step 2 is taken over all `N` sections rather than over the two
ends. A corner in an intermediate section is a real crease in the surface. Split
at it, and the crease lands on a column boundary — a rail, which is an edge, and
`Face::normal_at` stays continuous inside every face. Skip it, and the crease
sits in a face interior where nothing records it.

So the two roles separate cleanly:

| | contributes topology | contributes breakpoints | contributes geometry |
|---|---|---|---|
| end sections | yes — the boundary edges | yes | yes |
| intermediate sections | no | yes | yes |

## The section abstraction

Three input kinds, one algorithm, three tails. They differ only in what happens
after the column faces are built, so one entry point takes all three.

```rust
pub struct OpenSection(ProfileKey);
pub struct ClosedSection(ProfileKey);
pub struct CappedSection(FaceKey);

pub trait LoftSection: Copy + Sealed {
    type Output;
    const CLOSES_RING: bool;

    fn traversal<'m, P: Payload>(self, model: &'m Model<P>, index: usize)
        -> Result<ProfileCurve<'m, P>, LoftError>;
    fn prepare_end<P: Payload>(self, edit: &mut ModelEdit<'_, P>, index: usize, cuts: &[Point3])
        -> Result<(), LoftError>;
    fn finish<P: Payload>(edit: &mut ModelEdit<'_, P>, columns: &LoftColumns, ends: [Self; 2])
        -> Result<Self::Output, LoftError>;
}

pub fn add_loft<S: LoftSection, P: Payload>(
    g: &mut Model<P>,
    sections: &[S],
    options: LoftOptions,
) -> Result<S::Output, LoftError>;
```

| Type | Built from | Tail | `Output` |
|---|---|---|---|
| `OpenSection` | a `Profile` that is not closed | nothing; the sheet stays open | `SheetKey` |
| `ClosedSection` | a `Closed<Profile>` | close the ring between the last and first column | `SheetKey` |
| `CappedSection` | a `Face` with no inner loop | close the ring, sew the end caps on, register a solid | `SolidKey` |

**Keys, not views.** A trait over `Profile`, `Closed<Profile>` and `Face`
cannot be used at `&mut Model<P>`: those views borrow the model the builder
mutates, so a caller holding one could never hand it over. Each section type
therefore *carries a key* and is *built from a view*, and the constructor is
where the type's promise is checked — `ClosedSection` only from a `Closed`
profile, `CappedSection` only from a face a loft can actually cap.

**Three types, not one enum with three cases.** A homogeneous `&[S]` makes a
mixed run unwritable, so there is no uniformity check and no
`MixedSectionKinds`; and `S::Output` makes the result follow the input, so
nothing hands back a `SheetKey` a caller has to unwrap from a variant it
already knows.

Planarity is **not** required of any of them. A cap keeps its own face's
surface, and a section is a closed curve in space whether or not one plane
holds it, so there is nothing for `Planar` to prove here.

`modeling::loft` sits above this and *infers* the kind from the profiles it is
handed. Inference has to face the case the types rule out, so that is the one
place a mixed run is reported — `LoftError::MixedProfiles`.

**Only the end sections are capped.** A `Capped` loft through five faces builds
two caps, from sections 0 and `N-1`; the intermediate faces are consumed for
their outer loop alone, since there is no cap in the middle of a loft. The
section type is what makes that legible — a caller passing five faces is
asking for a solid, and reading cappedness off the ends of a sequence that
could mix kinds would make the same call mean different things depending on
which end was which.

## Building it

```
add_loft
├─ uniform-kind check
├─ ProfileCurve per section
├─ direction agreement and seam alignment, anchored on section 0
├─ union of all breakpoints, merged within tolerance → the columns
├─ subdivide every section at the union
├─ per column:
│    ├─ make the column's N curves compatible
│    ├─ skin the N compatible curves → Surface::Nurbs
│    └─ add the face — a quad, or a band where the one column wraps
├─ sew adjacent columns along their shared rails
├─ close the ring                                        (Closed, Capped)
└─ sew the caps, register the solid                      (Capped)
```

### One column that wraps has no rail, and therefore no seam

A closed run whose sections share no breakpoint — a pair of circles — has one
column, and that column's two ends are the same curve. Built as a quad it
would need a rail there, and the rail would be a **seam**: an edge asserting a
crease the surface does not have, on a wall that is smooth all the way round.

So it is built as a **band** instead, the way
`add_full_revolved_band_face` builds a cylinder wall. Two closed boundary
loops, one per end section, each running the whole of `u`; no rails between
them; and a scaffold cut joining the two so the face still occupies one
2-cell. Both loops are `LoopKind::Wrapping { axis: Axis2::U }`, because
neither closes in parameter space — each is a straight run across the domain
that closes only on the surface. A boundary loop is a *marked* closed edge
where its section began on a corner and an *unmarked* one where it did not, so
a circle handed in without a vertex comes back out without one.

A pair of circular faces therefore lofts to a frustum of **three faces, two
edges and no vertex at all**, which is the same shape a revolution of the same
profile produces. Nothing is healed away afterwards; the seam is never built.

Two things this costs, both paid in `sew_edges` rather than by the caller:

- **The sew has to be total over all three shapes of edge.** A band's boundary
  can be an unmarked closed edge, and there is no vertex at either end of one
  to reconcile, where every other sweep in the crate assumes two.
- **A merged closure point has to be re-owned.** A corner-free closed edge
  records where it closes by owning that 0-cell. Sewing two of them brings the
  two 0-cells onto one orbit, and merging the edges leaves the orbit claimed by
  the key that went away as well as by the one that stayed — two claims on one
  cell, which commit refuses by name.

`add_revolved_profile` is the skeleton to follow — one column where it has one
source edge. Per face, mirroring `add_revolved_quad_face`: eight darts, four
vertices, four edges, one profile, one face.

- **Boundary curves**, in loop order: the section-0 column curve, the rail at
  `u = 1`, the section-`(N-1)` column curve reversed, the rail at `u = 0`
  reversed.
- **Rails** are the skinned surface's `u = const` isocurves, extracted exactly.
  They are curved whenever `N > 2`, and a rail derived any other way would not
  lie on the two faces that share it.
- **pcurves** are the four sides of the `[0, 1]²` domain box — plain
  `Curve2::line`s. A skinned surface's parameterization *is* the quad.

Nothing here touches the GMap directly: sections arrive as typed views and edits
go through `ModelEdit`, per the builder-layer rule.

## Compatibility, per column

A column is `N` trimmed curves that have to become `N` NURBS curves sharing a
degree and a knot vector before they can be skinned.

| Requirement | Today |
|---|---|
| Common traversal direction | `NurbsCurve::reversed` — **have** |
| Clamped | `NurbsCurve::clamped` — **have** |
| Common domain `[0, 1]` | **missing** — affine remap of the knot vector |
| Common degree | **missing** — degree elevation, P&T A5.9 |
| Common knot vector | **missing** — refinement to the union, P&T A5.4. `insert_knot` exists and can be looped, at `O(n·m)` and one full rebuild per knot |

Elevation runs before refinement, since elevation inserts knots of its own and
refining first would leave the vectors different again. Normalization runs before
the knot union, since interior knots of `[0, 5]` and `[0, 1]` are not comparable
until both are mapped to the same domain.

Any column mixing a straight piece with a curved one exercises all of it: a
degree-1 non-rational line and a degree-2 rational arc have to meet at degree 2
with the arc's weights intact. Degree elevation is therefore on the critical
path for the first non-trivial shape, not a refinement of it.

## Skinning, per column

With all `N` curves compatible, the column is a grid `P[i][k]` — control point
`i`, section `k`.

1. **v-parameters.** Chord length along each row `i` across the sections, then
   averaged over all rows (P&T eq. 10.8). Averaging is what stops one wild row
   skewing the parameterization of the whole column.
2. **v-knots.** Averaged from those parameters — the recipe already in
   `interpolate_open` in `geometry/dim3/nurbs/curve.rs`.
3. **Rows.** For each `i`, interpolate the `N` control points across `v`. The
   coefficient matrix depends only on `(parameters, degree, knots)` and is
   therefore identical for every row: factor once, solve `n + 1` times.

Two obligations against the code as it stands:

- `interpolate_open` computes its own knots, fixes degree at 3 and solves once.
  Skinning needs its inner half as
  `interpolate_with_knots(points, parameters, degree, &knots)`, with the
  factorization reusable across rows.
- **The solve runs in homogeneous coordinates.** `interpolate_open` solves x, y
  and z with weight `1.0`. A rational section — any arc — then yields a surface
  that does not pass through it. Skinning interpolates `HPoint`, four
  coordinates, and divides once at the end.

`N = 2` reaches the same code with `q = 1`, where the interpolation reduces to
the two rows stacked. There is no separate two-section path: one construction,
one set of tests, and `q` chosen from `N` and the options.

## The geometry gaps, collected

| Gap | Where | Role |
|---|---|---|
| `NurbsCurve::normalized` — knot domain to `[0, 1]` | `nurbs/curve.rs` | compatibility |
| `NurbsCurve::refined(&[f64])` — P&T A5.4 | `nurbs/curve.rs` | compatibility |
| `NurbsCurve::elevated_degree(Degree)` — P&T A5.9 | `nurbs/curve.rs` | compatibility |
| `make_compatible(&mut [NurbsCurve])` | `nurbs/curve.rs` | per column |
| `interpolate_with_knots`, homogeneous, reusable factorization | `nurbs/curve.rs` | skinning |
| `NurbsSurface::skinned(&[NurbsCurve], Degree)` | `nurbs/surface.rs` | the surface |
| `NurbsSurface::isocurve_u` / `isocurve_v` | `nurbs/surface.rs` | rails |
| `ProfileCurve` | `src/topology/profile_curve.rs` | correspondence |

`isocurve_u` / `isocurve_v` pay twice. `exchange::step::convert::iso_curve`
answers `None` for `Surface::Nurbs` today, so every NURBS face needing a
synthesized seam is unexportable to STEP by name; exact isocurve extraction
closes that hole with the same function.

## Parameter spaces

The loft moves through four, three of which `plan/parameter_units.md` already
brands:

| Space | Carrier | State |
|---|---|---|
| Native curve parameter | `Param<Native>` | branded |
| Fraction of a named span | `Param<Normalized>` | branded |
| Fraction of a *traversal* | `Param<Normalized>`, resolved by `ProfileCurve` | this plan |
| Surface `(u, v)` | bare `f64`, `Point2` | `Uv<S>`, stage 4 there |

Worth recording for that stage: a skinned surface's `u` **is** the adaptor's
fraction and its `v` **is** the section fraction, so its domain is `[0, 1]²` and
the hop from `ProfileCurve` into the face's parameter space is the identity.
That is the cleanest instance of the cross-space hop `Uv<S>` is meant to type.

## Cost, stated plainly

The column count is the size of the merged breakpoint union, so it is the sum of
the sections' edge counts in the worst case, not the maximum. Ten five-edge
sections at unrelated rotations give up to fifty columns, hence fifty faces, and
every section is cut into fifty pieces. Each column then carries `N` curves
refined to their common knot vector, whose control point count is likewise a
union.

That is inherent to interpolating stated sections rather than approximating
them, and the merge tolerance is the only lever on it. Two sections whose
corners differ by less than that tolerance must collapse to one column, or every
column acquires a hairline neighbour.

## Not in scope

Each refuses by name rather than producing an approximation.

- **Degenerate sections.** A point section — the apex of a cone — makes a
  degenerate surface row. `Surface::is_degenerate_at` and
  `Surface::degenerate_rows` exist and the face layer already handles degenerate
  rows for cones and spheres, so the ground is prepared, but a zero-length
  traversal has no parametrization and the adaptor refuses it.
- **Inner loops.** A `Face` with a hole lofted to another with a hole has a
  correspondence problem between the holes on top of the one between the outer
  loops, and a cardinality question when the counts differ.
  `LoftError::SectionHasInnerLoops` refuses it, rather than lofting the outer
  loops alone and leaving each cap with a hole nothing walls in.
- **Lofts closed in `v`.** A sequence that returns to its first section wants a
  periodic `v` knot vector and no caps. It is a variation on the same skin, and
  the periodic-support work in `plan/periodic_supports.md` is its prerequisite.
- **Guide rails and tangency conditions.** Constraining the loft to be tangent
  to an adjacent face at an end section adds derivative rows to the
  interpolation (P&T §9.3). The homogeneous solver written here is the piece
  that would carry it.

## Stages

Staged by layer, so each lands testable on its own.

1. **`ProfileCurve`.** Arc-length spans, `point_at`, `locate`, `breakpoints`,
   `subdivided`, `rotated_to`, `reversed`, both constructors, the repeated-edge
   refusal. Direction agreement and seam alignment as free functions over it.
   Testable against an open polyline, a rectangle and a circle with no loft in
   sight.
2. **NURBS compatibility.** `normalized`, `refined`, `elevated_degree`,
   `make_compatible`. Each with tests asserting the curve is *unmoved*: an
   elevated or refined curve evaluates identically to its source at sampled
   parameters, which is the whole contract.
3. **Skinning.** Homogeneous `interpolate_with_knots` with reused factorization,
   `NurbsSurface::skinned` for general `N` and `q`, `isocurve_u`/`isocurve_v`.
   Asserts `S(u, v_k)` reproduces every section, rational ones included.
4. **`builders/loft.rs`.** `LoftSection`, `LoftOptions`, `LoftError`, the
   breakpoint union, the column faces, the lateral sews, ring closure, caps.
5. **Bindings.** Python and wasm entry points, bare `f64` across the boundary
   per the existing convention.

Stages 1–3 are topology-free.

## Definition of done

- Sections with unlike edge counts loft correctly: a circle and a rectangle in
  parallel planes give a four-face sheet whose corner edges run from each
  rectangle corner to the circle, and every face passes through both sections
  within `LINEAR_TOLERANCE` at sampled fractions.
- A loft through four or more sections interpolates **every** section, not only
  the ends, and records no edge where it crosses an intermediate one.
- A corner in an intermediate section lands on a rail: the face on each side of
  it has a continuous normal across its own interior.
- The same sections as faces loft to a valid solid with an outward-facing shell
  and two planar caps; `Model` validation passes.
- `v_degree = 1` through `N` sections gives a surface linear in `v` between
  consecutive sections, from the same code path as the smooth case.
- A loft of open profiles gives an open sheet; a mixed run cannot be written
  at all, and the one place a kind is inferred rather than stated —
  `modeling::loft::loft` over profiles — refuses a mixed one by name.
- Two circles loft to **one** face bounded by those two circles and nothing
  else: no rail, no seam edge, and no vertex neither circle had. As faces they
  give a three-face frustum that `Model` validation accepts.
- Reversing one input section gives the same shape rather than a bowtie, and a
  long chain of sections does not accumulate seam drift.
- Tests live under `tests/builders/loft.rs`, `tests/geometry/dim3/nurbs/` and
  `tests/topology/` for the adaptor, mirroring the source layout.

## Notes for the implementation

- **Splitting sections costs nothing geometrically.** `TrimmedCurve::sub` takes
  an `Interval<Normalized>` and narrows the stored interval without touching the
  support, so a quarter circle stays an exact `Curve::Circle` right up until
  `to_nurbs` is called on it for the skin.
- **The breakpoint merge tolerance is not `LINEAR_TOLERANCE`.** A fraction is
  not a distance. Derive it per section from the section's length, so that a
  merge means "these corners are closer than the kernel can tell apart in
  space", which is the question actually being asked.
- **A closed section's breakpoint set includes `0`** whenever a corner is
  there, and an unmarked circle's is empty because nothing meets where it
  closes. Rotating moves the start, so breakpoints are read off each span's own
  cornerness rather than shifted modulo one — the vertex that was at `0` is
  still a vertex, and the cut the rotation made is not one. A closed traversal
  of a single corner-free edge is rotated by *sliding its span* rather than by
  cutting it, which leaves one part and no phantom boundary.
- **One column is a whole shape, not a degenerate case.** Two sections with no
  breakpoint between them — a pair of circles — give one column whose two rails
  are sewn to each other. That is the seam of a wall that wraps, and skipping
  the sew leaves the tube split open along it.
- **One column is the unit of failure.** A column whose curves cannot be made
  compatible should name the column and the sections involved, not the loft as a
  whole; with fifty columns a bare failure is unactionable.
