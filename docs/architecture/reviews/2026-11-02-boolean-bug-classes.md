# Boolean bug classes: invariant audit

> **Status: in progress.** Class B ("an edge lies on the other operand's
> surface") is audited below, and its mechanism B1 is fixed. Classes A, C, D
> and E are still listed by symptom only. Further mechanisms, found by
> running the Boolean fuzz (`tests/fuzz/boolean.rs`) and fixed, are recorded
> under [Mechanisms found by the fuzz](#mechanisms-found-by-the-fuzz).

Bugs reported against the Boolean arrive one reproduction at a time, but each
one breaks an assumption some stage of the pipeline makes without checking it.
This audit names those assumptions, so that a fix covers every configuration
that breaks the same one, not only the case that was reported.

For each class it records:

- the experiments that separate one mechanism from another;
- the invariant the code relies on, quoted from the code where it says so;
- every place in the code that relies on the same invariant;
- the family of cases a fix has to cover.

The cases can be seen stage by stage with
`cargo run --release --example boolean_bug_classes`, which sends them to the
debug viewer with the trace of each Boolean (`solids::traced`).

## Classes by symptom

| Class | Symptom | Issues |
|---|---|---|
| A | Coincident contacts: the operands share a region of one surface | #29, #31, #39 |
| B | An edge of one operand lies on the other's surface | #3, #37 |
| C | Solver coverage: a surface pair is not traced to the end | #35, #39 |
| D | Scale and aspect ratio | #38 |
| E | Periodic faces: ring placement, seams | remaining `ImprintNotRealized` |

## Class B

### Experiments

A 30 × 20 × 10 block and a cylinder of radius 4, varying three things one at
a time: where the axis sits relative to a block face, where the cylinder's
ends sit relative to the block's top and bottom, and the cylinder's
rotation about its own axis, which moves its parametric seam.

| Axis | Ends | Result |
|---|---|---|
| on a block edge or face | both outside the block | ok |
| on a block edge or face | coplanar with the top and bottom | imprint not realized on the cylinder wall |
| on a block face | bottom inside the block | intersection loop open where the bottom circle meets the face |
| 0.5 or 3.9 from a block face | coplanar with the top and bottom | imprint not realized on the cylinder wall |
| 0.5 or 3.9 from a block face | bottom inside the block | ok |
| inside the block | coplanar, or top above | ok |
| tangent to a block face | any | imprint not realized |

Turning the cylinder about its axis changes none of these, so the seam plays
no part. What looked like one class is three mechanisms, below.

### B1. An imprint bridges two boundary loops of a face

**Seen in:** #3 (cylinder through a block edge, ends coplanar), #37 fused,
and every coplanar-end case whatever the axis offset.

**What happens.** The cylinder's ends lie in the block's top and bottom
planes, so the block faces cut the cylinder wall along lines that run from
one rim circle to the other. The wall is a ring face whose two boundary loops
are those rims. Each line joins the two loops: one such bridge leaves the
face whole, turning the ring into a disc, and only a second one splits it.
The splitter never forms the bridge, and the Boolean fails with
`ImprintNotRealized` on the wall. When the ends stick out, the same lines stop
on circles traced across the wall, close into loops, and the split works.

**Fixed.** A chain whose ends sit on two loops joined by a cut now joins the
two loops into one (`apply_face_bridge`, `src/builders/faces/support.rs`): the
chain takes the place of the cut, and the face lies on both sides of its
edges. A second chain between the same loops then has both ends on one loop
and chords it. Two loops not joined by a cut directly (two holes of one face,
each cut to the outer loop) are still refused.

**Invariant.** An open imprint runs between two corners of one boundary loop.
The splitter says so in `split_one_face_by_imprints`
(`src/builders/faces/imprints.rs`): "A chord runs between two corners of one
loop, so each bounding loop is tried on its own". `FaceImprintCut::from_chain`
is handed the edges of one loop at a time, so a chain whose ends sit on two
different loops matches neither, and nothing realizes it.

**Not specific to cylinders.** A square plate with a square hole, imprinted
with a segment from the outer edge to the hole, fails the same way with one
segment and with two (`split_face_by_imprints`, checked directly). Any
Boolean whose cut crosses a hole of a face hits it: a slot milled across a
drilled plate and a pocket whose sides reach a drilled hole both fail on the
plate's top face (gallery cases B4 and B8), and so would a block cutting
through a tube's wall, or a coplanar contact whose region boundary runs from
a face's outer loop to one of its holes.

**Family to cover.**

- Planar face with one hole: one bridge (face count unchanged, loops merged
  into one), two bridges (two faces), bridges from a corner and from mid-edge.
- Planar face with two holes: a bridge between the holes; a chain outer →
  hole → hole → outer.
- Ring face (cylinder wall between its rims): one cross chord, two cross
  chords, a cross chord plus an ordinary chord on one rim.
- Solids: a cylinder with coplanar ends through a block edge and through a
  block face (#3); a block cutting a tube's wall; a slot across a plate with
  a hole; all four operations.

### B2. A sampled boundary polygon taken for a conservative bound

**Seen in:** #37 cut (cylinder bottom inside the block, axis on a block
edge), and the same with the axis on a block face. The case fails only when
the face's plane passes exactly through the cylinder axis.

**What happens.** The block face's plane crosses the cylinder's bottom disc
along a diameter. `FaceTrimDomain::line_intervals`
(`src/topology/trim_domain.rs`) bounds the infinite line by the extent of the
trim's sampled polygons along it, then intersects that bounded segment with
the exact pcurves. The polygons are inscribed in the boundary, so along a
diameter their extent stops short of the circle by the chord sag: about
`-18.9995 .. -11.0001` against the circle's `-19 .. -11`. The segment then
ends inside the disc, crosses nothing, and the chord is dropped on both
sides. The section is left open where the wall's lines end, and the network
reports `OpenIntersectionLoop`.

**Invariant.** The polygon extent along a line covers every crossing of the
line with the exact boundary. The code says the polygons "only establish a
conservative finite parameter range", but an inscribed polygon falls short of
a convex boundary by up to the domain's `chord` budget, which the trim
domain already records as "upper bound on how far `polygons` may stray".

The bound only matters at the line's two extremes, which come from the loop
reaching furthest along it, the outer loop. So the crossings it misses are
those where the line meets a *curved outer* boundary at the boundary's
extreme point in the line's direction, that is, at right angles: a diameter
of a disc or of a cylinder's cap, a line square to an arc bounding a planar
face. A line through a circular hole is not affected: the outer loop
encloses the hole, so the hole's crossings are always inside the bound.

**Other readers of the polygon extent.**

- `FaceTrimDomain::domain_bounds` is the outer polygon's box. It seeds the
  probe grid in `classify::interior_parameter`, which only needs a point
  inside, so falling short is harmless there. In
  `measure::bounding_box` it gives the `u` range in which a revolved face's
  meridian is searched for extremes. A meridian extreme lying exactly on a
  curved trim boundary could be missed there. To check when fixing B2.
- `broad_phase::face_bounds` bounds a face by its pcurves' control points,
  which do enclose the curves. Sound.

**Family to cover.**

- A line through the centre of a disc face, at several angles and with the
  disc's circle edge starting at several angles.
- A line at right angles to the arc of a planar face bounded by lines and
  an arc (a slot's rounded end), and to an elliptic or NURBS outer boundary,
  at its extreme point.
- Solids: a block face through the axis of a cylinder whose end is inside the
  block, the axis on an edge (#37 cut) and on a face; the same with the axis
  off the face by a little more and a little less than the chord budget; a
  sphere or a cone whose planar cap a block face cuts along a diameter.

### B3. A line contact imprinted as a slit

**Seen in:** a cylinder tangent to a block face, along a line ending inside
the face or spanning it.

**What happens.** The cylinder touches the block face along one line. The
contact is found as a section, and each face is asked to split along it. On
the block face the line ends inside the face; on the cylinder wall it runs
between the rims, or ends inside too. A section that ends inside a face, or
that bridges a ring once, divides nothing, and the splitter reports
`ImprintNotRealized`.

**Invariant.** Every section a face is imprinted with separates two regions
of that face. A tangent contact does not separate anything: the solids touch
along it without crossing. The regularized union of two solids touching
along a line is not a manifold solid, so this case cannot succeed as asked.
It should be refused with a named error, as a point contact already is
(`DisconnectedResult`), instead of failing in the splitter. A cut or an
intersection that touches tangentially leaves the touching operand's faces
unchanged, so it should succeed without imprinting the line at all.

**Where to look.** `Contact::Imprint` already carries a `tangent` flag, and
the narrow phase keeps `tangent_face_imprints` apart. The audit has to follow
where those are dropped or kept before splitting.

**Family to cover.** A cylinder tangent to a planar face from outside and
from inside, along part and along the whole of the face; a sphere tangent to
a cylinder along a circle; two cylinders tangent along a line; all four
operations, each checked against the expected outcome (refusal for a
non-manifold result, success otherwise).

### Class B issues, reassigned

| Issue | Case | Mechanism |
|---|---|---|
| #3 | axis on an edge, ends coplanar, cut | B1 |
| #37 | axis on an edge, ends coplanar, fused | B1 |
| #37 | axis on an edge, top above the block, cut | B2 |

## Mechanisms found by the fuzz

Each of these broke an assumption no reported issue had isolated. The first
three involve a cylinder, which is why the cylinder was in most refused cases;
the last three are what kept a torus or a sphere from being cut.

### A section clipped on one branch of a periodic face

**What happened.** A plane cuts a cylinder wall along an ellipse going once
round it. Its pcurve on the wall runs over one period from wherever the
ellipse starts, so part of it lies a period away from the branch the wall's
rims are written on. `FaceTrimDomain::classify` answers in the quotient, but
`FaceTrimDomain::crossings` met the pcurve with the rims as written, on one
branch only. The crossings on the far part were missed, that part of the
section was dropped, and the network reported `OpenIntersectionLoop`.

**Invariant.** Every question a trim domain answers about a periodic support
is answered in the quotient. `crossings` now meets the loops with every
whole-period translate of the span that reaches them, and `clip_branch`
brings each fragment back onto the branch the face is written on.

### A vertex anchored on a cut's dart

**What happened.** A vertex is anchored on one dart of its 0-cell. Where a cut
leaves a loop at that vertex, the anchor may be one of the cut's own darts.
`CutAttachment::move_after` moves the cut to another corner, and the vertex
went with it: the corner it left had no vertex, the corner it reached had two,
and an edge ending there read as closed. The Boolean failed at assembly with
`NonIsomorphicSpanSubdivision`, one side of a span being a whole ellipse.

**Invariant.** Moving scaffold never moves a vertex. `move_after` and
`apply_face_bridge` re-anchor any vertex anchored on the darts they move.

### An analytic section's pcurve near a sphere's pole

**What happened.** A circle passing close to a sphere's pole turns through
half the longitudes over a stretch as short as its distance to the pole.
Evenly spaced samples, doubled a few times, could not follow it, the section
was reported uncertified and the Boolean refused, even where that stretch was
nowhere near either face. The fit now halves only the gaps it strays over.

### A closed section taken for an open one

**What happened.** A section that closes, on itself or after going once round
a period, is traced or fitted round to where it started and arrives there to
its solver's residual, about `1e-8`. The splitter told closed from open, and
wrapping from not, at `LINEAR_TOLERANCE`. A closed curve on a torus was then
an open imprint ending nowhere, and a circle round a sphere's pole axis was
no wrapping chain; neither was realized.

**Invariant.** What a closed section is told apart from misses by a length of
the face, not by a residual, so closure is read at `CLOSURE_TOLERANCE`
(`src/builders/faces/imprints.rs`). Two sections meeting at a node are still
matched at `LINEAR_TOLERANCE`: both are cut at that node.

### Wrapping chains handed in with anything else

**What happened.** Chains going round a period were split along only when
every imprint of the face belonged to one. A sphere crossed by a torus's tube
gets one circle round its pole axis and one island beside it, and a torus
gets two circles round its tube and the islands another face leaves; the
whole face was then left to the planar paths, which know neither.

**Invariant.** The chains are taken first, whatever else is handed in, and
each ring or cap they leave is then split by the imprints lying on it
(`split_face_by_imprints_edit`).

### A cut labelled twice

**What happened.** A label sits on the dart it was recorded at, and
`disown_cell` clears a whole cell where `own_cell` writes one dart. A chord
across a ring's wall hands the wall's cut to the half that keeps both rims;
relabelling it from another of its darts left the old label beside the new
one, the cut named both halves, and the edit was refused
(`ConflictingOwnership`).

**Invariant.** A cell carries one label. A cut is disowned before it is given
to another face, in the chord split and in `CutAttachment::move_after`, which
now labels the cut in the block that moves it.

