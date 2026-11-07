# Builder API

This document defines the naming and return-value convention for the NGK
builder layer. Builders mutate one `Model` transaction and return a structured
description of the topology change they committed.

## Result names

An operation result is a noun phrase naming the event, qualified by the
logical entity that the event acted on:

```text
<Subject><NominalizedVerb>
```

The subject comes first. It is the logical vocabulary of the operation, not a
raw GMap cell or a dart. The result name has no `Result` suffix; the outer Rust
`Result<..., ...>` already expresses fallibility.

This makes the result read naturally at the call site:

```rust
let split = split_edge(&mut model, edge, parameter)?;
let extrusion = add_extruded_profile(&mut model, profile, direction)?;
```

The canonical names are:

| Current or proposed name | Canonical name | Reason |
| --- | --- | --- |
| `EdgeSplit` | `EdgeSplit` | An edge was split. |
| `FaceImprintSplit` | `FaceImprintSplit` | A face was split by an imprint. |
| `CellRemoval` | `CellRemoval` | A cell was removed. |
| `ProfileExtrusion` | `ProfileExtrusion` | A profile was extruded. |
| `Extrusion` | `FaceExtrusion` | A face was extruded. |
| `Chamfer` | `TargetChamfer` | A chamfer was applied to a `BlendTarget`, which may mix vertices, edges, profiles and faces. |
| `Fillet` | `TargetFillet` | A fillet was applied to a `BlendTarget`. |
| `BooleanResult` | `SolidBoolean` | A Boolean operation acted on solids. |
| `BooleanPreparation` | `BooleanOperandPreparation` | Boolean operand preparation was performed. |
| `RevolvedEdge` | `EdgeRevolution` | An edge was revolved. |
| `RevolvedProfile` | `ProfileRevolution` | A profile was revolved. |
| `RevolvedFace` | `FaceRevolution` | A face was revolved. |
| `FaceSweep` | `FaceSweep` | A face was swept along a spine. |
| `AppendEdge` | `ProfileAppend` | An edge was appended to a profile. |
| — | `EdgeOffset`, `ProfileOffset`, `FaceOffset`, `SheetOffset`, `SolidOffset` | An entity was offset into a moved copy. |
| `ClosedSolid` | `ClosedSolid` | Product-name exception for pure creation. |

## Pure creation exception

Some operations do not perform a meaningful event on an existing subject. A
primitive constructor such as `add_sphere` or `add_torus` only establishes a
new product; there is no useful subject-plus-event phrase to name.

When the whole operation is “this product now exists”, the result may be named
after the product itself. `ClosedSolid` is the current example. This exception
is limited to pure creation operations and does not apply to edits,
transformations, splits, merges, or operations that consume or modify an
existing logical entity.

## Rewiring inside a builder

A builder reads its own writes: it cuts an edge and then reads the face that
edge bounds. So the steps it takes between two reads must leave the labelling
describing the map. Take the step as an edit operator (`mark_edge`, `cut_edge`,
`sew_edges`, `unsew_edge`), which relabels with the rewiring, and a read after it
is valid. A rewiring no operator spells goes in `edit.deferred(|raw| ..)`, which
offers no view and is checked when it closes. Only a builder that does neither
leaves a read to meet an inconsistent labelling, and then the read records a
fault and the transaction fails with it instead of panicking; see
[transactions.md](transactions.md#reads-on-an-open-edit).

## Builder return contract

Every public mutating builder returns a structured operation result:

```rust
pub fn operation(...) -> Result<OperationResult, OperationError>;
```

The result struct:

- contains durable logical keys such as `EdgeKey`, `FaceKey`, or `SolidKey`;
- does not expose raw darts as durable identity;
- implements `OpResult` when it refers to committed model state;
- is stamped only by the public transaction boundary; and
- may provide a checked `view(&model)` for resolving its keys at the committed
  revision.

The edit-scoped implementation returns the same result type before stamping.
Private topology primitives may return individual keys or topology fragments
when they are implementation details rather than complete builder operations.

Queries and preflight checks are not builders and keep ordinary return types.
For example, `is_removable` may return `bool`, and `planned_merge` may return
`Result<MergeKind, CellRemovalError>`.

## Revolution sections

A face or profile revolution uses a planar meridian section: the revolution
axis lies in the section plane and the whole boundary stays on one side of it.
The side can be either side; the axis origin can be anywhere along the same
line. `SectionCrossesAxis` refuses a section spanning both sides instead of
returning overlapping walls. The builder does not split or combine positive
and negative radial regions.

The preflight bounds each actual curve span with its positive-weight NURBS
control hull, including curved interiors and inner loops. This is conservative:
an ambiguous control hull or unsupported weights are also refused, even if
the curve itself might stay on one side. `AxisOutsideSectionPlane` refuses an
axis outside the meridian plane. Refusals leave the model unchanged.

A section edge wholly on the axis sweeps no wall. In a partial turn the two
caps meet directly along that edge; in a full turn the source section is
consumed. An edge with one end on the axis contributes a triangular wall for
a partial turn, or a face bounded by one circular rim for a full turn.
Curved section edges with both ends on the axis are refused by the profile
and face builders. The isolated-edge builder has its own full-turn apex
construction; partial isolated-edge apex sweeps are refused.

Circular trajectories keep the requested angular span, including angles over
half a turn. A returning boundary circle is oriented against the sweep so its
span agrees with its pcurve rather than selecting the complementary arc.

The convention applies to the logical subject named by the operation. It does
not change the distinction between logical entities and raw GMap cells: a
result describing an edge uses an `EdgeKey`, while a result about a raw map
cell must say so explicitly.

## Assembling a shape from the one below it

Each kind of shape can be assembled from shapes of the kind below it:
`add_profile_from_edges` joins edges, `add_sheet_from_faces` joins faces and
`add_solid_from_sheets` bounds a solid with closed sheets. A face from a closed
profile is `add_face`.

Joining matches pieces by geometry, never by key: two edges join where their
ends coincide, two faces where each runs a boundary edge covering the same
span. Faces are welded with the same `sew_faces_along` the Boolean assembly
uses. Only whole spans join; edges that partly overlap are left apart rather
than imprinted.

The builders refuse what they cannot read one way. Disconnected pieces, an edge
run by more than two faces, or two faces wound against each other are errors:
reversing one face would decide which side of the sheet is its front, which
only the caller knows. A closed sheet's winding is decided by its role instead:
the outer sheet of a solid is wound outward and each void inward, whichever way
it arrived. The sheets must be disjoint, which the Boolean contact search
certifies, and each void must lie inside the outer sheet and outside every
other void.

## Profiles from a constraint system

`add_profiles_from_system` draws a solved [constraint system](solver.md) on a plane:
`curves` lists the segments, arcs and circles to draw, while independent points and
construction curves stay in the system unlisted. A system without a solution for its
current revision is refused, since its seed need not satisfy its constraints. The
facade `modeling::profiles::from_system` does the same in a fresh model. The code lives
in the private `builders::sketch`.

A system is drawn in one of two ways. `add_profiles_from_system` gives contours and open
chains, and refuses curves that meet anywhere but at a joint.
[`add_sketch_from_system`](#regions-from-a-constraint-system) gives the areas the curves
enclose, and cuts them wherever they meet.

The result ties each curve's id to the `EdgeKey` it drew, and each profile's key to its
curves. A key holds within its model, and it moves when a build adds or removes a curve
before it, because keys follow creation order. The id is what identifies a curve from
one build to the next: a curve keeps its id when it is edited, so its edge can be found
again even with another shape and another key.

Unlike `add_profile_from_edges`, joining does not go by geometry. Two curve ends are one
vertex where the system's coincidences say so, including through a chain running over
what is not drawn, because two ends can touch by accident and a coincidence is meant to
be removable. Joined ends share one position: stored ones already coincide exactly, and
an arc's evaluated end is within `LINEAR_TOLERANCE`, which is what the solver holds by
default. A larger gap is refused.

Each connected run of curves becomes one profile, open or closed, and a circle is a
profile of its own. Curves meeting anywhere but at a joint are refused by name: more than
two ends at one joint, a crossing, an end lying on another curve, ends touching without a
coincidence, an overlap. So is a pair whose intersection search cannot claim complete
coverage, because a missed crossing would give a wrong profile. Every pair of curves is
intersected, so the cost grows with the square of the number of curves listed.

## Regions from a constraint system

`add_sketch_from_system` adds one loose planar face per region: a closed area the curves
enclose, with one outer boundary and its holes. A rectangle with a circle inside it gives
a ring and a disc, and two crossing rectangles shaped like a cross give a centre and four
arms. The facade is `modeling::faces::from_system`. See
[ADR 0005](../adr/0005-a-system-draws-loose-regions.md) for why the result is faces and
why they are not sewn.

Topology comes from where the curves meet, not from what joins them. Curves meeting at a
crossing, a T or a touch form a node, with or without a coincidence. Coincidences only
give joined ends one shared position, as for profiles. The steps are pure 2D and read no
model:

1. **Cutting.** Every pair of listed curves is intersected, so the cost grows with the
   square of their number. Points within `JOINT_CONTACT` are one node, so three curves
   through one point make one node. A round curve refuses a node farther than
   `LINEAR_TOLERANCE` from its circle (`CutOffCurve`), because an edge ending there would
   not lie on its support. An overlap or an incomplete search is refused.
2. **Walking.** Each piece is walked both ways, turning at each node to the next piece
   clockwise from the one it arrived on, which keeps one area on the walk's left. Pieces
   leaving along one tangent are ordered by signed curvature, which is exact on lines and
   circles and is what separates an arc from the segment it leaves tangentially.
3. **Dropping.** A piece one walk runs both ways has the same area on both sides and
   bounds nothing: an overshoot, a dangling end, a segment joining a rectangle to a
   circle inside it. It is removed, and the stretches of one curve are re-joined at any
   node only they still touch, so nothing of it remains in the model, not even a vertex.
   A curve that bounds nothing simply appears in no region's boundary.
4. **Classifying.** A walk with positive area is a region's outer boundary. The others
   are the outside of a connected group of pieces: a hole of the smallest region
   enclosing that group, found by winding number, or the outside of the drawing. A
   boundary running through a node twice, as where a hole touches the outer boundary at
   one point, is refused (`PinchedRegion`).

Each region then gets its own edges and profiles, running the way they were walked, so
that outer boundaries turn counter-clockwise on the plane and holes clockwise.
`add_face_with_holes_edit` fills them on the sketch plane itself rather than one inferred
from the boundary, which gives every region the plane's normal.

Neighbouring regions therefore each hold a copy of the stretch they share, with the same
end positions. `add_sheet_from_faces` sews them by that geometry, and
`remove_redundant_cells` then fuses a sewn sheet into one face. A face must be extruded
loose or after that fusion: `add_extruded_face` fails on a face already sewn to another.

`SystemRegion::boundary` ties each edge to the curve it lies on. Regions have no id of
their own: their number changes with the system's dimensions, and their order is only
stable for a given input.

## Which model an operation's result lives in

The owned-`Shape` facade (`modeling`) and the language bindings follow one set
of rules about models:

1. A `Shape<K>` is a model plus its primary `K` entity. The model may hold
   other entities besides it.
2. An operation with a primary input produces its result in that input's
   model, kept whole. Rust consumes the input shape and edits its model in
   place; Python and wasm clone it, so the input model is never mutated.
3. A secondary input — a Boolean's tool — contributes only its primary entity
   and the entities that bound it, copied in.
4. A tool already in the target's model is used in place: by key at the
   builder level, by model identity in the bindings.
5. An operation with no primary input, such as a loft, builds a fresh model.

Every entity of the model the operation does not touch keeps its key, so a
caller holding the key of a face an earlier operation produced can still find
it after a later one.

## Extrusion cap correspondence

Extrusion pairs each bottom boundary edge with the translated top span,
including its direction. Cap winding can reverse the walk around a hole, so
the two boundary lists are not zipped by traversal order. The walls are sewn
to the corresponding edges; their vertical curves then meet the stored corners.
