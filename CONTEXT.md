# NGK

A CAD geometry kernel whose topology is a generalized map (3-GMap), with
geometry and user data attached to the logical entities read off that map.

## Language

### Two layers: map and entities

**The one rule: a bare noun is always the logical entity.** "Face" means a
logical face; a thing the map holds is a 2-cell or a raw face, never just "a
face".

**Dart**:
An oriented traversal locator in the map. Short-lived: any edit may destroy it.
_Avoid_: half-edge, handle

**N-cell** (raw cell):
What the map's involutions group at dimension N: 0-cell, 1-cell, 2-cell, 3-cell.
"Raw vertex", "raw edge", "raw face" are synonyms; `N-cell` is the default.
_Avoid_: a bare "vertex", "edge" or "face" for a map cell

**Entity** (logical entity):
What a shape is made of: a vertex, edge, face, profile, sheet or solid. It has a
durable key.
_Avoid_: cell, element

**Key**:
The durable identity of an entity within one model. It does not say which model
it came from.
_Avoid_: id, handle

**Model**:
One map plus everything a shape is: entities, geometry, payload. Every mutation
happens inside one of its transactions.

**Shape**:
A model plus its primary entity.
_Avoid_: body, part

**Transaction**:
The atomic boundary of one modeling operation: it commits whole or restores the
model it started from.

### Payload

**Payload**:
User data carried by entities, one type per dimension.
_Avoid_: attribute (that word names the kernel's stored record)

**Policy** (edit policy):
The rule that produces and maintains payload when entities are created, merged
or consumed. It belongs to the payload type, not to the call site.

**Origin**:
What a created entity came from: new, split from one source of its own kind, or
derived from sources of any kind.
_Avoid_: parent, history

**Merged / consumed**:
An entity folded into a named survivor, or removed with nothing inheriting it.

### Corners and edges

**Corner**:
A vertex in the role of bounding something. Corners are always vertices.
_Avoid_: endpoint (an edge can have no corner at all)

**Distinct corners / corner visits**:
How many different vertices bound a thing, versus how many times a walk arrives
at one. A marked edge has one distinct corner and two visits.

**Bounded edge**:
An edge with two distinct corners: a segment, an arc.

**Marked edge**:
A closed edge with one corner, which is both its start and its end: a circle
someone cut.

**Unmarked edge**:
A closed edge with no corner: a circle as built.
_Avoid_: "closed edge" when the two closed kinds must be told apart

**Cut**:
Adding a corner to an edge. Cutting an unmarked edge marks it; cutting a bounded
or marked edge separates it in two.

### Boundaries

These name boundary components of entities, never components of the map.

**Profile**:
A maximal connected set of edges. It exists without a face: an open wire is a
profile.
_Avoid_: wire (except when quoting STEP or OCCT)

**Loop**:
A face's walk along a closed profile: which side, and how often each edge is
run. A loop names its profile; it is not the profile.

**Sheet**:
A maximal connected set of faces.

**Shell**:
A closed sheet, bounding a solid.

### Embedding

**Embedded cell**:
A raw cell that carries no entity of its own dimension and lies in the interior
of an entity of higher dimension: its **owner**.

**Scaffold**:
An embedded cell the topology needs but the shape does not show: a bridge, a
seam, the 0-cell where a circle closes, a cut face joining a cavity to its outer
shell.

**Bridge**:
A raw edge a face owns, joining its outer boundary to a hole.

**Seam**:
A raw edge a face owns where a periodic surface closes on itself.

**Turn**:
Stepping around a shared boundary cell, crossing scaffold rather than stopping
at it. Profile and sheet walks turn.

### Geometry

**Support**:
An unbounded curve or surface carrying an entity: a line runs to infinity, a
circle closes.
_Avoid_: "the edge's curve" when the span matters

**Span**:
The part of a support an entity occupies, as an interval of native parameters.

**Trimmed curve**:
A support plus a span, held as one value.

**Pcurve**:
A face's boundary edge expressed as a trimmed curve in the face surface's
parameter space.
_Avoid_: UV curve, 2D curve

**Native parameter / fraction**:
A support's own parameter (a line's affine, a conic's angle, a NURBS knot
value), versus a traversal fraction of a named span, 0 at its start and 1 at
its end.
_Avoid_: normalized parameter (without naming the span)

### Measurement

**Mass properties**:
How much of a shape there is, where it sits (centroid) and how it is spread
about that point (inertia), at density one.
_Avoid_: mass (there is no density)

**Linear / surface / volume properties**:
Mass properties of a curve, a surface or a solid.

**Inertia**:
The second moment of a shape's measure about its centroid.
