# WS G: contacts and tolerances

**Owns:** `src/builders/boolean/{contacts,tolerance,clip,neighborhood,classify}.rs`.
**Decision:** D4 for tangent contacts. **Bench:** about 6 cases.

## G1. The grazing tolerance applied to transverse contacts (class D)

**Symptoms.** A slot cut in a cube of side 10 works at width 3e-3 and fails
at 1e-3 and below (`imprint 1 of face … was not realized`), whether the slot
runs through the cube or stops inside it. A cube united with itself turned
0.001° fails the same way. A block of side 1000 with a 1e-3 cube cut from it
works, so this is not a scale-to-tolerance problem.

**Root cause (to confirm).** `BooleanTolerances::graze` is
`sqrt(linear × model_scale)`, about 5e-4 on a part of size 17: right for a
double root, where a solver locates a tangent contact only to the square
root of its tolerance. `contacts.rs` passes `graze` to edge/edge and
edge/face probing and to the anchor snapping in `clip_deferred`, so distinct
transverse crossings closer than about `graze` are merged into one contact.
The minimum feature a Boolean can keep is then about 1e-4 of the model's
size instead of the linear tolerance.

**Fix.** Use `graze` only for contacts certified tangent (a double root:
the surfaces' normals parallel at the point), and the linear tolerance for
transverse crossings and anchor snapping.

**Family.** Slots, steps, holes and fins of width 1e-2, 1e-3, 1e-4, 1e-6 on
parts of size 1e-2, 1, 1e3; two boxes turned 1°, 0.1°, 0.01°, 0.001°
against each other; a cylinder offset 1e-5 from a block face.

## G2. Tangent contacts (class B3)

**Symptoms.** A cube united with a half cylinder tangent to two of its faces
(`half-cylinder-boss-tangent`), a cube intersected with its midsphere: the
contact is imprinted as a section, and a section that separates nothing is
not realized. The audit's B3 holds.

**Fix.** Keep tangent contacts apart (`Contact::Imprint { tangent }`
already says which) and imprint them only where the result needs an edge
there: never for a cut or intersection that only touches; for a union whose
result would be non-manifold, refuse by name (D4).

## G3. A point contact read as two solids

`tangent/cone-tip-on-face`: a cone whose apex touches a block's face from
inside, united with the block, reports `regularized Boolean result has 2
disconnected solids`. The cone's part inside the block is classified on its
own. A point contact from inside must not split the cell graph.

## G4. Booleans on Booleans' results

**Symptoms.** A half ball made by a Boolean, then used as an operand:
united with a plate under it, or intersected with a post, fails with
`NURBS conversion failed: parameter interval is degenerate: [0, 0]`;
united with a block crossing its sphere and its flat face, fails with
`intersection loop is open`.

**Root cause (to confirm).** The half ball's sphere face is bounded by the
equator and carries its pole as a degenerate point or zero-length span; some
probe converts that span to NURBS (`conic_arc_nurbs` refuses an interval of
zero length). The open loop is likely the audit's B2 (a sampled trim polygon
taken as a conservative bound) on the trimmed sphere face.

**Fix.** Probes skip degenerate spans (a pole is a point, probed as one);
B2's fix per the audit.

**Family.** Every Boolean in the tests, run again with its result as an
operand of a second Boolean against a block, a cylinder and a ball crossing
it. Hemisphere, napkin ring, half torus, drilled block, filleted block.
