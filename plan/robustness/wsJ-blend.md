# WS J: blends that consume faces or meet marked edges

**Owns:** `src/builders/blend/`. **Decision:** none.
**Bench:** 10 cases. Lower priority: these are the limits of an engine that
works, not wrong answers.

## Classes

1. **A blend that consumes a face entirely.** Two rounds or chamfers meeting
   tangentially across a face (`slab-half-round`, `block-ridge`,
   `hole-waist`), a round or chamfer equal to the rim's radius
   (`cylinder-rim-to-dome`, `cylinder-rim-to-point`), a concave blend wide
   enough to consume both faces (`l-block-inside-consumed`). Refused with
   "the blends at its two ends overlap", "crossing or inverted boundaries",
   "a rail runs off the edge beside it". The fix is one rule in the surgery:
   a face whose rails meet or cross is removed and its neighbours' rails are
   joined, and a rim that shrinks to a point leaves a vertex.
2. **Marked closed edges.** A Boolean leaves a corner on a circle
   (`boss-s-curve`: "it closes on a corner, where its blend would meet its
   own start"). Either the blend treats a marked closed edge like an
   unmarked one when the corner joins it to itself, or healing after the
   Boolean removes corners no other edge meets (the more general fix: such
   corners also hamper WS B and the tessellator).
3. **Corner cut together with the edges that meet there**
   (`chamfer/cube-all-edges`): refused as conflicting selections; it is the
   flat corner the description asks for.
4. **A vertex where four faces meet** (`octahedron-all-edges-mitred`): only
   three-valent vertices are blended.

## Tests

Each class on planar and curved faces, convex and concave, for fillet and
chamfer, with the bench cases as integration tests.
