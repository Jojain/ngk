# Revolution and rounded sweep walls

`src/builders/revolve/operations.rs` and
`src/builders/sweep/operations.rs` both construct triangular walls when one
section endpoint lies on a turning axis, omit section edges wholly on it,
and sew the section boundaries to caps. Each also checks NURBS control hulls
against an axis half-plane.

A shared internal wall builder could own the triangle/quad boundary contract,
including the direction of returning circular arcs and the optional swept
rails. The callers would retain section validation, support selection,
lineage, and shell assembly. Full-turn revolutions also need single-rim and
two-rim faces, which rounded sweep transitions do not construct.

Not checked: all sweep callers, payload-policy differences, parameter mappings
for generic and analytic supports, or whether sharing the assembly actually
reduces complexity enough to justify coupling the two builders.
