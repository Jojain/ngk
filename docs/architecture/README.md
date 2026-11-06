# Architecture

How NGK is built, and the trail of reviews and decisions that shaped it.

| Kind | Where | Holds |
|---|---|---|
| Vocabulary | [CONTEXT.md](../../CONTEXT.md) | The domain glossary, no implementation |
| Architecture | this folder | How each subsystem works today |
| Reviews | [reviews/](reviews/) | Surveys and reviews, dated, each with its status |
| Decisions | [../adr/](../adr/) | One short record per hard-to-reverse decision |
| Refactors | [refactors/](refactors/) | Refactors noticed in passing and not yet looked into |
| Plans | [../../plan/](../../plan/) | Work not done yet |

An architecture document describes the code as it is and changes in the same
commit as the code it describes. A review is a snapshot: it is not updated
afterwards, only given a new status line. A plan is deleted once its work has
landed and its lasting content has moved here or into an ADR.

## Subsystems

- [overview.md](overview.md): what NGK is, the layers, the model.
- [topology.md](topology.md): the map, entities, corners and edges,
  boundaries, embedding, identity and orientation.
- [transactions.md](transactions.md): the edit capability, commit order,
  lineage, payload policy, identity reconciliation.
- [geometry.md](geometry.md): supports and spans, parameter spaces,
  intersection dispatch.
- [solver.md](solver.md): editable 2D constraint systems, local solving and diagnostics.
- [sketch.md](sketch.md): from a solved constraint system to profiles.
- [builders.md](builders.md): builder result names and the return contract.
- [blend.md](blend.md): the chamfer and fillet engine.
- [offset.md](offset.md): offsets of edges, profiles, faces, sheets and solids.
- [measure.md](measure.md): mass properties by Green boundary integral.
- [tessellation.md](tessellation.md): shared boundary sampling and constrained triangulation.
- [exchange.md](exchange.md): STEP B-rep interchange and binary STL export.

## Reviews

| Date | Review | Status |
|---|---|---|
| 2026-10-22 | [Constraint solvers: representations and CAD integration](reviews/2026-10-22-constraint-solvers-survey.md) | reference: exploratory architecture survey |
| 2026-10-22 | [Boolean kernels: NGK, OCCT and SMLib](reviews/2026-10-22-boolean-kernels-comparison.md) | reference: source comparison and NGK verification |
| 2026-10-15 | [Mass properties algorithms](reviews/2026-10-15-mass-properties-algorithms.md) | accepted: [ADR 0001](../adr/0001-mass-properties-by-green-boundary-integral.md) |
| 2026-09-20 | [Builders, payload, policy](reviews/2026-09-20-builders-payload-policy.md) | accepted in part |
| 2026-09-04 | [BOOLE paper and ngk](reviews/2026-09-04-boole-paper.md) | reference |
| 2026-09-04 | [Boolean algorithm guide](reviews/2026-09-04-boolean-algorithm-guide.md) | reference |
| 2026-05-28 | [Model API direction](reviews/2026-05-28-model-api-direction.md) | superseded |

## Decisions

- [0001 Mass properties by Green boundary integral](../adr/0001-mass-properties-by-green-boundary-integral.md)
- [0003 Invariant-preserving edit operators](../adr/0003-invariant-preserving-edit-operators.md)
- [0004 Panics are not recoverable modeling errors](../adr/0004-panics-are-not-recoverable-errors.md)
