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
- [builders.md](builders.md): builder result names and the return contract.
- [blend.md](blend.md): the chamfer and fillet engine.
- [measure.md](measure.md): mass properties by Green boundary integral.

## Reviews

| Date | Review | Status |
|---|---|---|
| 2026-10-15 | [Mass properties algorithms](reviews/2026-10-15-mass-properties-algorithms.md) | accepted: [ADR 0001](../adr/0001-mass-properties-by-green-boundary-integral.md) |
| 2026-09-20 | [Builders, payload, policy](reviews/2026-09-20-builders-payload-policy.md) | accepted in part |
| 2026-09-04 | [BOOLE paper and ngk](reviews/2026-09-04-boole-paper.md) | reference |
| 2026-09-04 | [Boolean algorithm guide](reviews/2026-09-04-boolean-algorithm-guide.md) | reference |
| 2026-05-28 | [Model API direction](reviews/2026-05-28-model-api-direction.md) | superseded |

## Decisions

- [0001 Mass properties by Green boundary integral](../adr/0001-mass-properties-by-green-boundary-integral.md)
