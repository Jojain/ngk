# ngk implementation plans

This directory contains implementation plans for substantial kernel work.

| Plan | Status | Purpose |
|---|---|---|
| [NURBS surface/surface intersection](nurbs_surface_surface_intersection.md) | In progress | Replace the sampled triangle approximation with a topology-safe, tolerance-controlled intersection engine |
| [Boolean evaluation](boolean_evaluation.md) | Proposed | Complete the regularized solid Boolean: clipping, network finalization, fragment classification, selection, and GMap assembly |
| [Shape healing](shape_healing.md) | In progress | Remove redundant topology left by Booleans and imprints: `i`-removal of shape-free vertices and edges, fusing the cells they separate |

Statuses used by the plans:

- **Proposed** — designed but implementation has not started;
- **In progress** — at least one implementation milestone is active;
- **Blocked** — progress requires an unresolved technical or API decision;
- **Complete** — every definition-of-done item is satisfied.
