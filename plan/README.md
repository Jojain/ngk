# ngk implementation plans

This directory contains implementation plans for substantial kernel work.

| Plan | Status | Purpose |
|---|---|---|
| [NURBS surface/surface intersection](nurbs_surface_surface_intersection.md) | In progress | Replace the sampled triangle approximation with a topology-safe, tolerance-controlled intersection engine |
| [Boolean evaluation](boolean_evaluation.md) | Proposed | Complete the regularized solid Boolean: clipping, network finalization, fragment classification, selection, and GMap assembly |
| [Shape healing](shape_healing.md) | In progress | Remove redundant topology left by Booleans and imprints: `i`-removal of shape-free vertices and edges, fusing the cells they separate |
| [Analytical curves and surfaces](analytical_geometry.md) | In progress | Add sphere, cone, torus, ellipse, hyperbola and parabola supports behind a stated geometry contract, and close the paths where an unrecognized support silently degrades |
| [Seamless periodic faces](seamless_periodic_faces.md) | Complete | Stop storing a seam edge on periodic faces: ring faces, vertexless closed edges, boundaryless faces, an unwrapped-domain cut synthesized on demand, and a healing pass that takes an imported seam apart |
| [Curved-support pcurve rebuild](curved_support_pcurve_rebuild.md) | Proposed | Let healing rebuild a parameter curve on a curved support, so a rim split by a Boolean heals back into the one closed edge it started as |
| [STEP interop](step_interop.md) | In progress | Bidirectional ISO 10303-21 B-Rep exchange: a four-layer split, a registry-driven geometry mapping with a certified NURBS fallback, and seam synthesis in both directions |

Statuses used by the plans:

- **Proposed** — designed but implementation has not started;
- **In progress** — at least one implementation milestone is active;
- **Blocked** — progress requires an unresolved technical or API decision;
- **Complete** — every definition-of-done item is satisfied.
