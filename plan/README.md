# ngk implementation plans

This directory contains implementation plans for substantial kernel work.

| Plan | Status | Purpose |
|---|---|---|
| [Logical topology over GMap](logical_topology_over_gmap.done.md) | Complete | Pure GMap connectivity beneath stable logical entities, atomic model transactions, embedded scaffold, and full feature-parity migration |
| [One logical cell, one raw cell](one_logical_cell_one_raw_cell.done.md) | Complete | Enforce one same-dimensional raw cell per logical entity, including boundaryless faces and multi-shell solids |
| [NURBS surface/surface intersection](nurbs_surface_surface_intersection.md) | In progress | Replace the sampled triangle approximation with a topology-safe, tolerance-controlled intersection engine |
| [Boolean evaluation](boolean_evaluation.md) | Proposed | Complete the regularized solid Boolean: clipping, network finalization, fragment classification, selection, and GMap assembly |
| [Shape healing](shape_healing.md) | In progress | Remove redundant topology left by Booleans and imprints: `i`-removal of shape-free vertices and edges, fusing the cells they separate |
| [Analytical curves and surfaces](analytical_geometry.md) | In progress | Add sphere, cone, torus, ellipse, hyperbola and parabola supports behind a stated geometry contract, and close the paths where an unrecognized support silently degrades |
| [Seamless periodic faces](seamless_periodic_faces.done.md) | Complete | Stop storing a seam edge on periodic faces: ring faces, vertexless closed edges, boundaryless faces, an unwrapped-domain cut synthesized on demand, and a healing pass that takes an imported seam apart |
| [Curved-support pcurve rebuild](curved_support_pcurve_rebuild.md) | Proposed | Let healing rebuild a parameter curve on a curved support, so a rim split by a Boolean heals back into the one closed edge it started as |
| [Periodic supports](periodic_supports.md) | Proposed | Separate "closed" from "periodic parameterization": let a closed NURBS span cross its own seam, and leave room for a helix, which repeats without ever closing |
| [Transforms](transform.md) | In progress | A total `Rigid` motion for the common case, a general affine `Transform<D>` for the rest, the parameter remap a support owes when its parameterization moves, and the orientation reversal a mirror owes on top |
| [Parameter units](parameter_units.md) | Proposed | Stop `f64` meaning native parameter, normalized fraction, knot parameter and arc length at once: brand the scalar and its interval, and make a fraction inexpressible without the span it is a fraction of |
| [STEP interop](step_interop.done.md) | Complete | Bidirectional ISO 10303-21 B-Rep exchange through the supported profile, including analytic/NURBS geometry, seam synthesis, and cavities |

Statuses used by the plans:

- **Proposed** — designed but implementation has not started;
- **In progress** — at least one implementation milestone is active;
- **Blocked** — progress requires an unresolved technical or API decision;
- **Complete** — every definition-of-done item is satisfied.
