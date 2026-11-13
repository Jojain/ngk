# Assembly solver: what comes next

The rigid-body assembly solver is built and described in
[assembly.md](../docs/architecture/assembly.md): `Assembly3d` in
`ngk::solver::assembly`, the eleven standard joints, unwrapped and seeded
coordinates, imposed values, limits, serialization and the `ngk.solver.assembly`
Python module. Its numerical core is shared with the 2D solver
([solver.md](../docs/architecture/solver.md)). The requirements were agreed in a
design discussion whose decisions now live in that document. This plan keeps what was
deliberately left out, each kept possible by the design.

## Left out, kept possible

- **Couplings between joints** (gear ratio, rack and pinion, a screw between two
  joints): linear equations on named joint coordinates, which is why coordinates are
  first-class values with references.
- **Joint-coordinate targets**: a `Target3d` pulling a coordinate toward a value, for
  a slider in a GUI. Only point-to-position targets exist.
- **A geometric-constraint front** (face on face, concentric, distance between
  features) over the joints' elementary rows, for callers that prefer picking two
  faces to placing two frames. Today a caller extracts two frames and adds a joint.
- **Rigid groups** inside the solver, to switch a sub-assembly between rigid and
  flexible without reinserting it; today they are expressible as fixed joints.
- **Explicit branch selection**: a solve always takes the nearest solution.
- **WASM bindings and a visualization playground** like `solver-2d`.

## Limitations to revisit when a caller meets them

- **The active set only grows within one solve.** A target that first pushes a
  coordinate into a limit and then would pull it away in the same call stays on the
  limit; the next call starts with no side active.
- **A seed exactly upside down** on an orientation equation fails to converge rather
  than choosing a turn axis. A caller picking two facing faces passes one frame
  flipped.
- **Structural reduction** (pre-solving rigid sub-graphs) is absent. Measure large
  assemblies before adding it.
