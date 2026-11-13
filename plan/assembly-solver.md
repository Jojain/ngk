# Assembly solver

Status: stages 1 and 2 built; the solver as it is is described in
[assembly.md](../docs/architecture/assembly.md). This plan records the decisions for
a rigid-body positioning solver and the stages still to build. The 2D
sketch solver it sits next to is described in
[solver.md](../docs/architecture/solver.md); its requirements and design reasoning
are in [solvers.md](solvers.md) and [solver-architecture.md](solver-architecture.md).

The client is Nales: an assembly Component places Component Instances by joints
between them. This plan designs the NGK solver only. How Nales extracts attachment
frames from faces and edges, stores joints, and applies the solved poses to its
Instances is Nales' design.

## Scope

- **Rigid bodies only.** Each body has six degrees of freedom and the solver
  computes one pose (`Rigid`) per body. The geometry a body stands for never
  deforms and never reaches the solver.
- **Positioning and interactive dragging.** `solve` positions the assembly;
  `solve_toward` drags a body along the freedoms its joints leave, so a mechanism
  can be moved by hand. No dynamics (mass, forces), no collision or contact
  detection, no time-based motion study.
- **Flat.** The solver knows no hierarchy and no Component. See
  [Sub-assemblies](#sub-assemblies).

## Public shape

A stateful `Assembly3d` in `ngk::solver::assembly`, built on the same philosophy as
`System2d`:

- typed ids issued by `add_*` and never issued twice; `insert_*` takes back an id
  issued earlier so a caller rebuilds the system from its own source;
- `edit` applies a batch atomically as one revision; a failed batch leaves the
  system intact;
- removing a body refuses while joints reference it; `remove_cascade` removes them
  and reports their ids;
- `solve()` and `solve_toward(&[Target3d])` return `Result<Solution3d, SolveError>`;
  success publishes the whole configuration with an immutable solution, failure
  publishes nothing, and the last success stays readable;
- `SolveOptions` defaults to the kernel's `LINEAR_TOLERANCE` and
  `ANGULAR_TOLERANCE`;
- the system serializes its options, bodies, joints and captured values, as
  `System2d` does.

No dependency on `Model`, GMap, builders or Nales.

| Type | Role |
| --- | --- |
| `Assembly3d` | Editable system, last solution |
| `BodyId` | A body: a pose the solver moves rigidly, with no shape |
| `JointId`, `Joint3d` | A joint's identity and its definition: a `JointKind` and two attachments |
| `Target3d` | A temporary objective of one `solve_toward` call |
| `Solution3d` | Solved poses, joint coordinates, report |

### Bodies

A body is an id plus a seed pose. The caller keeps the correspondence between
bodies and whatever they stand for: one Component Instance, several shapes moved
together, or nothing at all (a datum, an abstract ground). There is deliberately no
`ShapeId`: the mapping is not one to one, and solver ids must not be mistaken for
a model's keys.

**Fixing** a body captures its pose when the fixing is created. Updating the body's
seed afterwards does not move it; replacing the fixing recaptures it, as fixing a
point does in `System2d`. Any number of bodies may be fixed, including none. With
none, the assembly keeps a free global motion and the solver moves it as little as
possible from the seeds.

## Joints

The public vocabulary is **joints between two frames**, one on each body. Below
it, a private layer of elementary relations on frames (origin on an axis, origin on
a plane, axes parallel, …) builds every joint. A public "geometric constraint"
catalog (face coincident with face, concentric, distance between features) is not
part of this plan. The private layer is shaped so one can be added later as
another front without touching the numerical core. Picking two faces in a GUI is
caller sugar: extract two frames, add a planar joint.

Why joints rather than geometric constraints: a joint's freedoms are known in
advance, so the graph of bodies and joints is the kinematic diagram, and
mobility and redundancy are reported in terms a user understands. Joint
coordinates are named, so they can be read, imposed, limited, and later coupled.
The two frames fix the orientation completely, so there is no aligned/opposed
ambiguity.

### Frame convention

A joint relates frame `a` local to body A and frame `b` local to body B, in that
order. The order matters because coordinates express B's frame in A's frame.

- At joint value zero the two frames coincide exactly: origin, X, Y and Z.
- Z carries the joint's axis: the revolute axis, the slide direction, the planar
  normal.
- There is no flip flag. A caller wanting the other sense passes the flipped frame.

### Catalog

The eleven standard joints (French *liaisons normalisées*):

| Joint | French | Freedoms | Coordinates (B in A) |
| --- | --- | --- | --- |
| `Fixed` | encastrement | 0 | none |
| `Revolute` | pivot | 1R | θ about Z |
| `Prismatic` | glissière | 1T | z along Z |
| `Helical` | hélicoïdale | 1 coupled | θ about Z; z = pitch·θ/2π, read only |
| `Cylindrical` | pivot glissant | 1T 1R | θ, z |
| `Universal` | rotule à doigt | 2R | α about X_A, then β about Y_B; twist about Z blocked |
| `Spherical` | rotule | 3R | a 3D rotation |
| `Planar` | appui plan | 2T 1R | x, y in A's XY plane, θ about Z |
| `CylinderOnPlane` | linéaire rectiligne | 2T 2R | x, y, θ, and φ about X_B; B's X axis stays in A's XY plane |
| `SphereInCylinder` | linéaire annulaire | 1T 3R | z along Z_A, and a 3D rotation |
| `SphereOnPlane` | ponctuelle | 2T 3R | x, y in A's XY plane, and a 3D rotation |

The helical pitch is signed (positive right-handed, negative left-handed) and
nonzero.

Refused at edit with `EditError`: a joint whose two frames are on the same body;
a zero pitch. A joint between two fixed bodies is accepted and checked at solve,
where it is a compatible redundancy or a named conflict.

### Coordinates

Every joint exposes its coordinates in the solution, named by role. A 3D rotation
reads as a rotation, never as three scalars.

- **Angles are unwrapped.** They are continuous from the seed and may run past one
  turn, which the helical joint and a limit such as `[-30°, 400°]` need. A pose
  only gives an angle modulo one turn, so each angular coordinate stores its
  unwrapped **seed value**. The solver picks the representative nearest that seed,
  and the caller writes the solved value back after a drag, as Nales writes back
  the seeds of a Sketch. Without a stored seed the value is read off the poses in
  `(-180°, 180°]`.
- **Imposing.** A scalar coordinate can be set to a value, which acts as a
  dimension ("revolute at 30°", "prismatic at 12 mm"). A 3D rotation cannot be
  imposed; to freeze it the caller replaces the joint. For the helical joint only θ
  is free, so only θ is imposed.

### Limits

- Any scalar coordinate accepts `[min, max]`, either side optional. A helical
  limit may be given on z and is converted to a limit on θ.
- A free 3D rotation (spherical, sphere in cylinder, sphere on plane) accepts
  only a **cone limit**: the angle between Z_A and Z_B is at most a maximum.
- Limits are inequalities. An inactive limit removes no freedom. `solve` brings a
  coordinate back within its bounds with minimal motion; `solve_toward` stops
  against a limit, and the target is reported as not reached, without an error.
- The report lists the active limits. Mobility is reported without limits, the
  active limits beside it.
- `min > max`, or an imposed value outside the bounds, is an `EditError`, found
  without solving. A loop closure that forces a coordinate past a bound is a
  `SolveError` naming the joint and the limit.

The numerical treatment of the inequalities inside the minimum-norm solver
(active set or another method) is open and is settled in stage 4.

## Solving

- **Variables.** Six per body: a translation and an incremental rotation vector
  about the current orientation, recomposed at each iteration. No Euler angles, no
  quaternion with a norm constraint. Joints are equations on poses, not reduced
  coordinates, so closed loops (a four-bar linkage) need no special case. Joint
  coordinates are read back from the poses.
- **Continuity.** No deliberate jump. `solve` starts from the seeds, `solve_toward`
  from the last solution, and each takes the nearest solution. A four-bar linkage
  stays on its branch; there is no explicit branch selection.
- **Under- and over-constraint.** The same contract as `System2d`: an
  underconstrained assembly solves with minimal motion; a compatible redundancy
  (a hyperstatic assembly) succeeds with a warning; an incompatibility is a
  `SolveError` naming the joints at fault.
- **Report.** Remaining freedoms per body, the assembly's mobility and its degree
  of redundancy (hyperstatism), active limits, residuals and iteration counts.
- **Targets.** `Target3d` brings a point local to a body toward a world position:
  the user grabs a part where they clicked and drags it, and every joint follows.
  Several targets may be given with weights. Targets do not consume freedoms.
- **Scale.** Interactive dragging (about 16 ms per `solve_toward`) up to about 50
  bodies and 100 joints, with independent connected components solved
  separately as in 2D. Larger assemblies (hundreds of bodies) must solve, without
  an interactivity requirement. No structural reduction (pre-solving rigid
  sub-graphs).

## Sub-assemblies

The solver is flat, and both cases are handled by the caller:

- a **rigid** sub-assembly is inserted as one body;
- a **flexible** sub-assembly, whose internal joints move in its parent, is
  flattened: the caller inserts its bodies and internal joints into the parent
  system with its own ids (`insert_*`), then attaches the parent's joints to its
  inner bodies.

What this requires now: nothing assumes one body per Component Instance, ids are
opaque and may come from the caller, and every pose is expressed in one world
frame.

## Stages

1. **Shared numerical core.** Extract from `src/solver/numerical/` what is not
   2D-specific: sparse forward derivatives, the damped minimum-norm SVD step,
   projection of targets onto the null space of the permanent Jacobian, rank and
   redundancy diagnostics, component decomposition. `System2d` uses it and its
   tests stay green. Two real clients justify the extraction.
2. **`Assembly3d` core.** Bodies, fixing, `Fixed`, `Revolute` and `Prismatic`,
   point-to-position targets, report with freedoms, mobility and redundancy.
   Rust only.
3. **The rest of the catalog.** The eight remaining joints, unwrapped angle seeds,
   imposed coordinates.
4. **Limits.** Scalar bounds and cone limits, the inequality method, active limits
   in the report.
5. **Serialization and Python bindings.** `Assembly3d` serializes like `System2d`;
   `ngk.solver` exposes it to Python for Nales.

Stages 1 and 2 are built: the numerical core is shared (`src/solver/numerical/`) and
`Assembly3d` solves fixed, revolute and prismatic joints, documented in
[assembly.md](../docs/architecture/assembly.md). Each stage that lands is documented
there; this plan keeps the agreed decisions until the last stage is built.

## Out of scope, kept possible

- **Couplings between joints** (gear ratio, rack and pinion, screw between two
  joints): linear equations on named joint coordinates, which is why those
  coordinates are first-class variables.
- **Joint-coordinate targets**: a `Target3d` pulling a coordinate toward a value,
  for a slider. Only point-to-position targets are in scope.
- **A geometric-constraint front** on the private relation layer.
- **Rigid groups** inside the solver, to switch a sub-assembly between rigid and
  flexible without reinserting it; today they are expressible as fixed joints.
- **Explicit branch selection.**
- **WASM bindings and a visualization playground** like `solver-2d`.
