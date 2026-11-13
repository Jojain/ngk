# Assembly solver

`ngk::solver::assembly` exposes a stateful `Assembly3d`: rigid bodies and the joints
between them, the seeds they start from, and the last successful solution. It shares
the numerical core of the [2D solver](solver.md) and its contract: typed ids, atomic
batches, `solve` and `solve_toward`, nothing published on failure. It depends on NGK
geometry values only, not on `Model`, GMap, builders or Nales. What was left out is
in [plan/assembly-solver.md](../../plan/assembly-solver.md).

## Public workflow

```rust
use ngk::geometry::{dim3::frame::Frame, dim3::utils::Point3, transform::Rigid};
use ngk::solver::assembly::{Assembly3d, Attachment, Joint3d, JointKind, Target3d};

let mut assembly = Assembly3d::default();
let ground = assembly.add_body(Rigid::identity())?;
assembly.fix(ground)?;
let arm = assembly.add_body(seed)?;
assembly.add_joint(Joint3d::new(JointKind::Revolute, 
    Attachment::new(ground, Frame::at(Point3::new(0., 0., 0.))),
    Attachment::new(arm, Frame::at(Point3::new(0., 0., 0.))),
))?;
let solution = assembly.solve()?;
let pose: Rigid = solution.pose(arm)?;
let dragged = assembly.solve_toward(&[
    Target3d::point(arm, Point3::new(1., 0., 0.), Point3::new(0., 2., 0.)),
])?;
```

## Bodies

A body is an id and a seed pose (`Rigid`). It holds no shape: whether it stands for
one shape, several moved together, or none is the caller's mapping, and no id of the
assembly is a model key. Seeds must be finite.

`fix` holds a body at its seed as it is when fixed; `update_body` afterwards moves the
seed, not the fixing, and fixing again recaptures it. A fixed body is a constant of
the problem, not a set of unknowns. Any number of bodies may be fixed, none included:
an assembly with none keeps a free global motion and moves as little as possible.

A success becomes the seeds of the next solve, so `solve_toward` drags from where the
last success left the bodies. Removing a body refuses while joints hold it;
`remove_cascade` removes them and reports their ids.

## Joints

A `Joint3d` holds a frame on each of two bodies (`Attachment`, a frame in the body's
own coordinates) and a `JointKind`. At joint value zero the two frames coincide —
origin, X, Y and Z — and Z carries the joint's axis. The order matters: a joint's
coordinates express the second frame in the first. There is no flip flag; a caller
wanting the other sense passes the flipped frame.

| Kind | Coordinates (second in first) | Equations |
|---|---|---|
| `Fixed` | none | origins coincide (3), orientations coincide (3) |
| `Revolute` | `Angle` | origins coincide (3), Z axes coincide (2) |
| `Prismatic` | `Slide` | second origin on the first Z axis (2), orientations coincide (3) |
| `Helical { pitch }` | `Angle`; `Slide` follows | on the axis (2), Z axes (2), slide = pitch · angle / 2π (1) |
| `Cylindrical` | `Angle`, `Slide` | on the axis (2), Z axes (2) |
| `Universal` | `Alpha`, `Beta` | origins coincide (3), first X ⟂ second Y (1) |
| `Spherical` | a free rotation | origins coincide (3) |
| `Planar` | `X`, `Y`, `Angle` | second origin on the first XY plane (1), Z axes (2) |
| `CylinderOnPlane` | `X`, `Y`, `Angle`, `Roll` | on the plane (1), second X in the plane (1) |
| `SphereInCylinder` | `Slide`, a free rotation | on the first Z axis (2) |
| `SphereOnPlane` | `X`, `Y`, a free rotation | on the first XY plane (1) |

`X`, `Y` and `Slide` are the second origin along the first frame's axes. `Angle` turns
the first X onto the second about the first Z. The universal joint is a cardan joint:
`Alpha` about the first X, then `Beta` about the second Y, with no twist about Z, so the
first X and the second Y stay square. `Roll` turns a cylinder lying on a plane about its
own axis, the second X. The helical pitch is the slide per turn, positive for a
right-handed screw. A free rotation has no scalar coordinates; `Solution3d::relative`
reads any joint's motion as the pose of its second frame in its first.

Refused at edit: a joint holding the same body twice, a body that is not in the
assembly, a frame that is not finite, orthonormal and right-handed, a helical pitch
that is zero or not finite, and an imposed value that is not finite or not on a free
coordinate of the joint.

### Coordinates, seeds and imposed values

Angles are unwrapped: continuous from where they start and free to run past a turn.
A pose only gives an angle modulo a turn, so each angular coordinate of a joint has a
**seed**, its unwrapped value. A solve reads each angle at the start poses and takes
the representative nearest the seed (the value in `(-π, π]` without one); during the
solve the angle stays within half a turn of that reference. A success replaces the
seeds with the solved values, so dragging a crank round counts its turns.
`set_coordinate_seed` gives a seed back, such as one stored with the caller's
document; only an angle of the joint has one. `update_joint` keeps the seeds of the
angles the new kind still has.

`Joint3d::with_value` imposes a coordinate, as a dimension: an angle in radians,
unwrapped, or a length in the caller's unit. It adds one equation and removes one
freedom. An imposed angle is its own reference, so a helical joint imposed at 720°
advances two pitches. A helical slide follows its angle and is not imposed.

### Orientation equations

An orientation row measures the angle `θ` between the frames as `2 tan(θ/2)`. For the
full orientation the three rows are `2 vee(R - Rᵀ) / (1 + tr R)`, with `R` the second
frame's axes in the first; for a Z axis the two rows are the second Z in the first
frame's X and Y, over `(1 + cos θ) / 2`. Near zero a row equals the angle, so angular
tolerances read directly; it vanishes only when the frames agree, and grows without
bound as they turn over. A smooth row such as `sin θ` would also vanish upside down, and
a correction from a seed turned over would settle there. A seed exactly upside down is
singular and fails to converge rather than choosing a turn axis.

## Solving

Each free body has six unknowns: its translation, normalized by the component's
origin and scale, and a rotation vector applied to its start orientation through
Rodrigues' formula, with Taylor coefficients near zero. There are no Euler angles and
no quaternion norm constraint. Joints are equations on the bodies' frames, not reduced
coordinates, so a closed loop needs no special case.

Free bodies joined by joints form a component; a fixed body joins nothing, being
constant, and is borrowed as a constant by every component whose joints hold it. A
joint between two fixed bodies is checked in a component with no unknowns. Each
component goes through the shared `numerical::Problem`: damped minimum-norm
restoration from the seeds, then targets pursued in the joints' null space. Every
joint is checked again on the published poses in the caller's units before anything
is published.

`Target3d::point` pulls a point given in a body's own coordinates toward a world
position, weighted among targets. An unreachable target stops where the joints allow
and is reported as not reached.

## Limits

`Joint3d::with_limit` bounds a scalar coordinate by a `Limit`, either side optional,
in the coordinate's units; a helical slide may be bounded, and follows its angle.
`with_cone` bounds a free rotation (spherical, sphere in cylinder, sphere on plane): the
second Z stays within an angle in `(0, pi]` of the first. Refused at edit: a limit on a
coordinate the joint does not have, a side that is not finite, no side at all, `min`
above `max`, a cone on a joint without a free rotation, and an imposed value outside
its limit, a helical slide driven by an imposed angle included.

Limits are inequalities, solved by an active set that only grows within one solve.
Each round restores feasibility from the seeds with the active sides held as
equalities, pursues the targets, and then adds the most violated inactive side, until
none is violated beyond tolerance. A seed beyond a limit thus comes back to it with
minimal motion, and a drag stops against it with its target reported as not reached.
The next solve starts with no side active, so a drag pulled back leaves the limit. An
active side whose restoration fails, a loop forcing a coordinate past its bound, is
`SolveError::Limit` naming the joint and the side (`Bound`). The published poses are
audited against every limit as against every joint.

`SolveReport::active_limits` lists the sides the solution stands against, held or
touching within tolerance. `mobility` does not count them.

## Diagnostics

`SolveReport` gives, linearized at the solution:

- `mobility`: unknowns minus the rank of the joints' Jacobian, over all components;
- `redundancy`: joint equations minus that rank, the degree of hyperstatism. A planar
  four-bar linkage of four revolute joints reports mobility 1 and redundancy 3;
- `freedoms`: per body, the rank of the null-space projection on its six unknowns
  (zero for a fixed body);
- `redundancies`: per joint, how many of its equations depend on the joints before it;
- `residuals` in the caller's units, targets and iteration counts.

`Solution3d` also holds every joint's coordinates (`coordinate`) and relative motion
(`relative`) at the solution.

Failure to converge is `SolveError::NonConvergence`, naming the component's joints
and their residuals. Two fixed bodies joined where they disagree fail this way.

## Sub-assemblies

The solver is flat: it knows no hierarchy and no component of a caller's document.
A rigid sub-assembly is inserted as one body. A flexible one, whose internal joints
move in its parent, is flattened by the caller: it inserts the sub-assembly's bodies
and joints into the parent assembly under its own ids (`insert_*`), then attaches the
parent's joints to the inner bodies. Every pose is in one world frame.

## Storage and Python

`Assembly3d` serializes its options, bodies with their seeds and fixings, and joints
with the seeds of their angles; revisions and solutions start afresh. Reading refuses
what an edit could not have produced: a pose that is not finite, an id stored twice, a
joint on a missing body or one `add_joint` would refuse, a seed that is not a finite
angle of its joint, a next id already in use.

`ngk.solver.assembly` exposes the assembly to Python: `Assembly3d` with `to_json` and
`from_json`, `BodyId`, `JointId`, `Attachment`, `Joint` (its kind, `pitch` for a
helical joint only, and `values`, `limits` and `cone` as keywords), `JointKind`,
`Coordinate`, `Limit`, `Target`, `Solution3d` and its report. Poses are
`ngk.geometry.Rigid` and frames `ngk.geometry.Frame`. `NonConvergenceError` carries
`joints` and `residuals`, `LimitError` its `limit`; both derive from `ValueError`.

## Visualization playground

The visualization app registers `assembly-3d` (sidebar: **Contraintes / Assemblage 3D**,
URL fragment `#/assembly-3d`). `bindings/wasm/visualization/assembly.rs` exposes
`AssemblyPlayground`, which holds an `Assembly3d` built from a preset (a slider-crank,
a three-axis arm with limits, a pendulum on a cone-limited ball joint) and answers each
drag with one `solve_toward` pulling the grabbed point toward the pointer, coalesced
once per animation frame. Bodies are drawn as three.js boxes and cylinders placed by
their solved poses. `cargo run --example assembly_demo` instead sends a driven
slider-crank to the debug viewer, one timeline entry per crank angle.

## Code boundaries

- `assembly/system.rs`: bodies, fixings, joints and their seeds, edits, revisions,
  publication.
- `assembly/ids.rs`, `joint.rs`, `target.rs`: identity and problem representation.
- `assembly/solution.rs`, `error.rs`: outputs and failures.
- `assembly/storage.rs`: the serial form of an assembly and what reading it refuses.
- `assembly/solving/prepare.rs`: components, unknown layout, scale, poses from unknowns.
- `assembly/solving/vector.rs`: space vectors of jets and the rotation of a vector.
- `assembly/solving/equations.rs`: attachment frames as expressions, the joints' rows,
  coordinates and their unwrapping references, and how far each side of a limit is
  passed.
- `assembly/solving/solve.rs`: the component with its active limits as a `Problem`,
  the active-set rounds, the publication audit and diagnostics.
- `bindings/python/solver/assembly.rs`: the Python module.
