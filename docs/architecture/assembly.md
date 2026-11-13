# Assembly solver

`ngk::solver::assembly` exposes a stateful `Assembly3d`: rigid bodies and the joints
between them, the seeds they start from, and the last successful solution. It shares
the numerical core of the [2D solver](solver.md) and its contract: typed ids, atomic
batches, `solve` and `solve_toward`, nothing published on failure. It depends on NGK
geometry values only, not on `Model`, GMap, builders or Nales. What is still to be
built is in [plan/assembly-solver.md](../../plan/assembly-solver.md).

## Public workflow

```rust
use ngk::geometry::{dim3::frame::Frame, dim3::utils::Point3, transform::Rigid};
use ngk::solver::assembly::{Assembly3d, Attachment, Joint3d, Target3d};

let mut assembly = Assembly3d::default();
let ground = assembly.add_body(Rigid::identity())?;
assembly.fix(ground)?;
let arm = assembly.add_body(seed)?;
assembly.add_joint(Joint3d::revolute(
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

| Kind | Leaves | Equations |
|---|---|---|
| `Fixed` | nothing | origins coincide (3), orientations coincide (3) |
| `Revolute` | rotation about Z | origins coincide (3), Z axes coincide (2) |
| `Prismatic` | translation along Z | second origin on the first Z axis (2), orientations coincide (3) |

Refused at edit: a joint holding the same body twice, a body that is not in the
assembly, a frame that is not finite, orthonormal and right-handed.

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

## Diagnostics

`SolveReport` gives, linearized at the solution:

- `mobility`: unknowns minus the rank of the joints' Jacobian, over all components;
- `redundancy`: joint equations minus that rank, the degree of hyperstatism. A planar
  four-bar linkage of four revolute joints reports mobility 1 and redundancy 3;
- `freedoms`: per body, the rank of the null-space projection on its six unknowns
  (zero for a fixed body);
- `redundancies`: per joint, how many of its equations depend on the joints before it;
- `residuals` in the caller's units, targets and iteration counts.

Failure to converge is `SolveError::NonConvergence`, naming the component's joints
and their residuals. Two fixed bodies joined where they disagree fail this way.

## Code boundaries

- `assembly/system.rs`: bodies, fixings, joints, edits, revisions, publication.
- `assembly/ids.rs`, `joint.rs`, `target.rs`: identity and problem representation.
- `assembly/solution.rs`, `error.rs`: outputs and failures.
- `assembly/solving/prepare.rs`: components, unknown layout, scale, poses from unknowns.
- `assembly/solving/vector.rs`: space vectors of jets and the rotation of a vector.
- `assembly/solving/equations.rs`: attachment frames as expressions and the joints' rows.
- `assembly/solving/solve.rs`: the component as a `Problem`, the publication audit and
  diagnostics.
