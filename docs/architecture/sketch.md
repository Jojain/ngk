# 2D sketches

`src/sketch/` turns a solved [constraint system](solver.md) into topology. It holds
what the solver must not know about: the solver never depends on `Model`, and this
module is where the two meet.

```rust
use ngk::geometry::Plane;
use ngk::model::Model;
use ngk::sketch::build_profiles;

let mut model = Model::new();
let built = build_profiles(&mut model, &system, Plane::xy(), &curves)?;
let edge = built.edge(&model, right_side)?;
```

`curves` lists the segments, arcs and circles to draw. Independent points and
construction curves stay in the system without being listed. `system` must have a
solution for its current revision: a seed does not satisfy the constraints.

## Identity

The result ties each curve's id to the `EdgeKey` it drew, and each profile's key to
its curves. A key holds within its model, and it changes when a build adds or removes
a curve before it, because keys follow creation order. The id is what identifies a
curve from one build to the next: a curve keeps its id when it is edited, so its edge
can be found again even with another shape and another key.

## Joints

Two curve ends are one vertex where the system's coincidences say so, including
through a chain running over what is not drawn. Positions alone never join ends,
since two ends can touch by accident and a coincidence is meant to be removable.
Joined ends share one position: stored ones already coincide exactly, and an arc's
evaluated end is within `LINEAR_TOLERANCE`, which is what the solver holds by default.
A larger gap is refused.

## Profiles

Each connected run of curves becomes one profile, open or closed. A circle is a
profile of its own. Faces come after: `add_face` takes a closed profile.

Curves meeting anywhere but at a joint are refused, naming them: more than two ends at
one joint, a crossing, an end lying on another curve, ends touching without a
coincidence, an overlap. So is a pair whose intersection search cannot claim complete
coverage, because a missed crossing would give a wrong profile.

Every pair of curves is intersected, so the cost grows with the square of the number of
curves listed.
