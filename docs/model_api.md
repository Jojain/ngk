# NGK Model API Direction

This document captures the API direction discussed for NGK's modeling layer.
It is a target design note, not a description of the current API. The current
Rust and Python APIs are prototypes and can be reshaped to match this model.

## Goals

NGK is a CAD kernel built around generalized maps. The core data structure is
the `GMap`, but the user-facing modeling API should not force users to think in
terms of darts, alpha links, cell representatives, or direct attribute storage.

The desired API has two related but distinct surfaces:

- Simple standalone modeling types that can create, copy, clone, and preview
  shapes.
- A mutable `Model` that owns one persistent `GMap` and is the place where CAD
  edits happen.

The long-term goal is not a history-based parametric modeler. Operations such
as merge, split, and boolean do not need to track a feature tree. They should
mutate one map and return explicit result handles describing what changed.

## Layering

The proposed architecture has three main layers.

### Builders

Builders are low-level topology construction helpers. They work close to the
`GMap` representation.

Responsibilities:

- Add darts.
- Sew and unsew alpha links.
- Attach vertex, edge, face, and solid attributes.
- Build pcurves and boundary data.
- Implement reusable topology construction routines used by higher layers.

Builders may accept `&mut GMap<P>` and return low-level handles such as `Dart`,
`EdgeKey`, `FaceKey`, or `SolidKey`.

They are useful internally and in focused tests, but they should not be the main
CAD user API.

Example shape of this layer:

```rust
builders::faces::add_rectangle(&mut g, plane, x_size, y_size)?;
builders::solids::add_extruded_face(&mut g, face_key, direction)?;
```

This is acceptable internally, but too representation-heavy for end users.

### Modeling

The `modeling` layer provides simple standalone shape builders and owned shape
values. These types are not persistent design objects. They are temporary,
copyable, cloneable values that know how to produce a `GMap` fragment.

Responsibilities:

- Create standalone edges, profiles, faces, sheets, and solids.
- Provide ergonomic constructors for common primitives.
- Hide low-level GMap construction.
- Produce `Shape<K, P>` values or concrete builder values that can later be
  inserted into a `Model`.

Examples:

```rust
let block = modeling::solids::block(1.0, 2.0, 3.0)?;
let face = modeling::faces::rectangle(Plane::xy(), 2.0, 3.0)?;
let profile = modeling::profiles::rectangle(Plane::xy(), 2.0, 3.0)?;
```

This layer is useful for:

- Tests.
- Visualization scripts.
- One-off generated shapes.
- Python convenience constructors.
- Preparing shape fragments before inserting them into a model.

Standalone shapes should be easy to copy and clone. They are not expected to
remember how they were made.

### Model

`Model` is the persistent mutable modeling context. It owns one `GMap<P>`.

Responsibilities:

- Own the single authoritative map.
- Insert standalone shapes into that map.
- Apply mutating CAD operations.
- Return handles and operation result structs.
- Coordinate payload propagation policies.
- Optionally validate topology after edits.

The core shape is:

```rust
pub struct Model<P: Payload = StandardPayload> {
    map: GMap<P>,
}
```

The user-facing idea:

```rust
let mut model = Model::new();

let block = modeling::solids::block(1.0, 2.0, 3.0)?;
let solid = model.insert(block)?;

let result = model.apply(/* split, boolean, or other edit */)?;
```

`Model` should make it clear that all inserted topology now belongs to the
model. After insertion, the model's handles are the source of truth.

## Shape Values

`Shape<K, P>` is a useful concept and should remain lightweight:

```rust
pub struct Shape<K: ShapeKind, P: Payload = StandardPayload> {
    map: GMap<P>,
    handle: K::Handle,
}
```

Important semantics:

- A `Shape` owns a small map fragment.
- A `Shape` has one root handle.
- A `Shape` can be cloned or copied into another map.
- A `Shape` is not the persistent model object after insertion.
- A `Shape` does not track design history.

Useful examples:

```rust
let source = modeling::solids::block(1.0, 2.0, 3.0)?;
let mut model = Model::new();
let solid_key = model.insert(source)?;
```

After `insert`, user code should use `solid_key` and the `Model`, not the old
standalone `Shape`.

## Model Mutations

There are two candidate styles for model-level operations:

- Struct operations.
- Function operations.

Both can work. The important point is that model-level API should be
user-facing and should not expose the fact that operations are implemented by
mutating a `GMap`.

### Option A: Struct Operations

In this style, operations are named structs implementing an operation trait.

```rust
pub trait ModelOp<P: Payload> {
    type Output;
    type Error;

    fn apply(self, model: &mut Model<P>) -> Result<Self::Output, Self::Error>;
}

impl<P: Payload> Model<P> {
    pub fn apply<O>(&mut self, op: O) -> Result<O::Output, O::Error>
    where
        O: ModelOp<P>,
    {
        op.apply(self)
    }
}
```

Example usage:

```rust
let result = model.apply(
    SplitSolid::new(solid)
        .by_plane(plane)
        .with_policy(policy),
)?;
```

Or:

```rust
let result = model.apply(Boolean::union(a, b).with_policy(policy))?;
```

Pros:

- Good discoverability in Rust docs.
- Easy to document each operation and its result.
- Easy to add builder-style configuration methods.
- Works well for complex operations with many parameters.
- Can validate parameters before applying.
- Can store prepared intermediate data if needed.
- Can support dry-run, preview, or analysis modes later.
- Error and output types are explicit per operation.

Cons:

- More boilerplate.
- More names and files to maintain.
- Can feel ceremonious for simple edits.
- Generic operation traits can become abstract if introduced too early.
- Operation structs may duplicate simple function signatures for small tasks.

Best fit:

- Boolean operations.
- Split operations.
- Fillet/chamfer/shell operations.
- Any operation with many options or policy hooks.
- Any operation whose result needs a rich typed report.

### Option B: Function Operations

In this style, operations are functions that receive a model and mutate it.

```rust
pub fn split_solid<P, S>(
    model: &mut Model<P>,
    solid: SolidKey,
    plane: Plane,
    policy: &mut S,
) -> Result<SplitSolidResult, SplitError>
where
    P: Payload,
    S: SplitPolicy<P>,
{
    // mutate model internally
}
```

Example usage:

```rust
let result = model::ops::split_solid(&mut model, solid, plane, &mut policy)?;
```

Or with convenience methods:

```rust
let result = model.split_solid(solid, plane, &mut policy)?;
```

Pros:

- Simple to write.
- Simple to read.
- Lower abstraction cost.
- Easy to test as ordinary functions.
- Good for early API exploration.
- Avoids premature operation trait design.

Cons:

- Less discoverable if many functions live in a broad module.
- Complex operations may accumulate long parameter lists.
- Optional settings can become awkward.
- Harder to compose uniformly unless a convention emerges.
- Less room for staged builders such as `SplitSolid::new(...).by_plane(...)`.

Best fit:

- Small operations.
- Early experiments.
- Thin wrappers around builder functionality.
- Operations with few parameters and obvious return values.

### Option C: Closure-Based Editing

We considered `Model::edit` accepting a closure. The simplest version exposes
the underlying `GMap`:

```rust
model.edit(|g| {
    // mutate GMap
})?;
```

This is not the desired user-facing API because it leaks the implementation.
If users receive `&mut GMap`, then low-level topology functions become the
informal public API.

A closure could instead receive a facade, but that still requires us to design
the facade methods. It does not remove the need for a model-level API.

Conclusion: closure-based editing may be useful internally or for tests, but it
should not be the main public API if the goal is to hide `GMap`.

## Recommended Direction For Model Operations

Start pragmatic:

- Add direct `Model` methods for core lifecycle actions.
- Use function operations for simple early model edits.
- Introduce struct operations once an operation has enough configuration or
  policy complexity to justify it.

Suggested initial `Model` surface:

```rust
impl<P: Payload> Model<P> {
    pub fn new() -> Self;
    pub fn map(&self) -> &GMap<P>;

    pub fn insert<K>(&mut self, shape: Shape<K, P>) -> Result<K::Handle, InsertError>
    where
        K: ShapeKind;

    pub fn remove<H>(&mut self, handle: H) -> Result<(), RemoveError>;
}
```

Then operations can begin as functions:

```rust
let split = model::ops::split_solid(&mut model, solid, plane, &mut policy)?;
```

As an operation grows, it can become a struct:

```rust
let split = model.apply(
    SplitSolid::new(solid)
        .by_plane(plane)
        .with_policy(policy),
)?;
```

This avoids committing too early. NGK can use the lightest API that fits each
operation while preserving the central rule: the `Model` owns the persistent
map.

## Direct Methods On Model

Not every mutation should be a method on `Model`.

If every operation becomes a method, `Model` will eventually become too large:

```rust
model.add_line(...);
model.add_circle(...);
model.add_rectangle(...);
model.add_block(...);
model.extrude_profile(...);
model.extrude_face(...);
model.revolve_profile(...);
model.revolve_face(...);
model.split_face(...);
model.split_solid(...);
model.boolean_union(...);
model.boolean_difference(...);
model.fillet(...);
model.chamfer(...);
```

That API is easy at first, but it couples `Model` to every operation and makes
the type harder to maintain.

Better rule:

- Put lifecycle operations directly on `Model`.
- Put rich CAD algorithms in operation modules or operation structs.
- Add direct convenience methods only for operations that are truly universal.

Good direct methods:

```rust
model.insert(shape)?;
model.remove(handle)?;
model.map();
model.validate();
```

Possible convenience methods later:

```rust
model.split_solid(...)?;
model.boolean_union(...)?;
```

But these should delegate to the operation implementation, not contain the
algorithm inline.

## Operation Results And Handle Identity

CAD topology edits should return explicit result structs. We should not pretend
that a handle always means the same design object after a split, merge, or
boolean.

Example:

```rust
pub struct SplitSolidResult {
    pub source: SolidKey,
    pub parts: Vec<SolidKey>,
    pub created_faces: Vec<FaceKey>,
    pub modified_faces: Vec<FaceKey>,
    pub removed_faces: Vec<FaceKey>,
}
```

For booleans:

```rust
pub struct BooleanResult {
    pub result: SolidKey,
    pub consumed: Vec<SolidKey>,
    pub created_faces: Vec<FaceKey>,
    pub modified_faces: Vec<FaceKey>,
}
```

These structs are important because NGK is not tracking history. The operation
result is the immediate contract describing what changed.

## Payload Policy

The `Payload` trait attaches user data to topology dimensions:

```rust
pub trait Payload {
    type V;
    type E;
    type F;
    type S;
}
```

The long-term model needs a strategy for deciding what happens to payload data
during topology edits.

Examples:

- Splitting an edge creates two edges. Which payloads do they receive?
- Splitting a face creates multiple faces. Do they inherit material?
- Boolean union consumes two solids and creates a new solid. Which payload wins?
- Merging faces may combine metadata. How?
- Generated intersection edges may need special payloads.

The preferred direction is to keep payloads as data and put behavior in policy
traits. Payloads should not need to contain edit logic themselves.

Example policy shape:

```rust
pub trait SplitPolicy<P: Payload> {
    fn split_vertex(&mut self, source: &P::V) -> P::V;
    fn split_edge(&mut self, source: &P::E) -> (P::E, P::E);
    fn split_face(&mut self, source: &P::F) -> Vec<P::F>;
    fn split_solid(&mut self, source: &P::S) -> Vec<P::S>;
}
```

For merge or boolean:

```rust
pub trait MergePolicy<P: Payload> {
    fn merge_vertices(&mut self, a: &P::V, b: &P::V) -> P::V;
    fn merge_edges(&mut self, a: &P::E, b: &P::E) -> P::E;
    fn merge_faces(&mut self, a: &P::F, b: &P::F) -> P::F;
    fn merge_solids(&mut self, a: &P::S, b: &P::S) -> P::S;
}
```

The exact trait boundaries may change. Split, merge, and boolean may need
separate policy traits because their questions are different.

### Default Policies

NGK should provide simple default policies for `StandardPayload`.

For `StandardPayload`, every payload is `()`, so policies are trivial.

For common user payloads, useful defaults may include:

- Clone source payload into every child.
- Prefer left-hand payload.
- Prefer right-hand payload.
- Use operation-generated defaults.
- Call user closures for each affected cell.

Example:

```rust
let mut policy = ClonePayloadPolicy;
let result = model::ops::split_solid(&mut model, solid, plane, &mut policy)?;
```

## Shape Builder Direction

The biggest missing ergonomic piece is a higher-level way to build profiles and
planar faces without manually building darts, loops, pcurves, and `FaceAttr`.

Target concepts:

```rust
let profile = modeling::profiles::ProfileBuilder::on(Plane::xy())
    .line_to([1.0, 0.0])
    .line_to([1.0, 1.0])
    .line_to([0.0, 1.0])
    .close()
    .build()?;
```

For faces:

```rust
let face = modeling::faces::PlanarFaceBuilder::on(Plane::xy())
    .outer_polygon(points)
    .hole_polygon(hole_points)
    .build()?;
```

For circular holes:

```rust
let face = modeling::faces::PlanarFaceBuilder::on(Plane::xy())
    .outer_circle(outer_radius)
    .hole_circle(inner_radius)
    .build()?;
```

These builders should:

- Validate sizes and geometry.
- Build loops.
- Compute pcurves.
- Handle loop orientation.
- Return `Shape<ProfileTag, P>` or `Shape<FaceTag, P>`.
- Avoid exposing darts unless explicitly requested for diagnostics.

This is the API gap exposed by scripts such as hollow cylinder and holed
pentagon: they currently have to manually build loops and pcurves.

## Possible Rust User Flows

### Standalone Shape Flow

```rust
let block = modeling::solids::block(1.0, 2.0, 3.0)?;
let tcv = to_tcv(&block, TcvOptions::default())?;
```

This is ideal for tests, examples, previews, and Python convenience functions.

### Insert Into Model

```rust
let mut model = Model::new();
let block = modeling::solids::block(1.0, 2.0, 3.0)?;
let solid = model.insert(block)?;
```

### Function Operation

```rust
let mut policy = ClonePayloadPolicy;
let split = model::ops::split_solid(&mut model, solid, plane, &mut policy)?;
```

### Struct Operation

```rust
let split = model.apply(
    SplitSolid::new(solid)
        .by_plane(plane)
        .with_policy(ClonePayloadPolicy),
)?;
```

Both operation styles can coexist. The public API can start with function
operations and promote complex operations to struct operations later.

## Python API Direction

The current Python API is a prototype and should not constrain the Rust design.
Python should wrap the desired modeling concepts, not expose Rust internals.

Python users should not see:

- `GMap`
- `Dart`
- Alpha involutions
- Cell representatives
- Raw topology attributes
- Pcurve maps

They should see:

- `Model`
- `Shape` objects such as `Solid`, `Face`, `Profile`, `Edge`
- Geometry objects such as `Point3`, `Vector3`, `Plane`
- Operation results such as `SplitSolidResult`
- Optional policy objects or callbacks

### Python Standalone Shape API

Python can keep simple constructor functions:

```python
import ngk

solid = ngk.block(1, 2, 3)
face = ngk.rectangle_face(2, 3)
profile = ngk.rectangle_profile(2, 3)
ngk.show(solid)
```

But these should be understood as standalone shapes, not model-owned topology.

A more explicit future API could be:

```python
solid = ngk.solids.block(1, 2, 3)
face = ngk.faces.rectangle(2, 3)
profile = ngk.profiles.rectangle(2, 3)
```

This mirrors Rust's `modeling` layer.

### Python Model API

Python should expose a mutable `Model`:

```python
model = ngk.Model()

solid = model.insert(ngk.solids.block(1, 2, 3))
result = model.split_solid(solid, ngk.Plane.yz(0.5))

ngk.show(model)
```

The inserted `solid` should be a model handle or model-owned wrapper, not the
same object as the standalone shape.

Possible Python shape:

```python
class Model:
    def insert(self, shape) -> SolidHandle | FaceHandle | ProfileHandle: ...
    def remove(self, handle) -> None: ...
    def split_solid(self, solid, plane, policy=None) -> SplitSolidResult: ...
    def boolean_union(self, a, b, policy=None) -> BooleanResult: ...
```

For Python, direct methods on `Model` may be more appropriate than exposing
operation structs everywhere. Python users generally expect method calls or
plain functions more than Rust-style operation builders.

### Python Operation Structs

Python can still expose operation objects when an operation has many options:

```python
result = model.apply(
    ngk.ops.SplitSolid(solid)
        .by_plane(plane)
        .with_policy(policy)
)
```

Pros:

- Mirrors Rust if Rust uses operation structs.
- Useful for complex operations with many options.
- Good for advanced users.

Cons:

- Less Pythonic for simple operations.
- More objects to document.
- Builder-style chaining can feel foreign if overused.

Recommended Python approach:

- Provide direct methods for common operations.
- Provide operation objects only for advanced or highly configurable operations.
- Internally, both can call the same Rust implementation.

### Python Policies

Python needs an ergonomic way to control payload propagation.

There are three possible levels:

1. Built-in policy names:

```python
model.split_solid(solid, plane, policy="clone")
model.boolean_union(a, b, policy="prefer_left")
```

2. Policy objects:

```python
policy = ngk.ClonePayloadPolicy()
result = model.split_solid(solid, plane, policy=policy)
```

3. Python callbacks:

```python
policy = ngk.SplitPolicy(
    face=lambda source, context: source.copy_with(tag="split"),
    edge=lambda source, context: source.copy(),
)
result = model.split_solid(solid, plane, policy=policy)
```

Callbacks are powerful but require care:

- They cross the Rust/Python boundary many times.
- They need clear context objects.
- They can be slow for operations touching many cells.
- Error handling must map cleanly to Python exceptions.

Recommended Python policy path:

- Start with built-in policies.
- Add policy objects once custom payloads exist.
- Add callbacks only when there is a real use case.

### Python Handles And Object Lifetime

Current Python wrappers use shared maps behind shape objects. That is fine for
standalone immutable shapes, but model-owned objects need different semantics.

For a mutable `Model`, handles should refer back to the owning model:

```python
model = ngk.Model()
solid = model.insert(ngk.solids.block(1, 2, 3))

# solid is valid as long as the model keeps that topology.
```

After a split or boolean, old handles may be invalidated:

```python
result = model.split_solid(solid, plane)

solid.valid        # maybe False
result.parts       # new solid handles
```

Python should make invalidation clear. Options:

- Raise an exception when using an invalid handle.
- Provide `handle.is_valid`.
- Return result objects that explicitly list removed and created handles.

The result-object approach is the most explicit and matches the Rust direction.

### Python Visualization

`ngk.show` should accept both standalone shapes and `Model`:

```python
ngk.show(ngk.solids.block(1, 2, 3))

model = ngk.Model()
solid = model.insert(ngk.solids.block(1, 2, 3))
ngk.show(model)
```

This keeps visualization decoupled from whether a shape is standalone or
model-owned.

## Open Questions

These need design decisions later:

- Should `Model::insert` preserve the inserted shape root handle type exactly,
  or should it return model-owned wrapper handles?
- Should Rust expose direct `Model` convenience methods for common operations,
  or keep all rich operations under `model::ops`?
- Should operation structs receive `&mut Model<P>` or only access an internal
  map editing API?
- How strict should handle invalidation be after mutating operations?
- Should payload policies be split by operation type, by cell dimension, or both?
- How much of the low-level builder layer should remain public?
- Should Python mirror Rust modules closely or present a more Pythonic facade?

## Suggested Next Steps

1. Make `Model` a real owner of one `GMap`.
2. Add `Model::map`, `Model::insert`, and a basic insert result path.
3. Keep standalone `modeling` constructors for simple shape creation.
4. Add higher-level profile and planar face builders.
5. Migrate scripts that currently build loops and pcurves manually to the new
   modeling builders.
6. Implement one model-level operation as a function operation first.
7. If that operation grows many options, promote it to a struct operation.
8. Design the first payload policy trait around a real edit, probably split.
9. Redesign Python around `Shape` constructors plus mutable `Model`, without
   preserving the current prototype API as a constraint.

