# Builder API

This document defines the naming and return-value convention for the NGK
builder layer. Builders mutate one `Model` transaction and return a structured
description of the topology change they committed.

## Result names

An operation result is a noun phrase naming the event, qualified by the
logical entity that the event acted on:

```text
<Subject><NominalizedVerb>
```

The subject comes first. It is the logical vocabulary of the operation, not a
raw GMap cell or a dart. The result name has no `Result` suffix; the outer Rust
`Result<..., ...>` already expresses fallibility.

This makes the result read naturally at the call site:

```rust
let split = split_edge(&mut model, edge, parameter)?;
let extrusion = add_extruded_profile(&mut model, profile, direction)?;
```

The canonical names are:

| Current or proposed name | Canonical name | Reason |
| --- | --- | --- |
| `EdgeSplit` | `EdgeSplit` | An edge was split. |
| `FaceImprintSplit` | `FaceImprintSplit` | A face was split by an imprint. |
| `CellRemoval` | `CellRemoval` | A cell was removed. |
| `ProfileExtrusion` | `ProfileExtrusion` | A profile was extruded. |
| `Extrusion` | `FaceExtrusion` | A face was extruded. |
| `Chamfer` | `TargetChamfer` | A chamfer was applied to a `BlendTarget`, which may mix vertices, edges, profiles and faces. |
| `Fillet` | `TargetFillet` | A fillet was applied to a `BlendTarget`. |
| `BooleanResult` | `SolidBoolean` | A Boolean operation acted on solids. |
| `BooleanPreparation` | `BooleanOperandPreparation` | Boolean operand preparation was performed. |
| `RevolvedEdge` | `EdgeRevolution` | An edge was revolved. |
| `RevolvedProfile` | `ProfileRevolution` | A profile was revolved. |
| `RevolvedFace` | `FaceRevolution` | A face was revolved. |
| `FaceSweep` | `FaceSweep` | A face was swept along a spine. |
| `AppendEdge` | `ProfileAppend` | An edge was appended to a profile. |
| `ClosedSolid` | `ClosedSolid` | Product-name exception for pure creation. |

## Pure creation exception

Some operations do not perform a meaningful event on an existing subject. A
primitive constructor such as `add_sphere` or `add_torus` only establishes a
new product; there is no useful subject-plus-event phrase to name.

When the whole operation is “this product now exists”, the result may be named
after the product itself. `ClosedSolid` is the current example. This exception
is limited to pure creation operations and does not apply to edits,
transformations, splits, merges, or operations that consume or modify an
existing logical entity.

## Builder return contract

Every public mutating builder returns a structured operation result:

```rust
pub fn operation(...) -> Result<OperationResult, OperationError>;
```

The result struct:

- contains durable logical keys such as `EdgeKey`, `FaceKey`, or `SolidKey`;
- does not expose raw darts as durable identity;
- implements `OpResult` when it refers to committed model state;
- is stamped only by the public transaction boundary; and
- may provide a checked `view(&model)` for resolving its keys at the committed
  revision.

The edit-scoped implementation returns the same result type before stamping.
Private topology primitives may return individual keys or topology fragments
when they are implementation details rather than complete builder operations.

Queries and preflight checks are not builders and keep ordinary return types.
For example, `is_removable` may return `bool`, and `planned_merge` may return
`Result<MergeKind, CellRemovalError>`.

## Revolution sections

A face or profile revolution uses a planar meridian section: the revolution
axis lies in the section plane and the whole boundary stays on one side of it.
The side can be either side; the axis origin can be anywhere along the same
line. `SectionCrossesAxis` refuses a section spanning both sides instead of
returning overlapping walls. The builder does not split or combine positive
and negative radial regions.

The preflight bounds each actual curve span with its positive-weight NURBS
control hull, including curved interiors and inner loops. This is conservative:
an ambiguous control hull or unsupported weights are also refused, even if
the curve itself might stay on one side. `AxisOutsideSectionPlane` refuses an
axis outside the meridian plane. Refusals leave the model unchanged.

A section edge wholly on the axis sweeps no wall. In a partial turn the two
caps meet directly along that edge; in a full turn the source section is
consumed. An edge with one end on the axis contributes a triangular wall for
a partial turn, or a face bounded by one circular rim for a full turn.
Curved section edges with both ends on the axis are refused by the profile
and face builders. The isolated-edge builder has its own full-turn apex
construction; partial isolated-edge apex sweeps are refused.

Circular trajectories keep the requested angular span, including angles over
half a turn. A returning boundary circle is oriented against the sweep so its
span agrees with its pcurve rather than selecting the complementary arc.

The convention applies to the logical subject named by the operation. It does
not change the distinction between logical entities and raw GMap cells: a
result describing an edge uses an `EdgeKey`, while a result about a raw map
cell must say so explicitly.

## Which model an operation's result lives in

The owned-`Shape` facade (`modeling`) and the language bindings follow one set
of rules about models:

1. A `Shape<K>` is a model plus its primary `K` entity. The model may hold
   other entities besides it.
2. An operation with a primary input produces its result in that input's
   model, kept whole. Rust consumes the input shape and edits its model in
   place; Python and wasm clone it, so the input model is never mutated.
3. A secondary input — a Boolean's tool — contributes only its primary entity
   and the entities that bound it, copied in.
4. A tool already in the target's model is used in place: by key at the
   builder level, by model identity in the bindings.
5. An operation with no primary input, such as a loft, builds a fresh model.

Every entity of the model the operation does not touch keeps its key, so a
caller holding the key of a face an earlier operation produced can still find
it after a later one.
