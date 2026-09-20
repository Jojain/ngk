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
| `Chamfer` | `TargetChamfer` | A chamfer was applied to a `ChamferTarget`, which may be edges, profiles, or vertices. |
| `BooleanResult` | `SolidBoolean` | A Boolean operation acted on solids. |
| `BooleanPreparation` | `BooleanOperandPreparation` | Boolean operand preparation was performed. |
| `RevolvedEdge` | `EdgeRevolution` | An edge was revolved. |
| `RevolvedProfile` | `ProfileRevolution` | A profile was revolved. |
| `RevolvedFace` | `FaceRevolution` | A face was revolved. |
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

The convention applies to the logical subject named by the operation. It does
not change the distinction between logical entities and raw GMap cells: a
result describing an edge uses an `EdgeKey`, while a result about a raw map
cell must say so explicitly.
