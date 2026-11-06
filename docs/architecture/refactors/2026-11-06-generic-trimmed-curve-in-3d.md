# Generic `Trimmed` in 3D

## Seen

`geometry/dim2/trimmed.rs` has `Trimmed<C = Curve2>`, generic over the support, with
`TrimmedCurve2`, `TrimmedLine2` and `TrimmedCircle2` as aliases. Its 3D twin,
`TrimmedCurve` in `geometry/dim3/trimmed.rs`, still holds a `Curve` only. This breaks
the "2D mirrors 3D exactly" rule of [geometry.md](../geometry.md).

The 2D type became generic because the 2D constraint solver needed to say that a
segment is a span of a line. Nothing in 3D needed it at that point, so 3D was left
alone rather than changed without a user.

## Refactor

Make `TrimmedCurve` an alias of a generic type as in 2D, with the same split:
methods any support answers on the generic impl, and those matching on the `Curve`
enum on the alias. `CurveGeometry` already has an impl per curve type, `NurbsCurve`
included, which 2D's `Curve2Geometry` does not.

## Not checked

- Whether every `TrimmedCurve` method can be written against `CurveGeometry`.
- Whether 2D and 3D could share one generic type, given their different point types.
- Whether any 3D code would actually use a typed span.
