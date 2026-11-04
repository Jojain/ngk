# One home for writing a curve's pcurve on a surface

`builders/blend/pcurve.rs` holds `pcurve_on`, `closed_pcurve_on` and
`onto_branch`: write a 3D curve's pcurve on a surface (exact on a plane, traced
elsewhere), follow a closed rail round a periodic direction, and shift a
parameter point onto a periodic branch. `builders/offset/reembed.rs` needs the
same three and calls them from the blend module, mapping `BlendError` to a
string.

A refactor would move them to a neutral module (beside
`builders::profiles::curve_pcurve`, or into `geometry` next to
`pcurve_on_surface`) with an error type of their own, and have both callers
map it.

Not checked: whether other builders (boolean, sweep, loft) carry their own
copies of the same logic, or whether the blend-specific deviation check belongs
in the shared version.
