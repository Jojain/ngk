# Mass properties

Length, area, volume, centroid and inertia, as a public API over `Model<P>`.

Nothing in the tree measures a shape today. The one number that exists is a
signed shell volume buried inside validation, it is not public, and — measured
below — it is wrong for every curved solid. The immediate motivation is that
tests currently cannot assert what a builder *built*: a swept elbow that was
self-intersecting passed `validate_all_solid_manifolds` and
`validate_all_solid_orientations` because both are combinatorial, and the only
assertions left were bounding-box extents, which the broken shape satisfied.
A volume would have caught it in one line.

## Current state, measured

`validate_oriented_shell_volume` (`src/topology/validation.rs`, private)
accumulates a signed volume by the divergence theorem and then **discards the
number**, keeping only its sign. Its two contributors are:

- a direct tetrahedron fan, for a face that is a `Surface::Plane` *and* all of
  whose edges are degree-1 — exact;
- `Face::signed_volume_contribution` (`src/topology/face.rs`, `pub(crate)`) for
  everything else, which fans the face's **pcurves** sampled at a fixed **32
  points per edge**.

Measured against the closed forms, via the primitives in `modeling::solids`:

| solid | true | current | error |
|---|---|---|---|
| block 2×3×4 | 24.000000 | 24.000000 | exact |
| cylinder r1 h2 | 6.283185 | **2.032427** | **−67%** |
| sphere r1 | 4.188790 | 4.123391 | −1.6% |
| torus R3 r1 | 59.217626 | 57.877026 | −2.3% |

The sphere and torus errors are the fixed 32-sample fan. The cylinder is not a
sampling error — it is three times out, and wrong enough to be a defect in its
own right. **Diagnose that before building on it**; it is the first thing a
correct implementation has to explain. (A cylinder's caps are circles, so they
are not "planar with degree-1 edges" and take the sampled path too.)

So this is not "expose a number that already exists". The number has to be made
correct first.

## Two traps

**`Face::boundary_signed_area` is not an area.** It is the signed area of the
face's loop in *parameter space*, used to read winding. A `Face::area()` built
on it would return a plausible-looking number with no relation to the surface's
area — on a cylinder it would be the (angle × height) rectangle. Do not reuse
it.

**The volume loop and the winding check are entangled.** The same `for face in
&faces` pass in `validate_oriented_shell_volume` both accumulates volume and
populates the `directed` / `owner` maps that the following pass uses to verify
neighbours run their shared edge opposite ways. That check must survive
extraction — it is what catches a shell wound inconsistently, and it is
independent of the volume. Pull the measurement out; leave the winding check
where it is.

## What to build

Three questions, asked of one shape: **how much** of it there is, **where** it
sits, and **how it is spread** about where it sits. Every kernel answers them
together because one integration pass produces all three, and every kernel
calls the answer *mass properties* — at a density of one, which is the only
density here.

### Name the bundles concretely

The obvious design is one generic `Moments<D>` bundle. Do not: it reads badly
(`props.measure` says nothing at the call site, and `Moments<Volume>` is
opaque) and it is a false economy, because the three are computed by three
genuinely different integrals — quadrature along a curve, quadrature over a
trimmed surface, the divergence theorem over a closed shell. A shared type
would advertise shared machinery that does not exist.

So: three plain structs, named for what they hold, matching the names OCCT
uses so they are findable (`BRepGProp::LinearProperties` / `SurfaceProperties`
/ `VolumeProperties`).

```rust
pub struct LinearProperties {
    pub length: f64,
    pub centroid: Point3,
    pub inertia: Inertia<Length>,
}

pub struct SurfaceProperties {
    pub area: f64,
    /* centroid, inertia */
}

pub struct VolumeProperties {
    pub volume: f64,
    /* centroid, inertia */
}
```

`solid.volume_properties()?.volume` reads as what it is, at every call site.

The three property bundles already carry the semantic distinction between
length, area and volume. Their scalar fields are therefore plain `f64`; the
dimension markers remain on inertia, where they prevent mixing second moments
from different kinds of geometry.

### What `inertia` is, and why it is branded

`centroid` is the average position of the shape, weighted by its measure —
the first moment, normalized. `inertia` is the **second** moment: a 3×3
tensor saying how the measure is distributed *about* that centroid. Its
diagonal holds the moment about each axis, `∫(y² + z²)` and its two mates;
its off-diagonal holds the products of inertia, which are zero exactly when
the axes are the shape's own principal ones.

It is taken **about the centroid** because that is the one reference every
kernel agrees on and the only one that needs no further explanation; the
parallel-axis theorem moves it anywhere else, and `moment_about(Axis3)` should
do that for the caller.

It is branded because the integral's units follow the dimension it was taken
over: a curve's second moment is a length³, a surface's a length⁴, a solid's a
length⁵. Adding a face's inertia to a solid's compiles, produces a plausible
matrix, and is meaningless — a silent confusion, which is the only kind these
types exist to catch.

Mirror the rest of `GProp_GProps` on it: `moment_about(Axis3)`,
`radius_of_gyration(Axis3)`, `principal()` for the eigen-decomposition.

### Where the methods live

On the typed views, where traversal already lives. The scalar shorthand avoids
building a bundle nobody asked for.

| view | shorthand | full |
|---|---|---|
| `Edge` | `length()` | `linear_properties()` |
| `Profile` | `length()` | `linear_properties()` |
| `Face` | `area()` | `surface_properties()` |
| `Sheet` | `area()` | `surface_properties()` |
| `Solid` | `volume()` | `volume_properties()` |

`Edge::length()` is free today: `self.trimmed_curve().length()`.

### The sign belongs to a shell, not to a solid

A *shell* has a signed volume, and its sign is its orientation — that is what
`validate_oriented_shell_volume` reads, and it stays internal to validation
where it is a check rather than a value. A *solid*'s volume is its outer shell
less its cavities, and is non-negative; if it comes out negative the solid is
inside-out, which is `validate_solid_orientation`'s answer to give, not a
number to hand back. Public volume properties therefore reject invalid shell
orientation rather than exposing a signed value and inviting a caller to
silently discard the orientation signal with `.abs()`.

### Deliberately not done

Restraint is as much the house idiom as the branding — `Param`'s module doc
spends a section on what it refuses to brand, and this should too.

- **No units.** No mm-versus-inch. The kernel is unit-agnostic and STEP
  converts at its boundary; a units library would infect every signature for a
  confusion that is not silent.
- **No unit arithmetic.** Nothing needs length-times-length or similar
  operators, and the property bundles already identify the measured quantity.
- **No branded centroid.** A point is a point in all three cases.
- **No density.** OCCT calls the measure `Mass` and folds density in; this
  does not. A caller wanting mass multiplies.
- **No error estimate on the result, for now.** The obvious next brand is a
  `Fidelity` beside each scalar — `Exact` where a closed form answered,
  `Bounded(f64)` where quadrature did — following
  `IntersectionQuality`'s `certified`. It is deliberately out of this pass.
  What must not be skipped is the prose: each method's doc says which cases
  are exact and what governs the error in the rest, so a caller knows whether
  an exact-equality assertion is safe before a type says so.

## How to compute

A trimmed NURBS face has no closed-form area, so every kernel integrates.
OCCT's own `VolumePropertiesGK` is Gauss–Kronrod *with an error estimate*,
which is the honest contract and the reason `error` is in the struct above.

Recommended first implementation — **tessellation-based**, using the existing
`src/tessellate/`:

- area = sum of triangle areas;
- volume = sum of signed tetrahedra from any reference point (the divergence
  theorem makes the choice of reference irrelevant on a closed shell);
- centroid and inertia = the standard per-triangle and per-tetrahedron
  formulas.

Why this first: it is *total* — every surface type tessellates — it is exact
for planar faces with straight edges, its accuracy is governed by one
tolerance the caller already understands, and it cannot reproduce the cylinder
defect above because it never touches the pcurve fan. Feed the tessellation
tolerance in and report the resulting `error`.

The alternative, exactness-first, is boundary-integral quadrature over each
face's trimmed domain via Green's theorem on its pcurves — the same route
OCCT takes. It is the better long-term answer and can replace the internals
later without moving the API. Do not start there.

## Reference values

From OCCT via the `.venv` (`OCP.BRepGProp`), for the same primitives. Inertia
is about the **centre of mass**, density 1.

| shape | volume | area | edge length | centroid | Ixx, Iyy, Izz |
|---|---|---|---|---|---|
| block 2×3×4 | 24.000000000 | 52.000000000 | 72.000000000 | (1, 1.5, 2) | 50, 40, 26 |
| cylinder r1 h2 | 6.283185307 | 18.849555922 | 29.132741229 | (0, 0, 1) | 3.665191429, 3.665191429, 3.141592654 |
| sphere r1 | 4.188790205 | 12.566370614 | 6.283185307 | (0, 0, 0) | 1.675516082 ×3 |
| torus R3 r1 | 59.217626407 | 118.435252813 | 62.831853072 | (0, 0, 0) | 303.490335333, 303.490335333, 577.371857464 |

Each is checkable against a closed form, so the table is a cross-check on
OCCT rather than a dependency on it: `m(b²+c²)/12` for the box,
`m(3r²+h²)/12` and `mr²/2` for the cylinder, `2mr²/5` for the sphere,
`m(4R²+5r²)/8` and `m(R²+3r²/4)` for the torus.

## Tasks, in order

1. **Pin the current wrongness.** A test that asserts the four volumes above
   against the existing machinery, so the cylinder defect is red before
   anything is changed. Delete it once superseded by the real tests — it exists
   to make the fix verifiable, not to stay.
2. **Diagnose the cylinder.** −67% is not quadrature error. Understand it
   before replacing the code, because the same fault may be in the pcurve fan
   that other things rely on.
3. **Build the measurement**, per the section above, as its own module
   (`src/measure/` or `src/topology/properties.rs` — not inside `validation`,
   which is about refusing, not reporting).
4. **Re-express `validate_oriented_shell_volume`** in terms of it: compute the
   signed shell volume, check the sign, and keep the winding pass untouched.
   One implementation, two callers.
5. **Add the view methods** in the table above.
6. **Document the accuracy of each method.** Which inputs are exact, and what
   governs the error in the rest. Prose now; a `Fidelity` type later if it
   turns out callers need to branch on it.

## Testing

- The four primitives against the table, with a stated relative tolerance.
  Volume, area and centroid for each; inertia at least for the block, whose
  closed form is unambiguous. A block should come out exact, so assert it
  tightly and let the curved cases carry the loose tolerance.
- **Additivity**: the volume of a block equals the sum of the volumes of the
  two blocks a boolean cut it into.
- **Invariance**: volume, area and inertia eigenvalues are unchanged by
  `builders::transform::rigid`; the centroid moves with it.
- **Sign**: an inside-out solid is refused by `validate_solid_orientation`,
  and never reaches a caller as a negative volume. Test that the refusal
  still fires after task 4 rewires it.
- **Cavity**: a block with a block-shaped cavity measures the difference.
- **A degenerate shape is caught.** The motivating case: the self-intersecting
  swept elbow in `tests/builders/sweep.rs`. A correct miter measures 80.0 and a
  correct round corner 79.5708 — both confirmed against OCCT — where the
  self-intersecting version measures 32.
- Use **build123d** as the oracle for anything without a closed form; see the
  `build123d-step-testing` skill for how the project already does this.

## Constraints

From `AGENTS.md`, and worth restating because they bite here:

- Measurement is a **read**. Nothing in this touches `ModelEdit`, creates a
  dart, or opens a transaction. It takes `&Model<P>` and returns numbers.
- Comments state what the code does, never a plan for it. No "stage 2", no
  reference to this file, from any doc comment.
- Deleting or changing behaviour means adapting its tests. Task 4 changes what
  `validate_oriented_shell_volume` is; any test naming it must be revisited.
- Vocabulary: a bare noun is the logical entity. This measures faces, sheets
  and solids, never 2-cells.
