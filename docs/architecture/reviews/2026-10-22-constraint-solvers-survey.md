# Constraint solver representations and CAD integration

Date: 2026-10-22.
Status: reference; exploratory survey, not an NGK design decision.

## Question and limits

How do established sketch and assembly constraint solvers represent geometry,
its unknown parameters, and the relationship with a host CAD application?
This is a representative architecture survey rather than an exhaustive
comparison of numerical performance or a reconstruction of proprietary
algorithms. Source-code inspection is possible for FreeCAD and SolveSpace;
the commercial examples below are limited to their public documentation.

The initial NGK direction remains a module inside the existing crate, with
sketch solving implemented before rigid-motion solving. This review does not
settle its data ownership, geometric catalog, or public API.

## Observable patterns

### FreeCAD Sketcher / PlaneGCS

FreeCAD accepts its host geometric objects, including line segments, circles,
and B-splines. Its Sketch layer creates solver parameters and GCS geometric
records, solves their relationships, and writes the resulting values back to
the host geometries. GCS geometry references numerical parameters; this is
not another complete boundary-representation kernel.

The source evidence and details are in
[the open-source notes](2026-10-22-constraint-solvers-open-source-notes.md),
which distinguish host geometry, solver parameters, write-back, and supported
curve representations. Primary entry points are
[Sketch.cpp](https://github.com/FreeCAD/FreeCAD/blob/d3f4da791daa9e94f340b81770d13f090ec679c6/src/Mod/Sketcher/App/Sketch.cpp)
and
[planegcs/Geo.h](https://github.com/FreeCAD/FreeCAD/blob/d3f4da791daa9e94f340b81770d13f090ec679c6/src/Mod/Sketcher/App/planegcs/Geo.h).

### SolveSpace / libslvs

SolveSpace exposes explicit parameter, entity, and constraint records through
its library interface. The solver is available separately, although its
maintainers describe the implementation as coupled to the application. This
is a useful distinction between an embeddable API and internally independent
code. See the official [library description](https://solvespace.com/library.pl)
and the source analysis in
[the open-source notes](2026-10-22-constraint-solvers-open-source-notes.md).

The public interface includes separate arrays of parameter, entity, and
constraint records with integer references between them. It has both a
batch-array solve path and an API retaining the problem between calls. The
free-form curve exposed here is a non-rational cubic Bezier, not a general
B-spline. This is a concrete precedent for referenced pools, but arrays of
records are not by themselves a structure-of-arrays layout.
[Source: public libslvs interface](https://github.com/solvespace/solvespace/blob/30e1c3ac0beb77fe3e098315b1bb662840ddc99a/include/slvs.h).

### Siemens D-Cubed

2D DCM is an embeddable component with geometric types, dimensions,
constraints, preferred solving outcomes, and diagnostic status. Its published
catalog includes analytical geometry, splines, and parametric curves. This
establishes that its API is geometric rather than a requirement for the host
to supply arbitrary equations itself.
[Source: 2D DCM product documentation](https://www.siemens.com/en-us/products/plm-components/d-cubed/2d-dcm/).

A public integration example registers application-provided Frustum functions
for geometric data, adds application objects to a dimension system, creates
constraints, and evaluates that system. DCM also has a dedicated NURBS-based
spline definition, supporting control-point and interpolation-based input;
rigid and flexible spline behavior are explicit choices.
[Source: Siemens spline integration example, part 1](https://blogs.sw.siemens.com/plm-components/flexible-spline-curve-control-in-the-d-cubed-dimensional-constraint-manager-dcm-part-1/).

The same example series represents a bounded control segment through an
unbounded line and separately constrained points. Application geometry need
not therefore map one-to-one to a solver primitive. It also demonstrates
variables connecting spline conditions to other dimensions.
[Source: Siemens spline integration example, part 2](https://blogs.sw.siemens.com/plm-components/flexible-spline-curve-control-in-the-d-cubed-dimensional-constraint-manager-dcm-part-2/).

### Spatial Constraint Design Solver

Spatial documents CDS as a separate component with geometric objects,
constraints, rigid sets, joints, and numerical variables. Its catalog includes
NURBS curves and surfaces. The host application manages interaction and
assembly structure; a modeling kernel performs shape construction and
trimming. CDS distinguishes updating a configuration from movement under
constraints and exposes diagnostics and controls for ambiguous solutions.
The public documentation establishes these integration roles, not its private
data layout or numerical implementation.
[Source: Spatial CDS documentation](https://www.spatial.com/solutions/3d-modeling/constraint-design-solver).

## What the numerical solver changes

SolveSpace documents constraints as equations and their general solution by
a modified Newton method, with special handling for some simpler systems.
Its initial configuration influences the chosen solution when several exist;
its treatment of underconstrained sketches is designed to make dragging
predictable.
[Source: SolveSpace technology description](https://solvespace.com/tech.pl).

An illustrative, solver-independent sketch model can contain:

| Geometry | Possible numerical unknowns |
|---|---|
| Point | Two coordinates |
| Segment | Coordinates of its two endpoints |
| Circle | Center coordinates and radius |
| B-spline | Selected pole coordinates and possibly weights or other permitted parameters |

For example, horizontal alignment can be expressed as `Ay - By = 0`, and a
distance as `(Ax-Bx)^2 + (Ay-By)^2 - d^2 = 0`. These are explanatory equations,
not claims about the exact residuals used by every solver. The representation
and the choice of movable parameters determine which changes are possible.

Supporting a spline type is consequently a more specific promise than
accepting a generic curve object: one must define whether its position,
control polygon, interpolation conditions, weights, or other parameters may
change. A fixed-shape spline and a deformable spline are different problems,
even when they share the same evaluated curve representation. The explicit
rigid/flexible distinction is documented by Siemens in the spline example
above.

## Rigid-motion solving

Siemens describes 3D DCM rigid sets as collections of application-defined
geometry that retain their relative arrangement. The component reports
remaining translational or rotational freedoms and produces transformations
for the application. It supports application-defined parametric curves as
well as its own spline definitions.
[Source: Siemens 3D DCM integration capabilities](https://blogs.sw.siemens.com/plm-components/d-cubed-3d-dcm-version-55-0/).

An architectural inference for NGK is that rigid-motion solving can reference
geometric attachment features while solving poses, without making the
shape's internal geometry deformable. This is a possible mapping to NGK,
not a settled API or an assertion that every assembly solver uses the same
parameterization.

## Implications to explore in NGK

These are interpretations of the sources, not decisions:

- Host geometric types at the public boundary and a parameter representation
  inside the solver are compatible; FreeCAD provides a concrete precedent.
- Reusing NGK geometry does not require exposing GMap or Model mutation to
  the numerical solver.
- A stateful module object can manage geometry correspondence, current
  parameter values, constraints, solve results, and numerical working state.
  These responsibilities need not imply an application undo history.
- Importing an existing curve must define which parameters are movable and
  provide stable references to the roles constraints can address.
- Full curve evaluation support and constraint support should be specified
  separately, particularly for B-splines and NURBS.
- Stable object references, batches, and parameter pools are API and ownership
  choices. SoA versus arrays of records is a distinct storage-layout choice;
  the observed pool-based APIs do not establish a need for a full SoA design.
- The next discussion should examine the host-to-solver correspondence using
  these precedents, rather than asking the user to choose an unconstrained
  architecture from first principles.
