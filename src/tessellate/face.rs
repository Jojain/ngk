//! Trimmed face tessellation.
//!
//! Real plan: sample the face's pcurves into a UV polygon-with-holes, run a
//! constrained Delaunay triangulation, lift back to 3D via
//! `surface.point_at(u, v)`. That CDT is not in the tree yet. What is here
//! splits on the one question that decides whether a grid can be trusted:
//!
//! - **The boundary *is* the parameter rectangle** — a cylinder wall running
//!   rim to rim, a whole-period wrapping loop — so there is nothing to trim.
//!   Meshed as a uniform UV grid, which is what lets a direction covering a
//!   whole period index its closing column back onto its opening one and come
//!   out watertight.
//! - **The boundary is anything else** — a fillet's slanted corners, a wall
//!   notched where another face joins it. A grid over the bounding rectangle
//!   would spill past the trim, so instead the sampled boundary is bridged
//!   around its holes into one simple polygon, ear-clipped, and each triangle
//!   then split until it follows the support's curvature. The trim is exact;
//!   the seam-welding a grid gets for free is not available here.
//! - **Neither** — a support with no `point_at` shortcut, a boundary that will
//!   not reduce to a simple polygon — is refused with a [`TessellateError`]
//!   naming the gap. A mesh that is not the face is worse than no mesh, because
//!   it is read as the face.

use super::{
    IndexedMesh, TessellateError, TessellateOpts, TessellateResult,
    surface::tessellate_surface_patch,
};
use crate::geometry::{
    Curve, Interval, LINEAR_TOLERANCE, Point2, Point3, PointCoincidence, Surface,
    SurfacePeriodicity,
};
use crate::model::Model;
use crate::topology::LoopKind;
use crate::topology::face::Face;
use crate::topology::orientation::Orientation;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::FaceKey;
use crate::topology::unwrapped_face_domain::UnwrappedFaceDomain;

const EPS: f64 = LINEAR_TOLERANCE;

/// Tessellates a trimmed face into an indexed triangle mesh.
///
/// The boundary is read on a synthesized [`UnwrappedFaceDomain`], so a loop written across a
/// periodic support's cut arrives as one continuous parameter-space polyline
/// rather than as pieces a whole period apart. Each loop must have one pcurve
/// per boundary edge.
///
/// Errors rather than meshing where the boundary cannot be read, where it will
/// not reduce to a triangulable polygon, or where the support has no shortcut
/// here — see [`TessellateError`]. A mesh that is not the face would be read as
/// the face, so a gap is named rather than papered over.
pub fn tessellate_face<P: Payload>(
    face: &Face<'_, P>,
    opts: TessellateOpts,
) -> TessellateResult<IndexedMesh> {
    // What a face runs out to is its enclosing loop, or — where it has none —
    // the support's own domain. A bite taken out of a torus leaves a face of
    // the second kind: still enclosed by nothing, now carrying a hole.
    if !face
        .loops()
        .iter()
        .any(|boundary| !matches!(boundary.kind(), LoopKind::Inner))
    {
        return tessellate_boundaryless_face(face, opts);
    }
    let domain =
        UnwrappedFaceDomain::of_face(face).map_err(|_| TessellateError::UnreadableBoundary)?;
    let segments = opts.curve.segments.max(1);
    let mut boundary = domain
        .loops()
        .iter()
        .map(|boundary| boundary.polyline(segments));
    let outer_uv = boundary.next().ok_or(TessellateError::UnreadableBoundary)?;
    if outer_uv.len() < 3 {
        return Err(TessellateError::UnreadableBoundary);
    }

    let inner_uv: Vec<Vec<Point2>> = boundary.filter(|loop_| !loop_.is_empty()).collect();

    let signed_outer_area = signed_area(&outer_uv);
    let ccw = signed_outer_area > 0.0;

    // A grid covers the rectangle it is given, all of it. That is the answer
    // only when the boundary *is* that rectangle; anywhere else the grid would
    // spill past the trim — over a fillet's slanted corners, across the notch
    // another face cuts in a wall — and the spill is read as surface that is
    // there. So the rectangle is the special case and the polygon is the
    // general one, rather than the other way round.
    //
    // A plane is absent from the rectangle case on purpose: it is flat, so a
    // grid over it buys nothing but triangles. It takes the polygon path
    // always, where it clips to its own boundary exactly and the curvature
    // refinement finds nothing to split.
    let bounds = grid_bounds(&domain, face, &outer_uv);
    if inner_uv.is_empty() && boundary_fills_rectangle(&outer_uv, bounds) {
        match face.surface() {
            Surface::Cylinder(_)
            | Surface::Sphere(_)
            | Surface::Cone(_)
            | Surface::Torus(_)
            | Surface::Ruled(_) => {
                return Ok(surface_grid_over_bounds(face.surface(), bounds, ccw, opts));
            }
            Surface::Revolution(surface) => {
                return Ok(revolution_surface_grid(
                    face.surface(),
                    surface.curve(),
                    &outer_uv,
                    &outer_uv,
                    signed_outer_area,
                    opts,
                ));
            }
            _ => {}
        }
    }

    trimmed_polygon_mesh(face.surface(), &outer_uv, &inner_uv, ccw, opts)
}

/// Tessellates the face stored at `key` in `g`.
///
/// This is the raw map/key bridge for traversal code that has not yet lifted a
/// [`Face`] view. Prefer [`tessellate_face`] when a typed face view is already
/// available.
pub fn tessellate_face_key<P: Payload>(
    g: &Model<P>,
    key: FaceKey,
    opts: TessellateOpts,
) -> TessellateResult<IndexedMesh> {
    let face = g.face(key).ok_or(TessellateError::UnreadableBoundary)?;
    tessellate_face(&face, opts)
}

/// Meshes a face nothing bounds from outside, minus any holes it carries.
///
/// There is no enclosing loop to read bounds or winding from: the surface's own
/// domain is the region, and the face's sense is the winding. A row of that
/// domain that collapses to a point — a sphere's pole — is already meshed as a
/// fan by [`tessellate_surface_patch`], and a direction spanning a whole period
/// is already closed there, so the two ends of the sphere and the seam that is
/// no longer stored all come out watertight.
///
/// Holes are then dropped out of that grid by the triangle, so a bite taken out
/// of a torus reads as a bite. The rim it leaves is stepped at the grid's
/// resolution rather than cut along the hole's own pcurve: the alternative is
/// the polygon path, and that would give up the seam-welding that is the whole
/// reason a boundaryless face is meshed as a grid. Meshing the hole over as
/// though it were not there is the one answer that would be read as the truth.
fn tessellate_boundaryless_face<P: Payload>(
    face: &Face<'_, P>,
    opts: TessellateOpts,
) -> TessellateResult<IndexedMesh> {
    let (u, v) = face.surface().domain();
    if !u.is_finite() || !v.is_finite() {
        return Err(TessellateError::UnboundedDomain);
    }
    let bounds = (u.start, u.end, v.start, v.end);
    let ccw = face.sense() == Orientation::Same;
    let mut mesh = surface_grid_over_bounds(face.surface(), bounds, ccw, opts);
    if face.loops().is_empty() {
        return Ok(mesh);
    }
    let domain =
        UnwrappedFaceDomain::of_face(face).map_err(|_| TessellateError::UnreadableBoundary)?;
    let holes = domain
        .loops()
        .iter()
        .map(|boundary| boundary.polyline(opts.curve.segments.max(1)))
        .collect::<Vec<_>>();
    mesh.indices = cull_triangles_in_holes(face, &mesh, &holes);
    Ok(mesh)
}

/// Keeps the triangles whose centre is not in a hole.
///
/// A triangle is placed by projecting its own centroid back to the surface,
/// which is exact for the supports that carry a hole like this, and folding the
/// answer onto the image the hole is written in — the hole may straddle the
/// domain's seam, and a centroid the other side of it is the same point.
fn cull_triangles_in_holes<P: Payload>(
    face: &Face<'_, P>,
    mesh: &IndexedMesh,
    holes: &[Vec<Point2>],
) -> Vec<u32> {
    let periods = match face.surface().periodicity() {
        SurfacePeriodicity::None => [None, None],
        SurfacePeriodicity::UPeriodic(u) => [Some(u), None],
        SurfacePeriodicity::VPeriodic(v) => [None, Some(v)],
        SurfacePeriodicity::UVPeriodic(u, v) => [Some(u), Some(v)],
    };
    let mut kept = Vec::with_capacity(mesh.indices.len());
    for triangle in mesh.indices.chunks_exact(3) {
        let centroid = Point3::from(
            triangle
                .iter()
                .map(|index| mesh.positions[*index as usize].coords)
                .sum::<nalgebra::Vector3<f64>>()
                / 3.0,
        );
        let Ok(uv) = face.surface().param_at(centroid) else {
            kept.extend_from_slice(triangle);
            continue;
        };
        let in_hole = holes.iter().any(|hole| {
            let mut folded = uv;
            for (axis, period) in periods.into_iter().enumerate() {
                let Some(period) = period.filter(|period| *period > 0.0) else {
                    continue;
                };
                let centre = hole.iter().map(|point| point[axis]).sum::<f64>() / hole.len() as f64;
                folded[axis] += ((centre - folded[axis]) / period).round() * period;
            }
            point_in_polygon(hole, folded)
        });
        if !in_hole {
            kept.extend_from_slice(triangle);
        }
    }
    kept
}

/// Crossing-count membership for a closed parameter-space polygon.
fn point_in_polygon(polygon: &[Point2], point: Point2) -> bool {
    let mut inside = false;
    for (a, b) in polygon
        .iter()
        .zip(polygon.iter().cycle().skip(1))
        .take(polygon.len())
    {
        if (a.y > point.y) != (b.y > point.y)
            && point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x
        {
            inside = !inside;
        }
    }
    inside
}

/// Shoelace signed area in UV. Positive ⇒ CCW.
fn signed_area(poly: &[Point2]) -> f64 {
    let n = poly.len();
    if n < 3 {
        return 0.0;
    }
    let mut s = 0.0;
    for i in 0..n {
        let p = poly[i];
        let q = poly[(i + 1) % n];
        s += p.x * q.y - q.x * p.y;
    }
    0.5 * s
}

// ---------- shortcuts ----------

/// The parameter rectangle to mesh: sampled where a loop bounds, whole where
/// the face wraps.
///
/// A wrapping loop bounds the axis it is *transverse* to, never the axis it
/// spans, so along a spanned axis the face covers its period entirely. Taking
/// that range from the domain rather than from sampled points is what keeps the
/// two ends of the period the same parameter to the last bit — and only then can
/// the grid recognise them as one column and close the mesh over the cut.
fn grid_bounds<P: Payload>(
    domain: &UnwrappedFaceDomain,
    face: &Face<'_, P>,
    outer_uv: &[Point2],
) -> (f64, f64, f64, f64) {
    let (u_min, u_max, v_min, v_max) = uv_bbox(outer_uv);
    let mut bounds = [(u_min, u_max), (v_min, v_max)];
    for axis in face
        .loops()
        .into_iter()
        .filter_map(|loop_| loop_.wrapping_axis())
    {
        if let (Some(period), Some(cut)) = (domain.period(axis), domain.cut(axis)) {
            bounds[axis.index()] = (cut, cut + period);
        }
    }
    let [(u_min, u_max), (v_min, v_max)] = bounds;
    (u_min, u_max, v_min, v_max)
}

fn revolution_surface_grid(
    surface: &Surface,
    profile_curve: &Curve,
    outer_uv: &[Point2],
    boundary_uv: &[Point2],
    signed_outer_area: f64,
    opts: TessellateOpts,
) -> IndexedMesh {
    let (mut u_min, mut u_max, v_min, v_max) = uv_bbox(boundary_uv);
    if (u_max - u_min).abs() <= EPS
        && let Some(domain) = finite_curve_domain(profile_curve)
    {
        u_min = domain.start;
        u_max = domain.end;
    }

    let ccw = if signed_outer_area.abs() > EPS {
        signed_outer_area > 0.0
    } else {
        outer_uv
            .first()
            .zip(outer_uv.last())
            .is_none_or(|(first, last)| last.y >= first.y)
    };
    surface_grid_over_bounds(surface, (u_min, u_max, v_min, v_max), ccw, opts)
}

/// The curve's domain when it is bounded; `None` for an unbounded support.
fn finite_curve_domain(curve: &Curve) -> Option<Interval> {
    let domain = curve.domain();
    domain.is_finite().then_some(domain)
}

fn surface_grid_over_bounds(
    surface: &Surface,
    bounds: (f64, f64, f64, f64),
    ccw: bool,
    opts: TessellateOpts,
) -> IndexedMesh {
    let (u_min, u_max, v_min, v_max) = bounds;
    let mut mesh = tessellate_surface_patch(surface, (u_min, u_max), (v_min, v_max), opts.surface);
    if !ccw {
        flip_winding(&mut mesh.indices);
        for n in &mut mesh.normals {
            *n = -*n;
        }
    }
    mesh
}

/// Meshes the boundary exactly, then splits until the result follows the
/// support.
///
/// Ear-clipping the sampled boundary — each hole bridged into it first — gives
/// triangles that stop precisely where the face stops, which is the whole point
/// of coming here rather than laying a grid over the bounding rectangle. What it
/// does not give is curvature: those triangles are flat, and on a curved support
/// a triangle spanning the patch cuts straight through it. So each one is then
/// split about its edge midpoints until it hugs the surface, by
/// [`refine_to_surface`].
///
/// Errors where the boundary will not reduce to a simple polygon, or will not
/// clip: a self-intersecting loop, a hole no bridge reaches. Those are the cases
/// a real CDT would carry and this shortcut cannot.
fn trimmed_polygon_mesh(
    surface: &Surface,
    outer_uv: &[Point2],
    inner_uv: &[Vec<Point2>],
    ccw: bool,
    opts: TessellateOpts,
) -> TessellateResult<IndexedMesh> {
    // A boundary that encloses nothing is not a hard triangulation case, it is
    // an absent boundary — every sample on one point, or all of them on one
    // line, which is what a face whose pcurves did not survive its import looks
    // like. Saying so separately keeps a broken import from reading as a gap in
    // the mesher.
    if signed_area(outer_uv).abs() <= EPS {
        return Err(TessellateError::DegenerateBoundary);
    }
    // One tolerance serves both halves of this path: it decides which boundary
    // samples carry curvature worth keeping, and then how far the interior may
    // be split before it follows the surface closely enough. Reading it off the
    // boundary rather than the finished polygon keeps it independent of what the
    // cleaning below decides to drop.
    let tolerance = grid_sag(surface, outer_uv, opts);
    let mut polygon = build_simple_polygon(surface, outer_uv, inner_uv, tolerance)
        .ok_or(TessellateError::UntriangulableBoundary)?;

    if signed_area(&polygon) < 0.0 {
        polygon.reverse();
    }

    let clipped = ear_clip(&polygon).ok_or(TessellateError::UntriangulableBoundary)?;
    if clipped.is_empty() {
        return Err(TessellateError::UntriangulableBoundary);
    }

    let triangles = clipped
        .chunks_exact(3)
        .map(|triangle| {
            [
                polygon[triangle[0] as usize],
                polygon[triangle[1] as usize],
                polygon[triangle[2] as usize],
            ]
        })
        .collect::<Vec<_>>();

    let mut mesh = refine_to_surface(surface, &triangles, tolerance);
    if !ccw {
        flip_winding(&mut mesh.indices);
        for normal in &mut mesh.normals {
            *normal = -*normal;
        }
    }
    Ok(mesh)
}

/// How far off the surface one cell of the equivalent grid would have sat.
///
/// This is the tolerance the refinement below aims at, and taking it from the
/// surface itself rather than from a constant is what makes it scale-free: the
/// same number, whether the parameters are radians or millimetres, and whether
/// the face is a fingernail or a bridge. A face meshed here then carries about
/// the fidelity it would have carried had its boundary been a rectangle and a
/// grid been laid over it — no visible change in quality with which path a face
/// took.
///
/// Measured at several places, because one cell can lie along a direction the
/// support happens not to bend in — a cylinder's ruling — and report flat for a
/// surface that is not.
fn grid_sag(surface: &Surface, polygon: &[Point2], opts: TessellateOpts) -> f64 {
    let (u_min, u_max, v_min, v_max) = uv_bbox(polygon);
    let du = (u_max - u_min) / opts.surface.nu.max(1) as f64;
    let dv = (v_max - v_min) / opts.surface.nv.max(1) as f64;

    let sag_at = |corner: Point2| {
        let far = Point2::new(corner.x + du, corner.y + dv);
        (nalgebra::center(
            &surface.point_at(corner.x, corner.y),
            &surface.point_at(far.x, far.y),
        ) - surface.point_at(0.5 * (corner.x + far.x), 0.5 * (corner.y + far.y)))
        .norm()
    };

    let centroid = Point2::from(
        polygon
            .iter()
            .map(|p| p.coords)
            .sum::<nalgebra::Vector2<f64>>()
            / polygon.len() as f64,
    );
    let worst = std::iter::once(centroid)
        .chain(polygon.iter().copied())
        .map(sag_at)
        .fold(0.0f64, f64::max);

    // A plane never bends, so its sag is zero and nothing below ever splits —
    // which is exactly right, and why a planar face comes back from here with
    // the triangles ear-clip produced and no more.
    worst.max(EPS)
}

/// Splits parameter-space triangles until they all lie on the surface.
///
/// The decision is made per *edge*, not per triangle: an edge is marked when the
/// surface at its parameter midpoint is further than `tolerance` from the chord
/// between its ends. Two triangles sharing an edge therefore always agree about
/// it, which is what keeps the result conforming — a triangle split where its
/// neighbour was not would leave a vertex partway along their shared edge that
/// the neighbour's straight edge misses, and on a curved support that T-junction
/// opens as a visible crack.
///
/// Agreeing about each edge is not quite enough on its own, because a triangle
/// with two marked edges cannot be cut in two without hanging a node. So the
/// marking is closed first — any triangle carrying two marked edges has its
/// third marked as well — and afterwards every triangle has one marked edge and
/// bisects, or three and quarters. That closure is what makes this adaptive
/// rather than uniform: a long edge across the middle of a patch splits without
/// dragging every triangle in the face down with it.
///
/// Midpoints are taken in parameter space, so two triangles meeting on an edge
/// compute the same midpoint to the bit, and the dedupe below hands them one
/// shared vertex.
fn refine_to_surface(surface: &Surface, triangles: &[[Point2; 3]], tolerance: f64) -> IndexedMesh {
    // A bound on the passes as well as the tolerance: a degenerate
    // parameterization can report a deviation forever, and must not spin here.
    const MAX_PASSES: u32 = 6;

    type VertexKey = (u64, u64);
    let vertex_key = |p: &Point2| (p.x.to_bits(), p.y.to_bits());
    let edge_key = |a: &Point2, b: &Point2| -> (VertexKey, VertexKey) {
        let (a, b) = (vertex_key(a), vertex_key(b));
        if a <= b { (a, b) } else { (b, a) }
    };
    let off_surface = |a: Point2, b: Point2| {
        (nalgebra::center(&surface.point_at(a.x, a.y), &surface.point_at(b.x, b.y))
            - surface.point_at(0.5 * (a.x + b.x), 0.5 * (a.y + b.y)))
        .norm()
            > tolerance
    };

    let mut triangles = triangles.to_vec();
    for _ in 0..MAX_PASSES {
        let mut marked = triangles
            .iter()
            .flat_map(|t| (0..3).map(|e| (t[e], t[(e + 1) % 3])))
            .filter(|(a, b)| off_surface(*a, *b))
            .map(|(a, b)| edge_key(&a, &b))
            .collect::<std::collections::HashSet<_>>();
        if marked.is_empty() {
            break;
        }

        // Close the marking: a triangle with two marked edges takes its third,
        // so that every triangle below splits either in two or in four and none
        // is left with a node hanging on an unsplit side.
        loop {
            let mut changed = false;
            for triangle in &triangles {
                let edges: [_; 3] =
                    std::array::from_fn(|e| edge_key(&triangle[e], &triangle[(e + 1) % 3]));
                if edges.iter().filter(|edge| marked.contains(*edge)).count() >= 2 {
                    // Every edge, not the first that takes: short-circuiting
                    // here would leave the third unmarked and the node hanging.
                    for edge in edges {
                        changed |= marked.insert(edge);
                    }
                }
            }
            if !changed {
                break;
            }
        }

        triangles = triangles
            .iter()
            .flat_map(|t| {
                let split: [bool; 3] =
                    std::array::from_fn(|e| marked.contains(&edge_key(&t[e], &t[(e + 1) % 3])));
                match split.iter().filter(|s| **s).count() {
                    0 => vec![*t],
                    3 => {
                        let ab = nalgebra::center(&t[0], &t[1]);
                        let bc = nalgebra::center(&t[1], &t[2]);
                        let ca = nalgebra::center(&t[2], &t[0]);
                        vec![[t[0], ab, ca], [ab, t[1], bc], [ca, bc, t[2]], [ab, bc, ca]]
                    }
                    // Exactly one, the closure above having ruled out two: cut
                    // from its midpoint to the opposite corner.
                    _ => {
                        let e = split.iter().position(|s| *s).expect("one edge is marked");
                        let (from, to, opposite) = (t[e], t[(e + 1) % 3], t[(e + 2) % 3]);
                        let mid = nalgebra::center(&from, &to);
                        vec![[from, mid, opposite], [mid, to, opposite]]
                    }
                }
            })
            .collect();
    }

    let mut mesh = IndexedMesh::default();
    let mut index_of = std::collections::HashMap::<(u64, u64), u32>::new();
    for triangle in &triangles {
        for uv in triangle {
            let index = *index_of
                .entry((uv.x.to_bits(), uv.y.to_bits()))
                .or_insert_with(|| {
                    mesh.positions.push(surface.point_at(uv.x, uv.y));
                    mesh.normals.push(surface.normal_at(uv.x, uv.y));
                    (mesh.positions.len() - 1) as u32
                });
            mesh.indices.push(index);
        }
    }
    mesh
}

/// Whether the sampled boundary is the whole of `bounds` and nothing less.
///
/// Its area is the test. A boundary that traces the rectangle's four sides
/// encloses exactly the rectangle's area; one that cuts a corner off, or is
/// notched where another face joins it, encloses less. Comparing areas rather
/// than checking that every sample sits on the border is what catches the notch
/// in a wall whose samples all *do* sit on the border — the notch is interior.
fn boundary_fills_rectangle(outer_uv: &[Point2], bounds: (f64, f64, f64, f64)) -> bool {
    let (u_min, u_max, v_min, v_max) = bounds;
    let rectangle = (u_max - u_min) * (v_max - v_min);
    if rectangle <= EPS {
        return false;
    }
    // Relative, because parameter spaces are not all the same size: an angle
    // runs to a handful of radians where a length runs to hundreds.
    (signed_area(outer_uv).abs() - rectangle).abs() / rectangle <= 1e-6
}

fn build_simple_polygon(
    surface: &Surface,
    outer_uv: &[Point2],
    inner_uv: &[Vec<Point2>],
    tolerance: f64,
) -> Option<Vec<Point2>> {
    let mut polygon = clean_loop(surface, outer_uv, tolerance);
    if polygon.len() < 3 {
        return None;
    }
    if signed_area(&polygon) < 0.0 {
        polygon.reverse();
    }

    for hole in inner_uv {
        let mut hole = clean_loop(surface, hole, tolerance);
        if hole.len() < 3 {
            continue;
        }
        if signed_area(&hole) > 0.0 {
            hole.reverse();
        }
        polygon = bridge_hole(&polygon, &hole)?;
    }

    Some(polygon)
}

/// Drops the boundary samples that carry nothing, and only those.
///
/// A point repeating its predecessor carries nothing anywhere. A point straight
/// between its neighbours in *parameter* space is the harder case: on a plane it
/// carries nothing either, but on a curved support a straight run of parameters
/// is a curved run of surface, and dropping those points throws the curvature
/// away — leaving a boundary that cuts the corner and triangles so large that
/// refinement has to rebuild from scratch what the sampling already knew. So the
/// second test is not whether the point is straight in parameter space, but
/// whether the 3D chord past it still follows the surface to `tolerance`.
fn clean_loop(surface: &Surface, points: &[Point2], tolerance: f64) -> Vec<Point2> {
    let mut cleaned = Vec::new();
    for point in points {
        if cleaned
            .last()
            .is_none_or(|previous: &Point2| !previous.coincides(point, EPS))
        {
            cleaned.push(*point);
        }
    }
    if cleaned.len() > 1 && cleaned[0].coincides(cleaned.last().expect("non-empty"), EPS) {
        cleaned.pop();
    }

    let mut changed = true;
    while changed && cleaned.len() >= 3 {
        changed = false;
        let n = cleaned.len();
        let mut next = Vec::with_capacity(n);
        for i in 0..n {
            let prev = cleaned[(i + n - 1) % n];
            let curr = cleaned[i];
            let after = cleaned[(i + 1) % n];
            let collinear = orient(prev, curr, after).abs() <= EPS;
            let between = (curr - prev).dot(&(after - curr)) >= -EPS;
            if collinear && between && chord_follows_surface(surface, prev, curr, after, tolerance)
            {
                changed = true;
            } else {
                next.push(curr);
            }
        }
        cleaned = next;
    }

    cleaned
}

/// Whether dropping `curr` would leave the boundary where the surface is.
///
/// The chord from `prev` to `after` is compared with the surface at `curr`'s own
/// parameters, sampled along the chord at the fraction `curr` sits at. On a
/// plane the two coincide exactly and the point goes; on anything that bends
/// they separate and it stays.
fn chord_follows_surface(
    surface: &Surface,
    prev: Point2,
    curr: Point2,
    after: Point2,
    tolerance: f64,
) -> bool {
    let span = (after - prev).norm();
    if span <= EPS {
        return true;
    }
    let along = (curr - prev).norm() / span;
    let chord = surface
        .point_at(prev.x, prev.y)
        .coords
        .lerp(&surface.point_at(after.x, after.y).coords, along);
    (surface.point_at(curr.x, curr.y).coords - chord).norm() <= tolerance
}

fn bridge_hole(polygon: &[Point2], hole: &[Point2]) -> Option<Vec<Point2>> {
    let hole_idx = rightmost_vertex(hole);
    let hole_point = hole[hole_idx];
    let polygon_idx = (0..polygon.len())
        .filter(|idx| bridge_is_visible(hole_point, hole_idx, polygon[*idx], *idx, polygon, hole))
        .min_by(|a, b| {
            let da = (polygon[*a] - hole_point).norm_squared();
            let db = (polygon[*b] - hole_point).norm_squared();
            da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
        })?;

    let mut bridged = Vec::with_capacity(polygon.len() + hole.len() + 2);
    bridged.extend_from_slice(&polygon[..=polygon_idx]);
    for offset in 0..hole.len() {
        bridged.push(hole[(hole_idx + offset) % hole.len()]);
    }
    bridged.push(hole_point);
    bridged.push(polygon[polygon_idx]);
    bridged.extend_from_slice(&polygon[polygon_idx + 1..]);
    Some(bridged)
}

fn rightmost_vertex(points: &[Point2]) -> usize {
    (0..points.len())
        .max_by(|a, b| {
            points[*a]
                .x
                .partial_cmp(&points[*b].x)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then_with(|| {
                    points[*b]
                        .y
                        .partial_cmp(&points[*a].y)
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
        })
        .unwrap_or(0)
}

fn bridge_is_visible(
    a: Point2,
    hole_idx: usize,
    b: Point2,
    polygon_idx: usize,
    polygon: &[Point2],
    hole: &[Point2],
) -> bool {
    if a.coincides(b, EPS) {
        return false;
    }

    for i in 0..polygon.len() {
        let j = (i + 1) % polygon.len();
        if i == polygon_idx || j == polygon_idx {
            continue;
        }
        if segments_intersect_strict(a, b, polygon[i], polygon[j]) {
            return false;
        }
    }

    for i in 0..hole.len() {
        let j = (i + 1) % hole.len();
        if i == hole_idx || j == hole_idx {
            continue;
        }
        if segments_intersect_strict(a, b, hole[i], hole[j]) {
            return false;
        }
    }

    true
}

fn ear_clip(polygon: &[Point2]) -> Option<Vec<u32>> {
    if polygon.len() < 3 {
        return None;
    }

    let mut vertices = (0..polygon.len()).collect::<Vec<_>>();
    let mut indices = Vec::with_capacity((polygon.len() - 2) * 3);
    let mut guard = 0;

    while vertices.len() > 3 {
        let mut clipped = false;
        let len = vertices.len();
        for i in 0..len {
            let prev = vertices[(i + len - 1) % len];
            let curr = vertices[i];
            let next = vertices[(i + 1) % len];
            let a = polygon[prev];
            let b = polygon[curr];
            let c = polygon[next];

            if orient(a, b, c) <= EPS {
                continue;
            }
            if vertices.iter().any(|idx| {
                *idx != prev
                    && *idx != curr
                    && *idx != next
                    && !polygon[*idx].coincides(a, EPS)
                    && !polygon[*idx].coincides(b, EPS)
                    && !polygon[*idx].coincides(c, EPS)
                    && point_in_triangle(polygon[*idx], a, b, c)
            }) {
                continue;
            }

            indices.extend_from_slice(&[prev as u32, curr as u32, next as u32]);
            vertices.remove(i);
            clipped = true;
            break;
        }

        if !clipped {
            return None;
        }
        guard += 1;
        if guard > polygon.len() * polygon.len() {
            return None;
        }
    }

    indices.extend_from_slice(&[vertices[0] as u32, vertices[1] as u32, vertices[2] as u32]);
    Some(indices)
}

fn point_in_triangle(p: Point2, a: Point2, b: Point2, c: Point2) -> bool {
    orient(a, b, p) >= -EPS && orient(b, c, p) >= -EPS && orient(c, a, p) >= -EPS
}

fn segments_intersect_strict(a: Point2, b: Point2, c: Point2, d: Point2) -> bool {
    if a.coincides(c, EPS) || a.coincides(d, EPS) || b.coincides(c, EPS) || b.coincides(d, EPS) {
        return false;
    }

    let o1 = orient(a, b, c);
    let o2 = orient(a, b, d);
    let o3 = orient(c, d, a);
    let o4 = orient(c, d, b);

    if o1.abs() <= EPS && on_segment(a, c, b) {
        return true;
    }
    if o2.abs() <= EPS && on_segment(a, d, b) {
        return true;
    }
    if o3.abs() <= EPS && on_segment(c, a, d) {
        return true;
    }
    if o4.abs() <= EPS && on_segment(c, b, d) {
        return true;
    }

    (o1 > EPS) != (o2 > EPS) && (o3 > EPS) != (o4 > EPS)
}

fn on_segment(a: Point2, p: Point2, b: Point2) -> bool {
    p.x >= a.x.min(b.x) - EPS
        && p.x <= a.x.max(b.x) + EPS
        && p.y >= a.y.min(b.y) - EPS
        && p.y <= a.y.max(b.y) + EPS
}

fn orient(a: Point2, b: Point2, c: Point2) -> f64 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

// ---------- helpers ----------

fn uv_bbox(points: &[Point2]) -> (f64, f64, f64, f64) {
    let mut u_min = f64::INFINITY;
    let mut u_max = f64::NEG_INFINITY;
    let mut v_min = f64::INFINITY;
    let mut v_max = f64::NEG_INFINITY;
    for p in points {
        u_min = u_min.min(p.x);
        u_max = u_max.max(p.x);
        v_min = v_min.min(p.y);
        v_max = v_max.max(p.y);
    }
    (u_min, u_max, v_min, v_max)
}

fn flip_winding(indices: &mut [u32]) {
    for tri in indices.chunks_mut(3) {
        if tri.len() == 3 {
            tri.swap(1, 2);
        }
    }
}
