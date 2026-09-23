//! Trimmed face tessellation.
//!
//! Real plan: sample the face's pcurves into a UV polygon-with-holes, run a
//! constrained Delaunay triangulation, lift back to 3D via
//! `surface.point_at(u, v)`. That CDT is not in the tree yet. What is here:
//!
//! - **A face nothing bounds** — a whole sphere or torus — is its support's
//!   parameter rectangle, meshed as a uniform UV grid, which is what lets a
//!   direction covering a whole period index its closing column back onto its
//!   opening one and come out watertight.
//! - **Every bounded face** has each boundary pcurve sampled exactly where its
//!   edge is, so the edge line, this face and the face beyond meet at the same
//!   points. The polygon is cut into cells along every direction the support
//!   bends in, each cell ear-clipped, and each triangle split where it strays
//!   from the surface — never across the face's own boundary, which stays as
//!   sampled.
//! - **Neither** — a boundary that will not reduce to simple polygons — is
//!   refused with a [`TessellateError`] naming the gap. A mesh that is not the
//!   face is worse than no mesh, because it is read as the face.

use super::{
    IndexedMesh, TessellateError, TessellateOpts, TessellateResult,
    curve::segments_for,
    strips::{CutLines, cut_into_slabs, fold_into_period},
    surface::tessellate_surface_patch,
};
use nalgebra::Vector2;

use crate::geometry::{
    Axis2, LINEAR_TOLERANCE, Point2, Point3, PointCoincidence, Surface, SurfacePeriodicity,
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
    mesh_face(face, opts).map(welded)
}

/// Meshes `face` piece by piece, each piece with vertices of its own.
fn mesh_face<P: Payload>(
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
    // Each pcurve is cut as its edge is, so the edge line and every face along
    // it meet at the same points.
    let mut boundary = domain.loops().iter().map(|boundary| {
        boundary.polyline_on(face.surface(), |span| segments_for(span, opts.curve))
    });
    let outer_uv = boundary.next().ok_or(TessellateError::UnreadableBoundary)?;
    if outer_uv.len() < 3 {
        return Err(TessellateError::UnreadableBoundary);
    }

    let inner_uv: Vec<Vec<Point2>> = boundary.filter(|loop_| !loop_.is_empty()).collect();

    let signed_outer_area = signed_area(&outer_uv);
    let ccw = signed_outer_area > 0.0;

    // A loop running past one period cannot be meshed as a polygon in any one
    // period, so it is folded back into the period first.
    if let Some((axis, period)) = overrun_axis(&domain) {
        // Folded onto the outer loop's own period: where wrapping loops enclose
        // the face, the outer loop spans exactly one period and was closed
        // across its two ends, and those closing runs have to land on the fold
        // lines to be recognised as the cut they are rather than as boundary.
        let cut = outer_uv
            .iter()
            .map(|point| point[axis.index()])
            .fold(f64::INFINITY, f64::min);
        let tolerance = grid_sag(face.surface(), &outer_uv, opts);
        let (strips, fold) = fold_into_period(&outer_uv, &inner_uv, axis.index(), cut, period, EPS)
            .ok_or(TessellateError::UntriangulableBoundary)?;
        let mut mesh = IndexedMesh::default();
        for strip in strips {
            let piece = trimmed_polygon_mesh(
                face.surface(),
                &strip.outer,
                &strip.holes,
                ccw,
                opts,
                &[fold],
                tolerance,
            )?;
            append_mesh(&mut mesh, piece);
        }
        return Ok(mesh);
    }

    // Every bounded face takes the polygon path, a whole cylinder wall included:
    // a grid laid over the parameter rectangle puts its columns wherever the
    // grid count falls, and its rim then misses the points the rim edge and
    // the face beyond it are drawn through. The polygon path samples each rim
    // as its edge is sampled, and cuts the face into cells along every
    // direction it bends in, which is the grid's own resolution.
    let tolerance = grid_sag(face.surface(), &outer_uv, opts);
    trimmed_polygon_mesh(
        face.surface(),
        &outer_uv,
        &inner_uv,
        ccw,
        opts,
        &[],
        tolerance,
    )
}

/// The periodic axis some loop of `domain` runs more than one period along.
///
/// A face written within one period spans at most that period; a wrapping
/// loop spans it exactly. Only a loop that winds on past it — a band round a
/// cylinder — overruns.
fn overrun_axis(domain: &UnwrappedFaceDomain) -> Option<(Axis2, f64)> {
    let (min, max) = domain.bounds();
    [Axis2::U, Axis2::V].into_iter().find_map(|axis| {
        let period = domain.period(axis)?;
        let span = max[axis.index()] - min[axis.index()];
        (span > period * (1.0 + 1.0e-9) + EPS).then_some((axis, period))
    })
}

/// `mesh` with every set of coincident vertices merged into one.
///
/// A face meshed in pieces -- cells, strips, the two sides of a period it was
/// folded across -- repeats every vertex the pieces share, each lifted from
/// its own parameters. Merged, the face is one connected surface again: a
/// cylinder wall closes into a tube rather than a sheet cut down its seam, and
/// each shared point shades once. A triangle two of whose corners merge had no
/// area to begin with and is dropped.
fn welded(mesh: IndexedMesh) -> IndexedMesh {
    const CELL: f64 = 4.0 * EPS;
    let cell_of = |point: &Point3| {
        [point.x, point.y, point.z].map(|coordinate| (coordinate / CELL).floor() as i64)
    };
    let mut cells = std::collections::HashMap::<[i64; 3], Vec<u32>>::new();
    let mut remap = Vec::with_capacity(mesh.positions.len());
    let mut result = IndexedMesh::default();
    for (position, normal) in mesh.positions.iter().zip(&mesh.normals) {
        let cell = cell_of(position);
        let existing = (0..27).find_map(|offset: i64| {
            let neighbour = [
                cell[0] + offset % 3 - 1,
                cell[1] + (offset / 3) % 3 - 1,
                cell[2] + offset / 9 - 1,
            ];
            cells
                .get(&neighbour)?
                .iter()
                .copied()
                .find(|index| result.positions[*index as usize].coincides(*position, EPS))
        });
        let index = existing.unwrap_or_else(|| {
            let index = result.positions.len() as u32;
            result.positions.push(*position);
            result.normals.push(*normal);
            cells.entry(cell).or_default().push(index);
            index
        });
        remap.push(index);
    }
    for triangle in mesh.indices.chunks_exact(3) {
        let [a, b, c] = [0, 1, 2].map(|corner| remap[triangle[corner] as usize]);
        if a != b && b != c && c != a {
            result.indices.extend_from_slice(&[a, b, c]);
        }
    }
    result
}

/// Appends `piece` to `mesh`, re-indexing its triangles.
fn append_mesh(mesh: &mut IndexedMesh, piece: IndexedMesh) {
    let offset = mesh.positions.len() as u32;
    mesh.positions.extend(piece.positions);
    mesh.normals.extend(piece.normals);
    mesh.indices
        .extend(piece.indices.into_iter().map(|index| index + offset));
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
    let bounds = (
        u.start.value(),
        u.end.value(),
        v.start.value(),
        v.end.value(),
    );
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
        .map(|boundary| boundary.polyline_on(face.surface(), |span| segments_for(span, opts.curve)))
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
pub(super) fn point_in_polygon(polygon: &[Point2], point: Point2) -> bool {
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
pub(super) fn signed_area(poly: &[Point2]) -> f64 {
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

/// Meshes a polygon with holes, first cutting it into cells along every
/// direction its support bends in.
///
/// Ear-clipping a long boundary gives slivers spanning the whole of it, and
/// refining a sliver only makes more of them: a wall eight turns of a helix
/// long would come out as millions of needles. Cut first into slabs one
/// [`cell_steps`] cell wide -- the size a grid over the support would give
/// its cells -- each piece is already small wherever the surface bends and
/// clips into a handful of triangles, while a direction the support is
/// straight in is left whole. An axis is cut at most once, so the recursion
/// ends after both.
///
/// `cuts` names lines the polygon was already cut along, whose stretches of
/// boundary refinement may split -- unlike the face's own boundary, which
/// every neighbour meets at exactly the points it was sampled at.
fn trimmed_polygon_mesh(
    surface: &Surface,
    outer_uv: &[Point2],
    inner_uv: &[Vec<Point2>],
    ccw: bool,
    opts: TessellateOpts,
    cuts: &[CutLines],
    tolerance: f64,
) -> TessellateResult<IndexedMesh> {
    slabbed_polygon_mesh(
        surface, outer_uv, inner_uv, ccw, opts, [false; 2], cuts, tolerance,
    )
}

/// How many cells a polygon may run along an axis before it is cut.
const SLAB_STEPS: f64 = 1.0;

fn slabbed_polygon_mesh(
    surface: &Surface,
    outer_uv: &[Point2],
    inner_uv: &[Vec<Point2>],
    ccw: bool,
    opts: TessellateOpts,
    cut: [bool; 2],
    cuts: &[CutLines],
    tolerance: f64,
) -> TessellateResult<IndexedMesh> {
    let steps = cell_steps(surface, opts);
    let (u_min, u_max, v_min, v_max) = uv_bbox(outer_uv);
    let reach = [(u_max - u_min) / steps.x, (v_max - v_min) / steps.y];
    let axis = (0..2)
        .filter(|axis| !cut[*axis] && reach[*axis] > SLAB_STEPS)
        .max_by(|a, b| reach[*a].total_cmp(&reach[*b]));
    let Some(axis) = axis else {
        return clipped_polygon_mesh(surface, outer_uv, inner_uv, ccw, cuts, tolerance);
    };
    let (strips, slabs) = cut_into_slabs(outer_uv, inner_uv, axis, steps[axis], EPS)
        .ok_or(TessellateError::UntriangulableBoundary)?;
    let cuts = cuts.iter().copied().chain([slabs]).collect::<Vec<_>>();
    let mut cut = cut;
    cut[axis] = true;
    let mut mesh = IndexedMesh::default();
    for strip in strips {
        let piece = slabbed_polygon_mesh(
            surface,
            &strip.outer,
            &strip.holes,
            ccw,
            opts,
            cut,
            &cuts,
            tolerance,
        )?;
        append_mesh(&mut mesh, piece);
    }
    Ok(mesh)
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
fn clipped_polygon_mesh(
    surface: &Surface,
    outer_uv: &[Point2],
    inner_uv: &[Vec<Point2>],
    ccw: bool,
    cuts: &[CutLines],
    tolerance: f64,
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
    // be split before it follows the surface closely enough. It is read once
    // for the whole face, so a piece a cut left small is not held to a
    // tighter budget than the face it came from.
    let mut polygon = build_simple_polygon(surface, outer_uv, inner_uv)
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

    // The face's own boundary stays as sampled. A bridge to a hole is walked
    // both ways and a cut line was drawn by the cutter; neither is shared with
    // a neighbour, and both may split like the interior.
    let sides = polygon
        .iter()
        .zip(polygon.iter().cycle().skip(1))
        .map(|(a, b)| (vertex_bits(a), vertex_bits(b)))
        .collect::<std::collections::HashSet<_>>();
    let locked = polygon
        .iter()
        .zip(polygon.iter().cycle().skip(1))
        .filter(|(a, b)| {
            !sides.contains(&(vertex_bits(b), vertex_bits(a)))
                && !cuts.iter().any(|lines| lines.hold(**a, **b))
        })
        .map(|(a, b)| edge_bits(a, b))
        .collect::<std::collections::HashSet<_>>();
    let mut mesh = refine_to_surface(
        surface,
        &triangles,
        tolerance,
        support_steps(surface),
        &locked,
    );
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
///
/// The cell is also never longer than [`cell_steps`] allows: a period over
/// the grid count on a periodic support -- the very cell a whole cylinder
/// wall is gridded at. Without that cap a face running eight turns of a helix would
/// take its cells a quarter-turn long and be let off with a tolerance the
/// size of its own radius.
fn grid_sag(surface: &Surface, polygon: &[Point2], opts: TessellateOpts) -> f64 {
    let (u_min, u_max, v_min, v_max) = uv_bbox(polygon);
    let cells = cell_steps(surface, opts);
    let (nu, nv) = (opts.surface.nu.max(1) as f64, opts.surface.nv.max(1) as f64);
    let du = ((u_max - u_min) / nu).min(cells.x);
    let dv = ((v_max - v_min) / nv).min(cells.y);

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
/// Each triangle then splits along exactly its marked edges: one marked edge
/// bisects it, two cut it in three through both midpoints, three quarter it.
/// No edge is split that was not marked, so no node hangs, and none is split
/// just to keep a neighbour company: a strip running along a helix splits
/// along the helix and never across it, where it is straight.
///
/// Midpoints are taken in parameter space, so two triangles meeting on an edge
/// compute the same midpoint to the bit, and the dedupe below hands them one
/// shared vertex.
///
/// A sag test alone can be fooled, though. An edge running a whole turn of a
/// helical face has its midpoint straight above its chord, and one running
/// two turns has it exactly on it, so a face meshed from a handful of long
/// triangles would keep them and skip every turn in between. So an edge is
/// also marked while it is longer than `steps` — the scale on which the
/// support can turn at all, below which its midpoint does speak for it.
fn refine_to_surface(
    surface: &Surface,
    triangles: &[[Point2; 3]],
    tolerance: f64,
    steps: Vector2<f64>,
    locked: &std::collections::HashSet<EdgeBits>,
) -> IndexedMesh {
    // A bound on the passes as well as the tolerance: a degenerate
    // parameterization can report a deviation forever, and must not spin here.
    // The passes spent bringing edges down to `steps` come on top, since
    // those halve an edge whether or not it strays.
    const SAG_PASSES: u32 = 6;
    let too_long =
        |a: Point2, b: Point2| (b.x - a.x).abs() > steps.x || (b.y - a.y).abs() > steps.y;
    let longest = triangles
        .iter()
        .flat_map(|t| (0..3).map(|e| (t[e], t[(e + 1) % 3])))
        .map(|(a, b)| ((b.x - a.x).abs() / steps.x).max((b.y - a.y).abs() / steps.y))
        .fold(1.0_f64, f64::max);
    let passes = SAG_PASSES + longest.log2().ceil().min(16.0) as u32;

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
    for _ in 0..passes {
        let marked = triangles
            .iter()
            .flat_map(|t| (0..3).map(|e| (t[e], t[(e + 1) % 3])))
            .filter(|(a, b)| {
                !locked.contains(&edge_bits(a, b)) && (too_long(*a, *b) || off_surface(*a, *b))
            })
            .map(|(a, b)| edge_key(&a, &b))
            .collect::<std::collections::HashSet<_>>();
        if marked.is_empty() {
            break;
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
                    // Two, meeting at the corner opposite the one left whole:
                    // that corner keeps a triangle of its own, and what is left
                    // of the triangle is cut in two along a diagonal.
                    2 => {
                        let k = split.iter().position(|s| !*s).expect("one edge is whole");
                        let (a, b, corner) = (t[k], t[(k + 1) % 3], t[(k + 2) % 3]);
                        let near_b = nalgebra::center(&b, &corner);
                        let near_a = nalgebra::center(&corner, &a);
                        vec![
                            [near_b, corner, near_a],
                            [a, b, near_b],
                            [a, near_b, near_a],
                        ]
                    }
                    // Exactly one: cut from its midpoint to the opposite corner.
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

/// A parameter-space point, to the bit.
type VertexBits = (u64, u64);
/// A parameter-space segment, to the bit and either way round.
type EdgeBits = (VertexBits, VertexBits);

fn vertex_bits(point: &Point2) -> VertexBits {
    (point.x.to_bits(), point.y.to_bits())
}

fn edge_bits(a: &Point2, b: &Point2) -> EdgeBits {
    let (a, b) = (vertex_bits(a), vertex_bits(b));
    if a <= b { (a, b) } else { (b, a) }
}

/// The longest parameter steps over which `surface` cannot turn unseen.
///
/// A NURBS surface is one polynomial patch per knot span, and within one patch
/// its control net bounds how far it bends, so a span is the scale. A periodic
/// support closes on itself once a period, and an eighth of one is short
/// enough that no edge can reach round to where it started. A plane, or a
/// direction along which a support is straight, has no such scale.
fn support_steps(surface: &Surface) -> Vector2<f64> {
    let unbounded = Vector2::new(f64::INFINITY, f64::INFINITY);
    match surface {
        Surface::Nurbs(nurbs) => Vector2::new(
            knot_span_width(nurbs.knots_u().as_slice()),
            knot_span_width(nurbs.knots_v().as_slice()),
        ),
        _ => {
            let (u, v) = match surface.periodicity() {
                SurfacePeriodicity::None => return unbounded,
                SurfacePeriodicity::UPeriodic(u) => (Some(u), None),
                SurfacePeriodicity::VPeriodic(v) => (None, Some(v)),
                SurfacePeriodicity::UVPeriodic(u, v) => (Some(u), Some(v)),
            };
            let step = |period: Option<f64>| period.map_or(f64::INFINITY, |period| period / 8.0);
            Vector2::new(step(u), step(v))
        }
    }
}

/// The size of one tessellation cell along each parameter direction.
///
/// A grid over the support would give a periodic direction `n` cells a period
/// -- `nu` along `u`, `nv` along `v` -- and a NURBS direction `n / 8` cells a
/// knot span, a span being about an eighth of the turns a curved NURBS makes.
/// A direction the support is straight in needs no cells at all: a plane, a
/// cylinder along its axis, a NURBS direction of degree one, which bends only
/// at its knots and is given one cell a span.
fn cell_steps(surface: &Surface, opts: TessellateOpts) -> Vector2<f64> {
    let counts = [opts.surface.nu.max(1) as f64, opts.surface.nv.max(1) as f64];
    let steps = support_steps(surface);
    match surface {
        Surface::Nurbs(nurbs) => {
            let degrees = [nurbs.degree_u().get(), nurbs.degree_v().get()];
            Vector2::from_fn(|axis, _| match degrees[axis] {
                1 => steps[axis],
                _ => 8.0 * steps[axis] / counts[axis],
            })
        }
        _ => Vector2::from_fn(|axis, _| 8.0 * steps[axis] / counts[axis]),
    }
}

/// The mean width of a knot vector's non-empty spans.
fn knot_span_width(knots: &[f64]) -> f64 {
    let spans = knots
        .windows(2)
        .filter(|pair| pair[1] - pair[0] > EPS)
        .count();
    match (knots.first(), knots.last()) {
        (Some(first), Some(last)) if spans > 0 => (last - first) / spans as f64,
        _ => f64::INFINITY,
    }
}

fn build_simple_polygon(
    surface: &Surface,
    outer_uv: &[Point2],
    inner_uv: &[Vec<Point2>],
) -> Option<Vec<Point2>> {
    let mut polygon = clean_loop(surface, outer_uv);
    if polygon.len() < 3 {
        return None;
    }
    if signed_area(&polygon) < 0.0 {
        polygon.reverse();
    }

    for hole in inner_uv {
        let mut hole = clean_loop(surface, hole);
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
/// whether the surface itself runs straight past it.
///
/// Straight, not merely close: every boundary sample is also a point of the
/// edge line drawn over the face and of the neighbour's mesh along that edge,
/// so a sample the surface bends through is kept however little it bends. A
/// looser test drops points in runs, each judged against neighbours that the
/// same pass is dropping too, and the boundary comes away from its edge.
fn clean_loop(surface: &Surface, points: &[Point2]) -> Vec<Point2> {
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
            if collinear && between && chord_follows_surface(surface, prev, curr, after, EPS) {
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

            // Any ear with area at all: a slab a sliver wide near a sharp
            // corner has no ear as large as a fixed threshold, however many
            // it has.
            if orient(a, b, c) <= 0.0 {
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
            // Nothing convex is left to clip, only vertices lying straight
            // between their neighbours. Such a vertex encloses nothing; it is
            // cut off as the flat triangle it is, which keeps it a vertex of
            // the mesh for the edge drawn through it.
            let flat = (0..len).find(|&i| {
                let (a, b, c) = (
                    polygon[vertices[(i + len - 1) % len]],
                    polygon[vertices[i]],
                    polygon[vertices[(i + 1) % len]],
                );
                orient(a, b, c).abs() <= EPS
            })?;
            indices.extend_from_slice(&[
                vertices[(flat + len - 1) % len] as u32,
                vertices[flat] as u32,
                vertices[(flat + 1) % len] as u32,
            ]);
            vertices.remove(flat);
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
