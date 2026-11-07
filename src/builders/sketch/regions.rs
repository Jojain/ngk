use super::SketchError;
use super::arrangement::{Region, Stretch, regions};
use super::drawn::drawn_curves;
use super::joints::Joints;
use super::path::Path;
use crate::builders::edges::{add_circle_edit, add_edge_edit};
use crate::builders::faces::add_face_with_holes_edit;
use crate::builders::profiles::add_profile_from_edges_edit;
use crate::geometry::dim2::utils::Point2;
use crate::geometry::{Curve, Plane, Point3};
use crate::model::Model;
use crate::solver::{CurveId, System2d};
use crate::topology::ModelEdit;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey};

/// One closed area a system's curves enclose, as a loose face.
#[derive(Clone, Debug)]
pub struct SystemRegion {
    pub face: FaceKey,
    /// The edges this face runs, outer boundary first, each with the curve it lies on. A
    /// curve cut by another draws several edges, and a stretch two regions share is drawn
    /// once for each of them.
    pub boundary: Vec<(CurveId, EdgeKey)>,
}

/// What [`add_sketch_from_system`] added: one loose face per region.
///
/// The keys hold within this model until the faces are sewn, which merges the two edges
/// each shared stretch has. The order of the regions is the same for the same input.
#[derive(Clone, Debug)]
pub struct SketchFromSystem {
    pub regions: Vec<SystemRegion>,
}

/// Adds a face for each closed area a solved system's curves enclose on `plane`, its
/// coordinates mapping through the plane's axes.
///
/// A rectangle with a circle inside it gives a ring and a disc, and two crossing rectangles
/// give the area they share and what is left of each. Curves meeting form a corner there,
/// whether or not a coincidence joins them. What bounds no area is left out: a line
/// overshooting a rectangle still cuts it in two, and nothing remains of the overshoot.
///
/// The faces are loose, each with its own edges and all with the plane's normal, so each
/// can be used on its own. `add_sheet_from_faces` sews the ones that are connected.
///
/// Refuses curves that overlap, a region whose boundary runs through a point twice, and
/// what the intersection search cannot certify.
pub fn add_sketch_from_system<P: Payload>(
    model: &mut Model<P>,
    system: &System2d,
    plane: Plane,
    curves: &[CurveId],
) -> Result<SketchFromSystem, SketchError> {
    model.transaction(|edit| add_sketch_from_system_edit(edit, system, &plane, curves))
}

/// Works out the regions, then draws them inside the caller's transaction.
pub(crate) fn add_sketch_from_system_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    system: &System2d,
    plane: &Plane,
    curves: &[CurveId],
) -> Result<SketchFromSystem, SketchError> {
    let drawn = drawn_curves(system, curves)?;
    let joints = Joints::of(system, &drawn)?;
    let regions = regions(&drawn, &joints)?
        .iter()
        .map(|region| add_region_edit(edit, region, plane))
        .collect::<Result<_, _>>()?;
    Ok(SketchFromSystem { regions })
}

fn add_region_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    region: &Region,
    plane: &Plane,
) -> Result<SystemRegion, SketchError> {
    let mut boundary = vec![];
    let outer = add_boundary_edit(edit, &region.outer, plane, &mut boundary)?;
    let holes = region
        .holes
        .iter()
        .map(|hole| add_boundary_edit(edit, hole, plane, &mut boundary))
        .collect::<Result<Vec<_>, _>>()?;
    let face = add_face_with_holes_edit(edit, plane.clone(), outer, &holes)?;
    Ok(SystemRegion { face, boundary })
}

/// Draws one closed walk as a profile running the way it is walked.
fn add_boundary_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    stretches: &[Stretch],
    plane: &Plane,
    boundary: &mut Vec<(CurveId, EdgeKey)>,
) -> Result<ProfileKey, SketchError> {
    let mut keys = Vec::with_capacity(stretches.len());
    for stretch in stretches {
        let key = add_stretch_edit(edit, stretch.path, plane)?;
        boundary.push((stretch.curve, key));
        keys.push(key);
    }
    Ok(add_profile_from_edges_edit(edit, &keys)?)
}

/// Draws one stretch as an edge whose support runs the way the stretch is walked.
fn add_stretch_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    path: Path,
    plane: &Plane,
) -> Result<EdgeKey, SketchError> {
    let lift = |point: Point2| -> Point3 { plane.point_at(point.x, point.y) };
    let at = |center: Point2, counterclockwise: bool| {
        let normal = if counterclockwise {
            plane.normal()
        } else {
            -plane.normal()
        };
        Plane::new(lift(center), plane.x_dir(), normal)
    };
    let key = match path {
        Path::Line { start, end } => {
            let (start, end) = (lift(start), lift(end));
            add_edge_edit(edit, start, end, Curve::line(start, end))?
        }
        Path::Arc {
            center,
            radius,
            start,
            end,
            sweep,
        } => add_edge_edit(
            edit,
            lift(start),
            lift(end),
            Curve::circle(at(center, sweep > 0.), radius),
        )?,
        Path::Circle {
            center,
            radius,
            counterclockwise,
            corner,
        } => {
            let key = add_circle_edit(edit, at(center, counterclockwise), radius)?;
            if let Some(corner) = corner {
                edit.mark_edge(key, lift(corner))?;
            }
            key
        }
    };
    Ok(key)
}
