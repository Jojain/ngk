use std::env;
use std::error::Error;

use nalgebra::Vector3;
use ngk::builders::boolean::{BooleanOperand, BooleanOptions, prepare_boolean_with_external_tool};
use ngk::builders::edges::add_edge;
use ngk::builders::solids::add_extruded_face;
use ngk::geometry::{Curve, NurbsCurve, Plane, Point3};
use ngk::modeling::{faces, sweep::extrude_profile};
use ngk::topology::gmap::GMap;
use ngk::topology::shape::{EdgeTag, Shape};
use ngk::topology::shape_keys::{SheetKey, SolidKey};
use ngk::viz::debug_viewer::{DebugViewerOptions, show_gmap_with_options};

fn block_at(origin: Point3, size: f64) -> Result<(GMap, SolidKey), Box<dyn Error>> {
    let plane = Plane::from_xy(origin, Vector3::x(), Vector3::y());
    let base = faces::rectangle(plane, size, size)?;
    let (mut map, face) = base.into_map();
    let solid = add_extruded_face(&mut map, face, Vector3::new(0.0, 0.0, size))?;
    Ok((map, solid))
}

fn curved_sheet() -> Result<(GMap, SheetKey), Box<dyn Error>> {
    let points = [
        Point3::new(-0.5, -0.5, 1.0),
        Point3::new(0.25, -0.5, 0.55),
        Point3::new(1.0, -0.5, 1.45),
        Point3::new(1.75, -0.5, 0.55),
        Point3::new(2.5, -0.5, 1.0),
    ];
    let curve = Curve::Nurbs(NurbsCurve::interpolate(&points)?);
    let mut map = GMap::new();
    let edge = add_edge(&mut map, points[0], points[points.len() - 1], curve)?;
    let profile = Shape::<EdgeTag>::new(map, edge).into_profile();
    Ok(extrude_profile(profile.profile(), Vector3::new(0.0, 3.0, 0.0))?.into_map())
}

fn show_named(name: &str, map: &GMap) -> Result<(), Box<dyn Error>> {
    if env::var_os("NGK_SKIP_DEBUG_VIEWER").is_some() {
        return Ok(());
    }
    show_gmap_with_options(
        map,
        &DebugViewerOptions {
            name: name.to_owned(),
            ..Default::default()
        },
    )?;
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let (mut working_map, cube) = block_at(Point3::origin(), 2.0)?;
    let (curved_map, curved_sheet) = curved_sheet()?;

    show_named("curved boolean 1 - cube before", &working_map)?;
    show_named("curved boolean 2 - NURBS sheet before", &curved_map)?;

    // Lower sampling keeps this interactive debug example responsive. Use
    // `BooleanOptions::default()` when evaluating production tolerances.
    let mut options = BooleanOptions::default();
    options.intersections.curve_sample_count = 24;
    options.intersections.surface_u_sample_count = 12;
    options.intersections.surface_v_sample_count = 12;
    let preparation = prepare_boolean_with_external_tool(
        &mut working_map,
        BooleanOperand::Solid(cube),
        &curved_map,
        BooleanOperand::Sheet(curved_sheet),
        options,
    )?;

    show_named(
        "curved boolean 3 - cube and NURBS sheet prepared",
        &working_map,
    )?;

    let target_fragments = preparation
        .first_lineage
        .faces
        .values()
        .map(Vec::len)
        .sum::<usize>();
    let tool_fragments = preparation
        .second_lineage
        .faces
        .values()
        .map(Vec::len)
        .sum::<usize>();
    let target_edge_fragments = preparation
        .first_lineage
        .edges
        .values()
        .map(Vec::len)
        .sum::<usize>();
    let tool_edge_fragments = preparation
        .second_lineage
        .edges
        .values()
        .map(Vec::len)
        .sum::<usize>();
    println!(
        "network: {} events, {} spans, {} regions",
        preparation.network.events().len(),
        preparation.network.spans().len(),
        preparation.network.regions().len()
    );
    println!("cube face fragments: {target_fragments}");
    println!("curved-sheet face fragments: {tool_fragments}");
    println!("cube edge fragments: {target_edge_fragments}");
    println!("curved-sheet edge fragments: {tool_edge_fragments}");
    println!(
        "imported curved-sheet handle: {:?}",
        preparation.imported_tool
    );
    Ok(())
}
