use std::error::Error;

use nalgebra::Vector3;
use ngk::builders::boolean::{BooleanOperand, prepare_boolean_with_external_tool};
use ngk::builders::solids::add_extruded_face;
use ngk::geometry::{Plane, Point3};
use ngk::modeling::faces;
use ngk::topology::gmap::GMap;
use ngk::topology::shape_keys::SolidKey;
use ngk::viz::debug_viewer::{DebugViewerOptions, show_gmap_with_options};

fn block_at(origin: Point3, size: f64) -> Result<(GMap, SolidKey), Box<dyn Error>> {
    let plane = Plane::from_xy(origin, Vector3::x(), Vector3::y());
    let base = faces::rectangle(plane, size, size)?;
    let (mut map, face) = base.into_map();
    let solid = add_extruded_face(&mut map, face, Vector3::new(0.0, 0.0, size))?;
    Ok((map, solid))
}

fn show_named(name: &str, map: &GMap) -> Result<(), Box<dyn Error>> {
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
    let (mut working_map, target) = block_at(Point3::origin(), 1.0)?;
    let (tool_map, tool) = block_at(Point3::new(0.5, 0.5, 0.5), 1.0)?;

    show_named("boolean 1 - target before", &working_map)?;
    show_named("boolean 2 - external tool before", &tool_map)?;

    // This is the high-level entry point for an external tool. It copies the
    // tool into `working_map`, computes contacts, and splits both operands.
    let preparation = prepare_boolean_with_external_tool(
        &mut working_map,
        BooleanOperand::Solid(target),
        &tool_map,
        BooleanOperand::Solid(tool),
        Default::default(),
    )?;

    show_named("boolean 3 - both operands split", &working_map)?;

    let first_faces = preparation
        .first_lineage
        .faces
        .values()
        .map(Vec::len)
        .sum::<usize>();
    let second_faces = preparation
        .second_lineage
        .faces
        .values()
        .map(Vec::len)
        .sum::<usize>();
    println!(
        "network: {} events, {} spans, {} regions",
        preparation.network.events().len(),
        preparation.network.spans().len(),
        preparation.network.regions().len()
    );
    println!("target face fragments: {first_faces}");
    println!("tool face fragments: {second_faces}");
    println!("imported tool handle: {:?}", preparation.imported_tool);
    println!(
        "first source faces: {}, second source faces: {}",
        preparation.first_lineage.faces.len(),
        preparation.second_lineage.faces.len()
    );
    Ok(())
}
