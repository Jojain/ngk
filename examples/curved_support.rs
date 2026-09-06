//! Disposable modeling probe for the 23-T-24 curved support drawing.
//!
//! The probe builds the two bored bosses at their drawing dimensions and a
//! tangent-sided web at the documented 11 mm thickness. It then attempts the
//! two curved Boolean unions. If either union is not yet supported, the same
//! components are kept as a three-solid visual assembly so the geometry can
//! still be inspected in the debug viewer.

use std::env;
use std::error::Error;

use nalgebra::Vector3;
use ngk::builders::boolean::{BooleanOperation, BooleanOptions, boolean};
use ngk::geometry::{Plane, Point3};
use ngk::modeling::{faces, sweep::extrude_face};
use ngk::topology::TopologyEditError;
use ngk::topology::gmap::GMap;
use ngk::topology::shape::{Shape, SolidTag};
use ngk::topology::shape_keys::SolidKey;
use ngk::topology::validation::{validate_gmap, validate_solid_manifold};
use ngk::viz::debug_viewer::{DebugViewerOptions, show_gmap_with_options};

const CENTRE_DISTANCE: f64 = 125.0;

const LARGE_OUTER_RADIUS: f64 = 27.5;
const LARGE_INNER_RADIUS: f64 = 17.5;
const LARGE_LENGTH: f64 = 60.0;

const SMALL_OUTER_RADIUS: f64 = 15.0;
const SMALL_INNER_RADIUS: f64 = 10.0;
const SMALL_LENGTH: f64 = 32.0;

const WEB_THICKNESS: f64 = 11.0;

/// Creates one bored cylindrical boss whose right end lies on `z = 60`.
fn boss(
    centre_y: f64,
    outer_radius: f64,
    inner_radius: f64,
    length: f64,
) -> Result<Shape<SolidTag>, Box<dyn Error>> {
    let start_z = LARGE_LENGTH - length;
    let plane = Plane::from_xy(
        Point3::new(0.0, centre_y, start_z),
        Vector3::x(),
        Vector3::y(),
    );
    let face = faces::annulus(plane, outer_radius, inner_radius)?;
    Ok(extrude_face(face, Vector3::new(0.0, 0.0, length))?)
}

/// Creates the front-view web with straight sides tangent to both bosses.
///
/// The web enters each annular boss only outside its bore. This gives the
/// Boolean a volumetric overlap while preserving both through holes.
fn web() -> Result<Shape<SolidTag>, Box<dyn Error>> {
    let radius_delta = LARGE_OUTER_RADIUS - SMALL_OUTER_RADIUS;
    let normal_y = radius_delta / CENTRE_DISTANCE;
    let normal_x = (1.0 - normal_y * normal_y).sqrt();
    let half_width = |y: f64| (LARGE_OUTER_RADIUS - normal_y * y) / normal_x;

    let bottom_y = LARGE_INNER_RADIUS;
    let top_y = CENTRE_DISTANCE - SMALL_INNER_RADIUS;
    let bottom_half_width = half_width(bottom_y);
    let top_half_width = half_width(top_y);
    let front_z = LARGE_LENGTH - WEB_THICKNESS;

    let outline = [
        Point3::new(-bottom_half_width, bottom_y, front_z),
        Point3::new(bottom_half_width, bottom_y, front_z),
        Point3::new(top_half_width, top_y, front_z),
        Point3::new(-top_half_width, top_y, front_z),
    ];
    let face = faces::polygon(&outline)?;
    Ok(extrude_face(face, Vector3::new(0.0, 0.0, WEB_THICKNESS))?)
}

/// Copies `source` into `target` and returns the remapped solid key.
fn import_solid(target: &mut GMap, source: &Shape<SolidTag>) -> Result<SolidKey, Box<dyn Error>> {
    let key = target.transaction(|edit| {
        let dart = edit.merge(source.solid());
        Ok::<_, TopologyEditError>(
            edit.solid_key(dart)
                .expect("a merged solid should retain its registration"),
        )
    })?;
    Ok(key)
}

/// Builds the three drawing components without claiming that they are fused.
fn visual_assembly() -> Result<GMap, Box<dyn Error>> {
    let web = web()?;
    let large = boss(0.0, LARGE_OUTER_RADIUS, LARGE_INNER_RADIUS, LARGE_LENGTH)?;
    let small = boss(
        CENTRE_DISTANCE,
        SMALL_OUTER_RADIUS,
        SMALL_INNER_RADIUS,
        SMALL_LENGTH,
    )?;

    let (mut map, _) = web.into_map();
    import_solid(&mut map, &large)?;
    import_solid(&mut map, &small)?;
    Ok(map)
}

/// Attempts to turn the three components into one regularized solid.
fn fused_support() -> Result<(GMap, SolidKey), Box<dyn Error>> {
    let web = web()?;
    let large = boss(0.0, LARGE_OUTER_RADIUS, LARGE_INNER_RADIUS, LARGE_LENGTH)?;
    let small = boss(
        CENTRE_DISTANCE,
        SMALL_OUTER_RADIUS,
        SMALL_INNER_RADIUS,
        SMALL_LENGTH,
    )?;

    let (mut map, web_key) = web.into_map();
    let large_key = import_solid(&mut map, &large)?;
    let first_union = boolean(
        &mut map,
        web_key,
        large_key,
        BooleanOperation::Union,
        BooleanOptions::default(),
    )?;

    let small_key = import_solid(&mut map, &small)?;
    let options = BooleanOptions {
        heal: true,
        ..Default::default()
    };
    let second_union = boolean(
        &mut map,
        first_union.solid,
        small_key,
        BooleanOperation::Union,
        options,
    )?;
    Ok((map, second_union.solid))
}

fn show(name: &str, map: &GMap) -> Result<(), Box<dyn Error>> {
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

fn print_counts(label: &str, map: &GMap) {
    println!(
        "{label}: {} solid(s), {} face(s), {} edge(s), {} vertex/vertices, {} dart(s)",
        map.iter_solids().count(),
        map.iter_faces().count(),
        map.iter_edges().count(),
        map.iter_vertices().count(),
        map.dart_count(),
    );
}

fn main() -> Result<(), Box<dyn Error>> {
    let try_fusion = env::args().any(|argument| argument == "--fuse");
    if try_fusion {
        match fused_support() {
            Ok((map, solid)) => {
                validate_gmap(&map)?;
                validate_solid_manifold(&map, solid)?;
                print_counts("fused curved support", &map);
                println!("Both curved Boolean unions succeeded; displaying one solid.");
                show("23-T-24 curved support - fused", &map)?;
            }
            Err(error) => {
                eprintln!("Curved Boolean fusion failed: {error}");
                show_assembly()?;
            }
        }
    } else {
        println!("Boolean fusion is opt-in because this curved case is currently very slow.");
        println!("Re-run with --fuse to exercise it.");
        show_assembly()?;
    }
    Ok(())
}

/// Validates and displays the unfused three-solid approximation.
fn show_assembly() -> Result<(), Box<dyn Error>> {
    let map = visual_assembly()?;
    validate_gmap(&map)?;
    for (solid, _) in map.iter_solids() {
        validate_solid_manifold(&map, solid)?;
    }
    print_counts("visual curved-support assembly", &map);
    println!("Displaying the dimensioned approximation as three valid solids.");
    show("23-T-24 curved support - visual assembly", &map)
}
