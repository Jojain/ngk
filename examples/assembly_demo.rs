//! A slider-crank solved by the assembly solver and sent to the debug viewer.
//!
//! Start the viewer first (`npm run debug` in `visualization/`), then run
//! `cargo run --example assembly_demo`. Each crank angle arrives as one timeline entry.

use nalgebra::Vector3;
use ngk::geometry::dim3::frame::Frame;
use ngk::geometry::dim3::utils::Point3;
use ngk::geometry::transform::Rigid;
use ngk::modeling::solids::{block_at, cylinder_at};
use ngk::solver::assembly::{
    Assembly3d, Attachment, BodyId, Coordinate, Joint3d, JointId, JointKind, Solution3d,
};
use ngk::topology::shape::{Shape, SolidTag};
use ngk::viz::debug_viewer::{
    DebugDisplay, DebugNode, DebugStyle, DebugViewerOptions, DebugViewerPayload, send_payload,
};
use std::error::Error;

/// Crank pin radius from the crank axis, and the rod's length between its pins.
const CRANK: f64 = 1.0;
const ROD: f64 = 3.0;

/// A bar of `width` and `height` running along X from `start` to `end`, its bottom at `z`.
fn bar(start: f64, end: f64, width: f64, z: f64, height: f64) -> Shape<SolidTag> {
    block_at(
        Frame::at(Point3::new(start, -width / 2., z)),
        end - start,
        width,
        height,
    )
    .expect("a bar has positive sizes")
}

/// A pin: an upright cylinder at `(x, 0)`, its bottom at `z`.
fn pin(x: f64, z: f64, radius: f64, height: f64) -> Shape<SolidTag> {
    cylinder_at(Frame::at(Point3::new(x, 0., z)), radius, height).expect("a pin has positive sizes")
}

/// Each body's solids in its own coordinates, which are the world's at crank angle 0.
fn ground() -> Vec<Shape<SolidTag>> {
    vec![
        bar(-0.8, ROD + CRANK + 1.2, 1.6, -0.3, 0.1),
        pin(0., -0.2, 0.25, 0.2),
        bar(CRANK + 0.5, ROD + CRANK + 1.2, 0.3, -0.2, 0.15),
    ]
}
fn crank() -> Vec<Shape<SolidTag>> {
    vec![
        bar(0., CRANK, 0.3, 0., 0.15),
        pin(0., 0., 0.2, 0.15),
        pin(CRANK, 0., 0.12, 0.45),
    ]
}
fn rod() -> Vec<Shape<SolidTag>> {
    vec![bar(CRANK, CRANK + ROD, 0.2, 0.2, 0.1)]
}
fn slider() -> Vec<Shape<SolidTag>> {
    let x = CRANK + ROD;
    vec![
        bar(x - 0.35, x + 0.35, 0.5, -0.05, 0.25),
        pin(x, 0.2, 0.1, 0.25),
    ]
}

fn hinge(first: BodyId, second: BodyId, x: f64) -> Joint3d {
    let at = Frame::at(Point3::new(x, 0., 0.));
    Joint3d::new(
        JointKind::Revolute,
        Attachment::new(first, at.clone()),
        Attachment::new(second, at),
    )
}

fn node(
    name: &str,
    solids: Vec<Shape<SolidTag>>,
    pose: Rigid,
    color: &str,
) -> Result<DebugNode, Box<dyn Error>> {
    let moved: Vec<_> = solids.into_iter().map(|solid| solid.moved(pose)).collect();
    let mut objects = vec![];
    moved.append_debug_objects(&mut objects)?;
    let children = objects
        .into_iter()
        .enumerate()
        .map(|(i, object)| DebugNode::leaf(format!("{name}[{i}]"), object))
        .collect();
    Ok(DebugNode::group(name, children).styled(DebugStyle {
        color: Some(color.to_owned()),
        ..DebugStyle::default()
    }))
}

struct Linkage {
    ground: BodyId,
    crank: BodyId,
    rod: BodyId,
    slider: BodyId,
    slide: JointId,
}

fn send(solution: &Solution3d, linkage: &Linkage, name: &str) -> Result<(), Box<dyn Error>> {
    let nodes = vec![
        node(
            "ground",
            ground(),
            solution.pose(linkage.ground)?,
            "#8a8f98",
        )?,
        node("crank", crank(), solution.pose(linkage.crank)?, "#e8833a")?,
        node("rod", rod(), solution.pose(linkage.rod)?, "#3a7be8")?,
        node(
            "slider",
            slider(),
            solution.pose(linkage.slider)?,
            "#3aa865",
        )?,
    ];
    let options = DebugViewerOptions {
        name: name.to_owned(),
        ..DebugViewerOptions::default()
    };
    let payload = DebugViewerPayload {
        kind: "ngk.debug.v4".to_owned(),
        name: name.to_owned(),
        nodes,
    };
    send_payload(&payload, &options)?;
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut assembly = Assembly3d::default();
    let ground = assembly.add_body(Rigid::identity())?;
    assembly.fix(ground)?;
    // Seeds a little off, so the first solve has something to close.
    let crank = assembly.add_body(Rigid::translation(Vector3::new(0.05, -0.03, 0.)))?;
    let rod = assembly.add_body(Rigid::translation(Vector3::new(-0.1, 0.2, 0.)))?;
    let slider = assembly.add_body(Rigid::translation(Vector3::new(0.2, 0.15, 0.)))?;
    let rail = |x: f64| Frame::from_xz(Point3::new(x, 0., 0.), Vector3::y(), Vector3::x());
    let drive = hinge(ground, crank, 0.);
    let motor = assembly.add_joint(drive.clone())?;
    assembly.add_joint(hinge(crank, rod, CRANK))?;
    assembly.add_joint(hinge(rod, slider, CRANK + ROD))?;
    let slide = assembly.add_joint(Joint3d::new(
        JointKind::Prismatic,
        Attachment::new(ground, rail(0.)),
        Attachment::new(slider, rail(CRANK + ROD)),
    ))?;
    let linkage = Linkage {
        ground,
        crank,
        rod,
        slider,
        slide,
    };

    let free = assembly.solve()?;
    let report = free.report();
    println!(
        "free: mobility {}, redundancy {}, {} iterations",
        report.mobility, report.redundancy, report.iterations
    );
    send(&free, &linkage, "slider-crank, closed from its seeds")?;

    for step in 1..=12 {
        let degrees = 30. * step as f64;
        assembly.update_joint(
            motor,
            drive
                .clone()
                .with_value(Coordinate::Angle, degrees.to_radians()),
        )?;
        let solution = assembly.solve()?;
        let x = solution.coordinate(linkage.slide, Coordinate::Slide)?;
        println!(
            "crank {degrees:>5.0}°  slider at x = {x:.4}  ({} iterations)",
            solution.report().iterations
        );
        send(&solution, &linkage, &format!("crank {degrees:.0}°"))?;
    }
    Ok(())
}
