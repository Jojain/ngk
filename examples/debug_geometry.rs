use nalgebra::Vector3;
use ngk::geometry::{Curve, Cylinder, Plane, Point3, Surface};
use ngk::viz::debug_viewer::{
    DebugDisplay, DebugViewerError, DebugViewerOptions, show_with_options,
};

fn show_named(name: &str, value: &(impl DebugDisplay + ?Sized)) -> Result<(), DebugViewerError> {
    show_with_options(
        value,
        &DebugViewerOptions {
            name: name.to_owned(),
            ..DebugViewerOptions::default()
        },
    )
}

fn main() -> Result<(), DebugViewerError> {
    let point = Point3::new(0.75, 0.5, 0.25);
    let vector = Vector3::new(1.0, 0.5, 0.75);
    let plane = Plane::new(Point3::origin(), Vector3::x(), Vector3::z());
    let curve = Curve::circle(plane.clone(), 1.25);
    let surface = Surface::Cylinder(Cylinder::new(
        Point3::origin(),
        Vector3::x(),
        Vector3::z(),
        0.8,
    ));

    show_named("point", &point)?;
    show_named("vector", &vector)?;
    show_named("plane", &plane)?;
    show_named("circle", &curve)?;
    show_named("cylinder", &surface)?;
    Ok(())
}
