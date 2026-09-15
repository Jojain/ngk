use nalgebra::Vector3;
use ngk::geometry::Plane;
use ngk::modeling::{faces, sweep::extrude_face};
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

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let face = faces::annulus(Plane::xy(), 2.0, 0.85)?;
    show_named("holes 1 - face with inner loop", &face)?;

    let solid = extrude_face(face, Vector3::new(0.0, 0.0, 2.0))?;
    show_named("holes 2 - solid with through hole", &solid)?;

    Ok(())
}
