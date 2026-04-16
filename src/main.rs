use nalgebra::Vector3;
use ngk::geometry::surfaces::Plane;
use ngk::geometry::utils::Point3;
use ngk::topology::gmap::GMap;

fn main() {
    let plane = Plane::from_xy(
        Point3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    );
    let gmap = GMap::new(3);
    println!(
        "ngk demo: plane origin {:?}, gmap dim {}",
        plane.origin,
        gmap.dimension()
    );
}
