//! An edge and the faces along it are meshed to meet.
//!
//! The viewer draws each edge as its own polyline over the faces' meshes. If a
//! face samples the edge at other points than the edge line does, the line
//! dips under the surface between them and two neighbouring faces open a gap
//! along it. So every point of an edge's polyline has to be a vertex of every
//! face that edge bounds.

use nalgebra::Vector3;
use ngk::builders::sweep::{SweepFrame, SweepOptions};
use ngk::geometry::{Axis3, Frame, Helix, NativeParam, Plane, Point3};
use ngk::model::Model;
use ngk::modeling::sweep::sweep_face;
use ngk::modeling::{edges, faces, solids};
use ngk::tessellate::{
    CurveOpts, SurfaceOpts, TessellateOpts, tessellate_edge, tessellate_face_key,
};
use radians::Rad64;

const OPTS: TessellateOpts = TessellateOpts {
    curve: CurveOpts { segments: 64 },
    surface: SurfaceOpts { nu: 64, nv: 32 },
};

fn assert_edges_are_face_vertices(model: &Model, tolerance: f64) {
    for (face_key, _) in model.iter_faces() {
        let mesh = tessellate_face_key(model, face_key, OPTS).expect("face should mesh");
        for edge in model.face_unchecked(face_key).edges() {
            let line = tessellate_edge(model, edge.key(), OPTS).expect("edge should sample");
            for point in &line.points {
                let nearest = mesh
                    .positions
                    .iter()
                    .map(|vertex| (vertex - point).norm())
                    .fold(f64::INFINITY, f64::min);
                assert!(
                    nearest <= tolerance,
                    "{point:?} on edge {:?} is {nearest} from face {face_key:?}'s mesh",
                    edge.key()
                );
            }
        }
    }
}

#[test]
fn a_helical_sweep_meets_its_rails_at_every_point_they_are_drawn_at() {
    let axis = Axis3::z();
    let (radius, pitch, turns) = (2.0, 1.0, 3.0);
    let spine = edges::helix(
        axis,
        radius,
        pitch,
        Rad64::new(0.0),
        Rad64::new(std::f64::consts::TAU * turns),
    )
    .expect("helix");
    let start = Helix::from_axis(axis, radius, pitch).point_at(NativeParam::new(0.0));
    let outward = Vector3::new(start.x, start.y, 0.0).normalize();
    let section = faces::rectangle(
        Plane::from_xy(
            start - outward * 0.2 - Vector3::z() * 0.2,
            outward,
            Vector3::z(),
        ),
        0.4,
        0.4,
    )
    .expect("section");
    let solid = sweep_face(
        section,
        &spine.edge(),
        SweepOptions {
            frame: SweepFrame::Axial(axis),
            samples_per_segment: 48,
            ..SweepOptions::default()
        },
    )
    .expect("sweep");

    assert_edges_are_face_vertices(solid.model(), 1e-9);
}

#[test]
fn a_cylinder_fused_onto_a_block_meets_the_rims_it_shares_with_it() {
    let block = solids::block(2.0, 2.0, 1.0).expect("block");
    let cylinder =
        solids::cylinder_at(Frame::at(Point3::new(1.0, 1.0, 0.5)), 0.5, 1.5).expect("cylinder");
    let fused = solids::fuse(block, cylinder).expect("fuse");

    assert_edges_are_face_vertices(fused.model(), 1e-9);
}
