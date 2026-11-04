//! Reduced reproductions behind plan/robustness. Drop this file into
//! brep-bench/kernels/ngk/src/cases/boolean/ as zz_probe.rs, rebuild, and run
//! `PROBE=<name> brep-bench-ngk boolean/zz-probe out.stl` (W=<width> for slot_w).
//! It prints each Boolean stage and, on incomplete coverage, the diagnostics.
use crate::bench::*;
use crate::{Refusal, Solid, refuse};
use ngk::builders::boolean::BooleanOperation;
use ngk::modeling::solids;
pub fn build() -> Result<Solid, Refusal> {
    let which = std::env::var("PROBE").unwrap_or_default();
    let (a, b, op) = match which.as_str() {
        "napkin" => (ball(p(0.0,0.0,0.0),5.0)?, cylinder_z(0.0,0.0,-10.0,10.0,3.0)?, BooleanOperation::Difference),
        "napkin_x" => (solids::sphere_at(frame(p(0.0,0.0,0.0), v(1.0,0.0,0.0)),5.0)?, cylinder_z(0.0,0.0,-10.0,10.0,3.0)?, BooleanOperation::Difference),
        "napkin_short" => (ball(p(0.0,0.0,0.0),5.0)?, cylinder_z(0.0,0.0,-10.0,0.0,3.0)?, BooleanOperation::Difference),
        "ball_cyl_off" => (ball(p(0.0,0.0,0.0),5.0)?, cylinder_z(1.0,0.5,-10.0,10.0,2.0)?, BooleanOperation::Difference),
        "ball_cyl_side" => (ball(p(0.0,0.0,0.0),5.0)?, cylinder(p(0.0,-10.0,0.0), v(0.0,1.0,0.0), 2.0, 20.0)?, BooleanOperation::Difference),
        "ball_halved" => (ball(p(0.0,0.0,0.0),5.0)?, block(p(-10.0,-10.0,-10.0), p(0.0,10.0,10.0))?, BooleanOperation::Difference),
        "ball_halved_y" => (ball(p(0.0,0.0,0.0),5.0)?, block(p(-10.0,-10.0,-10.0), p(10.0,0.0,10.0))?, BooleanOperation::Difference),
        "ball_off_x" => (ball(p(0.0,0.0,0.0),5.0)?, block(p(-10.0,-10.0,-10.0), p(1.0,10.0,10.0))?, BooleanOperation::Difference),
        "golf1" => (ball(p(0.0,0.0,0.0),10.0)?, ball(p(0.0, 5.57, 9.02),1.2)?, BooleanOperation::Difference),
        "golf_pole" => (ball(p(0.0,0.0,0.0),10.0)?, ball(p(0.0, 0.0, 10.6),1.2)?, BooleanOperation::Difference),
        "golf_eq" => (ball(p(0.0,0.0,0.0),10.0)?, ball(p(10.6, 0.0, 0.0),1.2)?, BooleanOperation::Difference),
        "coax" => (cylinder_z(0.0,0.0,0.0,10.0,5.0)?, cylinder_z(0.0,0.0,6.0,16.0,5.0)?, BooleanOperation::Union),
        "stein" => (cylinder(p(-15.0,0.0,0.0), v(1.0,0.0,0.0),5.0,30.0)?, cylinder(p(0.0,-15.0,0.0), v(0.0,1.0,0.0),5.0,30.0)?, BooleanOperation::Intersection),
        "stein_unequal" => (cylinder(p(-15.0,0.0,0.0), v(1.0,0.0,0.0),5.0,30.0)?, cylinder(p(0.0,-15.0,0.0), v(0.0,1.0,0.0),3.0,30.0)?, BooleanOperation::Union),
        "cube_hair" => (block(p(-5.0,-5.0,0.0),p(5.0,5.0,10.0))?, block(p(-5.0,-5.0,0.0),p(5.0,5.0,10.0))?.rotated(ngk::geometry::Axis3::z(), deg(0.001)), BooleanOperation::Union),
        "cube_turn" => (block(p(-5.0,-5.0,0.0),p(5.0,5.0,10.0))?, block(p(-5.0,-5.0,0.0),p(5.0,5.0,10.0))?.rotated(ngk::geometry::Axis3::z(), deg(10.0)), BooleanOperation::Union),
        "cube_turn_tall" => (block(p(-5.0,-5.0,0.0),p(5.0,5.0,10.0))?, block(p(-5.0,-5.0,-1.0),p(5.0,5.0,11.0))?.rotated(ngk::geometry::Axis3::z(), deg(10.0)), BooleanOperation::Union),
        "hairline" => (block(p(0.0,0.0,0.0),p(10.0,10.0,10.0))?, block(p(5.0,-1.0,5.0),p(5.000001,11.0,11.0))?, BooleanOperation::Difference),
        "slot_1mm" => (block(p(0.0,0.0,0.0),p(10.0,10.0,10.0))?, block(p(5.0,-1.0,5.0),p(5.1,11.0,11.0))?, BooleanOperation::Difference),
        "half_cyl_tangent" => (block(p(0.0,0.0,0.0),p(10.0,10.0,10.0))?, cylinder_z(10.0,5.0,0.0,10.0,5.0)?, BooleanOperation::Union),
        "half_cyl_inside" => (block(p(0.0,0.0,0.0),p(10.0,10.0,10.0))?, cylinder_z(10.0,5.0,0.0,10.0,4.0)?, BooleanOperation::Union),
        "midsphere" => (block(p(-5.0,-5.0,-5.0),p(5.0,5.0,5.0))?, ball(p(0.0,0.0,0.0),50f64.sqrt())?, BooleanOperation::Intersection),
        "ballbox" => (block(p(-5.0,-5.0,-5.0),p(5.0,5.0,5.0))?, ball(p(0.0,0.0,0.0),6.0)?, BooleanOperation::Intersection),
        "torus_rod" => (cylinder_z(0.0,0.0,0.0,20.0,5.0)?, extrude(stadium(11.5,0.0,-9.0,6.0,2.0)?, v(0.0,0.0,38.0))?, BooleanOperation::Difference),
        "rib" => (cylinder_z(0.0,0.0,0.0,10.0,10.0)?, revolved(&[p(5.0,0.0,4.0),p(20.0,0.0,4.0),p(20.0,0.0,0.0),p(5.0,0.0,0.0)], p(0.0,0.0,0.0), v(0.0,0.0,1.0), 120.0)?, BooleanOperation::Union),
        "sector_box" => (revolved(&[p(5.0,0.0,4.0),p(20.0,0.0,4.0),p(20.0,0.0,0.0),p(5.0,0.0,0.0)], p(0.0,0.0,0.0), v(0.0,0.0,1.0), 120.0)?, block(p(-30.0,-30.0,2.0),p(30.0,30.0,10.0))?, BooleanOperation::Union),
        "cc_generic" => (cylinder(p(-15.0,0.0,0.0), v(1.0,0.0,0.0),5.0,30.0)?, cylinder(p(0.0,-15.0,1.0), v(0.0,1.0,0.0),3.0,30.0)?, BooleanOperation::Union),
        "cc_skew" => (cylinder(p(-15.0,0.0,0.0), v(1.0,0.0,0.0),5.0,30.0)?, cylinder(p(0.0,-15.0,1.0), v(0.3,1.0,0.2),3.0,30.0)?, BooleanOperation::Union),
        "cc_drill_block" => (block(p(-10.0,-10.0,-10.0),p(10.0,10.0,10.0))?, cylinder(p(-15.0,0.0,0.0), v(1.0,0.0,0.0),3.0,30.0)?, BooleanOperation::Difference),
        "cross_drill" => (cut(block(p(-10.0,-10.0,-10.0),p(10.0,10.0,10.0))?, &cylinder(p(-15.0,0.0,0.0), v(1.0,0.0,0.0),3.0,30.0)?)?, cylinder(p(0.0,-15.0,0.0), v(0.0,1.0,0.0),2.0,30.0)?, BooleanOperation::Difference),
        "sph_cyl_generic" => (ball(p(0.0,0.0,0.0),5.0)?, cylinder(p(2.0,-10.0,1.0), v(0.1,1.0,0.0),2.0,20.0)?, BooleanOperation::Union),
        "cone_plane" => (cone(p(0.0,0.0,0.0), v(0.0,0.0,1.0),5.0,1.0,10.0)?, block(p(-10.0,-10.0,4.0),p(10.0,10.0,20.0))?.rotated(ngk::geometry::Axis3::y(), deg(10.0)), BooleanOperation::Difference),
        "cone_plane_straight" => (cone(p(0.0,0.0,0.0), v(0.0,0.0,1.0),5.0,1.0,10.0)?, block(p(-10.0,-10.0,4.0),p(10.0,10.0,20.0))?, BooleanOperation::Difference),
        "cone_cyl" => (cone(p(0.0,0.0,0.0), v(0.0,0.0,1.0),5.0,1.0,10.0)?, cylinder(p(-10.0,0.0,4.0), v(1.0,0.0,0.0),1.0,20.0)?, BooleanOperation::Difference),
        "cone_sphere" => (cone(p(0.0,0.0,0.0), v(0.0,0.0,1.0),5.0,1.0,10.0)?, ball(p(2.0,0.0,5.0),2.0)?, BooleanOperation::Difference),
        "torus_plane" => (solids::torus(10.0,2.0)?, block(p(-20.0,-20.0,0.5),p(20.0,20.0,5.0))?, BooleanOperation::Difference),
        "torus_plane_tilt" => (solids::torus(10.0,2.0)?, block(p(-20.0,-20.0,0.5),p(20.0,20.0,5.0))?.rotated(ngk::geometry::Axis3::x(), deg(3.0)), BooleanOperation::Difference),
        "torus_cyl" => (solids::torus(10.0,2.0)?, cylinder_z(10.0,0.0,-5.0,5.0,1.0)?, BooleanOperation::Difference),
        "torus_sphere" => (solids::torus(10.0,2.0)?, ball(p(10.0,0.0,1.0),1.5)?, BooleanOperation::Difference),
        "sph_sph_generic" => (ball(p(0.0,0.0,0.0),5.0)?, ball(p(3.0,2.0,1.0),4.0)?, BooleanOperation::Union),
        "coax_cyl_diff_r" => (cylinder_z(0.0,0.0,0.0,10.0,5.0)?, cylinder_z(0.0,0.0,5.0,15.0,3.0)?, BooleanOperation::Union),
        "sph_cyl_coax" => (ball(p(0.0,0.0,0.0),5.0)?, cylinder_z(0.0,0.0,0.0,10.0,3.0)?, BooleanOperation::Union),
        "torus_cyl_coax" => (solids::torus(10.0,2.0)?, cylinder_z(0.0,0.0,-1.0,1.0,11.0)?, BooleanOperation::Union),
        "torus_cyl_off" => (solids::torus(10.0,2.0)?, cylinder_z(0.0,10.0,-5.0,5.0,1.0)?, BooleanOperation::Difference),
        "torus_sphere_off" => (solids::torus(10.0,2.0)?, ball(p(0.0,10.0,1.0),1.5)?, BooleanOperation::Difference),
        "cyl_sphere_seam" => (cylinder_z(0.0,0.0,0.0,10.0,5.0)?, ball(p(5.0,0.0,5.0),1.5)?, BooleanOperation::Difference),
        "cyl_sphere_off" => (cylinder_z(0.0,0.0,0.0,10.0,5.0)?, ball(p(0.0,5.0,5.0),1.5)?, BooleanOperation::Difference),
        "cone_plane_tilt_small" => (cone(p(0.0,0.0,0.0), v(0.0,0.0,1.0),5.0,1.0,10.0)?, block(p(-10.0,-10.0,4.0),p(10.0,10.0,20.0))?.rotated(ngk::geometry::Axis3::y(), deg(2.0)), BooleanOperation::Difference),
        "cyl_plane_tilt" => (cylinder_z(0.0,0.0,0.0,10.0,5.0)?, block(p(-10.0,-10.0,4.0),p(10.0,10.0,20.0))?.rotated(ngk::geometry::Axis3::y(), deg(10.0)), BooleanOperation::Difference),
        "sphere_two_planes" => (ball(p(0.0,0.0,0.0),5.0)?, block(p(-10.0,-10.0,-1.0),p(10.0,10.0,1.0))?, BooleanOperation::Difference),
        "sphere_two_planes_x" => (ball(p(0.0,0.0,0.0),5.0)?, block(p(-1.0,-10.0,-10.0),p(1.0,10.0,10.0))?, BooleanOperation::Difference),
        "cyl_two_bands" => (cylinder_z(0.0,0.0,0.0,10.0,5.0)?, cut(cylinder_z(0.0,0.0,4.0,6.0,8.0)?, &cylinder_z(0.0,0.0,3.0,7.0,4.0)?)?, BooleanOperation::Difference),
        "tube_on_face" => (block(p(-10.0,-10.0,-5.0),p(10.0,10.0,0.0))?, extrude(ngk::modeling::faces::annulus(plane(p(0.0,0.0,0.0), v(0.0,0.0,1.0)),5.0,2.0)?, v(0.0,0.0,5.0))?, BooleanOperation::Union),
        "square_tube_on_face" => (block(p(-10.0,-10.0,-5.0),p(10.0,10.0,0.0))?, cut(block(p(-5.0,-5.0,0.0),p(5.0,5.0,5.0))?, &block(p(-2.0,-2.0,-1.0),p(2.0,2.0,6.0))?)?, BooleanOperation::Union),
        "boss_on_face" => (block(p(-10.0,-10.0,-5.0),p(10.0,10.0,0.0))?, cylinder_z(0.0,0.0,0.0,5.0,5.0)?, BooleanOperation::Union),
        "block_on_drilled_face" => (cut(block(p(-10.0,-10.0,-5.0),p(10.0,10.0,0.0))?, &cylinder_z(0.0,0.0,-6.0,1.0,2.0)?)?, block(p(-5.0,-5.0,0.0),p(5.0,5.0,5.0))?, BooleanOperation::Union),
        "half_ball" => (ball(p(0.0,0.0,0.0),20.0)?, block(p(-30.0,-30.0,-30.0),p(30.0,30.0,0.0))?, BooleanOperation::Difference),
        "half_ball_plate" => (cylinder_z(0.0,0.0,-2.0,0.0,20.0)?, cut(ball(p(0.0,0.0,0.0),20.0)?, &block(p(-30.0,-30.0,-30.0),p(30.0,30.0,0.0))?)?, BooleanOperation::Union),
        "half_ball_post" => (cut(ball(p(0.0,0.0,0.0),18.0)?, &block(p(-30.0,-30.0,-30.0),p(30.0,30.0,0.0))?)?, cylinder_z(0.0,0.0,0.0,18.0,3.0)?, BooleanOperation::Intersection),
        "plane_angle_1e-3" => (block(p(0.0,0.0,0.0),p(10.0,10.0,10.0))?, block(p(-5.0,-5.0,5.0),p(15.0,15.0,15.0))?.rotated(ngk::geometry::Axis3::x(), deg(0.001)), BooleanOperation::Difference),
        "plane_angle_1e-1" => (block(p(0.0,0.0,0.0),p(10.0,10.0,10.0))?, block(p(-5.0,-5.0,5.0),p(15.0,15.0,15.0))?.rotated(ngk::geometry::Axis3::x(), deg(0.1)), BooleanOperation::Difference),
        "big_small" => (block(p(0.0,0.0,0.0),p(1000.0,1000.0,1000.0))?, block(p(1.0,1.0,999.999),p(1.001,1.001,1001.0))?, BooleanOperation::Difference),
        "slot_1e-4" => (block(p(0.0,0.0,0.0),p(10.0,10.0,10.0))?, block(p(5.0,-1.0,5.0),p(5.0001,11.0,11.0))?, BooleanOperation::Difference),
        "slot_1e-5" => (block(p(0.0,0.0,0.0),p(10.0,10.0,10.0))?, block(p(5.0,-1.0,5.0),p(5.00001,11.0,11.0))?, BooleanOperation::Difference),
        "slot_w" => { let w: f64 = std::env::var("W").unwrap().parse().unwrap(); (block(p(0.0,0.0,0.0),p(10.0,10.0,10.0))?, block(p(5.0,-1.0,5.0),p(5.0+w,11.0,11.0))?, BooleanOperation::Difference) },
        "slot_w_inside" => { let w: f64 = std::env::var("W").unwrap().parse().unwrap(); (block(p(0.0,0.0,0.0),p(10.0,10.0,10.0))?, block(p(5.0,2.0,5.0),p(5.0+w,8.0,11.0))?, BooleanOperation::Difference) },
        "half_ball_far" => (cut(ball(p(0.0,0.0,0.0),20.0)?, &block(p(-30.0,-30.0,-30.0),p(30.0,30.0,0.0))?)?, block(p(15.0,-2.0,-5.0),p(25.0,2.0,5.0))?, BooleanOperation::Union),
        "half_ball_x" => (cut(solids::sphere_at(frame(p(0.0,0.0,0.0), v(1.0,0.0,0.0)),20.0)?, &block(p(-30.0,-30.0,-30.0),p(30.0,30.0,0.0))?)?, cylinder_z(0.0,0.0,-2.0,0.0,20.0)?, BooleanOperation::Union),
        "ring_twice" => (cut(cylinder_z(0.0,0.0,0.0,10.0,5.0)?, &cylinder_z(0.0,0.0,-1.0,11.0,3.0)?)?, cylinder_z(6.0,0.0,-1.0,11.0,1.5)?, BooleanOperation::Difference),
        _ => return refuse("which?"),
    };
    let (res, trace) = solids::traced(a, [&b], op);
    for s in trace.stages() { eprintln!("  stage: {} faces={:?} curves={}", s.name, s.faces.iter().map(|(n,f)| (n.clone(), f.len())).collect::<Vec<_>>(), s.curves.iter().map(|(_,c)| c.len()).sum::<usize>()); }
    if let Err(ngk::builders::boolean::BooleanError::Imprint { source, .. }) = &res { if let ngk::builders::boolean::BooleanError::IncompleteIntersections { diagnostics } = source.as_ref() { eprintln!("DIAG {:?} overlaps={:?} uncert={}", diagnostics.coverage, diagnostics.unresolved_overlaps, diagnostics.branches_uncertified); } }
    match res { Ok(s) => { eprintln!("OK vol {:?} area {:?}", s.solid().volume(), s.solid().area()); Ok(s) }, Err(e) => Err(Refusal::from(e)) }
}
