use ngk::builders::add_polygon;
use ngk::geometry::utils::Point3;
use ngk::scripts;
use ngk::topology::StandardPayload;
use ngk::topology::gmap::GMap;
use ngk::topology::profile::{Loop, Profile};
use ngk::viz::ScriptResult;

fn script_handling(name: &str, result: &ScriptResult) {
    println!("available scripts:");
    for id in scripts::list() {
        println!("  - {id}");
    }

    let name = "two_faces_alpha2";
    match scripts::run(name) {
        Ok(result) => {
            let s = &result.scene;
            println!(
                "\n[{name}] scene: {} points, {} segments, {} arrows, {} α-links, {} labels",
                s.points.len(),
                s.segments.len(),
                s.arrows.len(),
                s.alpha_links.len(),
                s.labels.len(),
            );
            if let Some(g) = &result.gmap {
                println!(
                    "[{name}] gmap: dim={}, darts={}, vertex_point entries={}",
                    g.dimension,
                    g.dart_count,
                    g.vertex_points.len(),
                );
            }
        }
        Err(e) => eprintln!("script {name} failed: {e}"),
    }
}

fn main() {
    let mut map = GMap::<StandardPayload>::new();
    let d = add_polygon(
        &mut map,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
    );
    let l = Loop::new(Profile::new(&map, d)).expect("failed to create loop");
    for e in l.edges() {
        println!("edge dart: {}", e.dart.id());
    }
    for v in l.vertices() {
        println!("vertex dart: {}", v.dart.id());
        println!("vertex point: {:?}", v.point());
    }
    println!("loop start: {:?}", l.start().point());
    println!("loop end: {:?}", l.end().point());
}
