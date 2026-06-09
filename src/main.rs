use ngk::scripts;
use ngk::viz::ScriptResult;

fn script_handling(_name: &str, _result: &ScriptResult) {
    println!("available scripts:");
    for id in scripts::list() {
        println!("  - {id}");
    }

    let name = "two_faces_alpha2";
    match scripts::run(name) {
        Ok(result) => {
            let s = &result.scene;
            println!(
                "\n[{name}] scene: {} vertices, {} edges, {} faces, {} darts, {} α-links, {} labels",
                s.vertices.len(),
                s.edges.len(),
                s.faces.len(),
                s.darts.len(),
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
    use ngk::modeling::solids::block;
    use ngk::viz::debug_viewer::{DebugViewerOptions, show_with_options};

    let b = block(10.0, 5.0, 3.0).unwrap();
    show_with_options(
        &b,
        &DebugViewerOptions {
            name: "block".to_owned(),
            ..DebugViewerOptions::default()
        },
    )
    .unwrap();
}
