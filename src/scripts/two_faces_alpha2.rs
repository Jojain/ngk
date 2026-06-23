//! Two coplanar rectangles sharing one edge.

use crate::builders::faces::add_polygon;
use crate::geometry::Point3;
use crate::topology::StandardPayload;
use crate::topology::gmap::{Dart, Dim, GMap};
use crate::viz::{ScriptResult, VizHints};

pub fn run() -> Result<ScriptResult, String> {
    let g = build_gmap()?;
    Ok(ScriptResult::from_gmap_with_hints(&g, &VizHints::new()))
}

fn build_gmap() -> Result<GMap<StandardPayload>, String> {
    let mut g = GMap::<StandardPayload>::new();
    let p1 = Point3::new(0.0, 0.0, 0.0);
    let p2 = Point3::new(1.0, 0.0, 0.0);
    let p3 = Point3::new(1.0, 1.0, 0.0);
    let p4 = Point3::new(0.0, 1.0, 0.0);
    let p5 = Point3::new(2.0, 0.0, 0.0);
    let p6 = Point3::new(2.0, 1.0, 0.0);

    add_polygon(&mut g, &[p1, p2, p3, p4]);
    add_polygon(&mut g, &[p2, p5, p6, p3]);

    // add_polygon allocates two darts per edge, in corner order. The shared
    // edge is first face edge 1 (p2 -> p3) against second face edge 3 (p3 -> p2).
    g.edit(|edit| {
        edit.sew(Dim::Two, Dart::new(2), Dart::new(15))?;
        Ok(())
    })
    .map_err(|err| format!("failed to alpha2-sew shared edge: {err}"))?;

    Ok(g)
}

#[cfg(test)]
mod tests {
    use super::build_gmap;
    use crate::topology::gmap::{Dart, Dim};

    #[test]
    fn shared_edge_is_alpha2_sewn_with_opposite_orientation() {
        let g = build_gmap().expect("two-face gmap should build");

        assert_eq!(g.alpha(Dim::Two, Dart::new(2)), Dart::new(15));
        assert_eq!(g.alpha(Dim::Two, Dart::new(3)), Dart::new(14));
    }
}
