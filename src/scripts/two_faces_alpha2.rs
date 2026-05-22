//! Two coplanar rectangles sharing one edge.

use crate::builders::faces::add_polygon;
use crate::geometry::Point3;
use crate::topology::StandardPayload;
use crate::topology::gmap::{Dim, GMap};
use crate::topology::profile::Profile;
use crate::viz::{ScriptResult, VizHints};

pub fn run() -> Result<ScriptResult, String> {
    let mut g = GMap::<StandardPayload>::new();
    let p1 = Point3::new(0.0, 0.0, 0.0);
    let p2 = Point3::new(1.0, 0.0, 0.0);
    let p3 = Point3::new(1.0, 1.0, 0.0);
    let p4 = Point3::new(0.0, 1.0, 0.0);
    let p5 = Point3::new(2.0, 0.0, 0.0);
    let p6 = Point3::new(2.0, 1.0, 0.0);

    let da = add_polygon(&mut g, &[p1, p2, p3, p4]);
    let db = add_polygon(&mut g, &[p2, p5, p6, p3]);

    let edge_a = &Profile::new(&g, da).edges()[1];
    let edge_b = &Profile::new(&g, db).edges()[3];
    let d1 = edge_a.dart;
    let d2 = edge_a.end().dart;
    let d3 = edge_b.dart;
    let d4 = edge_b.end().dart;
    g.sew_unchecked(Dim::Two, d1, d3);
    g.sew_unchecked(Dim::Two, d2, d4);

    Ok(ScriptResult::from_gmap_with_hints(&g, &VizHints::new()))
}
