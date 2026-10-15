//! Scratch probe: how far the mass properties sit from the closed forms, and
//! what they cost.

use std::f64::consts::PI;
use std::time::Instant;

use ngk::geometry::Axis3;
use ngk::modeling::solids::{block, cylinder, sphere, torus};

fn report(name: &str, solid: ngk::topology::solid::Solid<'_>, volume: f64, area: f64, izz: f64) {
    let start = Instant::now();
    let v = solid.volume_properties().expect("volume");
    let tv = start.elapsed();
    let start = Instant::now();
    let a = solid.surface_properties().expect("area");
    let ta = start.elapsed();
    let rel = |got: f64, want: f64| (got - want) / want;
    println!(
        "{name:<10} V {:>14.9} ({:+.2e})  A {:>14.9} ({:+.2e})  Izz {:>14.9} ({:+.2e})  tV {:>8.2?} tA {:>8.2?}",
        v.volume,
        rel(v.volume, volume),
        a.area,
        rel(a.area, area),
        v.inertia.moment_about(Axis3::z()),
        rel(v.inertia.moment_about(Axis3::z()), izz),
        tv,
        ta
    );
}

fn main() {
    let b = block(2.0, 3.0, 4.0).unwrap();
    report("block", b.solid(), 24.0, 52.0, 26.0);
    let c = cylinder(1.0, 2.0).unwrap();
    report("cylinder", c.solid(), 2.0 * PI, 6.0 * PI, PI);
    let s = sphere(1.0).unwrap();
    report(
        "sphere",
        s.solid(),
        4.0 * PI / 3.0,
        4.0 * PI,
        8.0 * PI / 15.0,
    );
    let t = torus(3.0, 1.0).unwrap();
    let vt = 2.0 * PI * PI * 3.0;
    report(
        "torus",
        t.solid(),
        vt,
        4.0 * PI * PI * 3.0,
        vt * (9.0 + 0.75),
    );
    parts();
}

/// Times volume and area on imported parts, whose faces are trimmed by fitted
/// splines.
fn parts() {
    use ngk::exchange::step::{StepReadOptions, read_step};
    for name in [
        "buffer_stand",
        "curved_support",
        "lofted",
        "ppp0101_bearing_bracket",
        "ppp0102_post_cap",
        "ppp0105_paste_sleeve",
        "ppp0110_light_cap",
        "swept_circle",
    ] {
        let path = format!("tests/exchange/foreign/files/{name}.step");
        let text = std::fs::read_to_string(&path).expect("fixture");
        let import = read_step(&text, &StepReadOptions::default()).expect("imports");
        let solid = import.shapes[0].solid();
        let start = Instant::now();
        let volume = solid.volume().expect("volume");
        let tv = start.elapsed();
        let start = Instant::now();
        let area = solid.area().expect("area");
        let ta = start.elapsed();
        println!(
            "{name:<24} faces {:>4}  V {volume:>16.6}  A {area:>14.6}  tV {tv:>9.2?} tA {ta:>9.2?}",
            solid.faces().len()
        );
    }
}
