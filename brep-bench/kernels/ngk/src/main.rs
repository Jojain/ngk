//! The brep-bench runner for NGK (https://github.com/Jojain/ngk, the Nales Geometry Kernel).
//!
//!     brep-bench-ngk CASE OUT.stl
//!     brep-bench-ngk --version
//!
//! Builds the case `CASE` (`boolean/napkin-ring`) with its implementation,
//! `src/cases/boolean/napkin_ring.rs`, as an owned `Shape<SolidTag>`, then measures the solid
//! with the kernel's own tools: its volume and area (`Solid::volume`, `Solid::area`, integrated
//! over the B-rep by Green's theorem, no mesh), its faces by surface kind and its validity check
//! (`validate_solid_manifold` and `validate_solid_orientation`). It tessellates the solid into
//! OUT.stl and prints one line of JSON. A kernel error is a refusal: the kernel declined, saying
//! why. A case may also refuse in its own words, where NGK has no operation for what it
//! describes. The runner never reads the case's description.
use ngk::geometry::Surface;
use ngk::exchange::stl::{StlWriteOptions, write_stl_file};
use ngk::tessellate::{CurveOpts, SurfaceOpts, TessellateOpts};
use ngk::topology::shape::{Shape, SolidTag};
use ngk::topology::validation::{validate_solid_manifold, validate_solid_orientation};
use std::collections::BTreeMap;
use std::process::ExitCode;

#[allow(dead_code)]
mod bench;

/// The solid a case builds.
pub type Solid = Shape<SolidTag>;

/// A case's implementation: the solid it builds, or why it was not built.
type Build = fn() -> Result<Solid, Refusal>;

/// Why a case was not built: NGK's error, in its own words, or a case that refuses in its own
/// because NGK has no operation for what it describes.
#[derive(Debug)]
pub struct Refusal(pub String);

impl<E: std::error::Error> From<E> for Refusal {
    /// The error and every error it wraps, outermost first: a Boolean's "imprinting failed" says
    /// why only in its source.
    fn from(e: E) -> Self {
        let mut why = e.to_string();
        let mut source = e.source();
        while let Some(inner) = source {
            why.push_str(": ");
            why.push_str(&inner.to_string());
            source = inner.source();
        }
        Refusal(why)
    }
}

impl std::fmt::Display for Refusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

/// A case refusing in its own words.
pub fn refuse<T>(why: &str) -> Result<T, Refusal> {
    Err(Refusal(why.to_owned()))
}

/// One module per case file, and `CASES`, written by `build.rs`.
mod cases {
    use super::{Build, Refusal};
    include!(concat!(env!("OUT_DIR"), "/cases.rs"));
}

/// A face's surface as the reply counts it.
fn kind(s: &Surface) -> &'static str {
    match s {
        Surface::Plane(_) => "plane",
        Surface::Cylinder(_) => "cylinder",
        Surface::Cone(_) => "cone",
        Surface::Sphere(_) => "sphere",
        Surface::Torus(_) => "torus",
        Surface::Nurbs(_) => "bspline",
        Surface::Ruled(_) | Surface::Revolution(_) => "other",
    }
}

/// A JSON string.
fn quoted(s: &str) -> String {
    let mut out = String::from("\"");
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

fn refused(why: &str) -> String {
    format!("{{\"status\":\"refused\",\"reason\":{}}}", quoted(why))
}

/// The reply for a solid built: its measures, once its triangles are written.
fn measured(shape: &Solid, out: &str) -> String {
    let solid = shape.solid();
    let mut faces = BTreeMap::new();
    for f in solid.faces() {
        *faces.entry(kind(f.surface())).or_insert(0) += 1;
    }
    let faces = faces.iter().map(|(k, n)| format!("\"{k}\":{n}")).collect::<Vec<_>>().join(",");
    let volume = match solid.volume() {
        Ok(v) => v,
        Err(e) => return refused(&format!("volume not measured: {e}")),
    };
    let area = match solid.area() {
        Ok(a) => a,
        Err(e) => return refused(&format!("area not measured: {e}")),
    };
    let key = shape.key();
    let valid = validate_solid_manifold(shape.model(), key).is_ok()
        && validate_solid_orientation(shape.model(), key).is_ok();
    // Finer than the viewer's sampling, so curved walls bound the solid's volume closely.
    let n: usize = std::env::var("NGK_SEGMENTS").ok().and_then(|s| s.parse().ok()).unwrap_or(128);
    let opts = TessellateOpts {
        curve: CurveOpts { segments: n },
        surface: SurfaceOpts { nu: n, nv: n / 2 },
    };
    let options = StlWriteOptions { tessellation: opts, ..Default::default() };
    match write_stl_file(out, &solid, &options) {
        Ok(()) => format!(
            "{{\"status\":\"ok\",\"volume\":{volume:e},\"area\":{area:e},\"faces\":{{{faces}}},\"valid\":{valid}}}"
        ),
        Err(e) => refused(&format!("STL export: {e}")),
    }
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    if args.get(1).map(String::as_str) == Some("--version") {
        println!(
            "NGK {} (brep-bench runner {})",
            env!("KERNEL_VERSION"),
            env!("CARGO_PKG_VERSION")
        );
        return ExitCode::SUCCESS;
    }
    let [_, case, out] = &args[..] else {
        eprintln!("usage: brep-bench-ngk CASE OUT.stl");
        return ExitCode::from(2);
    };
    let Some(&(_, build)) = cases::CASES.iter().find(|(id, _)| id == case) else {
        eprintln!("no implementation of `{case}`");
        return ExitCode::from(2);
    };
    let reply = match build() {
        Ok(solid) => measured(&solid, out),
        Err(why) => refused(&why.to_string()),
    };
    println!("{reply}");
    ExitCode::SUCCESS
}
