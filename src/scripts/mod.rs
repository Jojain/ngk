//! Exploration scripts: named Rust functions that use the kernel to build a
//! small scene and return it as a [`ScriptResult`]. Scripts are the canonical
//! way to prototype in ngk — the wasm surface only exposes this registry.

use crate::viz::ScriptResult;

pub mod cylinder;
pub mod extruded_open_polyline;
pub mod hollow_cylinder;
pub mod two_faces_alpha2;
pub mod extruded_square;

pub type ScriptFn = fn() -> Result<ScriptResult, String>;

pub struct Script {
    pub id: &'static str,
    pub title: &'static str,
    pub run: ScriptFn,
}

pub const SCRIPTS: &[Script] = &[
    Script {
        id: "two_faces_alpha2",
        title: "Two faces α2-sewn on a shared edge",
        run: two_faces_alpha2::run,
    },
    Script {
        id: "hollow_cylinder",
        title: "Hollow cylinder (closed shell)",
        run: hollow_cylinder::run,
    },
    Script {
        id: "cylinder",
        title: "Extruded circular arc",
        run: cylinder::run,
    },
    Script {
        id: "extruded_square",
        title: "Extruded square",
        run: extruded_square::run,
    },
    Script {
        id: "extruded_open_polyline",
        title: "Extruded open polyline",
        run: extruded_open_polyline::run,
    },
];

pub fn list() -> Vec<&'static str> {
    SCRIPTS.iter().map(|s| s.id).collect()
}

pub fn run(name: &str) -> Result<ScriptResult, String> {
    let script = SCRIPTS
        .iter()
        .find(|s| s.id == name)
        .ok_or_else(|| format!("unknown script: {name}"))?;
    (script.run)()
}
