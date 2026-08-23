use std::fs;
use std::path::{Path, PathBuf};

const MIRRORED_FILES: &[&str] = &[
    "geometry/convert.rs",
    "geometry/curves.rs",
    "geometry/mod.rs",
    "geometry/nurbs.rs",
    "geometry/surfaces.rs",
    "geometry/values.rs",
    "modeling/mod.rs",
    "modeling/primitive.rs",
    "topology/common.rs",
    "topology/edge.rs",
    "topology/face.rs",
    "topology/gmap.rs",
    "topology/mod.rs",
    "topology/profile.rs",
    "topology/sheet.rs",
    "topology/solid.rs",
    "topology/vertex.rs",
    "visualization/mod.rs",
    "visualization/tcv.rs",
];

const SHARED_CLASSES: &[&str] = &[
    "Point3",
    "Vector3",
    "Line",
    "Circle",
    "NurbsCurve",
    "Plane",
    "Cylinder",
    "RuledSurface",
    "SurfaceOfRevolution",
    "NurbsSurface",
    "GMap",
    "Solid",
    "Shell",
    "Sheet",
    "Face",
    "Loop",
    "Profile",
    "Edge",
    "Vertex",
];

const SHARED_METHODS: &[(&str, &[&str])] = &[
    ("geometry/values.rs", &["x", "y", "z"]),
    (
        "geometry/curves.rs",
        &["start", "end", "plane", "radius", "point_at"],
    ),
    (
        "geometry/surfaces.rs",
        &[
            "origin",
            "x_dir",
            "y_dir",
            "normal",
            "axis",
            "radius",
            "curve",
            "direction",
            "point_at",
            "normal_at",
        ],
    ),
    (
        "geometry/nurbs.rs",
        &[
            "degree",
            "domain",
            "knots",
            "control_points",
            "degree_u",
            "degree_v",
            "domain_u",
            "domain_v",
            "knots_u",
            "knots_v",
            "point_at",
            "normal_at",
        ],
    ),
    (
        "topology/gmap.rs",
        &[
            "deserialize",
            "serialize",
            "dimension",
            "involution_count",
            "dart_count",
            "darts",
            "alpha",
            "is_free",
            "orbit",
            "cells",
            "cell_darts",
            "cell_representative",
            "incident_cells",
            "adjacent_cells",
            "vertices",
            "edges",
            "profiles",
            "faces",
            "sheets",
            "solids",
            "vertex",
            "edge",
            "profile",
            "face",
            "sheet",
            "solid",
        ],
    ),
    (
        "topology/solid.rs",
        &[
            "outer_shell",
            "inner_shells",
            "shells",
            "faces",
            "edges",
            "vertices",
            "face_count",
            "edge_count",
            "vertex_count",
        ],
    ),
    (
        "topology/sheet.rs",
        &[
            "is_closed",
            "darts",
            "faces",
            "edges",
            "vertices",
            "reversed",
        ],
    ),
    (
        "topology/face.rs",
        &[
            "surface",
            "outer_loop",
            "inner_loops",
            "loops",
            "edges",
            "vertices",
            "reversed",
        ],
    ),
    (
        "topology/profile.rs",
        &[
            "is_closed",
            "start",
            "end",
            "darts",
            "edges",
            "vertices",
            "reversed",
        ],
    ),
    (
        "topology/edge.rs",
        &[
            "start", "end", "length", "curve", "darts", "vertices", "faces", "sheets", "reversed",
        ],
    ),
    ("topology/vertex.rs", &["point", "edges", "faces", "sheets"]),
    (
        "modeling/primitive.rs",
        &["block", "line", "rectangle_profile", "rectangle_face"],
    ),
];

fn bindings_root(language: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("bindings")
        .join(language)
}

fn rust_sources(root: &Path) -> String {
    let mut sources = String::new();
    let mut pending = vec![root.to_path_buf()];
    while let Some(path) = pending.pop() {
        for entry in fs::read_dir(path).expect("binding directory must be readable") {
            let entry = entry.expect("binding directory entry must be readable");
            let path = entry.path();
            if path.is_dir() {
                pending.push(path);
            } else if path.extension().is_some_and(|extension| extension == "rs") {
                sources.push_str(&fs::read_to_string(path).expect("binding source must be UTF-8"));
            }
        }
    }
    sources
}

fn source(root: &Path, relative: &str) -> String {
    fs::read_to_string(root.join(relative)).expect("binding source must be readable UTF-8")
}

#[test]
fn python_and_wasm_binding_trees_are_mirrored() {
    let python = bindings_root("python");
    let wasm = bindings_root("wasm");

    for relative in MIRRORED_FILES {
        assert!(python.join(relative).is_file(), "missing Python {relative}");
        assert!(wasm.join(relative).is_file(), "missing WASM {relative}");
    }
}

#[test]
fn python_and_wasm_export_the_same_core_classes() {
    let python = rust_sources(&bindings_root("python"));
    let wasm = rust_sources(&bindings_root("wasm"));

    for class in SHARED_CLASSES {
        assert!(
            python.contains(&format!("#[pyclass(name = \"{class}\"")),
            "Python does not export {class}"
        );
        assert!(
            wasm.contains(&format!("#[wasm_bindgen(js_name = {class})]")),
            "WASM does not export {class}"
        );
    }
}

#[test]
fn python_and_wasm_expose_the_same_core_methods() {
    let python = bindings_root("python");
    let wasm = bindings_root("wasm");

    for (relative, methods) in SHARED_METHODS {
        let python_source = source(&python, relative);
        let wasm_source = source(&wasm, relative);
        for method in *methods {
            let declaration = format!("fn {method}(");
            assert!(
                python_source.contains(&declaration),
                "Python {relative} does not expose {method}"
            );
            assert!(
                wasm_source.contains(&declaration),
                "WASM {relative} does not expose {method}"
            );
        }
    }
}

#[test]
fn geometry_classes_do_not_expose_string_kind_tags() {
    for language in ["python", "wasm"] {
        let geometry = rust_sources(&bindings_root(language).join("geometry"));
        assert!(
            !geometry.contains("fn kind("),
            "{language} geometry still exposes kind"
        );
    }
}
