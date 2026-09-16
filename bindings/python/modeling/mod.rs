mod common;
mod edges;
mod faces;
mod profiles;
mod solids;

use pyo3::prelude::*;
use pyo3::types::PyModule;

/// The fully qualified name the compiled extension is imported under.
///
/// Matches `module-name` in `pyproject.toml`. A submodule created with
/// [`PyModule::new`] only carries its own leaf name, so this is what lets
/// each one register under its real dotted path in `sys.modules` and be
/// found by `from ngk._ngk.modeling.faces import ...`.
const EXTENSION_QUALNAME: &str = "ngk._ngk";

pub(super) fn register(parent: &Bound<'_, PyModule>) -> PyResult<()> {
    let py = parent.py();
    let modeling = new_submodule(py, "modeling", EXTENSION_QUALNAME)?;
    let modeling_qualname = format!("{EXTENSION_QUALNAME}.modeling");

    let edges_module = new_submodule(py, "edges", &modeling_qualname)?;
    edges::register(&edges_module)?;
    modeling.add_submodule(&edges_module)?;

    let profiles_module = new_submodule(py, "profiles", &modeling_qualname)?;
    profiles::register(&profiles_module)?;
    modeling.add_submodule(&profiles_module)?;

    let faces_module = new_submodule(py, "faces", &modeling_qualname)?;
    faces::register(&faces_module)?;
    modeling.add_submodule(&faces_module)?;

    let solids_module = new_submodule(py, "solids", &modeling_qualname)?;
    solids::register(&solids_module)?;
    modeling.add_submodule(&solids_module)?;

    parent.add_submodule(&modeling)?;
    Ok(())
}

/// Creates a submodule and registers it in `sys.modules` under its full
/// dotted path, so `import`/`from ... import ...` can find it directly
/// rather than only through attribute access on its parent.
fn new_submodule<'py>(
    py: Python<'py>,
    name: &str,
    parent_qualname: &str,
) -> PyResult<Bound<'py, PyModule>> {
    let module = PyModule::new_bound(py, name)?;
    let qualname = format!("{parent_qualname}.{name}");
    py.import_bound("sys")?
        .getattr("modules")?
        .set_item(&qualname, &module)?;
    Ok(module)
}
