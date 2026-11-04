//! Finds the case implementations and the NGK version for the runner.
//!
//! Every `src/cases/<category>/<name_snake>.rs` is one case, `<category>/<name>` with the name's
//! underscores as hyphens, and must define `pub fn build() -> Result<Solid, E>`,
//! where `E` is any NGK error or, for a case that refuses in its own words, the runner's
//! `Refusal`. This writes `$OUT_DIR/cases.rs`: a module per file (a raw identifier, so `box.rs` is
//! one) and `CASES`, the table from case id to its `build`, its error made a `Refusal`. Adding a
//! case is adding its file.
//!
//! It also sets `KERNEL_VERSION` to the version of `kentron` in `Cargo.lock`.
use std::fmt::Write as _;
use std::fs;
use std::path::Path;

fn main() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let cases = root.join("src/cases");
    println!("cargo:rerun-if-changed={}", cases.display());

    let mut categories: Vec<_> = fs::read_dir(&cases)
        .expect("src/cases reads")
        .map(|e| e.expect("a directory entry").path())
        .filter(|p| p.is_dir())
        .collect();
    categories.sort();

    let (mut modules, mut table) = (String::new(), String::new());
    for dir in &categories {
        println!("cargo:rerun-if-changed={}", dir.display());
        let category = dir.file_name().unwrap().to_str().expect("a UTF-8 category");
        let mut files: Vec<_> = fs::read_dir(dir)
            .expect("a category reads")
            .map(|e| e.expect("a directory entry").path())
            .filter(|p| p.extension().is_some_and(|x| x == "rs"))
            .collect();
        files.sort();
        writeln!(modules, "pub mod {category} {{").unwrap();
        for file in &files {
            let name = file.file_stem().unwrap().to_str().expect("a UTF-8 name");
            writeln!(modules, "    #[path = {:?}]\n    pub mod r#{name};", file.display().to_string())
                .unwrap();
            writeln!(
                table,
                "    ({:?}, || {category}::r#{name}::build().map_err(Refusal::from)),",
                format!("{category}/{}", name.replace('_', "-"))
            )
            .unwrap();
        }
        writeln!(modules, "}}").unwrap();
    }
    let out = Path::new(&std::env::var("OUT_DIR").unwrap()).join("cases.rs");
    let text = format!(
        "{modules}\n/// Every case id and the function that builds it.\npub const CASES: &[(&str, \
         Build)] = &[\n{table}];\n"
    );
    fs::write(out, text).expect("cases.rs writes");

    println!("cargo:rerun-if-changed=Cargo.lock");
    let lock = fs::read_to_string(root.join("Cargo.lock")).unwrap_or_default();
    let version = lock
        .split("[[package]]")
        .find(|p| p.contains("name = \"ngk\"\n"))
        .and_then(|p| p.lines().find_map(|l| l.strip_prefix("version = \"")))
        .map_or("unknown", |v| v.trim_end_matches('"'));
    println!("cargo:rustc-env=KERNEL_VERSION={version}");
}
