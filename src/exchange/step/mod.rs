//! STEP (ISO 10303) interchange.
//!
//! The feature is split into four layers with three seams, so that no single
//! module lexes, resolves references, interprets units, builds geometry and
//! sews topology at once:
//!
//! | Layer | Knows about | Does **not** know about |
//! |---|---|---|
//! | `part21` | ISO 10303-21 syntax only | any AP, any entity name, any NGK type |
//! | `schema` | entity names, reference resolution, units | NGK types |
//! | `convert` | geometry mapping and parameter maps | topology, the GMap, darts |
//! | `topology` | shells, faces, loops, stitching, seams | Part 21 text |
//!
//! Only `part21` exists so far; the layers above it arrive with the stages in
//! `plan/step_interop.md`.

pub mod part21;
