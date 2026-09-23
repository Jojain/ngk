//! **L4 — the topology mapping.** Shells, faces, loops, edge sharing, seams.
//!
//! Knows `topology::`, `builders::` and `healing::`; knows nothing of Part 21
//! text. It emits entity records through the same builder every other layer
//! uses, and never formats a character itself.

mod bridge;
pub mod export;
pub mod import;
pub mod seam;
