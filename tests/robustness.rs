//! Reproductions of the failures the brep-bench run of 2026-11-04 exposed, as plain tests, one
//! module per area of work. Each asserts the right answer: a closed-form
//! volume, a valid solid, a tessellation that closes, or a refusal that is an error and never a
//! panic. A reproduction that fails today is `#[ignore]`d with the class it belongs to, and loses
//! the attribute in the change that fixes the class.
//!
//!     cargo test --test robustness                    # what passes today
//!     cargo test --test robustness -- --ignored       # what is still open
//!     cargo test --test robustness -- --include-ignored sweep::
//!
//! `brep-bench/` holds the reduced reproductions this was written from and the whole bench.

#[path = "support/robust.rs"]
mod robust;

#[path = "robustness/contacts.rs"]
mod contacts;
#[path = "robustness/edit_safety.rs"]
mod edit_safety;
#[path = "robustness/intersection.rs"]
mod intersection;
#[path = "robustness/splitter.rs"]
mod splitter;
#[path = "robustness/sweep.rs"]
mod sweep;
#[path = "robustness/tessellation.rs"]
mod tessellation;
