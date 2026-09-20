//! Owned-`Shape` facade over the low-level [`crate::builders`] API.
//!
//! Each constructor allocates one fresh model, runs the corresponding builder,
//! and returns the builder's primary handle in an owned [`Shape`]. The facade
//! adds no validation that belongs to a builder and preserves errors through
//! their composing types. Fallible operations are free functions; only
//! infallible operations are inherent methods on `Shape`.
//!
//! Payload-generic constructors come in pairs. The plain function chooses
//! [`crate::StandardPayload`], while its `_with` companion lets the caller
//! choose `P`, because a constructor has no shape argument from which Rust
//! could infer the payload. Operations that receive a `Shape` infer `P` and
//! therefore remain single generic functions.

pub mod edges;
pub mod faces;
pub mod loft;
pub mod profiles;
pub mod revolve;
pub mod solids;
pub mod sweep;
pub mod transform;
