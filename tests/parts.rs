//! Part tests: whole mechanical parts modeled end to end through the public
//! modeling API.
//!
//! Each file builds one recognisable part the way a user would — sketch,
//! extrude, revolve, sweep, fuse — and checks the finished solid: one closed,
//! outward, manifold shell with the volume the part should have. A failure
//! here names a capability the kernel lacks in combination, which the
//! single-operation suites under `builders/` and `modeling/` do not see.

#[path = "parts/bolt.rs"]
mod bolt;
