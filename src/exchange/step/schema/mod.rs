//! **L2 — the AP entity model.** Typed records, units and product structure.
//!
//! Knows entity names, attribute order, and where AP203, AP214 and AP242
//! differ; knows nothing of NGK types. Keeping the boundary here is what makes
//! application-protocol differences somebody else's problem: the layer above
//! never learns which AP a file came from.

pub mod product;
