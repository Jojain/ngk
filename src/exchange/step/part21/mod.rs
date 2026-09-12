//! **L1 — ISO 10303-21 syntax.** Tokens, records, and the instance table.
//!
//! This layer knows Part 21 and nothing above it: no entity names, no
//! application protocol, no kernel type. It is the dependency boundary of the
//! STEP stack — small, frozen by the standard, and identical for AP203, AP214,
//! AP242 and IFC alike — which is what keeps the choice of parser reversible.
//!
//! Reading is built on `winnow`; writing is ours, since no crate supplies
//! Part 21 output.

pub mod parse;
pub mod value;
pub mod write;

pub use parse::{SyntaxError, decode_text, parse_exchange};
pub use value::{
    DanglingReference, DuplicateEntityId, EntityId, StepExchange, Instance, Record, Value,
};
pub use write::{WriteError, encode_text, exchange_to_string, format_real, write_exchange};
