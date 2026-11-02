//! solids builder API.

mod from_sheets;
mod operations;

pub use from_sheets::{SolidFromSheetsError, add_solid_from_sheets};
pub use operations::*;

pub(crate) use from_sheets::add_solid_from_sheets_edit;
