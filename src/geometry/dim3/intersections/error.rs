use thiserror::Error;

use crate::geometry::NurbsError;

#[derive(Debug, Error)]
pub enum IntersectionError {
    #[error("invalid intersection options")]
    InvalidOptions,
    #[error("NURBS conversion failed")]
    Nurbs(#[from] NurbsError),
}
