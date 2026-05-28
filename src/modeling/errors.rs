use thiserror::Error;

#[derive(Debug, Clone, Error, PartialEq)]
pub enum PrimitiveError {
    #[error("block {axis} size must be greater than 0, got {value}")]
    InvalidSize { axis: &'static str, value: f64 },
    #[error("failed to create the block base face")]
    FaceCreationFailed,
    #[error("failed to extrude the block base face")]
    SolidCreationFailed,
}
