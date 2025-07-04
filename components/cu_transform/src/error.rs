use thiserror::Error;

#[derive(Error, Debug)]
pub enum TransformError {
    #[error("Transform from frame '{from}' to frame '{to}' not found")]
    TransformNotFound { from: String, to: String },

    #[error("Transform at requested time {0} not available")]
    TransformTimeNotAvailable(cu29::clock::CuTime),

    #[error("Frame '{0}' does not exist")]
    FrameNotFound(String),

    #[error("Cycle detected in transform tree")]
    CyclicTransformTree,

    #[error("Error during transform interpolation: {0}")]
    InterpolationError(String),

    #[error("Serialization error: {0}")]
    SerializationError(String),

    #[error("Unknown error: {0}")]
    Unknown(String),
}

pub type TransformResult<T> = Result<T, TransformError>;
