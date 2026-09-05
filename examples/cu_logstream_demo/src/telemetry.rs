//! Typed views of Copper-owned twin frames and status.
pub use cu29_logstream::{CuTwinRecordingState as RecordingState, CuTwinStatus as Status};
pub type Frame = cu29_logstream::twin::TwinFrame<crate::DataSet>;
