use crate::pacing::STRUCTURED_RECORD_BYTES;
use cu29_traits::{CuError, CuResult};
/// Preallocated native records shared by all destinations on one telemetry worker.
/// A full pool drops the incoming record of that kind. Counts and byte limits must be positive.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TelemetryBufferConfig {
    /// Maximum encoded payload per CopperList capture.
    pub max_capture_bytes: usize,
    /// Maximum encoded payload per keyframe. Structured logs use the smaller of this and 4096 bytes.
    pub max_keyframe_bytes: usize,
    /// Number of capture buffers.
    pub capture_count: u16,
    /// Number of keyframe buffers.
    pub keyframe_count: u16,
    /// Number of structured-log buffers.
    pub structured_log_count: u16,
}

impl TelemetryBufferConfig {
    /// Reserve four captures, two keyframes and four structured logs.
    pub const fn new(max_capture_bytes: usize, max_keyframe_bytes: usize) -> Self {
        Self {
            max_capture_bytes,
            max_keyframe_bytes,
            capture_count: 4,
            keyframe_count: 2,
            structured_log_count: 4,
        }
    }

    pub(crate) fn structured_bytes(self) -> usize {
        self.max_keyframe_bytes.min(STRUCTURED_RECORD_BYTES)
    }

    pub(crate) fn payload_bytes(self) -> CuResult<usize> {
        if self.max_capture_bytes == 0 || self.max_keyframe_bytes == 0 {
            return Err("telemetry record limits must be positive".into());
        }
        if self.capture_count == 0 || self.keyframe_count == 0 || self.structured_log_count == 0 {
            return Err("telemetry buffer counts must be positive".into());
        }
        let pools = [
            (self.max_capture_bytes, self.capture_count),
            (self.max_keyframe_bytes, self.keyframe_count),
            (self.structured_bytes(), self.structured_log_count),
        ];
        pools
            .into_iter()
            .try_fold(0usize, |total, (bytes, count)| {
                total.checked_add(bytes.checked_mul(usize::from(count))?)
            })
            .filter(|&bytes| bytes <= isize::MAX as usize)
            .ok_or_else(|| CuError::from("telemetry buffer size overflow"))
    }
}
