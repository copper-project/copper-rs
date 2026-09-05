//! Example-specific telemetry values shared by the receiver and native UI.
use cu29_logstream::{StreamIdentity, telemetry::TelemetryPublisher};
use std::time::Instant;

/// A small current snapshot; frame delivery may be paused independently.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Status {
    pub latest: Option<u64>,
    pub anchor: Option<u64>,
    pub gaps: usize,
    pub dropped: usize,
    pub packets: usize,
    pub archived: u64,
    pub identity: Option<StreamIdentity>,
    pub last_packet: Option<Instant>,
    pub state: RecordingState,
    pub twin: cu29_logstream::twin::TwinStatus,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum RecordingState {
    #[default]
    Waiting,
    Recording,
    Closed,
    Failed,
}

pub type Frame = cu29_logstream::twin::TwinFrame<crate::DataSet>;
impl cu29_logstream::twin::TwinReceiverStatus for Status {
    fn with_twin(mut self, status: cu29_logstream::twin::TwinStatus) -> Self {
        self.twin = status;
        self
    }
}

pub type Publisher = TelemetryPublisher<Frame, Status>;
