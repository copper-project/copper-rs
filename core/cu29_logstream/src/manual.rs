//! Caller-serviced native telemetry for a serial runtime.
use crate::protocol::{CuTelemetryRecord, CuTelemetrySession, CuTelemetryTx, TxMode};
use crate::{TelemetryBufferConfig, capture::CaptureDataSet, structured::StructuredLogOutput};
use alloc::{boxed::Box, string::ToString, sync::Arc, vec};
use core::{
    fmt,
    sync::atomic::{AtomicUsize, Ordering},
};
use crossbeam_queue::ArrayQueue;
use cu29_clock::RobotClock;
use cu29_runtime::{curuntime::SyncCopperListsManager, manual_output::ManualRecord};
use cu29_traits::{CuError, CuResult};
use cu29_unifiedlog::SectionContext;

struct LogBuffer {
    data: Box<[u8]>,
    len: usize,
}
struct Logs {
    free: ArrayQueue<LogBuffer>,
    ready: ArrayQueue<LogBuffer>,
    drops: AtomicUsize,
}

#[doc(hidden)]
pub struct ManualLogs {
    logs: Arc<Logs>,
    current: Option<LogBuffer>,
    overflow: bool,
}
impl fmt::Debug for ManualLogs {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("ManualLogs")
    }
}
impl StructuredLogOutput for ManualLogs {
    fn begin(&mut self) {
        if self.current.is_none() {
            self.current = self.logs.free.pop();
        }
        if let Some(buffer) = &mut self.current {
            buffer.len = 0;
        }
        self.overflow = false;
    }
    fn write(&mut self, bytes: &[u8]) {
        if let Some(buffer) = &mut self.current {
            if !self.overflow && bytes.len() <= buffer.data.len() - buffer.len {
                buffer.data[buffer.len..buffer.len + bytes.len()].copy_from_slice(bytes);
                buffer.len += bytes.len();
            } else {
                self.overflow = true;
            }
        }
    }
    fn finish(&mut self, success: bool) {
        if let Some(buffer) = self.current.take() {
            if success && !self.overflow {
                let admitted = self.logs.ready.push(buffer);
                debug_assert!(admitted.is_ok());
                return;
            }
            let _ = self.logs.free.push(buffer);
        }
        if success {
            self.logs.drops.fetch_add(1, Ordering::Relaxed);
        }
    }
}

#[doc(hidden)]
pub struct ManualDestination {
    pub session_id: [u8; 16],
    pub buffers: TelemetryBufferConfig,
    pub make_tx: Box<dyn FnOnce(RobotClock) -> CuResult<Box<dyn CuTelemetryTx>> + Send>,
}
#[doc(hidden)]
pub struct ManualTelemetry {
    tx: Box<dyn CuTelemetryTx>,
    inbox: ManualInbox,
}
struct ManualInbox {
    session: CuTelemetrySession,
    opened: bool,
    bytes: Box<[u8]>,
    buffers: TelemetryBufferConfig,
    selective: bool,
    logs: Arc<Logs>,
    log_sequence: u64,
    prefer_log: bool,
}

impl ManualDestination {
    pub fn start<P: CaptureDataSet>(
        self,
        clock: RobotClock,
        context: SectionContext,
    ) -> CuResult<(ManualTelemetry, ManualLogs)> {
        self.buffers.payload_bytes()?;
        let count = usize::from(self.buffers.structured_log_count);
        let logs = Arc::new(Logs {
            free: ArrayQueue::new(count),
            ready: ArrayQueue::new(count),
            drops: AtomicUsize::new(0),
        });
        for _ in 0..count {
            let _ = logs.free.push(LogBuffer {
                data: vec![0; self.buffers.structured_bytes()].into_boxed_slice(),
                len: 0,
            });
        }
        let schema = P::stream_schema();
        let selective = !schema.reconstruction.is_empty();
        let session = CuTelemetrySession {
            identity: crate::StreamIdentity {
                session_id: self.session_id,
                sender_id: context.instance_id,
            },
            schema,
            context,
        };
        let bytes = vec![
            0;
            self.buffers
                .max_capture_bytes
                .max(self.buffers.max_keyframe_bytes)
        ]
        .into_boxed_slice();
        Ok((
            ManualTelemetry {
                tx: (self.make_tx)(clock)?,
                inbox: ManualInbox {
                    session,
                    opened: false,
                    bytes,
                    buffers: self.buffers,
                    selective,
                    logs: logs.clone(),
                    log_sequence: 0,
                    prefer_log: false,
                },
            },
            ManualLogs {
                logs,
                current: None,
                overflow: false,
            },
        ))
    }
}

impl ManualTelemetry {
    pub fn service<P: CaptureDataSet, const N: usize>(
        &mut self,
        lists: &mut SyncCopperListsManager<P, N>,
    ) -> CuResult<()> {
        // A single iteration can produce a session, a keyframe, a capture and a log.
        self.service_records(lists, TxMode::Running, 4).map(|_| ())
    }
    pub fn close<P: CaptureDataSet, const N: usize>(
        &mut self,
        lists: &mut SyncCopperListsManager<P, N>,
    ) -> CuResult<()> {
        // Bound shutdown even if an external logger keeps producing or the protocol never drains.
        let budget = 2
            + usize::from(self.inbox.buffers.capture_count)
            + usize::from(self.inbox.buffers.keyframe_count)
            + usize::from(self.inbox.buffers.structured_log_count);
        for _ in 0..budget {
            if !self.service_records(lists, TxMode::Draining, 1)? {
                return Ok(());
            }
        }
        Err("manual telemetry did not finish draining within its record budget".into())
    }
    fn service_records<P: CaptureDataSet, const N: usize>(
        &mut self,
        lists: &mut SyncCopperListsManager<P, N>,
        mode: TxMode,
        budget: usize,
    ) -> CuResult<bool> {
        let (sent, next) = crate::protocol::service_tx(&mut *self.tx, mode, budget, |tx| {
            self.inbox.send_next(tx, lists)
        })?;
        Ok(sent != 0 || next.is_some())
    }
    pub fn dropped_logs(&self) -> usize {
        self.inbox.logs.drops.load(Ordering::Relaxed)
    }
}

impl ManualInbox {
    fn send_log(&mut self, tx: &mut dyn CuTelemetryTx) -> CuResult<bool> {
        let Some(buffer) = self.logs.ready.pop() else {
            return Ok(false);
        };
        let result = tx.send(CuTelemetryRecord::StructuredLog {
            sequence: self.log_sequence,
            entry: &buffer.data[..buffer.len],
        });
        let _ = self.logs.free.push(buffer);
        result?;
        self.log_sequence = self
            .log_sequence
            .checked_add(1)
            .ok_or_else(|| CuError::from("structured log sequence exhausted"))?;
        Ok(true)
    }
    fn send_next<P: CaptureDataSet, const N: usize>(
        &mut self,
        tx: &mut dyn CuTelemetryTx,
        lists: &mut SyncCopperListsManager<P, N>,
    ) -> CuResult<bool> {
        let mut progress = true;
        if !self.opened {
            tx.send(CuTelemetryRecord::Session(&self.session))?;
            self.opened = true;
        } else if self.prefer_log && self.send_log(tx)? {
            self.prefer_log = false;
        } else {
            let mut sent = false;
            lists.service_manual_output(|record| {
                sent = true;
                let native = match record {
                    ManualRecord::Capture(list) => {
                        let out = &mut self.bytes[..self.buffers.max_capture_bytes];
                        let n = if self.selective {
                            crate::capture::encode_capture_into(list, out)
                                .map_err(|e| CuError::from(e.to_string()))?
                        } else {
                            bincode::encode_into_slice(list, out, bincode::config::standard())
                                .map_err(|e| CuError::from(e.to_string()))?
                        };
                        CuTelemetryRecord::CopperList {
                            id: list.id,
                            capture: &out[..n],
                        }
                    }
                    ManualRecord::KeyFrame(frame) => {
                        let out = &mut self.bytes[..self.buffers.max_keyframe_bytes];
                        let n = bincode::encode_into_slice(frame, out, bincode::config::standard())
                            .map_err(|e| CuError::from(e.to_string()))?;
                        CuTelemetryRecord::KeyFrame {
                            copperlist_id: frame.culistid,
                            state: &out[..n],
                        }
                    }
                };
                tx.send(native)
            })?;
            self.prefer_log = sent;
            if !sent {
                progress = self.send_log(tx)?;
            }
        }
        Ok(progress)
    }
}
