//! Record-level adapter for the existing FEC router and optional return link.
use crate::feedback::{FEEDBACK_BUFFER_BYTES, FeedbackReporter};
use crate::protocol::{
    CuTelemetryEvent, CuTelemetryPoll, CuTelemetryRecord, CuTelemetryRx, CuTelemetrySession,
};
use crate::{
    CuFeedbackTx, CuStreamRx, CuStreamTxError, GapReason, RecordKind, SessionEvent, SessionRouter,
    SessionRouterLimits, StreamIdentity,
};
use cu29_clock::RobotClock;
use cu29_runtime::continuity::SourceGapReason;
use cu29_traits::{CuError, CuResult};

/// Resolves packet loss and verifies recovery references before emitting native records.
pub struct FecTelemetryRx<R> {
    rx: R,
    router: SessionRouter<1128, 64, 64>,
    feedback: Option<Box<dyn CuFeedbackTx>>,
    reporter: Option<FeedbackReporter>,
    clock: RobotClock,
    identity: Option<StreamIdentity>,
    latest: Option<u64>,
    pub(crate) sent: u64,
    pub(crate) dropped: u64,
    pub(crate) failed: bool,
}
impl<R> FecTelemetryRx<R> {
    pub fn new(
        rx: R,
        limits: SessionRouterLimits,
        feedback: Option<Box<dyn CuFeedbackTx>>,
    ) -> CuResult<Self> {
        if limits.max_sessions != 1 {
            return Err("a telemetry receiver serves one sender session".into());
        }
        Ok(Self {
            rx,
            router: SessionRouter::new(limits).map_err(|e| CuError::from(e.to_string()))?,
            feedback,
            reporter: None,
            clock: RobotClock::new(),
            identity: None,
            latest: None,
            sent: 0,
            dropped: 0,
            failed: false,
        })
    }
}
impl<R: CuStreamRx> CuTelemetryRx for FecTelemetryRx<R> {
    fn try_recv(
        &mut self,
        mut deliver: impl FnMut(CuTelemetryEvent<'_>) -> CuResult<()>,
    ) -> CuResult<CuTelemetryPoll> {
        use CuTelemetryRecord as Record;
        let mut packet = [0; 1200];
        let len = self
            .rx
            .try_recv(&mut packet)
            .map_err(|e| CuError::from(format!("FEC receive: {e:?}")))?;
        if let Some(len) = len {
            let bytes = packet
                .get(..len)
                .ok_or_else(|| CuError::from("invalid packet length"))?;
            self.router
                .receive_datagram(bytes, |event| {
                    let record = match event {
                        SessionEvent::Manifest(received) => {
                            let m = received.manifest();
                            self.identity = Some(m.identity);
                            if self.feedback.is_some() {
                                self.reporter = FeedbackReporter::new(
                                    m,
                                    crate::new_session_id(),
                                    self.clock.now(),
                                );
                            }
                            return deliver(CuTelemetryEvent::Record(Record::Session(
                                &CuTelemetrySession {
                                    identity: m.identity,
                                    context: m.context,
                                    schema: m.application_schema.clone(),
                                },
                            )));
                        }
                        SessionEvent::ContinuousRecord { record, .. } => {
                            let data = record.decoded();
                            self.latest = Some(data.object_id);
                            Record::CopperList {
                                id: data.object_id,
                                capture: data.payload,
                            }
                        }
                        SessionEvent::VerifiedRecoveryPoint {
                            keyframe,
                            recovery_point,
                            ..
                        } => Record::KeyFrame {
                            copperlist_id: recovery_point.copperlist_id,
                            state: keyframe.decoded().payload,
                        },
                        SessionEvent::Object { record, .. }
                            if record.decoded().kind == RecordKind::StructuredLog =>
                        {
                            Record::StructuredLog {
                                sequence: record.decoded().object_id,
                                entry: record.decoded().payload,
                            }
                        }
                        SessionEvent::Object { .. } => return Ok(()),
                        SessionEvent::Gap { gap, .. } => {
                            return deliver(CuTelemetryEvent::Gap {
                                first_id: gap.first_id,
                                last_id: gap.last_id,
                                reason: match gap.reason {
                                    GapReason::LateJoin => SourceGapReason::LateJoin,
                                    GapReason::SessionEnded => SourceGapReason::SessionEnded,
                                    GapReason::RlcWindowExpired => {
                                        SourceGapReason::RlcWindowExpired
                                    }
                                    GapReason::RecoveryPoint => SourceGapReason::RecoveryPoint,
                                },
                            });
                        }
                    };
                    deliver(CuTelemetryEvent::Record(record))
                })
                .map_err(|e| CuError::from(format!("FEC receive: {e}")))?;
        }
        if let (Some(reporter), Some(tx), Some(identity)) =
            (&mut self.reporter, &mut self.feedback, self.identity)
            && let Some(mut counters) = self.router.feedback_counters(identity)
        {
            counters.latest_copperlist = self.latest;
            if let Some(report) = reporter.report(self.clock.now(), counters) {
                let mut packet = [0; FEEDBACK_BUFFER_BYTES];
                let len = report
                    .encode_into(&mut packet)
                    .map_err(|e| CuError::from(e.to_string()))?;
                match tx.try_send_feedback(&packet[..len]) {
                    Ok(()) => self.sent += 1,
                    Err(CuStreamTxError::WouldBlock) => self.dropped += 1,
                    Err(CuStreamTxError::Failed(_)) => {
                        self.feedback = None;
                        self.failed = true;
                    }
                }
            }
        }
        Ok(if len.is_some() {
            CuTelemetryPoll::Progress
        } else {
            CuTelemetryPoll::Pending
        })
    }
}
