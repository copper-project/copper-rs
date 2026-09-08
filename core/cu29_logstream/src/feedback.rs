//! Optional advisory feedback. All accounting and adaptation run on stream workers.
//! Sender and receiver must be built with matching feedback layouts.

use crate::{Error, Result, StreamIdentity};
use bincode::{Decode, Encode};
use cu29_clock::{CuDuration, CuTime};
use cu29_runtime::config::LogStreamFeedbackConfig;

pub const FEEDBACK_BUFFER_BYTES: usize = 320;
const MAGIC: &[u8; 4] = b"CUFB";
const CRC: crc::Crc<u32> = crc::Crc::<u32>::new(&crc::CRC_32_ISCSI);
const LOSS_MARGIN_BP: u64 = 200;
const HEALTHY_REPORTS: u8 = 3;

/// Local sender policy for feedback cadence, timeout, and FEC adaptation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Encode, Decode)]
pub struct FeedbackPolicy {
    pub report_interval_ms: u32,
    pub timeout_ms: u32,
    pub adaptation: Option<AdaptationBounds>,
}

/// Return-report cadence and identity advertised to the receiver.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Encode, Decode)]
pub struct FeedbackRequirements {
    pub report_interval_ms: u32,
    pub destination: [u8; 16],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Encode, Decode)]
pub struct AdaptationBounds {
    pub min_repair_every_source_symbols: u16,
    pub max_repair_every_source_symbols: u16,
}

impl From<&LogStreamFeedbackConfig> for FeedbackPolicy {
    fn from(config: &LogStreamFeedbackConfig) -> Self {
        Self {
            report_interval_ms: config.report_interval_ms,
            timeout_ms: config.timeout_ms,
            adaptation: config.adaptation.map(|bounds| AdaptationBounds {
                min_repair_every_source_symbols: bounds.min_repair_every_source_symbols,
                max_repair_every_source_symbols: bounds.max_repair_every_source_symbols,
            }),
        }
    }
}

impl FeedbackPolicy {
    pub fn validate(self, baseline: u16) -> Result<()> {
        if self.report_interval_ms == 0
            || self.timeout_ms <= self.report_interval_ms
            || self.adaptation.is_some_and(|b| {
                b.min_repair_every_source_symbols == 0
                    || b.min_repair_every_source_symbols > baseline
                    || baseline > b.max_repair_every_source_symbols
            })
        {
            return Err(Error::InvalidConfig(
                "invalid feedback cadence or FEC bounds",
            ));
        }
        Ok(())
    }
}

/// Cumulative outcomes for a contiguous finalized source-symbol range. Initial
/// unseen history and the still-live coding window are excluded. ESI wraps at u32.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Encode, Decode)]
pub struct SourceOutcomes {
    pub first_esi: u32,
    pub finalized: u64,
    pub received: u64,
    pub recovered: u64,
    pub missing: u64,
}

/// Cumulative measurements since this receiver joined the destination. Taking
/// deltas between accepted reports tolerates lost feedback datagrams.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Encode, Decode)]
pub struct ReceiverReport {
    pub session_id: [u8; 16],
    pub sender_id: u32,
    pub destination: [u8; 16],
    pub receiver_id: [u8; 16],
    pub sequence: u64,
    pub elapsed_us: u64,
    pub received_bytes: u64,
    pub received_packets: u64,
    pub sources: SourceOutcomes,
    pub invalid_packets: u64,
    pub duplicate_packets: u64,
    pub expired_records: u64,
    pub buffered_records: u32,
    pub record_capacity: u32,
    pub latest_copperlist: Option<u64>,
    pub request_recovery: bool,
}

/// Stable destination binding without transmitting or looking up strings on workers.
pub fn destination_key(id: &str) -> [u8; 16] {
    blake3::hash(id.as_bytes()).as_bytes()[..16]
        .try_into()
        .unwrap()
}

impl ReceiverReport {
    fn valid(&self) -> bool {
        self.elapsed_us > 0
            && self.record_capacity > 0
            && self.buffered_records <= self.record_capacity
            && self
                .sources
                .received
                .checked_add(self.sources.recovered)
                .and_then(|n| n.checked_add(self.sources.missing))
                == Some(self.sources.finalized)
    }

    pub fn encode_into(&self, output: &mut [u8]) -> Result<usize> {
        if output.len() < FEEDBACK_BUFFER_BYTES {
            return Err(Error::BufferTooSmall {
                needed: FEEDBACK_BUFFER_BYTES,
                available: output.len(),
            });
        }
        if !self.valid() {
            return Err(Error::InvalidConfig("invalid receiver report"));
        }
        output[..4].copy_from_slice(MAGIC);
        let len = bincode::encode_into_slice(
            self,
            &mut output[4..FEEDBACK_BUFFER_BYTES - 4],
            bincode::config::standard().with_fixed_int_encoding(),
        )
        .map_err(|_| Error::InvalidConfig("feedback report exceeds packet capacity"))?
            + 4;
        let checksum = CRC.checksum(&output[..len]);
        output[len..len + 4].copy_from_slice(&checksum.to_le_bytes());
        Ok(len + 4)
    }

    pub fn decode(packet: &[u8]) -> Result<Self> {
        if packet.len() < 8 || packet.len() > FEEDBACK_BUFFER_BYTES || &packet[..4] != MAGIC {
            return Err(Error::InvalidConfig("invalid feedback framing"));
        }
        let end = packet.len() - 4;
        if CRC.checksum(&packet[..end]) != u32::from_le_bytes(packet[end..].try_into().unwrap()) {
            return Err(Error::InvalidConfig("invalid feedback checksum"));
        }
        let (report, used): (Self, usize) = bincode::decode_from_slice(
            &packet[4..end],
            bincode::config::standard()
                .with_fixed_int_encoding()
                .with_limit::<FEEDBACK_BUFFER_BYTES>(),
        )
        .map_err(|_| Error::InvalidConfig("invalid feedback payload"))?;
        if used != end - 4 || !report.valid() {
            return Err(Error::InvalidConfig("invalid feedback counters"));
        }
        Ok(report)
    }
}

/// A bounded ring distinguishes original arrivals from symbols recovered by FEC.
/// Advancing past a symbol finalizes it once; late duplicates cannot heal history.
pub(crate) struct SourceObserver<const W: usize> {
    base: Option<u32>,
    slots: [u8; W],
    head: usize,
    outcomes: SourceOutcomes,
}
impl<const W: usize> Default for SourceObserver<W> {
    fn default() -> Self {
        Self {
            base: None,
            slots: [0; W],
            head: 0,
            outcomes: SourceOutcomes::default(),
        }
    }
}
impl<const W: usize> SourceObserver<W> {
    pub(crate) fn advance(&mut self, base: u32) {
        let Some(previous) = self.base else {
            self.base = Some(base);
            self.outcomes.first_esi = base;
            return;
        };
        let delta = base.wrapping_sub(previous);
        if delta == 0 || delta >= 1 << 31 {
            return;
        }
        // Long outages cost at most W slot visits, never work proportional to the outage.
        for _ in 0..(delta as usize).min(W) {
            match self.slots[self.head] {
                1 => self.outcomes.received += 1,
                2 => self.outcomes.recovered += 1,
                _ => self.outcomes.missing += 1,
            }
            self.slots[self.head] = 0;
            self.head = (self.head + 1) % W;
        }
        if delta as usize > W {
            self.outcomes.missing += u64::from(delta) - W as u64;
            self.head = (self.head + (delta as usize - W) % W) % W;
        }
        self.outcomes.finalized += u64::from(delta);
        self.base = Some(base);
    }
    pub(crate) fn observe(&mut self, esi: u32, original: bool) {
        let Some(base) = self.base else {
            return;
        };
        let offset = esi.wrapping_sub(base) as usize;
        if offset >= W {
            return;
        }
        let slot = &mut self.slots[(self.head + offset) % W];
        if original {
            *slot = 1;
        } else if *slot == 0 {
            *slot = 2;
        }
    }
    pub(crate) fn outcomes(&self) -> SourceOutcomes {
        self.outcomes
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum FeedbackState {
    #[default]
    Waiting,
    Active,
    Stale,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FeedbackSnapshot {
    pub state: FeedbackState,
    pub accepted_reports: u64,
    pub rejected_reports: u64,
    pub invalid_reports: u64,
    pub last_received: Option<CuTime>,
    pub report: Option<ReceiverReport>,
    pub effective_repair_every_source_symbols: u16,
    pub baseline_repair_every_source_symbols: u16,
    pub receiver_rates_available: bool,
    pub source_metrics_available: bool,
    pub receiver_bytes_per_second: u64,
    pub receiver_packets_per_second: u64,
    pub source_loss_basis_points: u16,
    pub source_recovery_basis_points: u16,
}

/// Pure deterministic controller. It cannot alter the send budget or retained storage.
pub struct FeedbackController {
    policy: FeedbackPolicy,
    identity: StreamIdentity,
    destination: [u8; 16],
    snapshot: FeedbackSnapshot,
    last_step: CuTime,
    smoothed_loss: Option<u64>,
    healthy: u8,
}
impl FeedbackController {
    pub fn new(
        policy: FeedbackPolicy,
        identity: StreamIdentity,
        destination: [u8; 16],
        baseline: u16,
    ) -> Result<Self> {
        policy.validate(baseline)?;
        Ok(Self {
            policy,
            identity,
            destination,
            snapshot: FeedbackSnapshot {
                effective_repair_every_source_symbols: baseline,
                baseline_repair_every_source_symbols: baseline,
                ..FeedbackSnapshot::default()
            },
            last_step: CuTime::default(),
            smoothed_loss: None,
            healthy: 0,
        })
    }
    pub fn snapshot(&self) -> FeedbackSnapshot {
        self.snapshot
    }
    pub fn invalid_report(&mut self) {
        self.snapshot.invalid_reports += 1;
    }
    pub fn tick(&mut self, now: CuTime) {
        if self
            .snapshot
            .last_received
            .is_some_and(|last| elapsed(now, last) >= u64::from(self.policy.timeout_ms) * 1_000_000)
        {
            self.snapshot.state = FeedbackState::Stale;
            self.healthy = 0;
            self.smoothed_loss = None;
            if elapsed(now, self.last_step) >= u64::from(self.policy.report_interval_ms) * 1_000_000
            {
                let baseline = self.snapshot.baseline_repair_every_source_symbols;
                let current = &mut self.snapshot.effective_repair_every_source_symbols;
                if *current < baseline {
                    *current += 1;
                } else if *current > baseline {
                    *current -= 1;
                }
                self.last_step = now;
            }
        }
    }
    /// Returns whether an accepted report requests the existing retained recovery bundle.
    pub fn receive(&mut self, report: ReceiverReport, now: CuTime) -> bool {
        self.tick(now);
        let previous = self.snapshot.report;
        let same_peer = previous.is_some_and(|p| p.receiver_id == report.receiver_id);
        let invalid = !report.valid()
            || report.session_id != self.identity.session_id
            || report.sender_id != self.identity.sender_id
            || report.destination != self.destination
            || previous.is_some_and(|p| {
                if same_peer {
                    report.sequence <= p.sequence
                        || report.elapsed_us <= p.elapsed_us
                        || (p.sources.finalized > 0
                            && report.sources.first_esi != p.sources.first_esi)
                        || report.sources.finalized < p.sources.finalized
                        || report.sources.received < p.sources.received
                        || report.sources.recovered < p.sources.recovered
                        || report.sources.missing < p.sources.missing
                        || report.received_bytes < p.received_bytes
                        || report.received_packets < p.received_packets
                } else {
                    self.snapshot.state != FeedbackState::Stale
                }
            })
            || self.snapshot.last_received.is_some_and(|last| {
                self.snapshot.state != FeedbackState::Stale
                    && elapsed(now, last) < u64::from(self.policy.report_interval_ms) * 500_000
            });
        if invalid {
            self.snapshot.rejected_reports += 1;
            return false;
        }
        // First report establishes a measurement baseline, including after receiver restart.
        if let Some(previous) = previous.filter(|_| same_peer) {
            let micros = report.elapsed_us - previous.elapsed_us;
            self.snapshot.receiver_rates_available = true;
            self.snapshot.receiver_bytes_per_second =
                rate(report.received_bytes - previous.received_bytes, micros);
            self.snapshot.receiver_packets_per_second =
                rate(report.received_packets - previous.received_packets, micros);
            let n = report.sources.finalized - previous.sources.finalized;
            if n > 0 {
                self.snapshot.source_metrics_available = true;
                let lost = n.saturating_sub(report.sources.received - previous.sources.received);
                let recovered = report.sources.recovered - previous.sources.recovered;
                let loss = ((u128::from(lost) * 10_000) / u128::from(n)) as u64;
                self.snapshot.source_loss_basis_points = loss as u16;
                self.snapshot.source_recovery_basis_points = if lost == 0 {
                    10_000
                } else {
                    ((u128::from(recovered) * 10_000) / u128::from(lost)).min(10_000) as u16
                };
                self.adapt(loss, report.sources.missing > previous.sources.missing);
            }
        } else {
            self.healthy = 0;
            self.smoothed_loss = None;
            self.snapshot.receiver_rates_available = false;
            self.snapshot.source_metrics_available = false;
            self.snapshot.receiver_bytes_per_second = 0;
            self.snapshot.receiver_packets_per_second = 0;
            self.snapshot.source_loss_basis_points = 0;
            self.snapshot.source_recovery_basis_points = 0;
        }
        self.snapshot.report = Some(report);
        self.snapshot.last_received = Some(now);
        self.snapshot.state = FeedbackState::Active;
        self.snapshot.accepted_reports += 1;
        self.last_step = now;
        report.request_recovery
    }
    fn adapt(&mut self, loss: u64, unrecovered: bool) {
        let Some(bounds) = self.policy.adaptation else {
            return;
        };
        let smoothed = self.smoothed_loss.map_or(loss, |old| (old * 3 + loss) / 4);
        self.smoothed_loss = Some(smoothed);
        let target = ((10_000 - smoothed) / (smoothed + LOSS_MARGIN_BP)).clamp(
            u64::from(bounds.min_repair_every_source_symbols),
            u64::from(bounds.max_repair_every_source_symbols),
        ) as u16;
        let current = &mut self.snapshot.effective_repair_every_source_symbols;
        if unrecovered {
            *current = (*current / 2)
                .min(target)
                .max(bounds.min_repair_every_source_symbols);
            self.healthy = 0;
        } else if target < *current {
            *current = target;
            self.healthy = 0;
        } else if target > *current {
            self.healthy += 1;
            if self.healthy >= HEALTHY_REPORTS {
                *current += 1;
                self.healthy = 0;
            }
        } else {
            self.healthy = 0;
        }
    }
}
fn elapsed(now: CuTime, then: CuTime) -> u64 {
    now.as_nanos().saturating_sub(then.as_nanos())
}
fn rate(count: u64, micros: u64) -> u64 {
    (u128::from(count) * 1_000_000 / u128::from(micros.max(1))).min(u128::from(u64::MAX)) as u64
}

/// Receiver-side cadence and identity. The caller gathers counters from its router
/// and attempts one datagram send; backpressure never queues feedback history.
pub struct FeedbackReporter {
    pub requirements: FeedbackRequirements,
    identity: StreamIdentity,
    destination: [u8; 16],
    receiver_id: [u8; 16],
    started: CuTime,
    next_report: CuTime,
    sequence: u64,
}
impl FeedbackReporter {
    pub fn new(
        manifest: &crate::SessionManifest,
        receiver_id: [u8; 16],
        now: CuTime,
    ) -> Option<Self> {
        let requirements = manifest.requirements.feedback?;
        Some(Self {
            requirements,
            identity: manifest.identity,
            destination: requirements.destination,
            receiver_id,
            started: now,
            next_report: now + CuDuration::from_millis(u64::from(requirements.report_interval_ms)),
            sequence: 0,
        })
    }
    pub fn report(&mut self, now: CuTime, mut counters: ReceiverReport) -> Option<ReceiverReport> {
        if now < self.next_report {
            return None;
        }
        self.next_report =
            now + CuDuration::from_millis(u64::from(self.requirements.report_interval_ms));
        self.sequence = self.sequence.saturating_add(1);
        counters.session_id = self.identity.session_id;
        counters.sender_id = self.identity.sender_id;
        counters.destination = self.destination;
        counters.receiver_id = self.receiver_id;
        counters.sequence = self.sequence;
        counters.elapsed_us = (elapsed(now, self.started) / 1000).max(1);
        Some(counters)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn observer_handles_wrap_repair_then_original_and_long_outage() {
        let mut observer = SourceObserver::<4>::default();
        observer.advance(u32::MAX - 1);
        observer.observe(u32::MAX - 1, true);
        observer.observe(u32::MAX, false);
        observer.observe(u32::MAX, true);
        observer.observe(0, false);
        observer.advance(2);
        assert_eq!(
            observer.outcomes(),
            SourceOutcomes {
                first_esi: u32::MAX - 1,
                finalized: 4,
                received: 2,
                recovered: 1,
                missing: 1,
            }
        );
        observer.advance(1_000_002);
        assert_eq!(observer.outcomes().finalized, 1_000_004);
        assert_eq!(observer.outcomes().missing, 1_000_001);
        observer.observe(1_000_002, true);
        observer.advance(1_000_003);
        assert_eq!(observer.outcomes().received, 3);
    }
}
