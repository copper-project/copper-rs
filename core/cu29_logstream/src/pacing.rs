//! Bounded autonomous sender scheduling. All methods run off the real-time path.

use crate::rlc::RepairSchedule;
use crate::{
    Anchor, ContinuousEncoder, CuStreamTx, CuStreamTxError, Error, FiniteObjectEncoder,
    LogStreamSenderConfig, RecordKind, Result, decode_record, encode_anchor, encode_record,
};
use alloc::{boxed::Box, vec, vec::Vec};
use cu29_clock::{CuDuration, CuTime};

/// Autonomous repetition period, measured by the sender's local RobotClock.
/// Repetition is coalesced while the previous transmission is still pending.
pub const RECOVERY_REPEAT_INTERVAL: CuDuration = CuDuration(250_000_000);
const MAX_SENDS_PER_POLL: usize = 64;
const PENDING_BOUNDARIES: usize = 4;
const PENDING_KEYFRAMES: usize = 2;

/// Shared destination budget. Counts complete Copper packets, excluding carrier overhead.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PacingConfig {
    pub bitrate_bps: u64,
    pub burst_packets: u32,
    pub max_latency: CuDuration,
    /// Storage budget for sender buffers and the continuous encoder. Finite-object
    /// codec scratch allocations and thread stacks are additional, bounded by object size.
    pub memory_budget_bytes: usize,
}

/// Counters distinguish queue shedding from carrier backpressure.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SenderStats {
    pub packets_sent: u64,
    pub bytes_sent: u64,
    pub queue_drops: u64,
    pub expired_packets: u64,
    pub transport_drops: u64,
    pub recovery_rounds: u64,
    pub recovery_superseded: u64,
    pub shutdown_drops: u64,
    pub queue_peak: usize,
}

/// Fixed-capacity packet storage; filling and replacing it never grows its allocation.
struct Packets {
    bytes: Vec<u8>,
    lengths: Vec<usize>,
    times: Vec<CuTime>,
    ids: Vec<u64>,
    mtu: usize,
    head: usize,
    len: usize,
}

impl Packets {
    fn new(capacity: usize, mtu: usize) -> Self {
        Self {
            bytes: vec![0; capacity * mtu],
            lengths: vec![0; capacity],
            times: vec![CuTime::default(); capacity],
            ids: vec![0; capacity],
            mtu,
            head: 0,
            len: 0,
        }
    }

    fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
    }

    fn push(&mut self, bytes: &[u8], now: CuTime, id: u64) -> bool {
        if self.len == self.lengths.len() || bytes.len() > self.mtu {
            return false;
        }
        let slot = (self.head + self.len) % self.lengths.len();
        self.bytes[slot * self.mtu..slot * self.mtu + bytes.len()].copy_from_slice(bytes);
        self.lengths[slot] = bytes.len();
        self.times[slot] = now;
        self.ids[slot] = id;
        self.len += 1;
        true
    }

    fn get(&self, index: usize) -> &[u8] {
        let slot = (self.head + index) % self.lengths.len();
        &self.bytes[slot * self.mtu..slot * self.mtu + self.lengths[slot]]
    }

    fn pop(&mut self) {
        self.head = (self.head + 1) % self.lengths.len();
        self.len -= 1;
    }
}

struct PendingPackets {
    id: Option<u64>,
    packets: Packets,
}

struct RecoveryPackets {
    control: Packets,
    boundary: Packets,
}

/// A destination's single packet scheduler and retained recovery state.
///
/// `now` always belongs to the same local RobotClock. The caller owns transport
/// and wakeups; this core neither reads a global clock nor sleeps. Runtime objects
/// are never retained here. Optional feedback can later request the same retained
/// jobs through `request_recovery`, without bypassing this scheduler's budget.
pub struct SenderCore {
    config: LogStreamSenderConfig,
    continuous: Box<ContinuousEncoder<1128, 64>>,
    finite: FiniteObjectEncoder,
    repairs: RepairSchedule,
    scratch: Vec<u8>,
    data: Packets,
    manifest: Packets,
    latest: RecoveryPackets,
    pending_cl: Vec<PendingPackets>,
    pending_kf: Vec<PendingPackets>,
    latest_id: Option<u64>,
    manifest_cursor: usize,
    recovery_cursor: usize,
    next_repeat: CuTime,
    last_now: CuTime,
    credit: u128,
    capacity: u128,
    // Byte deficit fairness: three MTUs of replay for one MTU of recovery.
    deficit: [usize; 2],
    lane: usize,
    stopping: bool,
    stats: SenderStats,
}

impl SenderCore {
    /// Validate all storage arithmetic before allocating. `reserved_bytes` covers
    /// the driver's owned-record pool, which shares the destination budget.
    pub fn new(config: LogStreamSenderConfig, now: CuTime, reserved_bytes: usize) -> Result<Self> {
        config.validate()?;
        let policy = config.pacing;
        if policy.bitrate_bps == 0
            || policy.burst_packets == 0
            || policy.max_latency.as_nanos() == 0
        {
            return Err(Error::InvalidConfig("pacing bounds must be nonzero"));
        }
        let mtu = crate::PACKET_HEADER_LEN + config.continuous.fec.symbol_size();
        let object_bytes = usize::try_from(config.recovery.finite.max_object_bytes)
            .map_err(|_| Error::InvalidConfig("object limit exceeds usize"))?;
        let symbol = usize::from(config.recovery.finite.symbol_size);
        if symbol != config.continuous.fec.symbol_size()
            || symbol <= crate::rlc::FRAGMENT_HEADER_LEN
            || config.continuous.repair_every_source_symbols == 0
            || object_bytes < 256
        {
            return Err(Error::InvalidConfig(
                "scheduled lanes require matching valid symbols and object bounds",
            ));
        }
        // One source block keeps finite encoding and its retained packet bound explicit.
        if object_bytes.div_ceil(symbol) > 56_403
            || raptorq::ObjectTransmissionInformation::with_defaults(
                object_bytes as u64,
                symbol as u16,
            )
            .source_blocks()
                != 1
        {
            return Err(Error::InvalidConfig(
                "scheduled objects must fit one RaptorQ source block",
            ));
        }
        let repairs_per_object = config.recovery.finite.repair_symbols_per_block as usize;
        let finite_capacity = |bytes: usize| bytes.div_ceil(symbol).checked_add(repairs_per_object);
        let manifest_capacity = finite_capacity(config.recovery.manifest_record.len())
            .ok_or(Error::InvalidConfig("manifest packet bound overflow"))?;
        let control_capacity = finite_capacity(object_bytes)
            .and_then(|v| finite_capacity(256).and_then(|a| v.checked_add(a)))
            .ok_or(Error::InvalidConfig("recovery packet bound overflow"))?;
        let source_capacity = config
            .continuous
            .max_record_bytes
            .div_ceil(symbol - crate::rlc::FRAGMENT_HEADER_LEN);
        let boundary_capacity = source_capacity
            .checked_mul(2)
            .ok_or(Error::InvalidConfig("boundary packet bound overflow"))?;
        let packet_bytes = mtu + size_of::<usize>() + size_of::<CuTime>() + size_of::<u64>();
        let retained_packets = control_capacity
            .checked_mul(PENDING_KEYFRAMES + 1)
            .and_then(|v| {
                boundary_capacity
                    .checked_mul(PENDING_BOUNDARIES + 1)
                    .and_then(|b| v.checked_add(b))
            })
            .and_then(|v| v.checked_add(manifest_capacity))
            .ok_or(Error::InvalidConfig("retained packet bound overflow"))?;
        let fixed = retained_packets
            .checked_mul(packet_bytes)
            .and_then(|v| v.checked_add(size_of::<ContinuousEncoder<1128, 64>>()))
            .and_then(|v| v.checked_add(size_of::<Self>()))
            .and_then(|v| {
                v.checked_add(
                    (PENDING_BOUNDARIES + PENDING_KEYFRAMES) * size_of::<PendingPackets>(),
                )
            })
            .and_then(|v| v.checked_add(mtu))
            .and_then(|v| v.checked_add(config.recovery.manifest_record.len()))
            .and_then(|v| v.checked_add(reserved_bytes))
            .ok_or(Error::InvalidConfig("sender storage bound overflow"))?;
        let queue_capacity = policy
            .memory_budget_bytes
            .checked_sub(fixed)
            .map(|remaining| remaining / packet_bytes)
            .filter(|capacity| *capacity >= boundary_capacity.max(1))
            .ok_or(Error::InvalidConfig(
                "sender buffers exceed destination memory budget",
            ))?;
        let continuous = Box::new(ContinuousEncoder::new(
            config.continuous.identity,
            config.continuous.first_packet_sequence,
            config.continuous.lane,
            config.continuous.fec,
            config.continuous.max_record_bytes,
            config.continuous.initial_esi,
        )?);
        let mut finite = FiniteObjectEncoder::new(config.recovery.finite)?;
        let mut manifest = Packets::new(manifest_capacity, mtu);
        let manifest_id = decode_record(&config.recovery.manifest_record)?.object_id;
        finite.push_record_with(&config.recovery.manifest_record, |packet| {
            if !manifest.push(packet, now, manifest_id) {
                return Err(Error::InvalidConfig("manifest packet capacity"));
            }
            Ok(())
        })?;
        let recovery = || RecoveryPackets {
            control: Packets::new(control_capacity, mtu),
            boundary: Packets::new(boundary_capacity, mtu),
        };
        let capacity = u128::from(policy.burst_packets) * mtu as u128 * 8 * 1_000_000_000;
        Ok(Self {
            repairs: RepairSchedule::new(
                config.continuous.repair_every_source_symbols,
                config.continuous.first_repair_key,
                config.continuous.repair_density,
            ),
            config,
            continuous,
            finite,
            scratch: vec![0; mtu],
            data: Packets::new(queue_capacity, mtu),
            manifest,
            latest: recovery(),
            pending_cl: (0..PENDING_BOUNDARIES)
                .map(|_| PendingPackets {
                    id: None,
                    packets: Packets::new(boundary_capacity, mtu),
                })
                .collect(),
            pending_kf: (0..PENDING_KEYFRAMES)
                .map(|_| PendingPackets {
                    id: None,
                    packets: Packets::new(control_capacity, mtu),
                })
                .collect(),
            latest_id: None,
            manifest_cursor: 0,
            recovery_cursor: 0,
            next_repeat: now + RECOVERY_REPEAT_INTERVAL,
            last_now: now,
            credit: capacity,
            capacity,
            deficit: [0; 2],
            lane: 0,
            stopping: false,
            stats: SenderStats::default(),
        })
    }

    pub const fn stats(&self) -> SenderStats {
        self.stats
    }

    /// Admit an already encoded record from a bounded worker buffer. No borrow
    /// escapes this call. CL packets shed on overflow; retained recovery replaces
    /// only a complete matching boundary/keyframe pair.
    pub fn accept_record(&mut self, record: &[u8], now: CuTime) -> Result<()> {
        let decoded = decode_record(record)?;
        match decoded.kind {
            RecordKind::CopperList => {
                let retain = decoded
                    .object_id
                    .is_multiple_of(u64::from(self.config.recovery.anchor_interval));
                let slot = if retain {
                    Some(Self::reserve(
                        &mut self.pending_cl,
                        decoded.object_id,
                        &mut self.stats,
                    ))
                } else {
                    None
                };
                let stats = &mut self.stats;
                let pending = &mut self.pending_cl;
                let data = &mut self.data;
                self.continuous.push_record_with_repairs(
                    record,
                    &mut self.scratch,
                    &mut |packet| {
                        if let Some(slot) = slot
                            && !pending[slot].packets.push(packet, now, decoded.object_id)
                        {
                            return Err(Error::InvalidConfig("boundary packet capacity"));
                        }
                        if !data.push(packet, now, decoded.object_id) {
                            stats.queue_drops = stats.queue_drops.saturating_add(1);
                        }
                        stats.queue_peak = stats.queue_peak.max(data.len);
                        Ok(())
                    },
                    &mut self.repairs,
                )?;
            }
            RecordKind::KeyFrame => {
                if !decoded
                    .object_id
                    .is_multiple_of(u64::from(self.config.recovery.anchor_interval))
                {
                    return Ok(());
                }
                let manifest = decode_record(&self.config.recovery.manifest_record)?;
                let anchor = encode_record(
                    RecordKind::Anchor,
                    decoded.object_id,
                    &encode_anchor(&Anchor {
                        manifest_object_id: manifest.object_id,
                        manifest_record_digest: manifest.digest,
                        copperlist_id: decoded.object_id,
                        keyframe_object_id: decoded.object_id,
                        keyframe_record_digest: decoded.digest,
                    })?,
                )?;
                let slot = Self::reserve(&mut self.pending_kf, decoded.object_id, &mut self.stats);
                for bytes in [record, anchor.as_slice()] {
                    self.finite.push_record_with(bytes, |packet| {
                        if !self.pending_kf[slot]
                            .packets
                            .push(packet, now, decoded.object_id)
                        {
                            return Err(Error::InvalidConfig("recovery packet capacity"));
                        }
                        Ok(())
                    })?;
                }
            }
            _ => {
                return Err(Error::InvalidConfig(
                    "sender inbox accepts CLs and keyframes",
                ));
            }
        }
        self.promote_recovery();
        Ok(())
    }

    fn reserve(slots: &mut [PendingPackets], id: u64, stats: &mut SenderStats) -> usize {
        let index = slots
            .iter()
            .position(|s| s.id.is_none() || s.id == Some(id))
            .unwrap_or_else(|| {
                slots
                    .iter()
                    .enumerate()
                    .min_by_key(|(_, s)| s.id)
                    .unwrap()
                    .0
            });
        if slots[index].id.is_some() {
            stats.recovery_superseded += 1;
        }
        slots[index].id = Some(id);
        slots[index].packets.clear();
        index
    }

    fn promote_recovery(&mut self) {
        // Finish an in-flight bundle before replacing it; frequent captures must
        // not continually restart a large transfer on a slow link.
        if self.recovery_cursor < self.recovery_len() {
            return;
        }
        let pair = self
            .pending_cl
            .iter()
            .enumerate()
            .filter_map(|(cl, entry)| {
                let id = entry.id?;
                // An anchor must not make the receiver skip older source packets
                // still waiting in our own queue. Expiry releases this fence too.
                if self.data.len > 0 && self.data.ids[self.data.head] < id {
                    return None;
                }
                if self.latest_id.is_some_and(|latest| id <= latest) {
                    return None;
                }
                self.pending_kf
                    .iter()
                    .position(|kf| kf.id == Some(id))
                    .map(|kf| (id, cl, kf))
            })
            .min_by_key(|(id, _, _)| *id);
        if let Some((id, cl, kf)) = pair {
            core::mem::swap(&mut self.latest.boundary, &mut self.pending_cl[cl].packets);
            core::mem::swap(&mut self.latest.control, &mut self.pending_kf[kf].packets);
            self.pending_cl[cl].id = None;
            self.pending_kf[kf].id = None;
            self.latest_id = Some(id);
            self.recovery_cursor = 0;
            self.request_recovery();
        }
    }

    /// Coalesced request for retained bootstrap data. Future advisory feedback
    /// uses this same operation; it grants neither extra bandwidth nor retention.
    pub fn request_recovery(&mut self) {
        if self.stopping || self.control_packet().is_some() {
            return;
        }
        if self.manifest_cursor >= self.manifest.len {
            self.manifest_cursor = 0;
        }
        if self.recovery_cursor >= self.recovery_len() {
            self.recovery_cursor = 0;
        }
    }

    fn recovery_len(&self) -> usize {
        self.latest.control.len + self.latest.boundary.len
    }

    fn control_packet(&self) -> Option<&[u8]> {
        if self.manifest_cursor < self.manifest.len {
            return Some(self.manifest.get(self.manifest_cursor));
        }
        if self.recovery_cursor < self.latest.control.len {
            return Some(self.latest.control.get(self.recovery_cursor));
        }
        let boundary = self.recovery_cursor - self.latest.control.len;
        (boundary < self.latest.boundary.len).then(|| self.latest.boundary.get(boundary))
    }

    /// Disable periodic work. The driver enforces the finite shutdown deadline.
    pub fn begin_shutdown(&mut self) {
        self.stopping = true;
    }

    /// Account for work abandoned at the driver's finite shutdown deadline.
    pub fn discard_pending(&mut self) {
        self.stats.shutdown_drops += (self.data.len
            + self.manifest.len.saturating_sub(self.manifest_cursor)
            + self.recovery_len().saturating_sub(self.recovery_cursor))
            as u64;
        self.data.clear();
        self.manifest_cursor = self.manifest.len;
        self.recovery_cursor = self.recovery_len();
    }

    pub fn is_idle(&self) -> bool {
        self.data.len == 0 && self.control_packet().is_none()
    }

    /// Send a bounded amount of eligible work and return the next local deadline.
    /// Carrier WouldBlock consumes the attempt's budget and drops the packet.
    pub fn poll<T: CuStreamTx>(
        &mut self,
        now: CuTime,
        transport: &mut T,
    ) -> Result<Option<CuTime>> {
        if now < self.last_now {
            return Err(Error::InvalidConfig("sender clock moved backwards"));
        }
        let elapsed = (now - self.last_now).as_nanos();
        self.credit = self.capacity.min(
            self.credit
                .saturating_add(u128::from(elapsed) * u128::from(self.config.pacing.bitrate_bps)),
        );
        self.last_now = now;
        self.promote_recovery();
        if !self.stopping && now >= self.next_repeat {
            self.request_recovery();
            self.stats.recovery_rounds = self.stats.recovery_rounds.saturating_add(1);
            self.next_repeat = now + RECOVERY_REPEAT_INTERVAL;
        }
        let mut work = 0;
        while self.data.len > 0
            && now >= self.data.times[self.data.head] + self.config.pacing.max_latency
            && work < MAX_SENDS_PER_POLL
        {
            self.data.pop();
            self.stats.expired_packets += 1;
            work += 1;
        }
        while work < MAX_SENDS_PER_POLL {
            let has_data = self.data.len > 0;
            let has_control = self.control_packet().is_some();
            if !has_data && !has_control {
                break;
            }
            if (self.lane == 0 && !has_data) || (self.lane == 1 && !has_control) {
                self.deficit[self.lane] = 0;
                self.lane ^= 1;
            }
            let packet = if self.lane == 0 {
                self.data.get(0)
            } else {
                self.control_packet().unwrap()
            };
            let len = packet.len();
            if self.deficit[self.lane] < len {
                self.deficit[self.lane] += self.scratch.len() * if self.lane == 0 { 3 } else { 1 };
                self.lane ^= 1;
                continue;
            }
            let cost = len as u128 * 8 * 1_000_000_000;
            if self.credit < cost {
                let wait =
                    (cost - self.credit).div_ceil(u128::from(self.config.pacing.bitrate_bps));
                let mut deadline = now + CuDuration(wait.min(u128::from(u64::MAX)) as u64);
                if self.data.len > 0 {
                    deadline = deadline
                        .min(self.data.times[self.data.head] + self.config.pacing.max_latency);
                }
                return Ok(Some(if self.stopping {
                    deadline
                } else {
                    deadline.min(self.next_repeat)
                }));
            }
            match transport.try_send(packet) {
                Ok(()) => {
                    self.stats.packets_sent += 1;
                    self.stats.bytes_sent += len as u64;
                }
                Err(CuStreamTxError::WouldBlock) => self.stats.transport_drops += 1,
                Err(CuStreamTxError::Failed(message)) => return Err(Error::Transport(message)),
            }
            self.credit -= cost;
            self.deficit[self.lane] -= len;
            if self.lane == 0 {
                self.data.pop();
            } else if self.manifest_cursor < self.manifest.len {
                self.manifest_cursor += 1;
            } else {
                self.recovery_cursor += 1;
            }
            work += 1;
            self.promote_recovery();
        }
        Ok(if !self.is_idle() {
            Some(now)
        } else if self.stopping {
            None
        } else {
            Some(self.next_repeat)
        })
    }
}
