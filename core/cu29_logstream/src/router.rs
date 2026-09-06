//! Bounded ground-side session discovery and decoder routing.

use crate::{
    ContinuousDecoder, ContinuousReceiveEvent, CuStreamRx, CuStreamRxError, Error, FecScheme,
    FiniteObjectDecoder, FiniteObjectLimits, Lane, ReceiveError, ReceiverLimits, RecoveredRecord,
    Result, SessionManifest, StreamIdentity, WirePacketRef,
};
use alloc::{collections::VecDeque, vec::Vec};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SessionRouterLimits {
    pub max_sessions: usize,
    /// Per-session pre-manifest datagrams, each bounded by MAX_SYMBOL_SIZE + header.
    pub max_startup_packets: usize,
    /// Completed keyframe/recovery point records retained per session, in addition to FEC storage.
    pub max_recovery_records: usize,
    pub max_pending_events: usize,
    pub max_record_bytes: usize,
    pub max_buffered_records: usize,
    pub equation_capacity: usize,
    pub finite_objects: FiniteObjectLimits,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SessionRouterStats {
    pub datagrams_seen: usize,
    pub malformed_datagrams: usize,
    pub datagrams_before_manifest: usize,
    pub sessions_discovered: usize,
    pub manifests_accepted: usize,
    pub startup_packets_dropped: usize,
}

/// A decoded manifest bound to its verified, immutable wire record.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReceivedManifest {
    manifest: SessionManifest,
    record: RecoveredRecord,
}

impl ReceivedManifest {
    /// Takes ownership of canonical bytes and verifies their manifest envelope.
    pub fn decode_record(bytes: Vec<u8>) -> Result<Self> {
        Self::from_record(RecoveredRecord::from_bytes(bytes)?)
    }

    fn from_record(record: RecoveredRecord) -> Result<Self> {
        let manifest = SessionManifest::from_decoded(record.decoded())?;
        Ok(Self { manifest, record })
    }

    pub fn manifest(&self) -> &SessionManifest {
        &self.manifest
    }

    pub fn record(&self) -> &RecoveredRecord {
        &self.record
    }
}

/// Session events borrow receiver storage during delivery. Use `to_owned` only
/// when an application deliberately needs to retain encoded history.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SessionEvent<R = RecoveredRecord, M = ReceivedManifest> {
    Manifest(M),
    /// Emitted only after the recovery point, manifest and keyframe agree by digest and id.
    VerifiedRecoveryPoint {
        identity: StreamIdentity,
        recovery_point: crate::RecoveryPoint,
        recovery_record: R,
        keyframe: R,
    },
    ContinuousRecord {
        identity: StreamIdentity,
        record: R,
    },
    Gap {
        identity: StreamIdentity,
        gap: crate::CopperListGap,
    },
    Object {
        identity: StreamIdentity,
        record: R,
    },
}
/// A callback-scoped view; retaining it requires an explicit allocating `to_owned` call.
pub type SessionEventRef<'a> = SessionEvent<&'a RecoveredRecord, &'a ReceivedManifest>;
impl SessionEventRef<'_> {
    pub fn to_owned(&self) -> SessionEvent {
        match self {
            Self::Manifest(manifest) => SessionEvent::Manifest((*manifest).clone()),
            Self::VerifiedRecoveryPoint {
                identity,
                recovery_point,
                recovery_record,
                keyframe,
            } => SessionEvent::VerifiedRecoveryPoint {
                identity: *identity,
                recovery_point: recovery_point.clone(),
                recovery_record: (*recovery_record).clone(),
                keyframe: (*keyframe).clone(),
            },
            Self::ContinuousRecord { identity, record } => SessionEvent::ContinuousRecord {
                identity: *identity,
                record: (*record).clone(),
            },
            Self::Gap { identity, gap } => SessionEvent::Gap {
                identity: *identity,
                gap: *gap,
            },
            Self::Object { identity, record } => SessionEvent::Object {
                identity: *identity,
                record: (*record).clone(),
            },
        }
    }
}

/// Discovers sender identities and routes packets using the recovered manifest.
/// Event payloads remain in receiver storage until the callback succeeds.
pub struct SessionRouter<
    const MAX_SYMBOL_SIZE: usize,
    const MAX_WINDOW_SYMBOLS: usize,
    const MAX_EQUATIONS: usize,
> {
    limits: SessionRouterLimits,
    sessions: Vec<RoutedSession<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS, MAX_EQUATIONS>>,
    pending: VecDeque<PendingEvent>,
    stats: SessionRouterStats,
}

struct RecoveryRecord {
    record: RecoveredRecord,
    point: Option<crate::RecoveryPoint>,
}
// Cache indices remain stable: pending delivery completes before another control
// packet can mutate a session'session cache. A finite packet completes at most one object.
enum PendingEvent {
    Manifest(usize),
    CachedObject {
        session: usize,
        slot: usize,
    },
    Object {
        identity: StreamIdentity,
        record: RecoveredRecord,
    },
    RecoveryPoint {
        session: usize,
        point: usize,
        keyframe: usize,
    },
    Gap {
        identity: StreamIdentity,
        gap: crate::CopperListGap,
    },
}

struct RoutedSession<const S: usize, const W: usize, const E: usize> {
    identity: StreamIdentity,
    finite: FiniteObjectDecoder,
    manifest: Option<ReceivedManifest>,
    startup: VecDeque<Vec<u8>>,
    recovery: Vec<RecoveryRecord>,
    recovery_dirty: bool,
    last_recovery_point: Option<u64>,
    delivered_record: bool,
    continuous: Option<ContinuousDecoder<S, W, E>>,
}

impl<const MAX_SYMBOL_SIZE: usize, const MAX_WINDOW_SYMBOLS: usize, const MAX_EQUATIONS: usize>
    SessionRouter<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS, MAX_EQUATIONS>
{
    pub fn new(limits: SessionRouterLimits) -> Result<Self> {
        if limits.max_sessions == 0
            || limits.max_pending_events < 3
            || limits.max_recovery_records < 2
            || limits.max_record_bytes == 0
            || limits.max_buffered_records == 0
            || limits.equation_capacity == 0
        {
            return Err(Error::InvalidConfig("invalid session router capacities"));
        }
        Ok(Self {
            limits,
            sessions: Vec::with_capacity(limits.max_sessions),
            pending: VecDeque::with_capacity(limits.max_pending_events),
            stats: SessionRouterStats::default(),
        })
    }

    pub const fn stats(&self) -> SessionRouterStats {
        self.stats
    }

    pub fn session_manifest(&self, identity: StreamIdentity) -> Option<&SessionManifest> {
        self.sessions
            .iter()
            .find(|session| session.identity == identity)
            .and_then(|session| session.manifest.as_ref())
            .map(ReceivedManifest::manifest)
    }

    pub fn try_receive<T: CuStreamRx, E>(
        &mut self,
        transport: &mut T,
        packet: &mut [u8],
        emit: impl FnMut(SessionEventRef<'_>) -> core::result::Result<(), E>,
    ) -> core::result::Result<bool, ReceiveError<E>> {
        let len = match transport.try_recv(packet) {
            Ok(Some(len)) => len,
            Ok(None) => return Ok(false),
            Err(CuStreamRxError::BufferTooSmall { needed }) => {
                return Err(Error::BufferTooSmall {
                    needed,
                    available: packet.len(),
                }
                .into());
            }
            Err(CuStreamRxError::Failed(message)) => return Err(Error::Transport(message).into()),
        };
        if len > packet.len() {
            return Err(Error::BufferTooSmall {
                needed: len,
                available: packet.len(),
            }
            .into());
        }
        self.receive_datagram(&packet[..len], emit)?;
        Ok(true)
    }

    pub fn receive_datagram<E>(
        &mut self,
        datagram: &[u8],
        mut emit: impl FnMut(SessionEventRef<'_>) -> core::result::Result<(), E>,
    ) -> core::result::Result<(), ReceiveError<E>> {
        self.drain_events(&mut emit)?;
        self.stats.datagrams_seen += 1;
        let packet = match WirePacketRef::decode(datagram) {
            Ok(p) => p,
            Err(_) => {
                self.stats.malformed_datagrams += 1;
                return Ok(());
            }
        };
        let identity = StreamIdentity {
            session_id: packet.header.session_id,
            sender_id: packet.header.sender_id,
        };
        let index = match self
            .sessions
            .iter()
            .position(|session| session.identity == identity)
        {
            Some(i) => i,
            None if (packet.header.fec_scheme == FecScheme::RaptorQ
                && packet.header.lane == Lane::Control)
                || (packet.header.lane == Lane::ReplayCritical
                    && packet.header.record_kind == crate::RecordKind::CopperList) =>
            {
                if self.sessions.len() == self.limits.max_sessions {
                    return Err(Error::TooManySessions {
                        maximum: self.limits.max_sessions,
                    }
                    .into());
                }
                self.sessions.push(RoutedSession {
                    identity,
                    finite: FiniteObjectDecoder::new(
                        identity,
                        Lane::Control,
                        self.limits.finite_objects,
                    )?,
                    manifest: None,
                    startup: VecDeque::with_capacity(self.limits.max_startup_packets),
                    recovery: Vec::with_capacity(self.limits.max_recovery_records),
                    recovery_dirty: false,
                    last_recovery_point: None,
                    delivered_record: false,
                    continuous: None,
                });
                self.stats.sessions_discovered += 1;
                self.sessions.len() - 1
            }
            None => {
                self.stats.datagrams_before_manifest += 1;
                return Ok(());
            }
        };
        if packet.header.fec_scheme == FecScheme::RaptorQ {
            self.sessions[index].finite.receive_packet(packet)?;
            if let Some(record) = self.sessions[index].finite.pop_record() {
                self.receive_object(index, record)?;
            }
        } else if self.sessions[index].continuous.is_some() {
            Self::receive_continuous(&mut self.sessions[index], packet, &mut emit)?;
        } else {
            self.stats.datagrams_before_manifest += 1;
            let startup = &mut self.sessions[index].startup;
            if datagram.len() <= MAX_SYMBOL_SIZE + crate::PACKET_HEADER_LEN
                && startup.len() < self.limits.max_startup_packets
            {
                startup.push_back(datagram.to_vec());
            } else {
                self.stats.startup_packets_dropped += 1;
            }
        }
        self.drain_events(&mut emit)
    }
    /// Retries pending delivery without cloning encoded records. A consumer error
    /// retains the same bytes until a later call succeeds.
    pub fn drain_events<E>(
        &mut self,
        emit: &mut impl FnMut(SessionEventRef<'_>) -> core::result::Result<(), E>,
    ) -> core::result::Result<(), ReceiveError<E>> {
        loop {
            while let Some(pending) = self.pending.front() {
                let event = match pending {
                    PendingEvent::Manifest(i) => {
                        SessionEvent::Manifest(self.sessions[*i].manifest.as_ref().unwrap())
                    }
                    PendingEvent::CachedObject { session, slot } => SessionEvent::Object {
                        identity: self.sessions[*session].identity,
                        record: &self.sessions[*session].recovery[*slot].record,
                    },
                    PendingEvent::Object { identity, record } => SessionEvent::Object {
                        identity: *identity,
                        record,
                    },
                    PendingEvent::Gap { identity, gap } => SessionEvent::Gap {
                        identity: *identity,
                        gap: *gap,
                    },
                    PendingEvent::RecoveryPoint {
                        session,
                        point,
                        keyframe,
                    } => {
                        let session = &self.sessions[*session];
                        SessionEvent::VerifiedRecoveryPoint {
                            identity: session.identity,
                            recovery_point: session.recovery[*point]
                                .point
                                .as_ref()
                                .unwrap()
                                .clone(),
                            recovery_record: &session.recovery[*point].record,
                            keyframe: &session.recovery[*keyframe].record,
                        }
                    }
                };
                emit(event).map_err(ReceiveError::Consumer)?;
                self.pending.pop_front();
            }
            for session in &mut self.sessions {
                if let Some(decoder) = &mut session.continuous {
                    decoder.drain_events(|e| {
                        Self::deliver_continuous(
                            session.identity,
                            session.last_recovery_point,
                            &mut session.delivered_record,
                            e,
                            emit,
                        )
                    })?;
                }
            }
            if let Some(index) = self
                .sessions
                .iter()
                .position(|session| session.continuous.is_some() && !session.startup.is_empty())
            {
                let packet = self.sessions[index].startup.pop_front().unwrap();
                Self::receive_continuous(
                    &mut self.sessions[index],
                    WirePacketRef::decode(&packet)?,
                    emit,
                )?;
                continue;
            }
            for index in 0..self.sessions.len() {
                if self.sessions[index].recovery_dirty {
                    self.verify_recovery(index)?;
                }
                if !self.pending.is_empty() {
                    break;
                }
            }
            if self.pending.is_empty() {
                return Ok(());
            }
        }
    }

    fn deliver_continuous<E>(
        identity: StreamIdentity,
        last_recovery_point: Option<u64>,
        delivered: &mut bool,
        event: ContinuousReceiveEvent<'_>,
        emit: &mut impl FnMut(SessionEventRef<'_>) -> core::result::Result<(), E>,
    ) -> core::result::Result<(), E> {
        match event {
            ContinuousReceiveEvent::Record(record) => {
                emit(SessionEvent::ContinuousRecord { identity, record })?;
                *delivered = true;
            }
            ContinuousReceiveEvent::Gap(mut gap) => {
                if !*delivered && last_recovery_point.is_none() {
                    gap.reason = crate::GapReason::LateJoin;
                }
                emit(SessionEvent::Gap { identity, gap })?;
            }
        }
        Ok(())
    }

    fn receive_continuous<E>(
        session: &mut RoutedSession<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS, MAX_EQUATIONS>,
        packet: WirePacketRef<'_>,
        emit: &mut impl FnMut(SessionEventRef<'_>) -> core::result::Result<(), E>,
    ) -> core::result::Result<(), ReceiveError<E>> {
        session
            .continuous
            .as_mut()
            .unwrap()
            .receive_packet(packet, |e| {
                Self::deliver_continuous(
                    session.identity,
                    session.last_recovery_point,
                    &mut session.delivered_record,
                    e,
                    emit,
                )
            })
    }

    fn receive_object(&mut self, index: usize, mut record: RecoveredRecord) -> Result<()> {
        if record.decoded().kind == crate::RecordKind::Manifest {
            if let Some(existing) = &self.sessions[index].manifest {
                if existing.record.decoded().digest != record.decoded().digest {
                    return Err(Error::InconsistentObject);
                }
                return Ok(());
            }
            let manifest = ReceivedManifest::from_record(record)?;
            if manifest.manifest.identity != self.sessions[index].identity {
                return Err(Error::InconsistentObject);
            }
            self.sessions[index].continuous = Some(Self::decoder_from_manifest(
                self.limits,
                &manifest.manifest,
            )?);
            self.sessions[index].manifest = Some(manifest);
            self.sessions[index].recovery_dirty = true;
            self.stats.manifests_accepted += 1;
            self.push_event(PendingEvent::Manifest(index))?;
        } else {
            let point = match record.decoded().kind {
                crate::RecordKind::KeyFrame => {
                    record.validate_keyframe()?;
                    None
                }
                crate::RecordKind::RecoveryPoint => {
                    Some(crate::decode_recovery_point(record.decoded().payload)?)
                }
                _ => {
                    return self.push_event(PendingEvent::Object {
                        identity: self.sessions[index].identity,
                        record,
                    });
                }
            };
            let session = &mut self.sessions[index];
            if session.recovery.len() == self.limits.max_recovery_records {
                session.recovery.remove(0);
            }
            let slot = session.recovery.len();
            session.recovery.push(RecoveryRecord { record, point });
            session.recovery_dirty = true;
            self.push_event(PendingEvent::CachedObject {
                session: index,
                slot,
            })?;
        }
        Ok(())
    }

    fn verify_recovery(&mut self, index: usize) -> Result<()> {
        let session = &mut self.sessions[index];
        let Some(manifest) = &session.manifest else {
            return Ok(());
        };
        let mut candidate = None;
        for (point_index, entry) in session.recovery.iter().enumerate() {
            let Some(point) = &entry.point else {
                continue;
            };
            if point.copperlist_id != entry.record.decoded().object_id
                || !point.references_manifest(manifest.record.decoded())
                || session
                    .last_recovery_point
                    .is_some_and(|id| point.copperlist_id <= id)
            {
                continue;
            }
            for (kf_index, kf) in session.recovery.iter().enumerate() {
                if point.references_keyframe(kf.record.decoded())
                    && kf.record.keyframe_id() == Some(point.copperlist_id)
                    && candidate.is_none_or(|(_, _, id)| point.copperlist_id > id)
                {
                    candidate = Some((point_index, kf_index, point.copperlist_id));
                }
            }
        }
        session.recovery_dirty = false;
        let Some((point, keyframe, id)) = candidate else {
            return Ok(());
        };
        let decoder = session.continuous.as_mut().unwrap();
        let next = decoder.next_object_id();
        let needed = 1 + usize::from(id > next);
        if self.pending.len() + needed > self.limits.max_pending_events {
            session.recovery_dirty = true;
            return Err(Error::TooManyRecords {
                actual: self.pending.len() + needed,
                maximum: self.limits.max_pending_events,
            });
        }
        if id > next {
            self.pending.push_back(PendingEvent::Gap {
                identity: session.identity,
                gap: crate::CopperListGap {
                    first_id: next,
                    last_id: id - 1,
                    reason: if session.delivered_record {
                        crate::GapReason::RecoveryPoint
                    } else {
                        crate::GapReason::LateJoin
                    },
                },
            });
            decoder.resume_at(id);
        }
        session.last_recovery_point = Some(id);
        self.pending.push_back(PendingEvent::RecoveryPoint {
            session: index,
            point,
            keyframe,
        });
        Ok(())
    }

    pub fn finish_through<E>(
        &mut self,
        identity: StreamIdentity,
        last_id: u64,
        mut emit: impl FnMut(SessionEventRef<'_>) -> core::result::Result<(), E>,
    ) -> core::result::Result<(), ReceiveError<E>> {
        self.drain_events(&mut emit)?;
        let session = self
            .sessions
            .iter_mut()
            .find(|session| session.identity == identity)
            .ok_or(Error::InvalidConfig("unknown session"))?;
        session
            .continuous
            .as_mut()
            .ok_or(Error::InvalidConfig("session has no manifest"))?
            .finish_through(last_id, |event| match event {
                ContinuousReceiveEvent::Record(record) => {
                    emit(SessionEvent::ContinuousRecord { identity, record })
                }
                ContinuousReceiveEvent::Gap(gap) => emit(SessionEvent::Gap { identity, gap }),
            })
    }

    fn decoder_from_manifest(
        limits: SessionRouterLimits,
        manifest: &SessionManifest,
    ) -> Result<ContinuousDecoder<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS, MAX_EQUATIONS>> {
        if usize::from(manifest.plan.symbol_size) > MAX_SYMBOL_SIZE
            || usize::from(manifest.plan.continuous.window_symbols) > MAX_WINDOW_SYMBOLS
            || manifest.plan.max_record_bytes > limits.max_record_bytes as u64
            || usize::from(manifest.plan.continuous.window_symbols) > limits.max_buffered_records
        {
            return Err(Error::InvalidConfig(
                "session manifest exceeds receiver-local limits",
            ));
        }
        ContinuousDecoder::new(
            manifest.identity,
            Lane::ReplayCritical,
            manifest.plan.rlc_config()?,
            limits.equation_capacity,
            0,
            ReceiverLimits::new(
                manifest.plan.max_record_bytes as usize,
                limits.max_buffered_records,
            ),
        )
    }

    fn push_event(&mut self, event: PendingEvent) -> Result<()> {
        if self.pending.len() == self.limits.max_pending_events {
            return Err(Error::TooManyRecords {
                actual: self.pending.len() + 1,
                maximum: self.limits.max_pending_events,
            });
        }
        self.pending.push_back(event);
        Ok(())
    }
}
