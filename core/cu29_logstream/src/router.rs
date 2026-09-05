//! Bounded ground-side session discovery and decoder routing.

use crate::{
    ContinuousDecoder, ContinuousReceiveEvent, CuStreamRx, CuStreamRxError, Error, FecScheme,
    FiniteObjectDecoder, FiniteObjectLimits, Lane, ReceiveError, ReceiverLimits, RecoveredRecord,
    Result, SessionManifest, StreamIdentity, WirePacket,
};
use alloc::vec::Vec;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SessionRouterLimits {
    pub max_sessions: usize,
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
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SessionEvent {
    Manifest(SessionManifest),
    ContinuousRecord {
        identity: StreamIdentity,
        record: RecoveredRecord,
    },
    Gap {
        identity: StreamIdentity,
        gap: crate::CopperListGap,
    },
    Object {
        identity: StreamIdentity,
        record: RecoveredRecord,
    },
}

/// Discovers sender identities from RaptorQ control packets and routes all
/// subsequent packets using the decoder parameters in the recovered manifest.
pub struct SessionRouter<
    const MAX_SYMBOL_SIZE: usize,
    const MAX_WINDOW_SYMBOLS: usize,
    const MAX_EQUATIONS: usize,
> {
    limits: SessionRouterLimits,
    sessions: Vec<RoutedSession<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS, MAX_EQUATIONS>>,
    pending: Vec<SessionEvent>,
    stats: SessionRouterStats,
}

struct RoutedSession<
    const MAX_SYMBOL_SIZE: usize,
    const MAX_WINDOW_SYMBOLS: usize,
    const MAX_EQUATIONS: usize,
> {
    identity: StreamIdentity,
    finite: FiniteObjectDecoder,
    manifest: Option<SessionManifest>,
    continuous: Option<ContinuousDecoder<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS, MAX_EQUATIONS>>,
}

impl<const MAX_SYMBOL_SIZE: usize, const MAX_WINDOW_SYMBOLS: usize, const MAX_EQUATIONS: usize>
    SessionRouter<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS, MAX_EQUATIONS>
{
    pub fn new(limits: SessionRouterLimits) -> Result<Self> {
        if limits.max_sessions == 0
            || limits.max_pending_events == 0
            || limits.max_record_bytes == 0
            || limits.max_buffered_records == 0
            || limits.equation_capacity == 0
        {
            return Err(Error::InvalidConfig(
                "session router limits must all be nonzero",
            ));
        }
        Ok(Self {
            limits,
            sessions: Vec::with_capacity(limits.max_sessions),
            pending: Vec::with_capacity(limits.max_pending_events),
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
    }

    /// Pulls at most one complete packet from a nonblocking Copper resource.
    pub fn try_receive<T, E>(
        &mut self,
        transport: &mut T,
        packet: &mut [u8],
        emit: impl FnMut(SessionEvent) -> core::result::Result<(), E>,
    ) -> core::result::Result<bool, ReceiveError<E>>
    where
        T: CuStreamRx,
    {
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
            Err(CuStreamRxError::Failed(message)) => {
                return Err(Error::Transport(message).into());
            }
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
        mut emit: impl FnMut(SessionEvent) -> core::result::Result<(), E>,
    ) -> core::result::Result<(), ReceiveError<E>> {
        self.drain_events(&mut emit)?;
        self.stats.datagrams_seen = self.stats.datagrams_seen.saturating_add(1);
        let packet = match WirePacket::decode(datagram) {
            Ok(packet) => packet,
            Err(_) => {
                self.stats.malformed_datagrams = self.stats.malformed_datagrams.saturating_add(1);
                return Ok(());
            }
        };
        let identity = StreamIdentity {
            session_id: packet.header.session_id,
            sender_id: packet.header.sender_id,
        };

        let session_index = match self
            .sessions
            .iter()
            .position(|session| session.identity == identity)
        {
            Some(index) => index,
            None if packet.header.fec_scheme == FecScheme::RaptorQ
                && packet.header.lane == Lane::Control =>
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
                    continuous: None,
                });
                self.stats.sessions_discovered = self.stats.sessions_discovered.saturating_add(1);
                self.sessions.len() - 1
            }
            None => {
                self.stats.datagrams_before_manifest =
                    self.stats.datagrams_before_manifest.saturating_add(1);
                return Ok(());
            }
        };

        if packet.header.fec_scheme == FecScheme::RaptorQ {
            self.receive_finite(session_index, datagram)?;
        } else if let Some(decoder) = self.sessions[session_index].continuous.as_mut() {
            let pending = &mut self.pending;
            let maximum = self.limits.max_pending_events;
            decoder
                .receive_datagram(datagram, |event| {
                    if pending.len() == maximum {
                        return Err(Error::TooManyRecords {
                            actual: pending.len().saturating_add(1),
                            maximum,
                        });
                    }
                    pending.push(match event {
                        ContinuousReceiveEvent::Record(record) => SessionEvent::ContinuousRecord {
                            identity,
                            record: record.clone(),
                        },
                        ContinuousReceiveEvent::Gap(gap) => SessionEvent::Gap { identity, gap },
                    });
                    Ok(())
                })
                .map_err(|error| match error {
                    ReceiveError::Stream(error) | ReceiveError::Consumer(error) => {
                        ReceiveError::Stream(error)
                    }
                })?;
        } else {
            self.stats.datagrams_before_manifest =
                self.stats.datagrams_before_manifest.saturating_add(1);
        }
        self.drain_events(&mut emit)
    }

    pub fn drain_events<E>(
        &mut self,
        emit: &mut impl FnMut(SessionEvent) -> core::result::Result<(), E>,
    ) -> core::result::Result<(), ReceiveError<E>> {
        while let Some(event) = self.pending.first().cloned() {
            emit(event).map_err(ReceiveError::Consumer)?;
            self.pending.remove(0);
        }
        Ok(())
    }

    fn receive_finite(&mut self, session_index: usize, datagram: &[u8]) -> Result<()> {
        let mut recovered = Vec::new();
        self.sessions[session_index]
            .finite
            .receive_datagram(datagram, |record| {
                recovered.push(record.clone());
                Ok::<(), core::convert::Infallible>(())
            })
            .map_err(|error| match error {
                ReceiveError::Stream(error) => error,
                ReceiveError::Consumer(never) => match never {},
            })?;

        for record in recovered {
            if record.decoded()?.kind == crate::RecordKind::Manifest {
                let manifest = SessionManifest::decode_record(record.bytes())?;
                if manifest.identity != self.sessions[session_index].identity {
                    return Err(Error::InconsistentObject);
                }
                if let Some(existing) = &self.sessions[session_index].manifest {
                    if existing != &manifest {
                        return Err(Error::InconsistentObject);
                    }
                    continue;
                }
                let continuous = self.decoder_from_manifest(&manifest)?;
                self.sessions[session_index].continuous = Some(continuous);
                self.sessions[session_index].manifest = Some(manifest.clone());
                self.stats.manifests_accepted = self.stats.manifests_accepted.saturating_add(1);
                self.push_event(SessionEvent::Manifest(manifest))?;
            } else {
                self.push_event(SessionEvent::Object {
                    identity: self.sessions[session_index].identity,
                    record,
                })?;
            }
        }
        Ok(())
    }

    fn decoder_from_manifest(
        &self,
        manifest: &SessionManifest,
    ) -> Result<ContinuousDecoder<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS, MAX_EQUATIONS>> {
        if usize::from(manifest.plan.symbol_size) > MAX_SYMBOL_SIZE
            || usize::from(manifest.plan.continuous.window_symbols) > MAX_WINDOW_SYMBOLS
            || manifest.plan.max_record_bytes > self.limits.max_record_bytes as u64
            || usize::from(manifest.plan.continuous.window_symbols)
                > self.limits.max_buffered_records
        {
            return Err(Error::InvalidConfig(
                "session manifest exceeds receiver-local limits",
            ));
        }
        ContinuousDecoder::new(
            manifest.identity,
            Lane::ReplayCritical,
            manifest.plan.rlc_config()?,
            self.limits.equation_capacity,
            0,
            ReceiverLimits::new(
                manifest.plan.max_record_bytes as usize,
                self.limits.max_buffered_records,
            ),
        )
    }

    fn push_event(&mut self, event: SessionEvent) -> Result<()> {
        if self.pending.len() == self.limits.max_pending_events {
            return Err(Error::TooManyRecords {
                actual: self.pending.len().saturating_add(1),
                maximum: self.limits.max_pending_events,
            });
        }
        self.pending.push(event);
        Ok(())
    }
}
