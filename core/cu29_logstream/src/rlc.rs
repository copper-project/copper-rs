use crate::{
    DecodedRecord, Error, FecScheme, FecSymbolKind, Lane, PACKET_HEADER_LEN, RecordKind, Result,
    WireHeader, WirePacket, decode_record, encode_packet_into,
};
use alloc::{boxed::Box, vec, vec::Vec};
use bincode::{Decode, Encode};
use core::fmt::{Display, Formatter};
use cu_fec::{
    DensityThreshold, EncodingSymbolId, Field, RepairParameters, RepairPayloadId, RlcConfig,
    RlcDecoder, RlcEncoder, SourcePayloadId, SourceStatus,
};

const FRAGMENT_MAGIC: [u8; 4] = *b"CUFR";
const FRAGMENT_VERSION: u8 = 1;
pub(crate) const FRAGMENT_HEADER_LEN: usize = 32;

/// Identity shared by one sender's continuous stream.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Encode, Decode)]
pub struct StreamIdentity {
    pub session_id: [u8; 16],
    pub sender_id: u32,
}

/// Receiver-side bounds for one continuous stream.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ReceiverLimits {
    /// Largest accepted framed semantic record.
    pub max_record_bytes: usize,
    /// Maximum number of incomplete and completed-but-ordered records retained.
    /// This must be at least the active RLC window size.
    pub max_buffered_records: usize,
}

impl ReceiverLimits {
    pub const fn new(max_record_bytes: usize, max_buffered_records: usize) -> Self {
        Self {
            max_record_bytes,
            max_buffered_records,
        }
    }
}

/// Why a range of CopperLists can no longer be reconstructed exactly.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GapReason {
    /// One or more required source symbols aged out of the configured RLC window.
    RlcWindowExpired,
    /// History unavailable before a verified late-join boundary.
    LateJoin,
    /// Recovery advanced to a verified recovery point after an outage.
    RecoveryPoint,
    /// The caller finalized the session before the range could be recovered.
    SessionEnded,
}

/// An inclusive range of unrecoverable CopperList identifiers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CopperListGap {
    /// First missing CopperList identifier, inclusive.
    pub first_id: u64,
    /// Last missing CopperList identifier, inclusive.
    pub last_id: u64,
    /// Condition that made this range definitive.
    pub reason: GapReason,
}

/// Ordered output from a continuous receiver.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ContinuousReceiveEvent<'a> {
    /// One complete, digest-verified semantic record.
    Record(&'a RecoveredRecord),
    /// An inclusive CopperList range that can no longer be recovered.
    Gap(CopperListGap),
}

/// Separates malformed stream input from a downstream consumer failure.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ReceiveError<E> {
    /// Packet, FEC, assembly, or semantic-record failure.
    Stream(Error),
    /// Failure returned by the supplied event consumer.
    Consumer(E),
}

impl<E> From<Error> for ReceiveError<E> {
    fn from(error: Error) -> Self {
        Self::Stream(error)
    }
}

impl<E: Display> Display for ReceiveError<E> {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Stream(error) => write!(formatter, "log stream receive failed: {error}"),
            Self::Consumer(error) => write!(formatter, "log stream consumer failed: {error}"),
        }
    }
}

#[cfg(feature = "std")]
impl<E: core::fmt::Debug + Display> std::error::Error for ReceiveError<E> {}

/// Current bounded receiver occupancy, exposed for monitoring and tests.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ReceiverOccupancy {
    /// Source symbols marked as processed inside the active RLC window.
    pub processed_symbols: usize,
    /// Records awaiting one or more source symbols.
    pub incomplete_records: usize,
    /// Complete records waiting for an earlier record or definitive gap.
    pub ready_records: usize,
}

/// Observed delivery and recovery behavior for one continuous stream.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ContinuousRecoveryStats {
    pub datagrams_seen: usize,
    pub valid_datagrams: usize,
    pub invalid_datagrams: usize,
    pub inconsistent_datagrams: usize,
    pub duplicate_symbols: usize,
    pub expired_datagrams: usize,
    pub source_symbols_received: usize,
    pub source_symbols_recovered: usize,
    pub repair_symbols_received: usize,
    pub innovative_repairs: usize,
    pub redundant_repairs: usize,
    pub records_recovered: usize,
    pub records_emitted: usize,
    pub records_expired: usize,
    pub gaps_emitted: usize,
    pub missing_copperlists: u64,
    pub peak_buffered_records: usize,
}

impl ContinuousRecoveryStats {
    /// Percentage of expected systematic symbols that did not arrive intact.
    pub const fn destruction_basis_points(self, expected_source_symbols: usize) -> u16 {
        if expected_source_symbols == 0 {
            return 0;
        }
        let destroyed = expected_source_symbols.saturating_sub(self.source_symbols_received);
        ((destroyed as u64 * 10_000) / expected_source_symbols as u64) as u16
    }

    /// Percentage of missing systematic symbols reconstructed by FEC.
    pub const fn source_recovery_basis_points(self, expected_source_symbols: usize) -> u16 {
        let missing = expected_source_symbols.saturating_sub(self.source_symbols_received);
        if missing == 0 {
            return 10_000;
        }
        let recovered = if self.source_symbols_recovered < missing {
            self.source_symbols_recovered
        } else {
            missing
        };
        ((recovered as u64 * 10_000) / missing as u64) as u16
    }

    /// Percentage of expected semantic records reconstructed and digest-verified.
    pub const fn semantic_recovery_basis_points(self, expected_records: usize) -> u16 {
        if expected_records == 0 {
            return 0;
        }
        let recovered = if self.records_recovered < expected_records {
            self.records_recovered
        } else {
            expected_records
        };
        ((recovered as u64 * 10_000) / expected_records as u64) as u16
    }
}

/// One fully assembled and digest-verified semantic record.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RecoveredRecord {
    bytes: Vec<u8>,
}

impl RecoveredRecord {
    pub(crate) fn from_bytes(bytes: Vec<u8>) -> Self {
        Self { bytes }
    }

    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }

    pub fn decoded(&self) -> Result<DecodedRecord<'_>> {
        decode_record(&self.bytes)
    }
}

/// Stateful RFC 8681 encoder for a continuous CopperList lane.
///
/// Record serialization, fragmentation, hashing, FEC, and packet allocation run
/// on the semantic-output worker, never on Copper's real-time task path.
pub struct ContinuousEncoder<const MAX_SYMBOL_SIZE: usize, const MAX_WINDOW_SYMBOLS: usize> {
    identity: StreamIdentity,
    lane: Lane,
    max_record_bytes: usize,
    packet_sequence: u64,
    fec: RlcEncoder<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS>,
}

impl<const MAX_SYMBOL_SIZE: usize, const MAX_WINDOW_SYMBOLS: usize>
    ContinuousEncoder<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS>
{
    pub fn new(
        identity: StreamIdentity,
        first_packet_sequence: u64,
        lane: Lane,
        config: RlcConfig,
        max_record_bytes: usize,
        initial_esi: EncodingSymbolId,
    ) -> Result<Self> {
        validate_config(config, max_record_bytes)?;
        Ok(Self {
            identity,
            lane,
            max_record_bytes,
            packet_sequence: first_packet_sequence,
            fec: RlcEncoder::new(config, initial_esi)?,
        })
    }

    pub const fn config(&self) -> RlcConfig {
        self.fec.config()
    }

    /// Fragments and emits one already-framed CopperList record as systematic symbols.
    pub fn push_record(&mut self, record: &[u8], output: &mut Vec<Vec<u8>>) -> Result<usize> {
        let mut datagram = vec![0; PACKET_HEADER_LEN + self.config().symbol_size()];
        self.push_record_with(record, &mut datagram, |packet| {
            output.push(packet.to_vec());
            Ok(())
        })
    }

    /// Fragments a record and emits each systematic packet immediately from
    /// reusable caller-owned storage.
    pub fn push_record_with(
        &mut self,
        record: &[u8],
        datagram: &mut [u8],
        mut emit: impl FnMut(&[u8]) -> Result<()>,
    ) -> Result<usize> {
        self.push_record_inner(record, datagram, &mut emit, None)
            .map(|(source_symbols, _)| source_symbols)
    }

    pub(crate) fn push_record_with_repairs(
        &mut self,
        record: &[u8],
        datagram: &mut [u8],
        emit: &mut impl FnMut(&[u8]) -> Result<()>,
        repair_schedule: &mut RepairSchedule,
    ) -> Result<(usize, usize)> {
        self.push_record_inner(record, datagram, emit, Some(repair_schedule))
    }

    fn push_record_inner(
        &mut self,
        record: &[u8],
        datagram: &mut [u8],
        emit: &mut impl FnMut(&[u8]) -> Result<()>,
        mut repair_schedule: Option<&mut RepairSchedule>,
    ) -> Result<(usize, usize)> {
        let decoded = decode_record(record)?;
        if decoded.kind != RecordKind::CopperList {
            return Err(Error::InconsistentObject);
        }
        if record.len() > self.max_record_bytes {
            return Err(Error::ObjectTooLarge {
                actual: record.len() as u64,
                maximum: self.max_record_bytes as u64,
            });
        }
        let record_len = u32::try_from(record.len())
            .map_err(|_| Error::InvalidConfig("record length exceeds u32"))?;
        let fragment_capacity = fragment_capacity(self.config())?;
        let fragment_count = record.len().div_ceil(fragment_capacity);
        let fragment_count = u32::try_from(fragment_count)
            .map_err(|_| Error::InvalidConfig("fragment count exceeds u32"))?;
        let mut symbol = [0_u8; MAX_SYMBOL_SIZE];
        let mut repair_symbols = 0usize;

        for (fragment_index, chunk) in record.chunks(fragment_capacity).enumerate() {
            let active_symbol = &mut symbol[..self.config().symbol_size()];
            active_symbol.fill(0);
            encode_fragment_header(
                active_symbol,
                decoded.kind,
                decoded.object_id,
                record_len,
                fragment_index as u32,
                fragment_count,
                chunk.len() as u16,
            );
            active_symbol[FRAGMENT_HEADER_LEN..FRAGMENT_HEADER_LEN + chunk.len()]
                .copy_from_slice(chunk);
            let esi = self.fec.push_source(active_symbol)?;
            let mut fec_metadata = [0_u8; 12];
            fec_metadata[..4].copy_from_slice(&SourcePayloadId::new(esi).to_bytes());
            let header = self.packet_header(
                decoded.kind,
                decoded.object_id,
                fragment_count,
                FecSymbolKind::Source,
                fec_metadata,
            );
            self.emit_packet_with(header, active_symbol, datagram, emit)?;
            if let Some(schedule) = repair_schedule.as_mut() {
                schedule.source_symbols_since_repair =
                    schedule.source_symbols_since_repair.saturating_add(1);
                if schedule.source_symbols_since_repair >= schedule.every {
                    schedule.source_symbols_since_repair -= schedule.every;
                    self.push_repair_with(
                        RepairParameters::new(schedule.next_repair_key, schedule.density),
                        datagram,
                        &mut *emit,
                    )?;
                    schedule.next_repair_key = schedule.next_repair_key.wrapping_add(1);
                    repair_symbols = repair_symbols.saturating_add(1);
                }
            }
        }
        Ok((fragment_count as usize, repair_symbols))
    }

    /// Emits one repair symbol over the encoder's current sliding window.
    pub fn push_repair(
        &mut self,
        parameters: RepairParameters,
        output: &mut Vec<Vec<u8>>,
    ) -> Result<RepairPayloadId> {
        let mut datagram = vec![0; PACKET_HEADER_LEN + self.config().symbol_size()];
        self.push_repair_with(parameters, &mut datagram, |packet| {
            output.push(packet.to_vec());
            Ok(())
        })
    }

    /// Emits one repair packet immediately from reusable caller-owned storage.
    pub fn push_repair_with(
        &mut self,
        parameters: RepairParameters,
        datagram: &mut [u8],
        mut emit: impl FnMut(&[u8]) -> Result<()>,
    ) -> Result<RepairPayloadId> {
        let mut symbol = [0_u8; MAX_SYMBOL_SIZE];
        let active_symbol = &mut symbol[..self.config().symbol_size()];
        let id = self.fec.encode_repair(parameters, active_symbol)?;
        let mut fec_metadata = [0_u8; 12];
        fec_metadata[..8].copy_from_slice(&id.to_bytes());
        let header = self.packet_header(
            RecordKind::CopperList,
            0,
            0,
            FecSymbolKind::Repair,
            fec_metadata,
        );
        self.emit_packet_with(header, active_symbol, datagram, &mut emit)?;
        Ok(id)
    }

    fn packet_header(
        &self,
        record_kind: RecordKind,
        object_id: u64,
        fragment_count: u32,
        symbol_kind: FecSymbolKind,
        fec_metadata: [u8; 12],
    ) -> WireHeader {
        WireHeader {
            lane: self.lane,
            record_kind,
            fec_scheme: fec_scheme(self.config().field()),
            symbol_kind,
            session_id: self.identity.session_id,
            sender_id: self.identity.sender_id,
            packet_sequence: self.packet_sequence,
            object_id,
            fec_metadata,
            fragment_count,
        }
    }

    fn emit_packet_with(
        &mut self,
        header: WireHeader,
        payload: &[u8],
        datagram: &mut [u8],
        emit: &mut impl FnMut(&[u8]) -> Result<()>,
    ) -> Result<()> {
        let encoded = encode_packet_into(header, payload, datagram)?;
        emit(&datagram[..encoded])?;
        self.packet_sequence = self.packet_sequence.wrapping_add(1);
        Ok(())
    }
}

pub(crate) struct RepairSchedule {
    every: usize,
    source_symbols_since_repair: usize,
    next_repair_key: u16,
    density: DensityThreshold,
}

impl RepairSchedule {
    pub(crate) const fn new(every: usize, next_repair_key: u16, density: DensityThreshold) -> Self {
        Self {
            every,
            source_symbols_since_repair: 0,
            next_repair_key,
            density,
        }
    }
}

/// Stateful, bounded RFC 8681 decoder and ordered CopperList assembler.
///
/// Recovered records are delivered to the caller immediately and are never
/// retained in an ever-growing history. Exact gap reporting assumes the sender
/// serializes CopperLists in increasing identifier order, as generated Copper
/// runtimes do.
pub struct ContinuousDecoder<
    const MAX_SYMBOL_SIZE: usize,
    const MAX_WINDOW_SYMBOLS: usize,
    const MAX_EQUATIONS: usize,
> {
    identity: StreamIdentity,
    lane: Lane,
    limits: ReceiverLimits,
    fec: Box<RlcDecoder<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS, MAX_EQUATIONS>>,
    processed_symbols: Vec<EncodingSymbolId>,
    assemblies: Vec<RecordAssembly>,
    ready: Vec<ReadyRecord>,
    next_object_id: u64,
    observed_window_base: Option<EncodingSymbolId>,
    retention_floor: Option<EncodingSymbolId>,
    stats: ContinuousRecoveryStats,
}

impl<const MAX_SYMBOL_SIZE: usize, const MAX_WINDOW_SYMBOLS: usize, const MAX_EQUATIONS: usize>
    ContinuousDecoder<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS, MAX_EQUATIONS>
{
    pub fn new(
        identity: StreamIdentity,
        lane: Lane,
        config: RlcConfig,
        equation_capacity: usize,
        first_object_id: u64,
        limits: ReceiverLimits,
    ) -> Result<Self> {
        validate_config(config, limits.max_record_bytes)?;
        if limits.max_buffered_records < config.window_symbols() {
            return Err(Error::InvalidConfig(
                "buffered-record capacity must cover the RLC symbol window",
            ));
        }
        Ok(Self {
            identity,
            lane,
            limits,
            fec: Box::new(RlcDecoder::new(config, equation_capacity)?),
            processed_symbols: Vec::with_capacity(config.window_symbols()),
            assemblies: Vec::with_capacity(limits.max_buffered_records),
            ready: Vec::with_capacity(limits.max_buffered_records),
            next_object_id: first_object_id,
            observed_window_base: None,
            retention_floor: None,
            stats: ContinuousRecoveryStats::default(),
        })
    }

    pub const fn next_object_id(&self) -> u64 {
        self.next_object_id
    }

    /// Advances ordering at a verified semantic boundary, retaining the FEC
    /// window and already assembled records at or beyond that boundary.
    pub(crate) fn resume_at(&mut self, boundary: u64) {
        self.next_object_id = self.next_object_id.max(boundary);
        self.assemblies
            .retain(|record| record.object_id >= self.next_object_id);
        self.discard_stale_ready();
    }

    pub const fn stats(&self) -> ContinuousRecoveryStats {
        self.stats
    }

    pub fn occupancy(&self) -> ReceiverOccupancy {
        ReceiverOccupancy {
            processed_symbols: self.processed_symbols.len(),
            incomplete_records: self.assemblies.len(),
            ready_records: self.ready.len(),
        }
    }

    /// Retries delivery of ordered records or gaps already ready in receiver state.
    ///
    /// This is useful after a consumer error and when shutting down a transport
    /// after its final datagram. A consumer failure leaves the current event
    /// pending so a later call can retry it.
    pub fn drain_events<E>(
        &mut self,
        mut emit: impl FnMut(ContinuousReceiveEvent<'_>) -> core::result::Result<(), E>,
    ) -> core::result::Result<(), ReceiveError<E>> {
        self.expire_and_emit(&mut emit)
    }

    /// Finalizes ordered delivery through an inclusive CopperList identifier.
    ///
    /// Any unresolved record or wholly absent identifier through `last_object_id`
    /// is emitted as a terminal gap. Calling this method is an explicit promise
    /// that no more useful repair symbols for that range will arrive.
    pub fn finish_through<E>(
        &mut self,
        last_object_id: u64,
        mut emit: impl FnMut(ContinuousReceiveEvent<'_>) -> core::result::Result<(), E>,
    ) -> core::result::Result<(), ReceiveError<E>> {
        self.process_new_symbols()?;
        self.emit_through(last_object_id, GapReason::SessionEnded, &mut emit)?;
        let unresolved = self
            .assemblies
            .iter()
            .filter(|assembly| assembly.object_id <= last_object_id)
            .count();
        self.assemblies
            .retain(|assembly| assembly.object_id > last_object_id);
        self.stats.records_expired = self.stats.records_expired.saturating_add(unresolved);
        self.discard_stale_ready();
        Ok(())
    }

    /// Validates and submits one datagram. Corrupt or foreign packets are counted
    /// and ignored before they can contaminate the FEC decoder. Recovered records
    /// and definitive gaps are emitted in CopperList-id order.
    pub fn receive_datagram<E>(
        &mut self,
        datagram: &[u8],
        mut emit: impl FnMut(ContinuousReceiveEvent<'_>) -> core::result::Result<(), E>,
    ) -> core::result::Result<(), ReceiveError<E>> {
        self.stats.datagrams_seen = self.stats.datagrams_seen.saturating_add(1);
        let packet = match WirePacket::decode(datagram) {
            Ok(packet) => packet,
            Err(_) => {
                self.stats.invalid_datagrams = self.stats.invalid_datagrams.saturating_add(1);
                return Ok(());
            }
        };
        if packet.header.session_id != self.identity.session_id
            || packet.header.sender_id != self.identity.sender_id
            || packet.header.lane != self.lane
            || packet.header.fec_scheme != fec_scheme(self.fec.config().field())
            || packet.header.record_kind != RecordKind::CopperList
        {
            self.stats.inconsistent_datagrams = self.stats.inconsistent_datagrams.saturating_add(1);
            return Ok(());
        }
        if packet.payload.len() != self.fec.config().symbol_size() {
            self.stats.invalid_datagrams = self.stats.invalid_datagrams.saturating_add(1);
            return Ok(());
        }

        let accepted = match packet.header.symbol_kind {
            FecSymbolKind::Source => self.receive_source_packet(&packet)?,
            FecSymbolKind::Repair => self.receive_repair_packet(&packet)?,
        };
        if !accepted {
            return Ok(());
        }
        self.stats.valid_datagrams = self.stats.valid_datagrams.saturating_add(1);
        self.update_retention_floor();
        self.expire_and_emit(&mut emit)?;
        self.process_new_symbols()?;
        self.expire_and_emit(&mut emit)
    }

    fn receive_source_packet(&mut self, packet: &WirePacket) -> Result<bool> {
        if packet.header.fec_metadata[4..]
            .iter()
            .any(|byte| *byte != 0)
        {
            self.stats.invalid_datagrams = self.stats.invalid_datagrams.saturating_add(1);
            return Ok(false);
        }
        let id = SourcePayloadId::from_bytes(packet.header.fec_metadata[..4].try_into().unwrap());
        let fragment = match decode_fragment(&packet.payload, fragment_capacity(self.fec.config())?)
        {
            Ok(fragment) => fragment,
            Err(_) => {
                self.stats.invalid_datagrams = self.stats.invalid_datagrams.saturating_add(1);
                return Ok(false);
            }
        };
        if packet.header.object_id != fragment.object_id
            || packet.header.fragment_count as usize != fragment.fragment_count
        {
            self.stats.inconsistent_datagrams = self.stats.inconsistent_datagrams.saturating_add(1);
            return Ok(false);
        }
        let report = match self.fec.receive_source(id.esi(), &packet.payload) {
            Ok(report) => report,
            Err(cu_fec::Error::WindowExpired) => {
                self.stats.expired_datagrams = self.stats.expired_datagrams.saturating_add(1);
                return Ok(false);
            }
            Err(error) => return Err(error.into()),
        };
        if report.status() == SourceStatus::Duplicate {
            self.stats.duplicate_symbols = self.stats.duplicate_symbols.saturating_add(1);
            return Ok(false);
        }
        self.stats.source_symbols_received = self.stats.source_symbols_received.saturating_add(1);
        self.stats.source_symbols_recovered = self
            .stats
            .source_symbols_recovered
            .saturating_add(report.recovered());
        Ok(true)
    }

    fn receive_repair_packet(&mut self, packet: &WirePacket) -> Result<bool> {
        if packet.header.object_id != 0
            || packet.header.fragment_count != 0
            || packet.header.fec_metadata[8..]
                .iter()
                .any(|byte| *byte != 0)
        {
            self.stats.invalid_datagrams = self.stats.invalid_datagrams.saturating_add(1);
            return Ok(false);
        }
        let raw_id: [u8; 8] = packet.header.fec_metadata[..8].try_into().unwrap();
        let id = RepairPayloadId::from_bytes(raw_id)?;
        let report = match self.fec.receive_repair(id, &packet.payload) {
            Ok(report) => report,
            Err(cu_fec::Error::WindowExpired) => {
                self.stats.expired_datagrams = self.stats.expired_datagrams.saturating_add(1);
                return Ok(false);
            }
            Err(error) => return Err(error.into()),
        };
        self.stats.repair_symbols_received = self.stats.repair_symbols_received.saturating_add(1);
        if report.is_innovative() {
            self.stats.innovative_repairs = self.stats.innovative_repairs.saturating_add(1);
        } else {
            self.stats.redundant_repairs = self.stats.redundant_repairs.saturating_add(1);
        }
        self.stats.source_symbols_recovered = self
            .stats
            .source_symbols_recovered
            .saturating_add(report.recovered());
        Ok(true)
    }

    fn process_new_symbols(&mut self) -> Result<()> {
        while let Some(esi) = self
            .fec
            .known_symbols()
            .map(|(esi, _)| esi)
            .find(|esi| !self.processed_symbols.contains(esi))
        {
            let symbol = self
                .fec
                .symbol(esi)
                .expect("known symbol disappeared from the retained RLC window")
                .to_vec();
            self.process_fragment(esi, &symbol)?;
            self.processed_symbols.push(esi);
        }
        Ok(())
    }

    fn process_fragment(&mut self, esi: EncodingSymbolId, symbol: &[u8]) -> Result<()> {
        let fragment = decode_fragment(symbol, fragment_capacity(self.fec.config())?)?;
        if fragment.record_len > self.limits.max_record_bytes {
            return Err(Error::ObjectTooLarge {
                actual: fragment.record_len as u64,
                maximum: self.limits.max_record_bytes as u64,
            });
        }
        let assembly_index = match self.assemblies.iter().position(|assembly| {
            assembly.kind == fragment.kind && assembly.object_id == fragment.object_id
        }) {
            Some(index) => index,
            None => {
                let count = self
                    .assemblies
                    .len()
                    .saturating_add(self.ready.len())
                    .saturating_add(1);
                if count > self.limits.max_buffered_records {
                    return Err(Error::TooManyRecords {
                        actual: count,
                        maximum: self.limits.max_buffered_records,
                    });
                }
                self.assemblies.push(RecordAssembly::new(esi, &fragment));
                self.observe_peak_buffered_records();
                self.assemblies.len() - 1
            }
        };
        let complete = self.assemblies[assembly_index].insert(esi, &fragment)?;
        if !complete {
            return Ok(());
        }

        let assembly = self.assemblies.swap_remove(assembly_index);
        let decoded = decode_record(&assembly.bytes)?;
        if decoded.kind != assembly.kind || decoded.object_id != assembly.object_id {
            return Err(Error::InconsistentObject);
        }
        self.ready.push(ReadyRecord {
            object_id: assembly.object_id,
            first_esi: assembly.first_esi,
            record: RecoveredRecord::from_bytes(assembly.bytes),
        });
        self.stats.records_recovered = self.stats.records_recovered.saturating_add(1);
        self.observe_peak_buffered_records();
        Ok(())
    }

    fn expire_and_emit<E>(
        &mut self,
        emit: &mut impl FnMut(ContinuousReceiveEvent<'_>) -> core::result::Result<(), E>,
    ) -> core::result::Result<(), ReceiveError<E>> {
        let Some((base_esi, _)) = self.fec.window() else {
            return Ok(());
        };
        self.processed_symbols
            .retain(|esi| !esi_is_before(*esi, base_esi));

        if let Some(retention_floor) = self.retention_floor {
            loop {
                let expired = self
                    .assemblies
                    .iter()
                    .enumerate()
                    .filter(|(_, assembly)| assembly.has_expired_fragment(retention_floor))
                    .min_by_key(|(_, assembly)| assembly.object_id)
                    .map(|(index, _)| index);
                let Some(index) = expired else {
                    break;
                };
                let object_id = self.assemblies[index].object_id;
                self.emit_through(object_id, GapReason::RlcWindowExpired, emit)?;
                self.assemblies.swap_remove(index);
                self.stats.records_expired = self.stats.records_expired.saturating_add(1);
            }
        }

        self.emit_ready(self.retention_floor, emit)
    }

    fn emit_ready<E>(
        &mut self,
        retention_floor: Option<EncodingSymbolId>,
        emit: &mut impl FnMut(ContinuousReceiveEvent<'_>) -> core::result::Result<(), E>,
    ) -> core::result::Result<(), ReceiveError<E>> {
        loop {
            self.discard_stale_ready();
            if self.emit_next_ready(emit)? {
                continue;
            }

            let Some(retention_floor) = retention_floor else {
                return Ok(());
            };
            let expired_boundary = self
                .ready
                .iter()
                .filter(|record| {
                    record.object_id > self.next_object_id
                        && !esi_is_after(record.first_esi, retention_floor)
                })
                .min_by_key(|record| record.object_id)
                .map(|record| record.object_id);
            let Some(boundary) = expired_boundary else {
                return Ok(());
            };
            self.emit_gap(boundary - 1, GapReason::RlcWindowExpired, emit)?;
        }
    }

    fn emit_through<E>(
        &mut self,
        last_id: u64,
        reason: GapReason,
        emit: &mut impl FnMut(ContinuousReceiveEvent<'_>) -> core::result::Result<(), E>,
    ) -> core::result::Result<(), ReceiveError<E>> {
        while self.next_object_id <= last_id {
            self.discard_stale_ready();
            if self.emit_next_ready(emit)? {
                continue;
            }
            let next_ready = self
                .ready
                .iter()
                .map(|record| record.object_id)
                .filter(|object_id| *object_id <= last_id)
                .min();
            let gap_last = next_ready.map_or(last_id, |object_id| object_id - 1);
            self.emit_gap(gap_last, reason, emit)?;
        }
        Ok(())
    }

    fn emit_next_ready<E>(
        &mut self,
        emit: &mut impl FnMut(ContinuousReceiveEvent<'_>) -> core::result::Result<(), E>,
    ) -> core::result::Result<bool, ReceiveError<E>> {
        let Some(index) = self
            .ready
            .iter()
            .position(|record| record.object_id == self.next_object_id)
        else {
            return Ok(false);
        };
        emit(ContinuousReceiveEvent::Record(&self.ready[index].record))
            .map_err(ReceiveError::Consumer)?;
        self.ready.swap_remove(index);
        self.stats.records_emitted = self.stats.records_emitted.saturating_add(1);
        self.next_object_id = self.next_object_id.wrapping_add(1);
        Ok(true)
    }

    fn emit_gap<E>(
        &mut self,
        last_id: u64,
        reason: GapReason,
        emit: &mut impl FnMut(ContinuousReceiveEvent<'_>) -> core::result::Result<(), E>,
    ) -> core::result::Result<(), ReceiveError<E>> {
        if last_id < self.next_object_id {
            return Ok(());
        }
        let first_id = self.next_object_id;
        emit(ContinuousReceiveEvent::Gap(CopperListGap {
            first_id,
            last_id,
            reason,
        }))
        .map_err(ReceiveError::Consumer)?;
        self.stats.gaps_emitted = self.stats.gaps_emitted.saturating_add(1);
        self.stats.missing_copperlists = self
            .stats
            .missing_copperlists
            .saturating_add(last_id - first_id + 1);
        self.next_object_id = last_id.wrapping_add(1);
        Ok(())
    }

    fn discard_stale_ready(&mut self) {
        self.ready
            .retain(|record| record.object_id >= self.next_object_id);
    }

    fn observe_peak_buffered_records(&mut self) {
        self.stats.peak_buffered_records = self
            .stats
            .peak_buffered_records
            .max(self.assemblies.len().saturating_add(self.ready.len()));
    }

    fn update_retention_floor(&mut self) {
        let Some((base_esi, _)) = self.fec.window() else {
            return;
        };
        if self
            .observed_window_base
            .is_some_and(|previous| esi_is_after(base_esi, previous))
        {
            self.retention_floor = Some(base_esi);
        }
        self.observed_window_base = Some(base_esi);
    }
}

#[derive(Clone, Debug)]
struct ReadyRecord {
    object_id: u64,
    first_esi: EncodingSymbolId,
    record: RecoveredRecord,
}

#[derive(Debug)]
struct Fragment<'a> {
    kind: RecordKind,
    object_id: u64,
    record_len: usize,
    fragment_index: usize,
    fragment_count: usize,
    fragment_capacity: usize,
    payload: &'a [u8],
}

struct RecordAssembly {
    kind: RecordKind,
    object_id: u64,
    first_esi: EncodingSymbolId,
    bytes: Vec<u8>,
    received: Vec<bool>,
    received_count: usize,
    fragment_capacity: usize,
}

impl RecordAssembly {
    fn new(esi: EncodingSymbolId, fragment: &Fragment<'_>) -> Self {
        Self {
            kind: fragment.kind,
            object_id: fragment.object_id,
            first_esi: esi.wrapping_sub(fragment.fragment_index as u32),
            bytes: vec![0; fragment.record_len],
            received: vec![false; fragment.fragment_count],
            received_count: 0,
            fragment_capacity: fragment.fragment_capacity,
        }
    }

    fn insert(&mut self, esi: EncodingSymbolId, fragment: &Fragment<'_>) -> Result<bool> {
        if self.bytes.len() != fragment.record_len || self.received.len() != fragment.fragment_count
        {
            return Err(Error::InvalidFragment("record geometry changed"));
        }
        if self.first_esi.wrapping_add(fragment.fragment_index as u32) != esi {
            return Err(Error::InvalidFragment(
                "fragment ESI does not match record geometry",
            ));
        }
        let offset = fragment
            .fragment_index
            .checked_mul(self.fragment_capacity)
            .ok_or(Error::InvalidFragment("fragment offset overflow"))?;
        let end = offset
            .checked_add(fragment.payload.len())
            .ok_or(Error::InvalidFragment("fragment length overflow"))?;
        if end > self.bytes.len() {
            return Err(Error::InvalidFragment("fragment exceeds record"));
        }
        if self.received[fragment.fragment_index] {
            if self.bytes[offset..end] != *fragment.payload {
                return Err(Error::InvalidFragment("duplicate fragment differs"));
            }
            return Ok(self.received_count == self.received.len());
        }
        self.bytes[offset..end].copy_from_slice(fragment.payload);
        self.received[fragment.fragment_index] = true;
        self.received_count += 1;
        Ok(self.received_count == self.received.len())
    }

    fn has_expired_fragment(&self, base_esi: EncodingSymbolId) -> bool {
        self.received.iter().enumerate().any(|(index, received)| {
            !received && esi_is_before(self.first_esi.wrapping_add(index as u32), base_esi)
        })
    }
}

fn esi_is_before(candidate: EncodingSymbolId, reference: EncodingSymbolId) -> bool {
    (candidate.get().wrapping_sub(reference.get()) as i32) < 0
}

fn esi_is_after(candidate: EncodingSymbolId, reference: EncodingSymbolId) -> bool {
    (candidate.get().wrapping_sub(reference.get()) as i32) > 0
}

fn validate_config(config: RlcConfig, max_record_bytes: usize) -> Result<()> {
    let _ = fragment_capacity(config)?;
    if max_record_bytes == 0 {
        return Err(Error::InvalidConfig("maximum record size must be nonzero"));
    }
    if max_record_bytes > u32::MAX as usize {
        return Err(Error::InvalidConfig("maximum record size exceeds u32"));
    }
    Ok(())
}

fn fragment_capacity(config: RlcConfig) -> Result<usize> {
    config
        .symbol_size()
        .checked_sub(FRAGMENT_HEADER_LEN)
        .filter(|capacity| *capacity > 0 && *capacity <= u16::MAX as usize)
        .ok_or(Error::InvalidConfig(
            "RLC symbol cannot hold the fragment header and payload",
        ))
}

fn fec_scheme(field: Field) -> FecScheme {
    match field {
        Field::Gf2 => FecScheme::RlcGf2,
        Field::Gf256 => FecScheme::RlcGf256,
    }
}

fn encode_fragment_header(
    symbol: &mut [u8],
    kind: RecordKind,
    object_id: u64,
    record_len: u32,
    fragment_index: u32,
    fragment_count: u32,
    fragment_len: u16,
) {
    symbol[..4].copy_from_slice(&FRAGMENT_MAGIC);
    symbol[4] = FRAGMENT_VERSION;
    symbol[5] = kind as u8;
    symbol[6..8].fill(0);
    symbol[8..16].copy_from_slice(&object_id.to_be_bytes());
    symbol[16..20].copy_from_slice(&record_len.to_be_bytes());
    symbol[20..24].copy_from_slice(&fragment_index.to_be_bytes());
    symbol[24..28].copy_from_slice(&fragment_count.to_be_bytes());
    symbol[28..30].copy_from_slice(&fragment_len.to_be_bytes());
    symbol[30..32].fill(0);
}

fn decode_fragment(symbol: &[u8], fragment_capacity: usize) -> Result<Fragment<'_>> {
    if symbol.len() < FRAGMENT_HEADER_LEN || symbol[..4] != FRAGMENT_MAGIC {
        return Err(Error::InvalidFragment("missing fragment header"));
    }
    if symbol[4] != FRAGMENT_VERSION {
        return Err(Error::UnsupportedVersion(symbol[4]));
    }
    if symbol[6..8].iter().any(|byte| *byte != 0) || symbol[30..32].iter().any(|byte| *byte != 0) {
        return Err(Error::InvalidFragment("reserved bytes are nonzero"));
    }
    let kind = RecordKind::try_from(symbol[5])?;
    if kind != RecordKind::CopperList {
        return Err(Error::InvalidFragment(
            "continuous lane requires CopperList records",
        ));
    }
    let object_id = u64::from_be_bytes(symbol[8..16].try_into().unwrap());
    let record_len = u32::from_be_bytes(symbol[16..20].try_into().unwrap()) as usize;
    let fragment_index = u32::from_be_bytes(symbol[20..24].try_into().unwrap()) as usize;
    let fragment_count = u32::from_be_bytes(symbol[24..28].try_into().unwrap()) as usize;
    let fragment_len = u16::from_be_bytes(symbol[28..30].try_into().unwrap()) as usize;
    if record_len == 0 || fragment_count == 0 || fragment_index >= fragment_count {
        return Err(Error::InvalidFragment("invalid fragment geometry"));
    }
    let expected_count = record_len.div_ceil(fragment_capacity);
    if fragment_count != expected_count {
        return Err(Error::InvalidFragment(
            "fragment count does not match record length",
        ));
    }
    let expected_len = if fragment_index + 1 == fragment_count {
        record_len - fragment_index * fragment_capacity
    } else {
        fragment_capacity
    };
    if fragment_len != expected_len || FRAGMENT_HEADER_LEN + fragment_len > symbol.len() {
        return Err(Error::InvalidFragment(
            "fragment length does not match geometry",
        ));
    }
    if symbol[FRAGMENT_HEADER_LEN + fragment_len..]
        .iter()
        .any(|byte| *byte != 0)
    {
        return Err(Error::InvalidFragment("fragment padding is nonzero"));
    }
    Ok(Fragment {
        kind,
        object_id,
        record_len,
        fragment_index,
        fragment_count,
        fragment_capacity,
        payload: &symbol[FRAGMENT_HEADER_LEN..FRAGMENT_HEADER_LEN + fragment_len],
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fragment_header_round_trips_and_rejects_padding() {
        let mut symbol = [0_u8; 64];
        encode_fragment_header(&mut symbol, RecordKind::CopperList, 42, 40, 1, 2, 8);
        symbol[FRAGMENT_HEADER_LEN..FRAGMENT_HEADER_LEN + 8].copy_from_slice(b"fragment");
        let decoded = decode_fragment(&symbol, 32).unwrap();
        assert_eq!(decoded.object_id, 42);
        assert_eq!(decoded.fragment_index, 1);
        assert_eq!(decoded.payload, b"fragment");

        symbol[63] = 1;
        assert_eq!(
            decode_fragment(&symbol, 32).unwrap_err(),
            Error::InvalidFragment("fragment padding is nonzero")
        );
    }
}
