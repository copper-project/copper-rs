//! Bounded finite-object transport using systematic RFC 6330 RaptorQ.

use crate::{
    Error, FecScheme, FecSymbolKind, Lane, PACKET_HEADER_LEN, ReceiveError, RecordKind,
    RecoveredRecord, Result, StreamIdentity, WireHeader, WirePacketRef, decode_record,
    encode_packet_into,
};
use alloc::{vec, vec::Vec};
use raptorq::{Decoder, Encoder, EncodingPacket, ObjectTransmissionInformation, PayloadId};

const RFC6330_MAX_TRANSFER_LENGTH: u64 = 942_574_504_275;
const RFC6330_MAX_SOURCE_SYMBOLS_PER_BLOCK: u64 = 56_403;

/// Sender policy for one finite-object lane.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FiniteObjectSenderConfig {
    pub identity: StreamIdentity,
    pub first_packet_sequence: u64,
    pub lane: Lane,
    /// RaptorQ symbol bytes, excluding the Copper wire header.
    pub symbol_size: u16,
    /// Hard limit checked before constructing the RaptorQ encoder.
    pub max_object_bytes: u64,
    /// Repair symbols emitted for every RaptorQ source block.
    pub repair_symbols_per_block: u32,
}

/// Receiver-local limits checked before any RaptorQ decoder is constructed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FiniteObjectLimits {
    pub max_object_bytes: u64,
    pub max_symbol_size: u16,
    pub max_concurrent_objects: usize,
}

impl FiniteObjectLimits {
    pub const fn new(
        max_object_bytes: u64,
        max_symbol_size: u16,
        max_concurrent_objects: usize,
    ) -> Self {
        Self {
            max_object_bytes,
            max_symbol_size,
            max_concurrent_objects,
        }
    }
}

/// Observed finite-object receive behavior.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FiniteObjectRecoveryStats {
    pub datagrams_seen: usize,
    pub valid_datagrams: usize,
    pub invalid_datagrams: usize,
    pub inconsistent_datagrams: usize,
    pub duplicate_symbols: usize,
    pub source_symbols_received: usize,
    pub repair_symbols_received: usize,
    pub objects_recovered: usize,
    pub peak_concurrent_objects: usize,
}

/// RaptorQ encoder for immutable framed semantic records.
///
/// Construction and encoding allocate and must run on a non-real-time output worker.
pub struct FiniteObjectEncoder {
    config: FiniteObjectSenderConfig,
    packet_sequence: u64,
}

impl FiniteObjectEncoder {
    pub fn new(config: FiniteObjectSenderConfig) -> Result<Self> {
        validate_sender_config(config)?;
        Ok(Self {
            packet_sequence: config.first_packet_sequence,
            config,
        })
    }

    pub const fn config(&self) -> FiniteObjectSenderConfig {
        self.config
    }

    /// Encodes one already-framed semantic record and appends its datagrams.
    pub fn push_record(&mut self, record: &[u8], output: &mut Vec<Vec<u8>>) -> Result<usize> {
        self.push_record_with(record, |datagram| {
            output.push(datagram.to_vec());
            Ok(())
        })
    }

    /// Encodes one record and emits each datagram immediately.
    pub fn push_record_with(
        &mut self,
        record: &[u8],
        mut emit: impl FnMut(&[u8]) -> Result<()>,
    ) -> Result<usize> {
        let decoded = decode_record(record)?;
        let record_len = u64::try_from(record.len())
            .map_err(|_| Error::InvalidConfig("finite object length exceeds u64"))?;
        if record_len > self.config.max_object_bytes {
            return Err(Error::ObjectTooLarge {
                actual: record_len,
                maximum: self.config.max_object_bytes,
            });
        }

        let encoder = Encoder::with_defaults(record, self.config.symbol_size);
        let oti = encoder.get_config().serialize();
        let datagram_len = PACKET_HEADER_LEN
            .checked_add(self.config.symbol_size as usize)
            .ok_or(Error::InvalidConfig(
                "finite-object datagram length overflow",
            ))?;
        let mut datagram = vec![0; datagram_len];
        let mut emitted = 0_usize;

        for block in encoder.get_block_encoders() {
            for packet in block.source_packets() {
                self.emit_packet(
                    decoded,
                    FecSymbolKind::Source,
                    oti,
                    packet,
                    &mut datagram,
                    &mut emit,
                )?;
                emitted = emitted.saturating_add(1);
            }
            for packet in block.repair_packets(0, self.config.repair_symbols_per_block) {
                self.emit_packet(
                    decoded,
                    FecSymbolKind::Repair,
                    oti,
                    packet,
                    &mut datagram,
                    &mut emit,
                )?;
                emitted = emitted.saturating_add(1);
            }
        }
        Ok(emitted)
    }

    fn emit_packet(
        &mut self,
        record: crate::DecodedRecord<'_>,
        symbol_kind: FecSymbolKind,
        oti: [u8; 12],
        packet: EncodingPacket,
        datagram: &mut [u8],
        emit: &mut impl FnMut(&[u8]) -> Result<()>,
    ) -> Result<()> {
        let (payload_id, payload) = packet.split();
        let header = WireHeader {
            lane: self.config.lane,
            record_kind: record.kind,
            fec_scheme: FecScheme::RaptorQ,
            symbol_kind,
            session_id: self.config.identity.session_id,
            sender_id: self.config.identity.sender_id,
            packet_sequence: self.packet_sequence,
            object_id: record.object_id,
            fec_metadata: oti,
            fragment_count: u32::from_be_bytes(payload_id.serialize()),
        };
        let encoded = encode_packet_into(header, &payload, datagram)?;
        emit(&datagram[..encoded])?;
        self.packet_sequence = self.packet_sequence.wrapping_add(1);
        Ok(())
    }
}

/// Bounded assembler for independently reorderable finite RaptorQ objects.
pub struct FiniteObjectDecoder {
    identity: StreamIdentity,
    lane: Lane,
    limits: FiniteObjectLimits,
    objects: Vec<ObjectDecoder>,
    ready: Vec<ReadyObject>,
    completed: Vec<CompletedObject>,
    stats: FiniteObjectRecoveryStats,
}

impl FiniteObjectDecoder {
    pub fn new(identity: StreamIdentity, lane: Lane, limits: FiniteObjectLimits) -> Result<Self> {
        validate_limits(limits)?;
        Ok(Self {
            identity,
            lane,
            limits,
            objects: Vec::with_capacity(limits.max_concurrent_objects),
            ready: Vec::with_capacity(limits.max_concurrent_objects),
            completed: Vec::with_capacity(limits.max_concurrent_objects),
            stats: FiniteObjectRecoveryStats::default(),
        })
    }

    pub const fn stats(&self) -> FiniteObjectRecoveryStats {
        self.stats
    }

    pub fn occupancy(&self) -> usize {
        self.objects.len().saturating_add(self.ready.len())
    }

    /// Accepts one datagram and emits a digest-verified record when its object completes.
    pub fn receive_datagram<E>(
        &mut self,
        datagram: &[u8],
        mut emit: impl FnMut(&RecoveredRecord) -> core::result::Result<(), E>,
    ) -> core::result::Result<(), ReceiveError<E>> {
        self.drain_records(&mut emit)?;
        let packet = match WirePacketRef::decode(datagram) {
            Ok(packet) => packet,
            Err(_) => {
                self.stats.datagrams_seen = self.stats.datagrams_seen.saturating_add(1);
                self.stats.invalid_datagrams = self.stats.invalid_datagrams.saturating_add(1);
                return Ok(());
            }
        };
        self.receive_packet(packet)?;
        self.drain_records(&mut emit)
    }

    pub(crate) fn receive_packet(&mut self, packet: WirePacketRef<'_>) -> Result<()> {
        self.stats.datagrams_seen = self.stats.datagrams_seen.saturating_add(1);
        if packet.header.session_id != self.identity.session_id
            || packet.header.sender_id != self.identity.sender_id
            || packet.header.lane != self.lane
            || packet.header.fec_scheme != FecScheme::RaptorQ
            || packet.header.record_kind == RecordKind::CopperList
        {
            self.stats.inconsistent_datagrams = self.stats.inconsistent_datagrams.saturating_add(1);
            return Ok(());
        }

        let oti = match validate_oti(packet.header.fec_metadata, self.limits) {
            Ok(oti) => oti,
            Err(_) => {
                self.stats.invalid_datagrams = self.stats.invalid_datagrams.saturating_add(1);
                return Ok(());
            }
        };
        if packet.payload.len() != oti.symbol_size() as usize {
            self.stats.invalid_datagrams = self.stats.invalid_datagrams.saturating_add(1);
            return Ok(());
        }

        let payload_id = PayloadId::deserialize(&packet.header.fragment_count.to_be_bytes());
        if payload_id.source_block_number() >= oti.source_blocks() {
            self.stats.invalid_datagrams = self.stats.invalid_datagrams.saturating_add(1);
            return Ok(());
        }
        if let Some(object) = self.ready.iter().find(|object| {
            object.kind == packet.header.record_kind && object.object_id == packet.header.object_id
        }) {
            if object.oti != oti {
                self.stats.inconsistent_datagrams =
                    self.stats.inconsistent_datagrams.saturating_add(1);
            } else {
                self.stats.duplicate_symbols = self.stats.duplicate_symbols.saturating_add(1);
            }
            return Ok(());
        }
        if let Some(object) = self.completed.iter().find(|object| {
            object.kind == packet.header.record_kind && object.object_id == packet.header.object_id
        }) {
            if object.oti != oti {
                self.stats.inconsistent_datagrams =
                    self.stats.inconsistent_datagrams.saturating_add(1);
            } else {
                self.stats.duplicate_symbols = self.stats.duplicate_symbols.saturating_add(1);
            }
            return Ok(());
        }
        let object_index = match self.objects.iter().position(|object| {
            object.kind == packet.header.record_kind && object.object_id == packet.header.object_id
        }) {
            Some(index) => {
                let object = &self.objects[index];
                if object.oti != oti {
                    self.stats.inconsistent_datagrams =
                        self.stats.inconsistent_datagrams.saturating_add(1);
                    return Ok(());
                }
                index
            }
            None => {
                let actual = self
                    .objects
                    .len()
                    .saturating_add(self.ready.len())
                    .saturating_add(1);
                if actual > self.limits.max_concurrent_objects {
                    return Err(Error::TooManyRecords {
                        actual,
                        maximum: self.limits.max_concurrent_objects,
                    });
                }
                self.objects.push(ObjectDecoder {
                    kind: packet.header.record_kind,
                    object_id: packet.header.object_id,
                    oti,
                    decoder: Decoder::new(oti),
                });
                self.stats.peak_concurrent_objects =
                    self.stats.peak_concurrent_objects.max(self.objects.len());
                self.objects.len() - 1
            }
        };

        match packet.header.symbol_kind {
            FecSymbolKind::Source => {
                self.stats.source_symbols_received =
                    self.stats.source_symbols_received.saturating_add(1);
            }
            FecSymbolKind::Repair => {
                self.stats.repair_symbols_received =
                    self.stats.repair_symbols_received.saturating_add(1);
            }
        }
        self.stats.valid_datagrams = self.stats.valid_datagrams.saturating_add(1);
        let completed = self.objects[object_index]
            .decoder
            .decode(EncodingPacket::new(payload_id, packet.payload.to_vec()));
        let Some(bytes) = completed else {
            return Ok(());
        };

        let object = self.objects.swap_remove(object_index);
        let record = RecoveredRecord::from_bytes(bytes)?;
        let decoded = record.decoded();
        if decoded.kind != object.kind || decoded.object_id != object.object_id {
            return Err(Error::InconsistentObject);
        }
        self.ready.push(ReadyObject {
            kind: object.kind,
            object_id: object.object_id,
            oti: object.oti,
            record,
        });
        self.stats.objects_recovered = self.stats.objects_recovered.saturating_add(1);
        Ok(())
    }

    /// Retries delivery of records retained after a consumer failure.
    pub fn drain_records<E>(
        &mut self,
        mut emit: impl FnMut(&RecoveredRecord) -> core::result::Result<(), E>,
    ) -> core::result::Result<(), ReceiveError<E>> {
        while let Some(ready) = self.ready.first() {
            emit(&ready.record).map_err(ReceiveError::Consumer)?;
            self.pop_record();
        }
        Ok(())
    }

    pub(crate) fn pop_record(&mut self) -> Option<RecoveredRecord> {
        if self.ready.is_empty() {
            return None;
        }
        let delivered = self.ready.remove(0);
        if self.completed.len() == self.limits.max_concurrent_objects {
            self.completed.remove(0);
        }
        self.completed.push(CompletedObject {
            kind: delivered.kind,
            object_id: delivered.object_id,
            oti: delivered.oti,
        });
        Some(delivered.record)
    }
}

struct ObjectDecoder {
    kind: RecordKind,
    object_id: u64,
    oti: ObjectTransmissionInformation,
    decoder: Decoder,
}

struct ReadyObject {
    kind: RecordKind,
    object_id: u64,
    oti: ObjectTransmissionInformation,
    record: RecoveredRecord,
}

struct CompletedObject {
    kind: RecordKind,
    object_id: u64,
    oti: ObjectTransmissionInformation,
}

fn validate_sender_config(config: FiniteObjectSenderConfig) -> Result<()> {
    if config.symbol_size == 0 {
        return Err(Error::InvalidConfig("RaptorQ symbol size must be nonzero"));
    }
    if config.max_object_bytes == 0 || config.max_object_bytes > RFC6330_MAX_TRANSFER_LENGTH {
        return Err(Error::InvalidConfig("invalid RaptorQ maximum object size"));
    }
    Ok(())
}

fn validate_limits(limits: FiniteObjectLimits) -> Result<()> {
    if limits.max_object_bytes == 0 || limits.max_object_bytes > RFC6330_MAX_TRANSFER_LENGTH {
        return Err(Error::InvalidConfig("invalid RaptorQ maximum object size"));
    }
    if limits.max_symbol_size == 0 {
        return Err(Error::InvalidConfig(
            "RaptorQ maximum symbol size must be nonzero",
        ));
    }
    if limits.max_concurrent_objects == 0 {
        return Err(Error::InvalidConfig(
            "finite-object receiver capacity must be nonzero",
        ));
    }
    Ok(())
}

fn validate_oti(
    encoded: [u8; 12],
    limits: FiniteObjectLimits,
) -> Result<ObjectTransmissionInformation> {
    if encoded[5] != 0 {
        return Err(Error::InvalidFecMetadata(
            "RaptorQ OTI reserved byte is nonzero",
        ));
    }
    let oti = ObjectTransmissionInformation::deserialize(&encoded);
    let transfer_length = oti.transfer_length();
    let symbol_size = oti.symbol_size();
    let source_blocks = oti.source_blocks();
    let sub_blocks = oti.sub_blocks();
    let alignment = oti.symbol_alignment();
    if transfer_length == 0 || transfer_length > limits.max_object_bytes {
        return Err(Error::ObjectTooLarge {
            actual: transfer_length,
            maximum: limits.max_object_bytes,
        });
    }
    if symbol_size == 0 || symbol_size > limits.max_symbol_size {
        return Err(Error::InvalidFecMetadata(
            "RaptorQ symbol size exceeds receiver limit",
        ));
    }
    if source_blocks == 0 || sub_blocks == 0 || alignment == 0 {
        return Err(Error::InvalidFecMetadata(
            "RaptorQ OTI contains a zero parameter",
        ));
    }
    if !symbol_size.is_multiple_of(alignment as u16) {
        return Err(Error::InvalidFecMetadata(
            "RaptorQ symbol alignment is invalid",
        ));
    }
    let total_source_symbols = transfer_length.div_ceil(symbol_size as u64);
    if source_blocks as u64 > total_source_symbols {
        return Err(Error::InvalidFecMetadata(
            "RaptorQ has more source blocks than source symbols",
        ));
    }
    if sub_blocks as u32 > u32::from(symbol_size / alignment as u16) {
        return Err(Error::InvalidFecMetadata(
            "RaptorQ sub-block count exceeds symbol geometry",
        ));
    }
    let largest_block = total_source_symbols.div_ceil(source_blocks as u64);
    if largest_block > RFC6330_MAX_SOURCE_SYMBOLS_PER_BLOCK {
        return Err(Error::InvalidFecMetadata(
            "RaptorQ source block is too large",
        ));
    }
    Ok(oti)
}
