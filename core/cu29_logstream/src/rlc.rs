use crate::record::decode_record;
use crate::{
    DecodedRecord, Error, FecScheme, FecSymbolKind, Lane, RecordKind, Result, WireHeader,
    WirePacket,
};
use alloc::{vec, vec::Vec};
use cu_fec::{
    EncodingSymbolId, Field, RepairParameters, RepairPayloadId, RlcConfig, RlcDecoder, RlcEncoder,
    SourcePayloadId,
};

const FRAGMENT_MAGIC: [u8; 4] = *b"CUFR";
const FRAGMENT_VERSION: u8 = 1;
const FRAGMENT_HEADER_LEN: usize = 32;

/// Identity shared by one sender's continuous stream.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StreamIdentity {
    pub session_id: [u8; 16],
    pub sender_id: u32,
}

/// Receiver-side bounds for one continuous stream.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ReceiverLimits {
    pub max_record_bytes: usize,
    pub max_incomplete_records: usize,
    pub max_source_symbols: usize,
    pub max_datagrams: usize,
}

impl ReceiverLimits {
    pub const fn new(
        max_record_bytes: usize,
        max_incomplete_records: usize,
        max_source_symbols: usize,
        max_datagrams: usize,
    ) -> Self {
        Self {
            max_record_bytes,
            max_incomplete_records,
            max_source_symbols,
            max_datagrams,
        }
    }
}

/// Observed delivery and recovery behavior for one continuous stream.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ContinuousRecoveryStats {
    pub datagrams_seen: usize,
    pub valid_datagrams: usize,
    pub invalid_datagrams: usize,
    pub inconsistent_datagrams: usize,
    pub duplicate_symbols: usize,
    pub source_symbols_received: usize,
    pub source_symbols_recovered: usize,
    pub repair_symbols_received: usize,
    pub innovative_repairs: usize,
    pub records_recovered: usize,
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
            self.emit_packet(header, active_symbol, output)?;
        }
        Ok(fragment_count as usize)
    }

    /// Emits one repair symbol over the encoder's current sliding window.
    pub fn push_repair(
        &mut self,
        parameters: RepairParameters,
        output: &mut Vec<Vec<u8>>,
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
        self.emit_packet(header, active_symbol, output)?;
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

    fn emit_packet(
        &mut self,
        header: WireHeader,
        payload: &[u8],
        output: &mut Vec<Vec<u8>>,
    ) -> Result<()> {
        let packet = WirePacket {
            header,
            payload: payload.to_vec(),
        };
        output.push(packet.encode()?);
        self.packet_sequence = self.packet_sequence.wrapping_add(1);
        Ok(())
    }
}

/// Stateful RFC 8681 decoder and partial-CopperList record assembler.
pub struct ContinuousDecoder<
    const MAX_SYMBOL_SIZE: usize,
    const MAX_WINDOW_SYMBOLS: usize,
    const MAX_EQUATIONS: usize,
> {
    identity: StreamIdentity,
    lane: Lane,
    limits: ReceiverLimits,
    fec: RlcDecoder<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS, MAX_EQUATIONS>,
    source_ids: Vec<EncodingSymbolId>,
    processed_symbols: Vec<EncodingSymbolId>,
    repair_ids: Vec<[u8; 8]>,
    assemblies: Vec<RecordAssembly>,
    recovered: Vec<RecoveredRecord>,
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
        limits: ReceiverLimits,
    ) -> Result<Self> {
        validate_config(config, limits.max_record_bytes)?;
        if limits.max_incomplete_records == 0
            || limits.max_source_symbols == 0
            || limits.max_datagrams == 0
        {
            return Err(Error::InvalidConfig("receiver limits must be nonzero"));
        }
        Ok(Self {
            identity,
            lane,
            limits,
            fec: RlcDecoder::new(config, equation_capacity)?,
            source_ids: Vec::new(),
            processed_symbols: Vec::new(),
            repair_ids: Vec::new(),
            assemblies: Vec::new(),
            recovered: Vec::new(),
            stats: ContinuousRecoveryStats::default(),
        })
    }

    pub const fn stats(&self) -> ContinuousRecoveryStats {
        self.stats
    }

    pub fn recovered_records(&self) -> &[RecoveredRecord] {
        &self.recovered
    }

    pub fn into_recovered_records(self) -> Vec<RecoveredRecord> {
        self.recovered
    }

    /// Validates and submits one datagram. Corrupt or foreign packets are counted
    /// and ignored before they can contaminate the FEC decoder.
    pub fn receive_datagram(&mut self, datagram: &[u8]) -> Result<()> {
        self.stats.datagrams_seen = self.stats.datagrams_seen.saturating_add(1);
        if self.stats.datagrams_seen > self.limits.max_datagrams {
            return Err(Error::TooManyDatagrams {
                actual: self.stats.datagrams_seen,
                maximum: self.limits.max_datagrams,
            });
        }
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
        self.process_new_symbols()
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
        if self.source_ids.contains(&id.esi()) {
            self.stats.duplicate_symbols = self.stats.duplicate_symbols.saturating_add(1);
            return Ok(false);
        }
        self.source_ids.push(id.esi());
        self.stats.source_symbols_received = self.stats.source_symbols_received.saturating_add(1);
        let report = self.fec.receive_source(id.esi(), &packet.payload)?;
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
        if self.repair_ids.contains(&raw_id) {
            self.stats.duplicate_symbols = self.stats.duplicate_symbols.saturating_add(1);
            return Ok(false);
        }
        let id = RepairPayloadId::from_bytes(raw_id)?;
        let report = self.fec.receive_repair(id, &packet.payload)?;
        self.repair_ids.push(raw_id);
        self.stats.repair_symbols_received = self.stats.repair_symbols_received.saturating_add(1);
        if report.is_innovative() {
            self.stats.innovative_repairs = self.stats.innovative_repairs.saturating_add(1);
        }
        self.stats.source_symbols_recovered = self
            .stats
            .source_symbols_recovered
            .saturating_add(report.recovered());
        Ok(true)
    }

    fn process_new_symbols(&mut self) -> Result<()> {
        let pending = self
            .fec
            .known_symbols()
            .filter(|(esi, _)| !self.processed_symbols.contains(esi))
            .map(|(esi, symbol)| (esi, symbol.to_vec()))
            .collect::<Vec<_>>();
        for (esi, symbol) in pending {
            let symbol_count = self.processed_symbols.len().saturating_add(1);
            if symbol_count > self.limits.max_source_symbols {
                return Err(Error::TooManySourceSymbols {
                    actual: symbol_count,
                    maximum: self.limits.max_source_symbols,
                });
            }
            self.process_fragment(&symbol)?;
            self.processed_symbols.push(esi);
        }
        Ok(())
    }

    fn process_fragment(&mut self, symbol: &[u8]) -> Result<()> {
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
                let count = self.assemblies.len().saturating_add(1);
                if count > self.limits.max_incomplete_records {
                    return Err(Error::TooManyRecords {
                        actual: count,
                        maximum: self.limits.max_incomplete_records,
                    });
                }
                self.assemblies.push(RecordAssembly::new(&fragment));
                self.assemblies.len() - 1
            }
        };
        let complete = self.assemblies[assembly_index].insert(&fragment)?;
        if !complete {
            return Ok(());
        }

        let assembly = self.assemblies.swap_remove(assembly_index);
        let decoded = decode_record(&assembly.bytes)?;
        if decoded.kind != assembly.kind || decoded.object_id != assembly.object_id {
            return Err(Error::InconsistentObject);
        }
        self.recovered.push(RecoveredRecord {
            bytes: assembly.bytes,
        });
        self.stats.records_recovered = self.stats.records_recovered.saturating_add(1);
        Ok(())
    }
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
    bytes: Vec<u8>,
    received: Vec<bool>,
    received_count: usize,
    fragment_capacity: usize,
}

impl RecordAssembly {
    fn new(fragment: &Fragment<'_>) -> Self {
        Self {
            kind: fragment.kind,
            object_id: fragment.object_id,
            bytes: vec![0; fragment.record_len],
            received: vec![false; fragment.fragment_count],
            received_count: 0,
            fragment_capacity: fragment.fragment_capacity,
        }
    }

    fn insert(&mut self, fragment: &Fragment<'_>) -> Result<bool> {
        if self.bytes.len() != fragment.record_len || self.received.len() != fragment.fragment_count
        {
            return Err(Error::InvalidFragment("record geometry changed"));
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
