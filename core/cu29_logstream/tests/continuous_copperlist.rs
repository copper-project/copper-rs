use bincode::{Decode, Encode};
use cu_fec::{DensityThreshold, EncodingSymbolId, Field, RepairParameters, RlcConfig};
use cu29_logstream::{
    ContinuousCopperListSink, ContinuousDecoder, ContinuousEncoder, ContinuousSenderConfig,
    CuStreamTx, CuStreamTxError, ImpairmentConfig, Lane, ReceiverLimits, RecordKind,
    StreamIdentity, decode_copperlist, encode_copperlist, encode_record, impair,
};
use cu29_runtime::copperlist::CopperList;
use cu29_traits::{ErasedCuStampedData, ErasedCuStampedDataSet, MatchingTasks, WriteStream};
use serde::Serialize;

const MAX_SYMBOL_SIZE: usize = 192;
const SYMBOL_SIZE: usize = 160;
const WINDOW_SYMBOLS: usize = 64;
const MAX_EQUATIONS: usize = 32;

#[derive(Debug, Default)]
struct BackpressureTx {
    attempts: usize,
    accepted: Vec<Vec<u8>>,
}

impl CuStreamTx for BackpressureTx {
    fn try_send(&mut self, datagram: &[u8]) -> Result<(), CuStreamTxError> {
        self.attempts += 1;
        if self.attempts.is_multiple_of(2) {
            Err(CuStreamTxError::WouldBlock)
        } else {
            self.accepted.push(datagram.to_vec());
            Ok(())
        }
    }
}

/// Test stand-in for the generated streaming view: all slot metadata remains,
/// while only selected payloads and reconstruction digests cross the link.
#[derive(Clone, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize)]
struct PartialDataSet {
    common_metadata: Vec<u8>,
    original_payload_presence: u16,
    captured_payload_presence: u16,
    captured_payloads: Vec<Vec<u8>>,
    reconstruction_digests: Vec<[u8; 32]>,
}

impl ErasedCuStampedDataSet for PartialDataSet {
    fn cumsgs(&self) -> Vec<&dyn ErasedCuStampedData> {
        Vec::new()
    }
}

impl MatchingTasks for PartialDataSet {
    fn get_all_task_ids() -> &'static [&'static str] {
        &["source", "captured", "reconstructed", "sink"]
    }
}

fn partial_copperlist(id: u64) -> CopperList<PartialDataSet> {
    let captured_a = (0..173)
        .map(|index| (id as u8).wrapping_add(index as u8).wrapping_mul(17))
        .collect::<Vec<_>>();
    let captured_b = (0..91)
        .map(|index| (id as u8).wrapping_sub(index as u8).wrapping_mul(29))
        .collect::<Vec<_>>();
    CopperList::new(
        id,
        PartialDataSet {
            common_metadata: (0..127)
                .map(|index| (index as u8).wrapping_add(id as u8))
                .collect(),
            original_payload_presence: 0b1111,
            captured_payload_presence: 0b0011,
            captured_payloads: vec![captured_a, captured_b],
            reconstruction_digests: vec![
                *blake3::hash(&[id as u8, 2]).as_bytes(),
                *blake3::hash(&[id as u8, 3]).as_bytes(),
            ],
        },
    )
}

#[test]
fn partial_copperlists_survive_seeded_stream_impairment() {
    let identity = StreamIdentity {
        session_id: *b"test-session-001",
        sender_id: 7,
    };
    let fec_config = RlcConfig::new(SYMBOL_SIZE, WINDOW_SYMBOLS, Field::Gf256).unwrap();
    let mut encoder = ContinuousEncoder::<MAX_SYMBOL_SIZE, WINDOW_SYMBOLS>::new(
        identity,
        u64::MAX - 5,
        Lane::ReplayCritical,
        fec_config,
        4_096,
        EncodingSymbolId::new(u32::MAX - 20),
    )
    .unwrap();

    let source = (4_200..4_204).map(partial_copperlist).collect::<Vec<_>>();
    let mut framed_records = Vec::new();
    let mut datagrams = Vec::new();
    let mut source_symbols = 0;
    for copperlist in &source {
        let payload = encode_copperlist(copperlist).unwrap();
        let record = encode_record(RecordKind::CopperList, copperlist.id, &payload).unwrap();
        source_symbols += encoder.push_record(&record, &mut datagrams).unwrap();
        framed_records.push(record);
    }
    for repair_key in 1..=20 {
        encoder
            .push_repair(
                RepairParameters::new(repair_key, DensityThreshold::FULL),
                &mut datagrams,
            )
            .unwrap();
    }
    assert!(source_symbols < WINDOW_SYMBOLS);

    let impaired = impair(
        &datagrams,
        ImpairmentConfig {
            seed: 0x5eed_cafe,
            drop_basis_points: 2_000,
            corrupt_basis_points: 750,
            duplicate_basis_points: 1_000,
            reorder: true,
        },
    )
    .unwrap();
    assert!(impaired.stats.dropped_datagrams > 0);
    assert!(impaired.stats.corrupted_datagrams > 0);
    assert!(impaired.stats.duplicated_datagrams > 0);

    let mut decoder = ContinuousDecoder::<MAX_SYMBOL_SIZE, WINDOW_SYMBOLS, MAX_EQUATIONS>::new(
        identity,
        Lane::ReplayCritical,
        fec_config,
        MAX_EQUATIONS,
        ReceiverLimits::new(4_096, 8, WINDOW_SYMBOLS, 256),
    )
    .unwrap();
    for datagram in &impaired.datagrams {
        decoder.receive_datagram(datagram).unwrap();
    }

    for expected in &source {
        let recovered = decoder
            .recovered_records()
            .iter()
            .find(|record| record.decoded().unwrap().object_id == expected.id)
            .expect("every partial CopperList should be recovered");
        let decoded_record = recovered.decoded().unwrap();
        let decoded: CopperList<PartialDataSet> =
            decode_copperlist(decoded_record.payload).unwrap();
        assert_eq!(decoded.id, expected.id);
        assert_eq!(decoded.msgs, expected.msgs);
        assert_eq!(
            recovered.bytes(),
            framed_records[(expected.id - 4_200) as usize]
        );
    }

    let stats = decoder.stats();
    assert!(stats.source_symbols_received < source_symbols);
    assert!(stats.source_symbols_recovered > 0);
    assert!(stats.repair_symbols_received > 0);
    assert!(stats.innovative_repairs > 0);
    assert!(stats.destruction_basis_points(source_symbols) > 0);
    assert_eq!(stats.source_recovery_basis_points(source_symbols), 10_000);
    assert_eq!(stats.semantic_recovery_basis_points(source.len()), 10_000);
}

#[test]
fn missing_unrepaired_fragments_do_not_claim_semantic_recovery() {
    let identity = StreamIdentity {
        session_id: *b"test-session-002",
        sender_id: 9,
    };
    let fec_config = RlcConfig::new(SYMBOL_SIZE, WINDOW_SYMBOLS, Field::Gf256).unwrap();
    let mut encoder = ContinuousEncoder::<MAX_SYMBOL_SIZE, WINDOW_SYMBOLS>::new(
        identity,
        0,
        Lane::ReplayCritical,
        fec_config,
        4_096,
        EncodingSymbolId::new(0),
    )
    .unwrap();
    let source = (9_000..9_002).map(partial_copperlist).collect::<Vec<_>>();
    let mut datagrams = Vec::new();
    let mut source_symbols = 0;
    for copperlist in &source {
        let payload = encode_copperlist(copperlist).unwrap();
        let record = encode_record(RecordKind::CopperList, copperlist.id, &payload).unwrap();
        source_symbols += encoder.push_record(&record, &mut datagrams).unwrap();
    }

    let mut omitted_objects = Vec::new();
    let retained = datagrams
        .into_iter()
        .filter(|datagram| {
            let object_id = cu29_logstream::WirePacket::decode(datagram)
                .unwrap()
                .header
                .object_id;
            if omitted_objects.contains(&object_id) {
                true
            } else {
                omitted_objects.push(object_id);
                false
            }
        })
        .collect::<Vec<_>>();

    let mut decoder = ContinuousDecoder::<MAX_SYMBOL_SIZE, WINDOW_SYMBOLS, MAX_EQUATIONS>::new(
        identity,
        Lane::ReplayCritical,
        fec_config,
        MAX_EQUATIONS,
        ReceiverLimits::new(4_096, 8, WINDOW_SYMBOLS, 256),
    )
    .unwrap();
    for datagram in &retained {
        decoder.receive_datagram(datagram).unwrap();
    }

    let stats = decoder.stats();
    assert!(decoder.recovered_records().is_empty());
    assert_eq!(stats.source_symbols_received, source_symbols - source.len());
    assert_eq!(stats.source_symbols_recovered, 0);
    assert_eq!(stats.source_recovery_basis_points(source_symbols), 0);
    assert_eq!(stats.semantic_recovery_basis_points(source.len()), 0);
}

#[test]
fn sender_drops_transport_backpressure_without_blocking_the_output_worker() {
    let identity = StreamIdentity {
        session_id: *b"test-session-003",
        sender_id: 11,
    };
    let fec = RlcConfig::new(SYMBOL_SIZE, WINDOW_SYMBOLS, Field::Gf256).unwrap();
    let mut sink = ContinuousCopperListSink::<
        PartialDataSet,
        BackpressureTx,
        MAX_SYMBOL_SIZE,
        WINDOW_SYMBOLS,
    >::new(
        BackpressureTx::default(),
        ContinuousSenderConfig {
            identity,
            first_packet_sequence: 0,
            lane: Lane::ReplayCritical,
            fec,
            max_record_bytes: 4_096,
            initial_esi: EncodingSymbolId::new(0),
            repair_every_source_symbols: 2,
            first_repair_key: 1,
            repair_density: DensityThreshold::FULL,
        },
    )
    .unwrap();

    sink.log(&partial_copperlist(12_000)).unwrap();

    let stats = sink.stats();
    assert_eq!(stats.records_encoded, 1);
    assert!(stats.datagrams_sent > 0);
    assert!(stats.datagrams_dropped > 0);
    assert_eq!(
        stats.datagrams_sent + stats.datagrams_dropped,
        stats.source_datagrams + stats.repair_datagrams
    );
}
