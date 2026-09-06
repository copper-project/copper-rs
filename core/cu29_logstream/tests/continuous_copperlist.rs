mod common;

use bincode::{Decode, Encode};
use common::link_sim::{LinkSimulationConfig, simulate_bad_link};
use cu_fec::{DensityThreshold, EncodingSymbolId, Field, RepairParameters, RlcConfig};
use cu29_logstream::{
    ContinuousCopperListSink, ContinuousDecoder, ContinuousEncoder, ContinuousReceiveEvent,
    ContinuousSenderConfig, CopperListGap, CuStreamTx, CuStreamTxError, Lane, ReceiverLimits,
    RecordKind, StreamIdentity, decode_copperlist, encode_copperlist, encode_record,
};
use cu29_runtime::copperlist::CopperList;
use cu29_traits::{ErasedCuStampedData, ErasedCuStampedDataSet, MatchingTasks, WriteStream};
use serde::Serialize;

const MAX_SYMBOL_SIZE: usize = 192;
const SYMBOL_SIZE: usize = 160;
const WINDOW_SYMBOLS: usize = 64;
const MAX_EQUATIONS: usize = 32;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ObservedEvent {
    Record(u64),
    Gap(CopperListGap),
}

fn observe_event(event: ContinuousReceiveEvent<'_>) -> ObservedEvent {
    match event {
        ContinuousReceiveEvent::Record(record) => ObservedEvent::Record(record.decoded().object_id),
        ContinuousReceiveEvent::Gap(gap) => ObservedEvent::Gap(gap),
    }
}

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
fn partial_copperlists_survive_a_deterministically_simulated_bad_link() {
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

    let simulated_link = simulate_bad_link(
        &datagrams,
        LinkSimulationConfig {
            seed: 0x5eed_cafe,
            drop_basis_points: 2_000,
            corrupt_basis_points: 750,
            duplicate_basis_points: 1_000,
            reorder: true,
        },
    );
    assert!(simulated_link.stats.dropped_datagrams > 0);
    assert!(simulated_link.stats.corrupted_datagrams > 0);
    assert!(simulated_link.stats.duplicated_datagrams > 0);

    let mut decoder = ContinuousDecoder::<MAX_SYMBOL_SIZE, WINDOW_SYMBOLS, MAX_EQUATIONS>::new(
        identity,
        Lane::ReplayCritical,
        fec_config,
        MAX_EQUATIONS,
        4_200,
        ReceiverLimits::new(4_096, WINDOW_SYMBOLS),
    )
    .unwrap();
    let mut recovered_records = Vec::new();
    let mut gaps = Vec::new();
    for datagram in &simulated_link.datagrams {
        decoder
            .receive_datagram(datagram, |event| {
                match event {
                    ContinuousReceiveEvent::Record(record) => {
                        recovered_records.push(record.clone());
                    }
                    ContinuousReceiveEvent::Gap(gap) => gaps.push(gap),
                }
                Ok::<(), core::convert::Infallible>(())
            })
            .unwrap();
    }
    assert!(gaps.is_empty());

    for expected in &source {
        let recovered = recovered_records
            .iter()
            .find(|record| record.decoded().object_id == expected.id)
            .expect("every partial CopperList should be recovered");
        let decoded_record = recovered.decoded();
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
        9_000,
        ReceiverLimits::new(4_096, WINDOW_SYMBOLS),
    )
    .unwrap();
    let mut event_count = 0;
    for datagram in &retained {
        decoder
            .receive_datagram(datagram, |_| {
                event_count += 1;
                Ok::<(), core::convert::Infallible>(())
            })
            .unwrap();
    }

    let stats = decoder.stats();
    assert_eq!(event_count, 0);
    assert_eq!(stats.source_symbols_received, source_symbols - source.len());
    assert_eq!(stats.source_symbols_recovered, 0);
    assert_eq!(stats.source_recovery_basis_points(source_symbols), 0);
    assert_eq!(stats.semantic_recovery_basis_points(source.len()), 0);

    let mut final_events = Vec::new();
    decoder
        .finish_through(9_001, |event| {
            final_events.push(observe_event(event));
            Ok::<(), core::convert::Infallible>(())
        })
        .unwrap();
    assert_eq!(
        final_events,
        vec![ObservedEvent::Gap(CopperListGap {
            first_id: 9_000,
            last_id: 9_001,
            reason: cu29_logstream::GapReason::SessionEnded,
        })]
    );
}

#[test]
fn receiver_reports_an_incomplete_record_when_its_missing_fragment_expires() {
    const RECORD_COUNT: u64 = 30;
    let identity = StreamIdentity {
        session_id: *b"test-session-006",
        sender_id: 19,
    };
    let fec_config = RlcConfig::new(SYMBOL_SIZE, 16, Field::Gf256).unwrap();
    let mut encoder = ContinuousEncoder::<MAX_SYMBOL_SIZE, WINDOW_SYMBOLS>::new(
        identity,
        0,
        Lane::ReplayCritical,
        fec_config,
        4_096,
        EncodingSymbolId::new(0),
    )
    .unwrap();
    let mut datagrams = Vec::new();
    for id in 0..RECORD_COUNT {
        let payload = encode_copperlist(&partial_copperlist(id)).unwrap();
        let record = encode_record(RecordKind::CopperList, id, &payload).unwrap();
        encoder.push_record(&record, &mut datagrams).unwrap();
    }
    let mut omitted_fragment = false;
    datagrams.retain(|datagram| {
        let packet = cu29_logstream::WirePacket::decode(datagram).unwrap();
        if packet.header.object_id == 4 && !omitted_fragment {
            omitted_fragment = true;
            false
        } else {
            true
        }
    });
    assert!(omitted_fragment);

    let mut decoder = ContinuousDecoder::<MAX_SYMBOL_SIZE, WINDOW_SYMBOLS, MAX_EQUATIONS>::new(
        identity,
        Lane::ReplayCritical,
        fec_config,
        MAX_EQUATIONS,
        0,
        ReceiverLimits::new(4_096, WINDOW_SYMBOLS),
    )
    .unwrap();
    let mut events = Vec::new();
    for datagram in &datagrams {
        decoder
            .receive_datagram(datagram, |event| {
                events.push(observe_event(event));
                Ok::<(), core::convert::Infallible>(())
            })
            .unwrap();
    }

    let gaps = events
        .iter()
        .filter_map(|event| match event {
            ObservedEvent::Gap(gap) => Some(*gap),
            ObservedEvent::Record(_) => None,
        })
        .collect::<Vec<_>>();
    assert_eq!(gaps.len(), 1);
    assert_eq!(gaps[0].first_id, 4);
    assert_eq!(gaps[0].last_id, 4);
    assert_eq!(decoder.stats().records_expired, 1);
    assert_eq!(decoder.stats().missing_copperlists, 1);
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

#[test]
fn receiver_runs_past_many_windows_with_bounded_state_and_wrapping_sequences() {
    const RECORD_COUNT: u64 = 600;
    let identity = StreamIdentity {
        session_id: *b"test-session-004",
        sender_id: 13,
    };
    let fec_config = RlcConfig::new(SYMBOL_SIZE, WINDOW_SYMBOLS, Field::Gf256).unwrap();
    let mut encoder = ContinuousEncoder::<MAX_SYMBOL_SIZE, WINDOW_SYMBOLS>::new(
        identity,
        u64::MAX - 32,
        Lane::ReplayCritical,
        fec_config,
        4_096,
        EncodingSymbolId::new(u32::MAX - 32),
    )
    .unwrap();
    let mut decoder = ContinuousDecoder::<MAX_SYMBOL_SIZE, WINDOW_SYMBOLS, MAX_EQUATIONS>::new(
        identity,
        Lane::ReplayCritical,
        fec_config,
        MAX_EQUATIONS,
        0,
        ReceiverLimits::new(4_096, WINDOW_SYMBOLS),
    )
    .unwrap();
    let mut record_ids = Vec::new();

    for id in 0..RECORD_COUNT {
        let payload = encode_copperlist(&partial_copperlist(id)).unwrap();
        let record = encode_record(RecordKind::CopperList, id, &payload).unwrap();
        let mut datagrams = Vec::new();
        encoder.push_record(&record, &mut datagrams).unwrap();
        for datagram in &datagrams {
            decoder
                .receive_datagram(datagram, |event| {
                    match event {
                        ContinuousReceiveEvent::Record(record) => {
                            record_ids.push(record.decoded().object_id);
                        }
                        ContinuousReceiveEvent::Gap(gap) => panic!("unexpected gap: {gap:?}"),
                    }
                    Ok::<(), core::convert::Infallible>(())
                })
                .unwrap();
            let occupancy = decoder.occupancy();
            assert!(occupancy.processed_symbols <= WINDOW_SYMBOLS);
            assert!(occupancy.incomplete_records + occupancy.ready_records <= WINDOW_SYMBOLS);
        }
    }

    assert_eq!(record_ids, (0..RECORD_COUNT).collect::<Vec<_>>());
    assert_eq!(decoder.stats().records_emitted, RECORD_COUNT as usize);
    assert!(decoder.stats().datagrams_seen > 256);
    assert!(decoder.stats().peak_buffered_records <= WINDOW_SYMBOLS);
}

#[test]
fn receiver_coalesces_wholly_missing_records_after_the_rlc_window_expires() {
    const RECORD_COUNT: u64 = 40;
    let identity = StreamIdentity {
        session_id: *b"test-session-005",
        sender_id: 17,
    };
    let fec_config = RlcConfig::new(SYMBOL_SIZE, 16, Field::Gf256).unwrap();
    let mut encoder = ContinuousEncoder::<MAX_SYMBOL_SIZE, WINDOW_SYMBOLS>::new(
        identity,
        0,
        Lane::ReplayCritical,
        fec_config,
        4_096,
        EncodingSymbolId::new(0),
    )
    .unwrap();
    let mut datagrams = Vec::new();
    for id in 0..RECORD_COUNT {
        let payload = encode_copperlist(&partial_copperlist(id)).unwrap();
        let record = encode_record(RecordKind::CopperList, id, &payload).unwrap();
        encoder.push_record(&record, &mut datagrams).unwrap();
    }
    datagrams.retain(|datagram| {
        let object_id = cu29_logstream::WirePacket::decode(datagram)
            .unwrap()
            .header
            .object_id;
        !(4..=6).contains(&object_id)
    });

    let mut decoder = ContinuousDecoder::<MAX_SYMBOL_SIZE, WINDOW_SYMBOLS, MAX_EQUATIONS>::new(
        identity,
        Lane::ReplayCritical,
        fec_config,
        MAX_EQUATIONS,
        0,
        ReceiverLimits::new(4_096, WINDOW_SYMBOLS),
    )
    .unwrap();
    let mut events = Vec::new();
    for datagram in &datagrams {
        decoder
            .receive_datagram(datagram, |event| {
                events.push(observe_event(event));
                Ok::<(), core::convert::Infallible>(())
            })
            .unwrap();
    }

    let gaps = events
        .iter()
        .filter_map(|event| match event {
            ObservedEvent::Gap(gap) => Some(*gap),
            ObservedEvent::Record(_) => None,
        })
        .collect::<Vec<_>>();
    assert_eq!(gaps.len(), 1);
    assert_eq!(gaps[0].first_id, 4);
    assert_eq!(gaps[0].last_id, 6);
    assert_eq!(decoder.stats().gaps_emitted, 1);
    assert_eq!(decoder.stats().missing_copperlists, 3);

    let record_ids = events
        .iter()
        .filter_map(|event| match event {
            ObservedEvent::Record(object_id) => Some(*object_id),
            ObservedEvent::Gap(_) => None,
        })
        .collect::<Vec<_>>();
    let expected = (0..RECORD_COUNT)
        .filter(|id| !(4..=6).contains(id))
        .collect::<Vec<_>>();
    assert_eq!(record_ids, expected);
}

#[test]
fn consumer_failure_leaves_the_record_available_for_retry() {
    let identity = StreamIdentity {
        session_id: *b"test-session-007",
        sender_id: 23,
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
    let payload = encode_copperlist(&partial_copperlist(42)).unwrap();
    let record = encode_record(RecordKind::CopperList, 42, &payload).unwrap();
    let mut datagrams = Vec::new();
    encoder.push_record(&record, &mut datagrams).unwrap();

    let mut decoder = ContinuousDecoder::<MAX_SYMBOL_SIZE, WINDOW_SYMBOLS, MAX_EQUATIONS>::new(
        identity,
        Lane::ReplayCritical,
        fec_config,
        MAX_EQUATIONS,
        42,
        ReceiverLimits::new(4_096, WINDOW_SYMBOLS),
    )
    .unwrap();
    let mut consumer_failed = false;
    for datagram in &datagrams {
        let result = decoder.receive_datagram(datagram, |event| match event {
            ContinuousReceiveEvent::Record(_) if !consumer_failed => {
                consumer_failed = true;
                Err("consumer busy")
            }
            _ => Ok(()),
        });
        if consumer_failed {
            assert_eq!(
                result,
                Err(cu29_logstream::ReceiveError::Consumer("consumer busy"))
            );
            break;
        }
        result.unwrap();
    }

    assert!(consumer_failed);
    assert_eq!(decoder.stats().records_emitted, 0);
    assert_eq!(decoder.occupancy().ready_records, 1);
    let mut retried_id = None;
    decoder
        .drain_events(|event| {
            if let ContinuousReceiveEvent::Record(record) = event {
                retried_id = Some(record.decoded().object_id);
            }
            Ok::<(), core::convert::Infallible>(())
        })
        .unwrap();
    assert_eq!(retried_id, Some(42));
    assert_eq!(decoder.stats().records_emitted, 1);
    assert_eq!(decoder.occupancy().ready_records, 0);
}
