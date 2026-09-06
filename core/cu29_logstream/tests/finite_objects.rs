mod common;

use common::link_sim::{LinkSimulationConfig, simulate_bad_link};
use cu_fec::{DensityThreshold, RepairParameters};
use cu29_logstream::{
    ContinuousDecoder, ContinuousEncoder, ContinuousReceiveEvent, EncodingSymbolId, Field,
    FiniteObjectDecoder, FiniteObjectEncoder, FiniteObjectLimits, FiniteObjectSenderConfig, Lane,
    ReceiverLimits, RecordKind, RlcConfig, StreamIdentity, WirePacket, decode_keyframe,
    decode_record, decode_recovery_point, encode_keyframe_and_recovery_point, encode_record,
};
use cu29_runtime::curuntime::KeyFrame;

const SYMBOL_SIZE: u16 = 256;
const MAX_OBJECT_BYTES: u64 = 64 * 1024;

fn identity() -> StreamIdentity {
    StreamIdentity {
        session_id: *b"object-session01",
        sender_id: 17,
    }
}

fn encoder() -> FiniteObjectEncoder {
    FiniteObjectEncoder::new(FiniteObjectSenderConfig {
        identity: identity(),
        first_packet_sequence: 0,
        lane: Lane::LargeObject,
        symbol_size: SYMBOL_SIZE,
        max_object_bytes: MAX_OBJECT_BYTES,
        repair_symbols_per_block: 16,
    })
    .unwrap()
}

fn decoder() -> FiniteObjectDecoder {
    FiniteObjectDecoder::new(
        identity(),
        Lane::LargeObject,
        FiniteObjectLimits::new(MAX_OBJECT_BYTES, SYMBOL_SIZE, 4),
    )
    .unwrap()
}

#[test]
fn raptorq_recovers_a_finite_record_over_a_bad_link() {
    let payload = (0..7_000)
        .map(|index| (index as u8).wrapping_mul(31))
        .collect::<Vec<_>>();
    let expected = encode_record(RecordKind::Manifest, 3, &payload).unwrap();
    let mut datagrams = Vec::new();
    encoder().push_record(&expected, &mut datagrams).unwrap();

    let damaged = simulate_bad_link(
        &datagrams,
        LinkSimulationConfig {
            seed: 0x6330_cafe,
            drop_basis_points: 2_000,
            corrupt_basis_points: 500,
            duplicate_basis_points: 1_000,
            reorder: true,
        },
    );
    assert!(damaged.stats.dropped_datagrams > 0);
    assert!(damaged.stats.corrupted_datagrams > 0);

    let mut decoder = decoder();
    let mut recovered = Vec::new();
    for datagram in &damaged.datagrams {
        decoder
            .receive_datagram(datagram, |record| {
                recovered.push(record.bytes().to_vec());
                Ok::<(), core::convert::Infallible>(())
            })
            .unwrap();
    }
    assert_eq!(recovered, vec![expected]);
    assert_eq!(decoder.occupancy(), 0);
    assert_eq!(decoder.stats().objects_recovered, 1);
    assert!(decoder.stats().repair_symbols_received > 0);
}

#[test]
fn finite_object_consumer_failure_retains_the_record_for_retry() {
    let expected = encode_record(RecordKind::Manifest, 4, b"retry-me").unwrap();
    let mut datagrams = Vec::new();
    encoder().push_record(&expected, &mut datagrams).unwrap();
    let mut receiver = decoder();
    let mut failed = false;
    for datagram in &datagrams {
        let result = receiver.receive_datagram(datagram, |_| Err("consumer unavailable"));
        if result.is_err() {
            failed = true;
            break;
        }
    }
    assert!(failed);
    assert_eq!(receiver.occupancy(), 1);

    let mut retried = Vec::new();
    receiver
        .drain_records(|record| {
            retried.push(record.bytes().to_vec());
            Ok::<(), core::convert::Infallible>(())
        })
        .unwrap();
    assert_eq!(retried, vec![expected]);
    assert_eq!(receiver.occupancy(), 0);
}

#[test]
fn keyframe_and_recovery_point_recover_independently_and_bind_by_digest() {
    let manifest = encode_record(RecordKind::Manifest, 9, b"resolved-session-plan").unwrap();
    let manifest_digest = decode_record(&manifest).unwrap().digest;
    let keyframe = KeyFrame {
        culistid: 4_200,
        timestamp: Default::default(),
        serialized_tasks: (0..5_000)
            .map(|index| (index as u8).wrapping_mul(17))
            .collect(),
    };
    let (keyframe_record, recovery_point_record) =
        encode_keyframe_and_recovery_point(&keyframe, 9, manifest_digest).unwrap();

    let mut encoder = encoder();
    let mut datagrams = Vec::new();
    encoder
        .push_record(&keyframe_record, &mut datagrams)
        .unwrap();
    encoder
        .push_record(&recovery_point_record, &mut datagrams)
        .unwrap();
    datagrams.reverse();
    datagrams.retain(|datagram| {
        let packet = WirePacket::decode(datagram).unwrap();
        !packet.header.packet_sequence.is_multiple_of(7)
    });

    let mut receiver = decoder();
    let mut recovered = Vec::new();
    for datagram in &datagrams {
        receiver
            .receive_datagram(datagram, |record| {
                recovered.push(record.bytes().to_vec());
                Ok::<(), core::convert::Infallible>(())
            })
            .unwrap();
    }
    assert_eq!(recovered.len(), 2);

    let recovered_keyframe = recovered
        .iter()
        .map(|bytes| decode_record(bytes).unwrap())
        .find(|record| record.kind == RecordKind::KeyFrame)
        .unwrap();
    let recovery_point_record = recovered
        .iter()
        .map(|bytes| decode_record(bytes).unwrap())
        .find(|record| record.kind == RecordKind::RecoveryPoint)
        .unwrap();
    let decoded_keyframe = decode_keyframe(recovered_keyframe.payload).unwrap();
    let recovery_point = decode_recovery_point(recovery_point_record.payload).unwrap();

    assert_eq!(decoded_keyframe.culistid, 4_200);
    assert_eq!(decoded_keyframe.serialized_tasks, keyframe.serialized_tasks);
    assert_eq!(recovery_point.copperlist_id, 4_200);
    assert!(recovery_point.references_keyframe(recovered_keyframe));
    assert!(recovery_point.references_manifest(decode_record(&manifest).unwrap()));
}

#[test]
fn a_late_receiver_restarts_the_continuous_stream_at_the_recovery_point() {
    const MAX_RLC_SYMBOL_SIZE: usize = 192;
    const WINDOW_SYMBOLS: usize = 64;
    const MAX_EQUATIONS: usize = 32;
    const RECOVERY_POINT_ID: u64 = 12;
    const LAST_ID: u64 = 20;

    let rlc = RlcConfig::new(160, WINDOW_SYMBOLS, Field::Gf256).unwrap();
    let mut continuous = ContinuousEncoder::<MAX_RLC_SYMBOL_SIZE, WINDOW_SYMBOLS>::new(
        identity(),
        0,
        Lane::ReplayCritical,
        rlc,
        1_024,
        EncodingSymbolId::new(0),
    )
    .unwrap();
    let mut continuous_datagrams = Vec::new();
    for id in 0..=LAST_ID {
        let record = encode_record(RecordKind::CopperList, id, &[id as u8; 96]).unwrap();
        continuous
            .push_record(&record, &mut continuous_datagrams)
            .unwrap();
    }
    for repair_key in 1..=12 {
        continuous
            .push_repair(
                RepairParameters::new(repair_key, DensityThreshold::FULL),
                &mut continuous_datagrams,
            )
            .unwrap();
    }

    // Everything before this boundary is intentionally unavailable to the late receiver.
    continuous_datagrams.retain(|datagram| {
        let packet = WirePacket::decode(datagram).unwrap();
        packet.header.symbol_kind == cu29_logstream::FecSymbolKind::Repair
            || packet.header.object_id >= RECOVERY_POINT_ID
    });

    let manifest = encode_record(RecordKind::Manifest, 1, b"late-join-manifest").unwrap();
    let keyframe = KeyFrame {
        culistid: RECOVERY_POINT_ID,
        timestamp: Default::default(),
        serialized_tasks: b"CUKF\x01".to_vec(),
    };
    let (keyframe_record, recovery_point_record) =
        encode_keyframe_and_recovery_point(&keyframe, 1, decode_record(&manifest).unwrap().digest)
            .unwrap();
    let mut objects = encoder();
    let mut object_datagrams = Vec::new();
    objects
        .push_record(&keyframe_record, &mut object_datagrams)
        .unwrap();
    objects
        .push_record(&recovery_point_record, &mut object_datagrams)
        .unwrap();
    object_datagrams.reverse();

    let mut object_receiver = decoder();
    let mut recovered_objects = Vec::new();
    for datagram in &object_datagrams {
        object_receiver
            .receive_datagram(datagram, |record| {
                recovered_objects.push(record.bytes().to_vec());
                Ok::<(), core::convert::Infallible>(())
            })
            .unwrap();
    }
    let recovery_point = recovered_objects
        .iter()
        .map(|record| decode_record(record).unwrap())
        .find(|record| record.kind == RecordKind::RecoveryPoint)
        .map(|record| decode_recovery_point(record.payload).unwrap())
        .unwrap();
    assert_eq!(recovery_point.copperlist_id, RECOVERY_POINT_ID);

    let mut receiver =
        ContinuousDecoder::<MAX_RLC_SYMBOL_SIZE, WINDOW_SYMBOLS, MAX_EQUATIONS>::new(
            identity(),
            Lane::ReplayCritical,
            rlc,
            MAX_EQUATIONS,
            recovery_point.copperlist_id,
            ReceiverLimits::new(1_024, WINDOW_SYMBOLS),
        )
        .unwrap();
    let mut recovered_ids = Vec::new();
    let mut gaps = Vec::new();
    for datagram in &continuous_datagrams {
        receiver
            .receive_datagram(datagram, |event| {
                match event {
                    ContinuousReceiveEvent::Record(record) => {
                        recovered_ids.push(record.decoded().unwrap().object_id);
                    }
                    ContinuousReceiveEvent::Gap(gap) => gaps.push(gap),
                }
                Ok::<(), core::convert::Infallible>(())
            })
            .unwrap();
    }
    assert!(gaps.is_empty());
    assert_eq!(
        recovered_ids,
        (RECOVERY_POINT_ID..=LAST_ID).collect::<Vec<_>>()
    );
}
