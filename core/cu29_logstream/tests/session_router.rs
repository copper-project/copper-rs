use cu29_logstream::{
    ApplicationSchema, ContinuousEncoder, EncodingSymbolId, FiniteObjectEncoder,
    FiniteObjectLimits, LogStreamPlan, RecordKind, SessionEvent, SessionRouter,
    SessionRouterLimits, StreamIdentity, encode_record,
};
use cu29_runtime::config::{
    LogStreamContentConfig, LogStreamContinuousFecConfig, LogStreamDestinationConfig,
    LogStreamFecConfig, LogStreamLinkConfig, LogStreamObjectFecConfig, LogStreamRepairDensity,
    LogStreamRlcField, LogStreamTransportConfig,
};

fn destination() -> LogStreamDestinationConfig {
    LogStreamDestinationConfig {
        id: "ground".into(),
        transport: LogStreamTransportConfig {
            type_: "test::Tx".into(),
            resource: "network.tx".into(),
        },
        link: LogStreamLinkConfig {
            mtu_bytes: 1200,
            bitrate_bps: 1_000_000,
            memory_budget_kib: 512,
            max_latency_ms: 250,
            burst_packets: 8,
        },
        fec: LogStreamFecConfig {
            continuous: LogStreamContinuousFecConfig {
                field: LogStreamRlcField::Gf256,
                window_symbols: 64,
                repair_every_source_symbols: 4,
                repair_density: LogStreamRepairDensity::Full,
            },
            objects: LogStreamObjectFecConfig {
                max_object_bytes: 4_194_304,
                repair_symbols_per_block: 8,
            },
        },
        content: LogStreamContentConfig {
            archive: true,
            live_viz: true,
            anchor_interval: 100,
        },
        max_record_bytes: 65_536,
    }
}

#[test]
fn manifest_bootstraps_continuous_decoder_without_out_of_band_fec_config() {
    let identity = StreamIdentity {
        session_id: *b"router-session01",
        sender_id: 23,
    };
    let plan = LogStreamPlan::resolve(&destination()).unwrap();
    let sender = plan
        .sender_config(identity, ApplicationSchema { outputs: vec![] })
        .unwrap();

    let mut finite = FiniteObjectEncoder::new(sender.recovery.finite).unwrap();
    let mut manifest_packets = Vec::new();
    finite
        .push_record(&sender.recovery.manifest_record, &mut manifest_packets)
        .unwrap();

    let record = encode_record(RecordKind::CopperList, 0, b"semantic-copperlist").unwrap();
    let mut continuous = ContinuousEncoder::<1128, 64>::new(
        identity,
        sender.continuous.first_packet_sequence,
        sender.continuous.lane,
        sender.continuous.fec,
        sender.continuous.max_record_bytes,
        EncodingSymbolId::new(0),
    )
    .unwrap();
    let mut continuous_packets = Vec::new();
    continuous
        .push_record(&record, &mut continuous_packets)
        .unwrap();

    let mut router = SessionRouter::<1128, 64, 64>::new(SessionRouterLimits {
        max_sessions: 2,
        max_pending_events: 16,
        max_record_bytes: 65_536,
        max_buffered_records: 64,
        equation_capacity: 64,
        finite_objects: FiniteObjectLimits::new(4_194_304, 1128, 4),
    })
    .unwrap();
    let mut events = Vec::new();
    for packet in manifest_packets {
        router
            .receive_datagram(&packet, |event| {
                events.push(event);
                Ok::<(), ()>(())
            })
            .unwrap();
    }
    assert!(matches!(events.as_slice(), [SessionEvent::Manifest(_)]));
    assert_eq!(router.session_manifest(identity).unwrap().plan, plan);

    for packet in continuous_packets {
        router
            .receive_datagram(&packet, |event| {
                events.push(event);
                Ok::<(), ()>(())
            })
            .unwrap();
    }
    assert!(matches!(
        events.last(),
        Some(SessionEvent::ContinuousRecord {
            identity: event_identity,
            record: recovered,
        }) if *event_identity == identity && recovered.bytes() == record
    ));
}
