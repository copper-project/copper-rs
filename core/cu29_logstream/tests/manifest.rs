use cu29_logstream::{
    ApplicationOutputSchema, ApplicationSchema, LogStreamPlan, SessionManifest, StreamIdentity,
};
use cu29_runtime::config::{
    LogStreamContinuousFecConfig, LogStreamDestinationConfig, LogStreamFecConfig,
    LogStreamLinkConfig, LogStreamObjectFecConfig, LogStreamRepairDensity, LogStreamRlcField,
    LogStreamTransportConfig,
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
        recovery_interval: 100,
        max_record_bytes: 65_536,
    }
}

fn schema() -> ApplicationSchema {
    ApplicationSchema {
        reconstruction: vec![],
        outputs: vec![ApplicationOutputSchema {
            task_id: "camera".into(),
            message_type: "app::Image".into(),
            payload_type: "app::Image".into(),
        }],
    }
}

#[test]
fn config_resolves_to_sender_config_and_manifest() {
    let plan = LogStreamPlan::resolve(&destination()).unwrap();
    assert_eq!(plan.symbol_size, 1128);
    assert_eq!(plan.continuous.repair_density, 15);

    let identity = StreamIdentity {
        session_id: *b"manifest-session",
        sender_id: 17,
    };
    let sender = plan.sender_config(identity, schema()).unwrap();
    sender.validate().unwrap();

    let manifest = SessionManifest::decode_record(&sender.recovery.manifest_record).unwrap();
    let payload = bincode::encode_to_vec(&manifest, bincode::config::standard()).unwrap();
    let expected =
        bincode::encode_to_vec((&identity, &plan, schema()), bincode::config::standard()).unwrap();
    assert_eq!(payload, expected, "manifest has no version prefix");
    assert_eq!(manifest.identity, identity);
    assert_eq!(manifest.plan, plan);
    assert_eq!(manifest.application_schema, schema());
    assert_eq!(sender.continuous.fec.symbol_size(), 1128);
    assert_eq!(sender.continuous.fec.window_symbols(), 64);
    assert_eq!(sender.recovery.recovery_interval, 100);
}

#[test]
fn generated_capacity_and_memory_budget_are_enforced() {
    let mut larger_mtu = destination();
    larger_mtu.link.mtu_bytes = 1201;
    // An MTU is an upper bound; retain preallocated symbol storage on larger links.
    assert_eq!(
        LogStreamPlan::resolve(&larger_mtu).unwrap().symbol_size,
        1128
    );

    let mut too_small = destination();
    too_small.link.memory_budget_kib = 64;
    assert!(LogStreamPlan::resolve(&too_small).is_err());
}

#[test]
fn symbol_size_respects_mtu_without_growing_preallocated_storage() {
    let mut config = destination();
    config.link.mtu_bytes = 1100;
    let mut plan = LogStreamPlan::resolve(&config).unwrap();
    assert_eq!(plan.symbol_size, 1045);
    plan.symbol_size += 1;
    assert!(plan.validate().is_err());
    config.link.mtu_bytes = cu29_logstream::PACKET_HEADER_LEN as u16;
    assert!(LogStreamPlan::resolve(&config).is_err());
}
