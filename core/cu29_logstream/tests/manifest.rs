use cu29_logstream::{
    ApplicationOutputSchema, ApplicationSchema, LogStreamPlan, SESSION_MANIFEST_VERSION,
    SessionManifest, StreamIdentity,
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
fn config_resolves_to_sender_config_and_versioned_manifest() {
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
    assert_eq!(manifest.version, SESSION_MANIFEST_VERSION);
    assert_eq!(manifest.version, 1, "the unreleased protocol remains v1");
    assert_eq!(manifest.identity, identity);
    assert_eq!(manifest.plan, plan);
    assert_eq!(manifest.application_schema, schema());
    assert_eq!(sender.continuous.fec.symbol_size(), 1128);
    assert_eq!(sender.continuous.fec.window_symbols(), 64);
    assert_eq!(sender.recovery.recovery_interval, 100);
}

#[test]
fn generated_capacity_and_memory_budget_are_enforced() {
    let mut too_large = destination();
    too_large.link.mtu_bytes = 1201;
    assert!(LogStreamPlan::resolve(&too_large).is_err());

    let mut too_small = destination();
    too_small.link.memory_budget_kib = 64;
    assert!(LogStreamPlan::resolve(&too_small).is_err());
}
