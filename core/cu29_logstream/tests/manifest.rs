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
        feedback: None,
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
    let expected = bincode::encode_to_vec(
        (&identity, plan.receiver_requirements(), schema()),
        bincode::config::standard(),
    )
    .unwrap();
    assert_eq!(payload, expected, "manifest has no version prefix");
    assert_eq!(manifest.identity, identity);
    assert_eq!(manifest.requirements, plan.receiver_requirements());
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
    assert_eq!(plan.symbol_size, 1049);
    plan.symbol_size += 1;
    assert!(plan.validate().is_err());
    config.link.mtu_bytes = cu29_logstream::PACKET_HEADER_LEN as u16;
    assert!(LogStreamPlan::resolve(&config).is_err());
}

#[test]
fn manifest_contains_only_receiver_requirements() {
    let plan = LogStreamPlan::resolve(&destination()).unwrap();
    let requirements = plan.receiver_requirements();
    let encoded = bincode::encode_to_vec(requirements, bincode::config::standard()).unwrap();
    // Bincode varints: symbol size 1128, GF(256), window 64, record bound 65536.
    assert_eq!(encoded, [0, 251, 104, 4, 1, 64, 252, 0, 0, 1, 0]);
    let old_plan = bincode::encode_to_vec(&plan, bincode::config::standard()).unwrap();
    assert_eq!(old_plan.len(), 40);
    assert_eq!(old_plan.len() - encoded.len(), 29);

    let identity = StreamIdentity {
        session_id: [7; 16],
        sender_id: 17,
    };
    let original = plan.sender_config(identity, schema()).unwrap();
    let mut local_policy = plan.clone();
    local_policy.destination_id = "another-destination-with-a-long-name".into();
    local_policy.mtu_bytes += 100;
    local_policy.bitrate_bps *= 2;
    local_policy.memory_budget_kib *= 2;
    local_policy.max_latency_ms *= 2;
    local_policy.burst_packets *= 2;
    local_policy.continuous.repair_every_source_symbols *= 2;
    local_policy.continuous.repair_density = 7;
    local_policy.objects.max_object_bytes *= 2;
    local_policy.objects.repair_symbols_per_block *= 2;
    local_policy.recovery_interval *= 2;
    let changed = local_policy.sender_config(identity, schema()).unwrap();
    assert_eq!(
        original.recovery.manifest_record,
        changed.recovery.manifest_record
    );
    assert_ne!(original.pacing, changed.pacing);
    assert_ne!(
        original.continuous.repair_density,
        changed.continuous.repair_density
    );
    assert_ne!(
        original.recovery.recovery_interval,
        changed.recovery.recovery_interval
    );
    // Sender-only policies still require validation before a sender is constructed.
    local_policy.bitrate_bps = 0;
    assert!(local_policy.sender_config(identity, schema()).is_err());
}

#[test]
fn malformed_requirements_are_rejected_after_record_verification() {
    let valid = LogStreamPlan::resolve(&destination())
        .unwrap()
        .receiver_requirements();
    let invalid = [
        cu29_logstream::ReceiverRequirements {
            symbol_size: 0,
            ..valid
        },
        cu29_logstream::ReceiverRequirements {
            symbol_size: 20,
            ..valid
        },
        cu29_logstream::ReceiverRequirements {
            symbol_size: 1129,
            ..valid
        },
        cu29_logstream::ReceiverRequirements {
            window_symbols: 0,
            ..valid
        },
        cu29_logstream::ReceiverRequirements {
            window_symbols: 65,
            ..valid
        },
        cu29_logstream::ReceiverRequirements {
            max_record_bytes: 0,
            ..valid
        },
        cu29_logstream::ReceiverRequirements {
            max_record_bytes: u64::from(u32::MAX) + 1,
            ..valid
        },
        cu29_logstream::ReceiverRequirements {
            max_record_bytes: u64::MAX,
            ..valid
        },
    ];
    for requirements in invalid {
        let record = SessionManifest::new(
            StreamIdentity {
                session_id: [0; 16],
                sender_id: 0,
            },
            requirements,
            schema(),
        )
        .encode_record()
        .unwrap();
        assert!(
            SessionManifest::decode_record(&record).is_err(),
            "{requirements:?}"
        );
    }
    for field in [
        cu29_logstream::ResolvedRlcField::Gf2,
        cu29_logstream::ResolvedRlcField::Gf256,
    ] {
        let requirements = cu29_logstream::ReceiverRequirements {
            feedback: None,
            symbol_size: 21,
            window_symbols: 1,
            max_record_bytes: u64::from(u32::MAX),
            field,
        };
        requirements.validate().unwrap();
    }
}

#[test]
fn manifest_rejects_truncated_payloads_unknown_fields_and_trailing_bytes() {
    let manifest = SessionManifest::new(
        StreamIdentity {
            session_id: [0; 16],
            sender_id: 0,
        },
        LogStreamPlan::resolve(&destination())
            .unwrap()
            .receiver_requirements(),
        schema(),
    );
    let payload = bincode::encode_to_vec(&manifest, bincode::config::standard()).unwrap();
    let frame = |payload: &[u8]| {
        cu29_logstream::encode_record(cu29_logstream::RecordKind::Manifest, 0, payload).unwrap()
    };
    for len in 0..payload.len() {
        assert!(
            SessionManifest::decode_record(&frame(&payload[..len])).is_err(),
            "length {len}"
        );
    }
    let mut trailing = payload.clone();
    trailing.push(0);
    assert!(SessionManifest::decode_record(&frame(&trailing)).is_err());
    let mut unknown_field = payload;
    // Session, sender ID, absent feedback, three symbol-size bytes, then field.
    unknown_field[21] = 2;
    assert!(SessionManifest::decode_record(&frame(&unknown_field)).is_err());
}

#[test]
fn receiver_requirements_remain_bound_by_the_manifest_digest() {
    let identity = StreamIdentity {
        session_id: [0; 16],
        sender_id: 0,
    };
    let requirements = LogStreamPlan::resolve(&destination())
        .unwrap()
        .receiver_requirements();
    let manifest = SessionManifest::new(identity, requirements, schema());
    let original = manifest.encode_record().unwrap();
    for requirements in [
        cu29_logstream::ReceiverRequirements {
            symbol_size: 1024,
            ..requirements
        },
        cu29_logstream::ReceiverRequirements {
            field: cu29_logstream::ResolvedRlcField::Gf2,
            ..requirements
        },
        cu29_logstream::ReceiverRequirements {
            window_symbols: 32,
            ..requirements
        },
        cu29_logstream::ReceiverRequirements {
            max_record_bytes: 4096,
            ..requirements
        },
    ] {
        let changed = SessionManifest::new(identity, requirements, schema())
            .encode_record()
            .unwrap();
        SessionManifest::decode_record(&changed).unwrap();
        assert_ne!(
            cu29_logstream::decode_record(&original).unwrap().digest,
            cu29_logstream::decode_record(&changed).unwrap().digest
        );
    }
}

#[test]
fn feedback_manifest_keeps_receiver_contract_and_omits_sender_adaptation() {
    use cu29_logstream::feedback::{AdaptationBounds, FeedbackPolicy, destination_key};
    let mut plan = LogStreamPlan::resolve(&destination()).unwrap();
    plan.feedback = Some(FeedbackPolicy {
        report_interval_ms: 500,
        timeout_ms: 2000,
        adaptation: Some(AdaptationBounds {
            min_repair_every_source_symbols: 1,
            max_repair_every_source_symbols: 8,
        }),
    });
    let identity = StreamIdentity {
        session_id: [7; 16],
        sender_id: 17,
    };
    let original = plan.sender_config(identity, schema()).unwrap();
    let manifest = SessionManifest::decode_record(&original.recovery.manifest_record).unwrap();
    let feedback = manifest.requirements.feedback.unwrap();
    assert_eq!(feedback.report_interval_ms, 500);
    assert_eq!(feedback.destination, destination_key("ground"));
    plan.feedback.as_mut().unwrap().timeout_ms = 3000;
    plan.feedback.as_mut().unwrap().adaptation = None;
    plan.continuous.repair_every_source_symbols = 2;
    let changed = plan.sender_config(identity, schema()).unwrap();
    assert_eq!(
        original.recovery.manifest_record,
        changed.recovery.manifest_record
    );
    assert_ne!(original.feedback, changed.feedback);
    plan.feedback.as_mut().unwrap().report_interval_ms = 600;
    assert_ne!(
        original.recovery.manifest_record,
        plan.sender_config(identity, schema())
            .unwrap()
            .recovery
            .manifest_record
    );
    plan.feedback.as_mut().unwrap().report_interval_ms = 500;
    plan.destination_id = "other".into();
    assert_ne!(
        original.recovery.manifest_record,
        plan.sender_config(identity, schema())
            .unwrap()
            .recovery
            .manifest_record
    );
    let mut invalid = manifest;
    invalid
        .requirements
        .feedback
        .as_mut()
        .unwrap()
        .report_interval_ms = 0;
    assert!(SessionManifest::decode_record(&invalid.encode_record().unwrap()).is_err());
}
