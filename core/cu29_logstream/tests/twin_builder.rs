#![cfg(feature = "std")]

use bincode::{Decode, Encode};
use cu29_logstream::capture::{CaptureDataSet, encode_capture_record_into};
use cu29_logstream::twin::LiveReplay;
use cu29_logstream::{
    ApplicationSchema, ContinuousEncoder, CuStreamRx, CuStreamRxError, CuTwin, EncodingSymbolId,
    FiniteObjectEncoder, FiniteObjectLimits, LogStreamPlan, ResolvedContinuousFec,
    ResolvedObjectFec, ResolvedRlcField, SessionRouterLimits, StreamIdentity,
};
use cu29_runtime::copperlist::CopperList;
use cu29_runtime::curuntime::KeyFrame;
use cu29_traits::{CuResult, ErasedCuStampedData, ErasedCuStampedDataSet, MatchingTasks};
use std::collections::VecDeque;
use std::num::NonZeroUsize;
use std::time::{Duration, Instant};

#[derive(Debug, Default, Encode, Decode, serde::Serialize)]
struct Cameras(Vec<Vec<u8>>);
impl ErasedCuStampedDataSet for Cameras {
    fn cumsgs(&self) -> Vec<&dyn ErasedCuStampedData> {
        Vec::new()
    }
}
impl MatchingTasks for Cameras {
    fn get_all_task_ids() -> &'static [&'static str] {
        &[]
    }
}
impl CaptureDataSet for Cameras {
    const RECONSTRUCTION: &'static [bool] = &[];
    fn stream_schema() -> ApplicationSchema {
        ApplicationSchema {
            outputs: vec![],
            reconstruction: vec![],
        }
    }
    fn encode_capture<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        self.encode(encoder)
    }
    fn validate_capture(&self) -> cu29_logstream::Result<()> {
        Ok(())
    }
    fn restore_sender_metadata(&mut self, _: &Self) {}
    #[cfg(feature = "verify-reconstruction")]
    fn encode_reconstruction<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        self.encode(encoder)
    }
}
struct App;
impl LiveReplay for App {
    type DataSet = Cameras;
    const MISSION_INDEX: u32 = 0;
    fn seal_archive_metadata(_: &mut cu29_unifiedlog::UnifiedLoggerWrite) -> CuResult<()> {
        Ok(())
    }
    fn build_twin() -> CuResult<(Self, cu29_clock::RobotClockMock)> {
        let (_, clock) = cu29_clock::RobotClock::mock();
        Ok((Self, clock))
    }
    fn replay_capture(
        &mut self,
        _: &cu29_clock::RobotClockMock,
        _: &mut CopperList<Cameras>,
        _: Option<&KeyFrame>,
    ) -> CuResult<()> {
        Ok(())
    }
}
#[derive(Debug)]
struct Rx(VecDeque<Vec<u8>>);
impl CuStreamRx for Rx {
    fn try_recv(&mut self, output: &mut [u8]) -> Result<Option<usize>, CuStreamRxError> {
        let Some(packet) = self.0.pop_front() else {
            return Ok(None);
        };
        output[..packet.len()].copy_from_slice(&packet);
        Ok(Some(packet.len()))
    }
}

fn packets(max_record_bytes: u64, count: u64) -> VecDeque<Vec<u8>> {
    let plan = LogStreamPlan {
        feedback: None,
        destination_id: "ground".into(),
        mtu_bytes: 1200,
        symbol_size: 1128,
        bitrate_bps: 1_000_000,
        memory_budget_kib: 1024,
        max_latency_ms: 250,
        burst_packets: 8,
        continuous: ResolvedContinuousFec {
            field: ResolvedRlcField::Gf256,
            window_symbols: 64,
            repair_every_source_symbols: 4,
            repair_density: 15,
        },
        objects: ResolvedObjectFec {
            max_object_bytes: 128 * 1024,
            repair_symbols_per_block: 8,
        },
        recovery_interval: 100,
        max_record_bytes,
    };
    let sender = plan
        .sender_config(
            StreamIdentity {
                session_id: *b"twin-builder-001",
                sender_id: 7,
            },
            Cameras::stream_schema(),
            cu29_unifiedlog::SectionContext {
                run_id: 1,
                instance_id: 7,
                mission_index: App::MISSION_INDEX,
            },
        )
        .unwrap();
    let mut packets = Vec::new();
    let mut finite = FiniteObjectEncoder::new(sender.recovery.finite).unwrap();
    finite
        .push_record(&sender.recovery.manifest_record, &mut packets)
        .unwrap();
    let manifest = cu29_logstream::decode_record(&sender.recovery.manifest_record).unwrap();
    let (keyframe, recovery) = cu29_logstream::encode_keyframe_and_recovery_point(
        &KeyFrame {
            culistid: 0,
            timestamp: Default::default(),
            serialized_tasks: vec![0x5a; 96 * 1024],
        },
        manifest.object_id,
        manifest.digest,
    )
    .unwrap();
    finite.push_record(&keyframe, &mut packets).unwrap();
    finite.push_record(&recovery, &mut packets).unwrap();
    let mut structured = FiniteObjectEncoder::new(cu29_logstream::FiniteObjectSenderConfig {
        lane: cu29_logstream::Lane::StructuredLog,
        ..sender.recovery.finite
    })
    .unwrap();
    for id in 0..5 {
        let entry = cu29_log::CuLogEntry::new(id, cu29_log::CuLogLevel::Info);
        let encoded = bincode::encode_to_vec(&entry, bincode::config::standard()).unwrap();
        let record = cu29_logstream::encode_record(
            cu29_logstream::RecordKind::StructuredLog,
            u64::from(id),
            &encoded,
        )
        .unwrap();
        structured.push_record(&record, &mut packets).unwrap();
    }
    let mut encoder = ContinuousEncoder::<1128, 64>::new(
        sender.continuous.identity,
        sender.continuous.lane,
        sender.continuous.fec,
        sender.continuous.max_record_bytes,
        EncodingSymbolId::new(0),
    )
    .unwrap();
    for id in 0..count {
        let list = CopperList::new(id, Cameras(vec![vec![id as u8; 16 * 1024]; 6]));
        let mut record = vec![0; max_record_bytes as usize];
        let len = encode_capture_record_into(&list, &mut record).unwrap();
        encoder.push_record(&record[..len], &mut packets).unwrap();
    }
    packets.into()
}

#[test]
fn configured_twin_archives_large_camera_records_and_keyframes_across_slabs() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("capture.copper");
    let limits = SessionRouterLimits {
        max_record_bytes: 128 * 1024,
        finite_objects: FiniteObjectLimits::new(128 * 1024, 1128, 2),
        ..Default::default()
    };
    let (mut twin, mut reader) = CuTwin::<App>::builder(Rx(packets(128 * 1024, 8)))
        .with_log_path(&path)
        .with_slab_size(1024 * 1024)
        .with_section_size(256 * 1024)
        .with_receiver_limits(limits)
        .with_replay_capacity(NonZeroUsize::new(16).unwrap())
        .with_log_capacity(NonZeroUsize::new(3).unwrap())
        .with_frame_capacity(NonZeroUsize::new(1).unwrap())
        .spawn()
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    while reader.status().archived != 8 || reader.status().twin.reconstructed != 8 {
        assert!(!reader.is_closed(), "{:?}", reader.status());
        assert!(Instant::now() < deadline, "{:?}", reader.status());
        reader.wait_timeout(Duration::from_millis(10));
    }
    let status = twin.stop().unwrap();
    assert_eq!(status.archived, 8);
    assert_eq!(status.structured_logs, 5);
    assert_eq!(reader.overwritten(), 7);
    assert_eq!(reader.try_read().unwrap().frame.copperlist.id, 7);
    assert!(reader.try_read().is_none());
    let mut logs = twin.take_log_reader().unwrap();
    assert_eq!(logs.overwritten(), 2);
    for id in 2..5 {
        assert_eq!(logs.try_read().unwrap().frame.entry.msg_index, id);
    }
    assert!(logs.try_read().is_none());
    let logger = cu29_unifiedlog::UnifiedLoggerRead::new(&path).unwrap();
    let mut stream = cu29_unifiedlog::UnifiedLoggerIOReader::new(
        logger,
        cu29_traits::UnifiedLogType::CopperList,
    );
    for id in 0..8 {
        let list: CopperList<Cameras> =
            bincode::decode_from_std_read(&mut stream, bincode::config::standard()).unwrap();
        assert_eq!(list.id, id);
        assert_eq!(list.msgs.0, vec![vec![id as u8; 16 * 1024]; 6]);
    }
    let logger = cu29_unifiedlog::UnifiedLoggerRead::new(&path).unwrap();
    let mut stream = cu29_unifiedlog::UnifiedLoggerIOReader::new(
        logger,
        cu29_traits::UnifiedLogType::FrozenTasks,
    );
    let keyframe: KeyFrame =
        bincode::decode_from_std_read(&mut stream, bincode::config::standard()).unwrap();
    assert_eq!(keyframe.serialized_tasks, vec![0x5a; 96 * 1024]);
    let slabs = std::fs::read_dir(dir.path())
        .unwrap()
        .map(|entry| entry.unwrap())
        .filter(|entry| entry.path() != path)
        .collect::<Vec<_>>();
    assert!(slabs.len() >= 2);
    for slab in slabs {
        assert_eq!(slab.metadata().unwrap().len(), 1024 * 1024);
    }
}

#[test]
fn invalid_configuration_fails_before_transport_or_filesystem_side_effects() {
    #[derive(Debug)]
    struct UnusedRx;
    impl CuStreamRx for UnusedRx {
        fn try_recv(&mut self, _: &mut [u8]) -> Result<Option<usize>, CuStreamRxError> {
            panic!("invalid configuration started the receiver")
        }
    }
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("new-directory/capture.copper");
    let cases = [
        (513, 128 * 1024, SessionRouterLimits::default()),
        (16 * 1024 * 1024, 512, SessionRouterLimits::default()),
        (16 * 1024 * 1024, 64 * 1024, SessionRouterLimits::default()),
        (1024, 128 * 1024, SessionRouterLimits::default()),
        (
            16 * 1024 * 1024,
            128 * 1024,
            SessionRouterLimits {
                max_sessions: 2,
                ..Default::default()
            },
        ),
        (
            16 * 1024 * 1024,
            128 * 1024,
            SessionRouterLimits {
                equation_capacity: 65,
                ..Default::default()
            },
        ),
        (
            16 * 1024 * 1024,
            128 * 1024,
            SessionRouterLimits {
                max_pending_events: 0,
                ..Default::default()
            },
        ),
        (
            16 * 1024 * 1024,
            128 * 1024,
            SessionRouterLimits {
                finite_objects: FiniteObjectLimits::new(65536, 1128, 0),
                ..Default::default()
            },
        ),
    ];
    for (slab, section, limits) in cases {
        assert!(
            CuTwin::<App>::builder(UnusedRx)
                .with_log_path(&path)
                .with_slab_size(slab)
                .with_section_size(section)
                .with_receiver_limits(limits)
                .spawn()
                .is_err()
        );
        assert!(!path.parent().unwrap().exists());
    }
}

#[test]
fn sender_exceeding_configured_record_budget_is_rejected() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("rejected.copper");
    let (mut twin, mut reader) = CuTwin::<App>::builder(Rx(packets(128 * 1024, 0)))
        .with_log_path(&path)
        .archive_only()
        .spawn()
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    while !reader.is_closed() {
        assert!(Instant::now() < deadline);
        reader.wait_timeout(Duration::from_millis(10));
    }
    assert!(
        twin.stop()
            .unwrap_err()
            .to_string()
            .contains("object has 131072 bytes; maximum is 4096")
    );
    assert!(
        !cu29_runtime::replay::first_slab_path(&path)
            .unwrap()
            .exists()
    );
}
