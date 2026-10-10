#![cfg(all(feature = "std", feature = "clock-sync"))]
#![allow(deprecated)]

use cu29::clock::sync::{ClockDomain, ClockObservation, SyncState};
use cu29::clock_sync::{ClockReference, ClockReferenceBundle};
use cu29::prelude::*;
use cu29::resource::{BundleContext, ResourceBundle, ResourceManager};
use std::sync::atomic::{AtomicUsize, Ordering};

const EPOCH: u64 = 1_800_000_000_000_000_000;
const DOMAIN: ClockDomain = ClockDomain {
    id: 0,
    identity: *b"testroot",
    session: 0,
};
static STARTS: AtomicUsize = AtomicUsize::new(0);
static STOPS: AtomicUsize = AtomicUsize::new(0);
static PROCESSES: AtomicUsize = AtomicUsize::new(0);
static LOST: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

pub struct PtpBundle;
cu29::bundle_resources!(PtpBundle: Reference = "reference");
impl ClockReferenceBundle for PtpBundle {
    type Reference = MockPtp;
}
impl ResourceBundle for PtpBundle {
    fn build(
        bundle: BundleContext<Self>,
        _: Option<&ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        manager.add_owned(bundle.key(PtpBundleId::Reference), MockPtp { anchor: None })
    }
}

pub struct MockPtp {
    anchor: Option<CuInstant>,
}
impl ClockReference for MockPtp {
    fn create_clock(&self) -> CuResult<RobotClock> {
        Ok(RobotClock::new())
    }
    fn start(&mut self) -> CuResult<()> {
        STARTS.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }
    fn domain(&self) -> ClockDomain {
        DOMAIN
    }
    fn poll(&mut self, clock: &RobotClock) -> CuResult<Option<ClockObservation>> {
        if LOST.load(Ordering::SeqCst) {
            return Ok(None);
        }
        let raw = clock.raw_now();
        Ok(Some(ClockObservation {
            raw_local: raw,
            parent_ns: EPOCH + (raw - *self.anchor.get_or_insert(raw)).as_nanos(),
            uncertainty: CuDuration(100),
            domain: DOMAIN,
        }))
    }
    fn stop(&mut self) -> CuResult<()> {
        STOPS.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }
}

#[derive(Reflect)]
pub struct Source;
impl Freezable for Source {}
impl CuSrcTask for Source {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(u64);
    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        Ok(Self)
    }
    fn start(&mut self, ctx: &CuContext) -> CuResult<()> {
        assert_eq!(ctx.clock.sync_status().unwrap().state, SyncState::Locked);
        assert!(ctx.now().0 >= EPOCH);
        Ok(())
    }
    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        PROCESSES.fetch_add(1, Ordering::SeqCst);
        output.set_payload(ctx.now().0);
        output.tov = Tov::Time(ctx.now());
        Ok(())
    }
}
#[derive(Reflect)]
pub struct Sink;
impl Freezable for Sink {}
impl CuSinkTask for Sink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(u64);
    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        Ok(Self)
    }
    fn process(&mut self, ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        assert!(input.payload().unwrap() >= &EPOCH);
        assert!(ctx.now().0 >= *input.payload().unwrap());
        Ok(())
    }
}

#[copper_runtime(config = "tests/clock_sync_config.ron")]
struct App {}

#[test]
fn runtime_acquires_before_consumers_and_stops_reference() {
    let scratch =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/clock-sync-tests");
    std::fs::create_dir_all(&scratch).unwrap();
    let directory = tempfile::tempdir_in(scratch).unwrap();
    let path = directory.path().join("live.copper");
    let clock = RobotClock::new();
    let observer = clock.clone();
    let mut app = App::builder()
        .with_clock(clock)
        .with_log_path(&path, None)
        .unwrap()
        .build()
        .unwrap()
        .start()
        .unwrap();
    assert_eq!(observer.sync_status().unwrap().state, SyncState::Locked);
    for _ in 0..5 {
        app.run_one_iteration().unwrap();
    }
    drop(app.stop().unwrap());
    assert_eq!(STARTS.load(Ordering::SeqCst), 1);
    assert_eq!(STOPS.load(Ordering::SeqCst), 1);
    use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};
    let read = || match UnifiedLoggerBuilder::new()
        .file_base_name(&path)
        .build()
        .unwrap()
    {
        UnifiedLogger::Read(reader) => reader,
        _ => panic!("Expected read logger"),
    };
    let records = cu29::clock_sync::read_clock_sync_records(UnifiedLoggerIOReader::new(
        read(),
        UnifiedLogType::RuntimeLifecycle,
    ))
    .unwrap();
    assert!(!records.is_empty());
    assert_eq!(records[0].culistid, 0);
    let mut stream = UnifiedLoggerIOReader::new(read(), UnifiedLogType::CopperList);
    let mut recorded = Vec::new();
    while let Ok(cl) = bincode::decode_from_std_read::<
        CopperList<
            <offline::ReplayApp as CuRecordedReplayApplication<
                cu29_unifiedlog::memmap::MmapSectionStorage,
                cu29_unifiedlog::UnifiedLoggerWrite,
            >>::RecordedDataSet,
        >,
        _,
        _,
    >(&mut stream, bincode::config::standard())
    {
        recorded.push(cl);
    }
    assert_eq!(recorded.len(), 5);
    let (replay_clock, mock) = RobotClock::mock();
    let mut callback = |_: offline::ReplayStep<'_>| SimOverride::ExecuteByRuntime;
    let mut replay = offline::ReplayApp::builder()
        .with_clock(replay_clock.clone())
        .with_sim_callback(&mut callback)
        .with_log_path(directory.path().join("replay.copper"), None)
        .unwrap()
        .build()
        .unwrap()
        .into_inner();
    use cu29_unifiedlog::{UnifiedLoggerWrite, memmap::MmapSectionStorage};
    type Replay = offline::ReplayApp;
    <Replay as CuRecordedReplayApplication<MmapSectionStorage, UnifiedLoggerWrite>>::restore_clock_sync(&mut replay, records[0]).unwrap();
    <Replay as CuRecordedReplayApplication<MmapSectionStorage, UnifiedLoggerWrite>>::set_recorded_clock_time(&mut replay, &mock, cu29::simulation::recorded_copperlist_timestamp(&recorded[0]).unwrap()).unwrap();
    <Replay as CuSimApplication<MmapSectionStorage, UnifiedLoggerWrite>>::start_all_tasks(
        &mut replay,
        &mut callback,
    )
    .unwrap();
    for cl in &recorded {
        let correction = *records
            .iter()
            .rev()
            .find(|record| record.culistid <= cl.id)
            .unwrap();
        <Replay as CuRecordedReplayApplication<MmapSectionStorage, UnifiedLoggerWrite>>::restore_clock_sync(&mut replay, correction).unwrap();
        let time = cu29::simulation::recorded_copperlist_timestamp(cl).unwrap();
        <Replay as CuRecordedReplayApplication<MmapSectionStorage, UnifiedLoggerWrite>>::set_recorded_clock_time(&mut replay, &mock, time).unwrap();
        assert_eq!(replay_clock.now(), time);
        assert_eq!(replay_clock.sync_status().unwrap().domain, DOMAIN);
        <Replay as CuRecordedReplayApplication<MmapSectionStorage, UnifiedLoggerWrite>>::replay_recorded_copperlist(&mut replay, &mock, cl, None).unwrap();
    }
    <Replay as CuSimApplication<MmapSectionStorage, UnifiedLoggerWrite>>::stop_all_tasks(
        &mut replay,
        &mut callback,
    )
    .unwrap();
    drop(replay);
    let replay_path = directory.path().join("replay.copper");
    let UnifiedLogger::Read(reader) = UnifiedLoggerBuilder::new()
        .file_base_name(&replay_path)
        .build()
        .unwrap()
    else {
        panic!("Expected replay logger");
    };
    let mut replay_stream = UnifiedLoggerIOReader::new(reader, UnifiedLogType::CopperList);
    for expected in &recorded {
        let actual: CopperList<<Replay as CuRecordedReplayApplication<MmapSectionStorage, UnifiedLoggerWrite>>::RecordedDataSet> = bincode::decode_from_std_read(&mut replay_stream, bincode::config::standard()).unwrap();
        assert_eq!(
            bincode::encode_to_vec(&actual.msgs, bincode::config::standard()).unwrap(),
            bincode::encode_to_vec(&expected.msgs, bincode::config::standard()).unwrap()
        );
    }
    assert_eq!(
        STARTS.load(Ordering::SeqCst),
        1,
        "Offline replay must not start PTP"
    );
    let record = RuntimeLifecycleRecord {
        timestamp: CuTime(EPOCH),
        event: RuntimeLifecycleEvent::ClockSync(records[0]),
    };
    let mut encoded = bincode::encode_to_vec(record, bincode::config::standard()).unwrap();
    encoded.pop();
    assert!(cu29::clock_sync::read_clock_sync_records(encoded.as_slice()).is_err());
    let mut config = cu29::config::read_configuration_str(
        include_str!("clock_sync_config.ron").to_owned(),
        None,
    )
    .unwrap();
    config
        .runtime
        .as_mut()
        .unwrap()
        .clock
        .as_mut()
        .unwrap()
        .max_age_ns = 200_000_000;
    let mut loss_app = App::builder()
        .with_config(config)
        .build()
        .unwrap()
        .start()
        .unwrap();
    let before = PROCESSES.load(Ordering::SeqCst);
    LOST.store(true, Ordering::SeqCst);
    std::thread::sleep(std::time::Duration::from_millis(250));
    assert!(loss_app.run_one_iteration().is_err());
    assert_eq!(
        PROCESSES.load(Ordering::SeqCst),
        before,
        "An expired reference must stop before any process hook"
    );
    drop(loss_app.stop().unwrap());
}

mod offline {
    use cu29::prelude::*;
    #[copper_runtime(config = "tests/clock_sync_config.ron", sim_mode = true)]
    struct App {}
    pub use self::default::{App as ReplayApp, SimStep as ReplayStep};
}
