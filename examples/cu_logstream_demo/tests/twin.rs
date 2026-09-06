#![cfg(feature = "demo")]

use cu_logstream_demo::{DataSet, List, read_keyframes, read_lists, run_sender, twin::Twin};
use cu29_logstream::{
    capture::{CapturedList, decode_capture, encode_capture_record_into},
    twin::{LiveTwin, ReconstructionState},
};

struct CountingAllocator;
thread_local! {
    static ALLOCATIONS: std::cell::Cell<Option<usize>> = const { std::cell::Cell::new(None) };
}
unsafe impl std::alloc::GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: std::alloc::Layout) -> *mut u8 {
        let _ = ALLOCATIONS.try_with(|count| {
            if let Some(value) = count.get() {
                count.set(Some(value + 1));
            }
        });
        unsafe { std::alloc::System.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: std::alloc::Layout) {
        unsafe { std::alloc::System.dealloc(ptr, layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: std::alloc::Layout, size: usize) -> *mut u8 {
        let _ = ALLOCATIONS.try_with(|count| {
            if let Some(value) = count.get() {
                count.set(Some(value + 1));
            }
        });
        unsafe { std::alloc::System.realloc(ptr, layout, size) }
    }
}
#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

static SERIAL: std::sync::Mutex<()> = std::sync::Mutex::new(());

fn fixture() -> (Vec<List>, Vec<cu29::curuntime::KeyFrame>) {
    let logs = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("logs");
    std::fs::create_dir_all(&logs).unwrap();
    let directory = tempfile::tempdir_in(logs).unwrap();
    let path = directory.path().join("onboard.copper");
    let socket = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
    run_sender(socket.local_addr().unwrap(), &path, 34, 0).unwrap();
    (read_lists(&path).unwrap(), read_keyframes(&path).unwrap())
}

fn capture(list: &List) -> CapturedList<DataSet> {
    let mut bytes = [0; 4096];
    let len = encode_capture_record_into(list, &mut bytes).unwrap();
    let record = cu29_logstream::decode_record(&bytes[..len]).unwrap();
    let (captured, _) = decode_capture::<DataSet>(record.payload).unwrap();
    assert!(
        captured
            .copperlist
            .msgs
            .get_derived_output()
            .payload()
            .is_none()
    );
    captured
}

fn assert_exact(expected: &List, actual: &List) {
    let encode =
        |list| cu29::bincode::encode_to_vec(list, cu29::bincode::config::standard()).unwrap();
    assert_eq!(
        encode(expected),
        encode(actual),
        "CopperList {} including sender metadata",
        expected.id
    );
}

#[test]
fn generated_twin_recovers_gaps_and_preserves_every_sender_field() {
    let _serial = SERIAL.lock().unwrap();
    let (lists, keyframes) = fixture();
    let mut twin = LiveTwin::<Twin>::new().unwrap();
    for list in &lists[..4] {
        let result = twin
            .reconstruct(
                capture(list),
                keyframes.iter().find(|k| k.culistid == list.id),
            )
            .unwrap()
            .unwrap();
        assert_exact(list, &result);
    }
    // Missing required iterations cannot be healed by a complete later input alone.
    assert!(
        twin.reconstruct(capture(&lists[9]), None)
            .unwrap()
            .is_none()
    );
    assert_eq!(twin.status.state, ReconstructionState::Recovering);
    for list in &lists[16..] {
        let result = twin
            .reconstruct(
                capture(list),
                keyframes.iter().find(|k| k.culistid == list.id),
            )
            .unwrap()
            .unwrap();
        assert_exact(list, &result);
    }
    assert_eq!(twin.status.reconstructed, 22);
    if cfg!(feature = "verify-reconstruction") {
        assert_eq!(twin.status.verified, 22);
        assert_eq!(twin.status.state, ReconstructionState::Verified);
    } else {
        assert_eq!(twin.status.verified, 0);
        assert_eq!(twin.status.state, ReconstructionState::Reconstructed);
    }
}

#[test]
fn a_payload_hidden_in_a_capture_is_rejected() {
    let _serial = SERIAL.lock().unwrap();
    let (lists, _) = fixture();
    let list = &lists[0];
    let bytes = cu29::bincode::encode_to_vec(list, cu29::bincode::config::standard()).unwrap();
    assert!(decode_capture::<DataSet>(&bytes).is_err());
}

#[cfg(feature = "verify-reconstruction")]
#[test]
fn divergence_suppresses_frames_until_a_matching_anchor() {
    let _serial = SERIAL.lock().unwrap();
    let (lists, keyframes) = fixture();
    let mut twin = LiveTwin::<Twin>::new().unwrap();
    twin.reconstruct(capture(&lists[0]), Some(&keyframes[0]))
        .unwrap()
        .unwrap();
    let mut corrupted = capture(&lists[1]);
    corrupted.digest = Some([0; 32]);
    assert!(twin.reconstruct(corrupted, None).is_err());
    assert_eq!(twin.status.state, ReconstructionState::Diverged);
    assert!(
        twin.reconstruct(capture(&lists[2]), None)
            .unwrap()
            .is_none()
    );
    assert_eq!(twin.status.state, ReconstructionState::Diverged);
    let boundary = &lists[16];
    let recovered = twin
        .reconstruct(
            capture(boundary),
            keyframes.iter().find(|k| k.culistid == 16),
        )
        .unwrap()
        .unwrap();
    assert_exact(boundary, &recovered);
    assert_eq!(twin.status.state, ReconstructionState::Verified);
    assert_eq!(twin.status.divergences, 1);
}

static ENTERED: std::sync::LazyLock<std::sync::Barrier> =
    std::sync::LazyLock::new(|| std::sync::Barrier::new(2));
static RELEASE: std::sync::LazyLock<std::sync::Barrier> =
    std::sync::LazyLock::new(|| std::sync::Barrier::new(2));
struct BlockedTwin(Twin);
impl cu29_logstream::twin::LiveReplay for BlockedTwin {
    type DataSet = DataSet;
    fn build_twin() -> cu29::CuResult<(Self, cu29::clock::RobotClockMock)> {
        let (app, clock) = Twin::build_twin()?;
        Ok((Self(app), clock))
    }
    fn replay_capture(
        &mut self,
        clock: &cu29::clock::RobotClockMock,
        captured: &mut List,
        keyframe: Option<&cu29::curuntime::KeyFrame>,
    ) -> cu29::CuResult<()> {
        ENTERED.wait();
        RELEASE.wait();
        self.0.replay_capture(clock, captured, keyframe)
    }
}

#[test]
fn replay_queue_overflow_invalidates_in_flight_output_without_blocking_capture() {
    let _serial = SERIAL.lock().unwrap();
    let (lists, keyframes) = fixture();
    let (publisher, mut reader) = cu29_logstream::telemetry::telemetry_channel(
        4.try_into().unwrap(),
        cu_logstream_demo::telemetry::Status::default(),
    );
    let worker =
        cu29_logstream::twin::TwinWorker::spawn::<BlockedTwin>(2.try_into().unwrap(), publisher)
            .unwrap();
    let identity = cu29_logstream::StreamIdentity {
        session_id: [1; 16],
        sender_id: 41,
    };
    worker.submit(capture(&lists[0]), identity, std::time::Instant::now());
    worker.anchor(keyframes[0].clone());
    ENTERED.wait(); // The replay worker is blocked inside task execution.
    for list in &lists[1..4] {
        worker.submit(capture(list), identity, std::time::Instant::now());
    }
    RELEASE.wait();
    drop(worker);
    assert!(
        reader.try_read().is_none(),
        "invalidated in-flight output reached the UI"
    );
    let status = reader.status();
    assert_eq!(status.twin.queue_overflows, 1);
    assert_eq!(status.twin.state, ReconstructionState::Recovering);
}

#[test]
fn recovery_discards_captures_waiting_for_an_old_anchor() {
    let _serial = SERIAL.lock().unwrap();
    let (lists, keyframes) = fixture();
    let (publisher, mut reader) = cu29_logstream::telemetry::telemetry_channel(
        4.try_into().unwrap(),
        cu_logstream_demo::telemetry::Status::default(),
    );
    let worker =
        cu29_logstream::twin::TwinWorker::spawn::<Twin>(4.try_into().unwrap(), publisher).unwrap();
    let identity = cu29_logstream::StreamIdentity {
        session_id: [1; 16],
        sender_id: 41,
    };
    worker.submit(capture(&lists[0]), identity, std::time::Instant::now());
    // Two acknowledged status updates ensure the worker has consumed the capture,
    // even if the first update was observed before its blocking receive.
    for archived in 1..=2 {
        worker.set_status(cu_logstream_demo::telemetry::Status {
            archived,
            ..Default::default()
        });
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
        while reader.status().archived != archived {
            assert!(
                std::time::Instant::now() < deadline,
                "worker did not acknowledge status"
            );
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }
    worker.recover();
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    while reader.status().twin.state != ReconstructionState::Recovering {
        assert!(
            std::time::Instant::now() < deadline,
            "worker did not recover"
        );
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
    worker.anchor(keyframes[0].clone());
    drop(worker);
    assert!(reader.try_read().is_none(), "old capture survived recovery");
    assert_eq!(reader.status().twin.reconstructed, 0);
}

mod stateful {
    use cu29::prelude::*;
    #[copper_runtime(config = "tests/stateful.ron", sim_mode = true)]
    struct Replay {}
    pub type Twin = default::Replay;
    pub type DataSet = default::CuStampedDataSet;
}

#[test]
fn stateful_reconstruction_restores_keyframes_and_feeds_downstream_tasks() {
    let _serial = SERIAL.lock().unwrap();
    let (lists, keyframes) = fixture();
    let mut twin = LiveTwin::<stateful::Twin>::new().unwrap();
    // The same onboard graph, but both the stateful sum and its downstream
    // modulo task are omitted from the stream in this reconstruction contract.
    for expected in lists[..4].iter().chain(&lists[16..]) {
        let full =
            cu29::bincode::encode_to_vec(expected, cu29::bincode::config::standard()).unwrap();
        let (list, _): (cu29::copperlist::CopperList<stateful::DataSet>, _) =
            cu29::bincode::decode_from_slice(&full, cu29::bincode::config::standard()).unwrap();
        let mut bytes = [0; 4096];
        let len = encode_capture_record_into(&list, &mut bytes).unwrap();
        let record = cu29_logstream::decode_record(&bytes[..len]).unwrap();
        let (captured, _) = decode_capture::<stateful::DataSet>(record.payload).unwrap();
        assert!(
            captured
                .copperlist
                .msgs
                .get_sum_output()
                .payload()
                .is_none()
        );
        assert!(
            captured
                .copperlist
                .msgs
                .get_derived_output()
                .payload()
                .is_none()
        );
        let actual = twin
            .reconstruct(
                captured,
                keyframes.iter().find(|k| k.culistid == expected.id),
            )
            .unwrap()
            .unwrap();
        assert_eq!(
            full,
            cu29::bincode::encode_to_vec(actual, cu29::bincode::config::standard()).unwrap()
        );
    }
    assert_eq!(twin.status.reconstructed, 22);
}

#[test]
fn capture_encoding_is_native_and_does_not_allocate() {
    use cu29_logstream::capture::CaptureView;
    let _serial = SERIAL.lock().unwrap();
    let (lists, _) = fixture();
    let list = &lists[0];
    let mut bytes = [0; 4096];
    encode_capture_record_into(list, &mut bytes).unwrap(); // Warm thread-local I/O instrumentation.
    ALLOCATIONS.with(|count| count.set(Some(0)));
    let encoded = encode_capture_record_into(list, &mut bytes);
    let allocations = ALLOCATIONS.with(|count| count.replace(None).unwrap());
    assert_eq!(allocations, 0, "capture encoding allocated after startup");
    let record = cu29_logstream::decode_record(&bytes[..encoded.unwrap()]).unwrap();
    let (capture, native) = decode_capture::<DataSet>(record.payload).unwrap();
    let expected = cu29::bincode::encode_to_vec(
        (list.id, list.get_state(), CaptureView(&list.msgs)),
        cu29::bincode::config::standard(),
    )
    .unwrap();
    assert_eq!(native, expected);
    let decoded = cu29_logstream::decode_copperlist::<DataSet>(native).unwrap();
    assert!(decoded.msgs.get_derived_output().payload().is_none());
    if cfg!(feature = "verify-reconstruction") {
        assert_eq!(record.payload.len(), native.len() + 32);
    } else {
        assert_eq!(
            record.payload, native,
            "production added a verification envelope"
        );
        assert_eq!(std::mem::size_of_val(&capture), std::mem::size_of::<List>());
    }
    // Any debug build can replay a normal capture without claiming verification.
    let (native_capture, _) = decode_capture::<DataSet>(native).unwrap();
    #[cfg(feature = "verify-reconstruction")]
    assert!(native_capture.digest.is_none());
    assert_eq!(native_capture.copperlist.id, list.id);
    let mut malformed = native.to_vec();
    malformed.push(0);
    assert!(decode_capture::<DataSet>(&malformed).is_err());
}

#[test]
fn twin_owns_shutdown_and_reports_transport_failure() {
    use cu29_logstream::{CuStreamRx, CuStreamRxError, CuTwinRecordingState};
    #[derive(Debug)]
    struct Rx(bool);
    impl CuStreamRx for Rx {
        fn try_recv(&mut self, _: &mut [u8]) -> Result<Option<usize>, CuStreamRxError> {
            if self.0 {
                Err(CuStreamRxError::Failed("test transport failed"))
            } else {
                Ok(None)
            }
        }
    }
    let _serial = SERIAL.lock().unwrap();
    let logs = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("logs");
    std::fs::create_dir_all(&logs).unwrap();
    let dir = tempfile::tempdir_in(logs).unwrap();
    let (twin, mut reader) = Twin::twin(Rx(false))
        .with_log_path(dir.path().join("unused.copper"))
        .spawn()
        .unwrap();
    drop(twin);
    assert!(
        reader.is_closed(),
        "dropping the twin must join its workers"
    );
    assert_eq!(reader.status().state, CuTwinRecordingState::Closed);
    let (mut twin, mut reader) = Twin::twin(Rx(true))
        .with_log_path(dir.path().join("failed.copper"))
        .spawn()
        .unwrap();
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    while !reader.is_closed() {
        assert!(std::time::Instant::now() < deadline);
        reader.status();
        reader.wait_timeout(std::time::Duration::from_millis(10));
    }
    assert_eq!(reader.status().state, CuTwinRecordingState::Failed);
    assert!(
        twin.stop()
            .unwrap_err()
            .to_string()
            .contains("test transport failed")
    );
}
