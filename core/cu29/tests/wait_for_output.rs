//! Recording barriers preserve typed lifecycles and wait for output completion.
#![cfg(feature = "std")]

use cu29::prelude::*;
use cu29::simulation::SimOverride;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, mpsc};
#[cfg(feature = "async-cl-io")]
use std::time::Duration;

#[derive(Reflect)]
struct Source;
impl Freezable for Source {}
impl CuSrcTask for Source {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(u32);

    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(42);
        Ok(())
    }
}

mod normal {
    use super::*;

    #[copper_runtime(config = "tests/wait_for_output.ron")]
    struct App {}

    #[cfg(feature = "async-cl-io")]
    pub(super) use default::App as TestApp;
}

mod simulation {
    use super::*;

    #[copper_runtime(config = "tests/wait_for_output.ron", sim_mode = true)]
    struct SimApp {}

    pub(super) use default::{SimApp as TestApp, SimStep};
}

#[cfg(feature = "async-cl-io")]
use normal::TestApp as App;
use simulation::TestApp as SimApp;

#[derive(Debug)]
struct Output {
    completed: Arc<AtomicUsize>,
    fail: bool,
    gate: Option<Mutex<mpsc::Receiver<()>>>,
}

impl<E: bincode::Encode> WriteStream<E> for Output {
    fn log(&mut self, _: &E) -> CuResult<()> {
        if let Some(gate) = self.gate.take() {
            gate.into_inner().unwrap().recv().unwrap();
        }
        if self.fail {
            return Err(CuError::from("injected recording failure"));
        }
        self.completed.fetch_add(1, Ordering::Release);
        Ok(())
    }
}

fn output(completed: &Arc<AtomicUsize>) -> Output {
    Output {
        completed: completed.clone(),
        fail: false,
        gate: None,
    }
}

fn log_path() -> (tempfile::TempDir, std::path::PathBuf) {
    let root =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/wait-for-output-tests");
    std::fs::create_dir_all(&root).unwrap();
    let dir = tempfile::TempDir::new_in(root).unwrap();
    let path = dir.path().join("recording.copper");
    (dir, path)
}

#[test]
fn test_sim_wait_completes_copperlists_and_keyframes_without_leaving_running() {
    let (_dir, path) = log_path();
    let completed = Arc::new(AtomicUsize::new(0));
    let mut callback = |step: simulation::SimStep<'_>| {
        if let simulation::SimStep::Source(cu29::simulation::CuTaskCallbackState::Process(
            _,
            output,
        )) = step
        {
            output.set_payload(42);
            SimOverride::ExecutedBySim
        } else {
            SimOverride::ExecuteByRuntime
        }
    };
    let mut app = SimApp::builder()
        .with_sim_callback(&mut callback)
        .with_log_path(&path, None)
        .unwrap()
        .build()
        .unwrap();
    app.copper_runtime_mut().copperlists_manager =
        cu29::curuntime::CopperListsManager::new_with_keyframes(
            Some(Box::new(output(&completed))),
            Some(Box::new(output(&completed))),
        )
        .unwrap();
    let mut app = app.start(&mut callback).unwrap();
    app.wait_for_output().unwrap(); // Empty output is also legal.
    for iteration in 1..=12 {
        app.run_one_iteration(&mut callback).unwrap();
        app.wait_for_output().unwrap();
        assert_eq!(completed.load(Ordering::Acquire), iteration * 2);
    }
    app.stop(&mut callback).unwrap();
}

#[cfg(feature = "async-cl-io")]
#[test]
fn test_wait_blocks_for_pending_recording_and_keeps_running() {
    let (_dir, path) = log_path();
    let completed = Arc::new(AtomicUsize::new(0));
    let (release, gate) = mpsc::channel();
    let mut sink = output(&completed);
    sink.gate = Some(Mutex::new(gate));
    let mut app = App::builder()
        .with_log_path(&path, None)
        .unwrap()
        .build()
        .unwrap();
    app.copper_runtime_mut().copperlists_manager =
        cu29::curuntime::CopperListsManager::new(Some(Box::new(sink))).unwrap();
    let mut app = app.start().unwrap();
    app.run_one_iteration().unwrap();
    assert_eq!(completed.load(Ordering::Acquire), 0);
    std::thread::scope(|scope| {
        scope.spawn(|| {
            std::thread::sleep(Duration::from_millis(100));
            release.send(()).unwrap();
        });
        app.wait_for_output().unwrap();
        assert_eq!(completed.load(Ordering::Acquire), 1);
    });
    app.run_one_iteration().unwrap();
    app.wait_for_output().unwrap();
    assert_eq!(completed.load(Ordering::Acquire), 2);
    app.stop().unwrap();
}

#[cfg(feature = "async-cl-io")]
#[test]
fn test_wait_reports_recording_failure() {
    let (_dir, path) = log_path();
    let completed = Arc::new(AtomicUsize::new(0));
    let mut sink = output(&completed);
    sink.fail = true;
    let mut app = App::builder()
        .with_log_path(&path, None)
        .unwrap()
        .build()
        .unwrap();
    app.copper_runtime_mut().copperlists_manager =
        cu29::curuntime::CopperListsManager::new(Some(Box::new(sink))).unwrap();
    let mut app = app.start().unwrap();
    app.run_one_iteration().unwrap();
    let error = app.wait_for_output().unwrap_err();
    assert!(error.to_string().contains("injected recording failure"));
    assert_eq!(completed.load(Ordering::Acquire), 0);
    // A wait error leaves the typed Running handle available for cleanup.
    assert!(app.stop().is_err());
}
