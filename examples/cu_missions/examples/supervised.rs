//! Run with `cargo run -p cu-missions --example supervised`.
use cu29::prelude::*;
use std::sync::atomic::{AtomicBool, Ordering};

#[copper_runtime(config = "examples/supervised.ron")]
struct App {}

fn main() -> CuResult<()> {
    let clock = RobotClock::default();
    let mut missions = AppRunner::builder()
        .with_clock(clock.clone())
        .with_log_path("missions.copper", None)?
        .build()?;
    missions.start(AppMission::Idle)?;
    let switch_at = clock.raw_now() + CuDuration::from_secs(1);
    if missions.run_until(|| clock.raw_now() >= switch_at)? == CuRunExit::Requested {
        missions.transition_to(AppMission::Recording)?;
        missions.wait_until_ready(CuDuration::from_secs(2), Camera::ready)?;
        let stop_at = clock.raw_now() + CuDuration::from_secs(1);
        if missions.run_until(|| clock.raw_now() >= stop_at)? == CuRunExit::Requested {
            Camera::request_eos();
            missions.drain_until(CuDuration::from_secs(2), Camera::drained)?;
            missions.transition_to(AppMission::Idle)?;
        }
    }
    missions.stop()
}

// This simulated camera can be opened by only one graph at a time.
static CAMERA_OPEN: AtomicBool = AtomicBool::new(false);

// The frame source acknowledges end-of-stream after processing its final frame.
static STREAMING: AtomicBool = AtomicBool::new(false);
static EOS_REQUESTED: AtomicBool = AtomicBool::new(false);

pub struct Camera;

impl Camera {
    fn ready() -> bool {
        STREAMING.load(Ordering::Acquire)
    }

    fn request_eos() {
        EOS_REQUESTED.store(true, Ordering::Release);
    }

    fn drained() -> bool {
        EOS_REQUESTED.load(Ordering::Acquire) && !Self::ready()
    }
}

impl Drop for Camera {
    fn drop(&mut self) {
        CAMERA_OPEN.store(false, Ordering::Release);
    }
}

pub struct CameraBundle;
bundle_resources!(CameraBundle: Device);

impl ResourceBundle for CameraBundle {
    fn build(
        bundle: BundleContext<Self>,
        _: Option<&ComponentConfig>,
        resources: &mut ResourceManager,
    ) -> CuResult<()> {
        if CAMERA_OPEN.swap(true, Ordering::AcqRel) {
            return Err("camera already open".into());
        }
        STREAMING.store(false, Ordering::Release);
        EOS_REQUESTED.store(false, Ordering::Release);
        resources.add_owned(bundle.key(CameraBundleId::Device), Camera)
    }
}

mod frame_resources {
    use super::*;
    cu29::resources!({ camera => Owned<Camera> });
}

#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct FrameSource {
    #[reflect(ignore)]
    _camera: Camera,
}

impl Freezable for FrameSource {}

impl CuSrcTask for FrameSource {
    type Resources<'r> = frame_resources::Resources;
    type Output<'m> = output_msg!(u64);

    fn new(_: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {
            _camera: resources.camera.0,
        })
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        if !Camera::drained() {
            output.set_payload(ctx.cl_id());
            STREAMING.store(!EOS_REQUESTED.load(Ordering::Acquire), Ordering::Release);
        }
        Ok(())
    }
}

#[test]
fn transitions_release_the_camera_and_keep_each_recording() {
    let root = tempfile::tempdir_in(env!("CARGO_MANIFEST_DIR")).unwrap();
    let clock = RobotClock::default();
    let path = root.path().join("missions.copper");
    let mut missions = AppRunner::builder()
        .with_instance_id(41)
        .with_clock(clock.clone())
        .with_log_path(&path, None)
        .unwrap()
        .build()
        .unwrap();
    missions.start(AppMission::Idle).unwrap();
    assert!(CAMERA_OPEN.load(Ordering::Acquire));
    for mission in [AppMission::Recording, AppMission::Idle] {
        missions.transition_to(mission).unwrap();
        assert!(!Camera::ready());
        assert!(matches!(
            missions.wait_until_ready(CuDuration::from_nanos(0), Camera::ready),
            Err(CuWaitError::Timeout)
        ));
        missions
            .wait_until_ready(CuDuration::from_secs(2), Camera::ready)
            .unwrap();
        let mut remaining = 3;
        let start = clock.raw_now();
        missions
            .run_until(|| {
                remaining -= 1;
                remaining == 0
            })
            .unwrap();
        assert!(clock.raw_now() - start >= CuDuration::from_millis(10));
        Camera::request_eos();
        assert!(!Camera::drained());
        missions
            .drain_until(CuDuration::from_secs(2), Camera::drained)
            .unwrap();
        assert!(Camera::drained());
    }
    missions.stop().unwrap();
    assert!(!CAMERA_OPEN.load(Ordering::Acquire));
    drop(missions);
    let mut reader = UnifiedLoggerRead::new(&path).unwrap();
    let mut runs = std::collections::BTreeMap::new();
    loop {
        let header = reader.raw_skip_section().unwrap();
        match header.entry_type {
            UnifiedLogType::LastEntry => break,
            UnifiedLogType::RuntimeLifecycle => {
                assert_eq!(header.context.instance_id, 41);
                runs.insert(header.context.run_id, header.context.mission_index);
            }
            _ => {}
        }
    }
    assert_eq!(runs.into_values().collect::<Vec<_>>(), [0, 1, 0]);
}

#[cfg(all(test, unix))]
#[test]
fn copper_shutdown_takes_precedence_over_the_caller_request() {
    use std::process::Command;
    const NAME: &str = "copper_shutdown_takes_precedence_over_the_caller_request";
    if std::env::var_os("CU_TEST_SUPERVISION_CHILD").is_none() {
        let status = Command::new(std::env::current_exe().unwrap())
            .args(["--exact", NAME])
            .env("CU_TEST_SUPERVISION_CHILD", "1")
            .status()
            .unwrap();
        assert!(status.success());
        return;
    }
    let clock = RobotClock::default();
    let mut missions = AppRunner::builder()
        .with_clock(clock.clone())
        .build()
        .unwrap();
    missions.start(AppMission::Idle).unwrap();
    assert_eq!(missions.run_until(|| true).unwrap(), CuRunExit::Requested);
    assert!(
        Command::new("kill")
            .args(["-INT", &std::process::id().to_string()])
            .status()
            .unwrap()
            .success()
    );
    let start = clock.raw_now();
    assert_eq!(
        missions
            .run_until(|| clock.raw_now() - start > CuDuration::from_secs(3))
            .unwrap(),
        CuRunExit::Shutdown
    );
    assert_eq!(missions.run_until(|| true).unwrap(), CuRunExit::Shutdown);
    assert!(matches!(
        missions.drain_until(CuDuration::from_secs(2), || true),
        Err(CuWaitError::Shutdown)
    ));
    missions.stop().unwrap();
}
