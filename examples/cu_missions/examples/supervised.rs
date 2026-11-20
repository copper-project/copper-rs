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
        let stop_at = clock.raw_now() + CuDuration::from_secs(1);
        missions.run_until(|| clock.raw_now() >= stop_at)?;
    }
    missions.stop()
}

// This simulated camera can be opened by only one graph at a time.
static CAMERA_OPEN: AtomicBool = AtomicBool::new(false);

pub struct Camera;

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
        output.set_payload(ctx.cl_id());
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
        .with_clock(clock)
        .with_log_path(&path, None)
        .unwrap()
        .build()
        .unwrap();
    missions.start(AppMission::Idle).unwrap();
    assert!(CAMERA_OPEN.load(Ordering::Acquire));
    for mission in [AppMission::Recording, AppMission::Idle] {
        missions.transition_to(mission).unwrap();
        let mut remaining = 3;
        let start = std::time::Instant::now();
        missions
            .run_until(|| {
                remaining -= 1;
                remaining == 0
            })
            .unwrap();
        assert!(start.elapsed() >= std::time::Duration::from_millis(10));
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
    let mut missions = AppRunner::builder().build().unwrap();
    missions.start(AppMission::Idle).unwrap();
    assert_eq!(missions.run_until(|| true).unwrap(), CuRunExit::Requested);
    assert!(
        Command::new("kill")
            .args(["-INT", &std::process::id().to_string()])
            .status()
            .unwrap()
            .success()
    );
    let start = std::time::Instant::now();
    assert_eq!(
        missions
            .run_until(|| start.elapsed() > std::time::Duration::from_secs(3))
            .unwrap(),
        CuRunExit::Shutdown
    );
    assert_eq!(missions.run_until(|| true).unwrap(), CuRunExit::Shutdown);
    missions.stop().unwrap();
}
