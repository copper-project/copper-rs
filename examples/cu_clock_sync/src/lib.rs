use cu29::clock::sync::{ClockDomain, ClockObservation};
use cu29::clock_sync::{ClockReference, ClockReferenceBundle};
use cu29::prelude::*;
use cu29::resource::{BundleContext, ResourceBundle, ResourceManager};
use cu29_unifiedlog::{UnifiedLoggerWrite, memmap::MmapSectionStorage};
use std::time::Duration;

/// Emits a timestamp on the shared timeline; inspect it through the logreader.
#[derive(Reflect)]
pub struct Timestamp;
impl Freezable for Timestamp {}
impl CuSrcTask for Timestamp {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(u64);
    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        Ok(Self)
    }
    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let time = ctx.now();
        output.set_payload(time.0);
        output.tov = Tov::Time(time);
        Ok(())
    }
}

#[derive(Reflect)]
pub struct Consume;
impl Freezable for Consume {}
impl CuSinkTask for Consume {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(u64);
    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        Ok(Self)
    }
    fn process(&mut self, _: &CuContext, _: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}

pub struct MockPtpBundle;
cu29::bundle_resources!(MockPtpBundle: Reference = "reference");
impl ClockReferenceBundle for MockPtpBundle {
    type Reference = MockPtp;
}
impl ResourceBundle for MockPtpBundle {
    fn build(
        bundle: BundleContext<Self>,
        _: Option<&ComponentConfig>,
        manager: &mut ResourceManager,
    ) -> CuResult<()> {
        manager.add_owned(
            bundle.key(MockPtpBundleId::Reference),
            MockPtp { anchor: None },
        )
    }
}

/// Deterministic parent model with a 20 ppm relative rate difference.
pub struct MockPtp {
    anchor: Option<CuInstant>,
}
const DOMAIN: ClockDomain = ClockDomain {
    id: 0,
    identity: *b"demo-ptp",
    session: 1,
};
impl ClockReference for MockPtp {
    fn create_clock(&self) -> CuResult<RobotClock> {
        Ok(RobotClock::new())
    }
    fn start(&mut self) -> CuResult<()> {
        Ok(())
    }
    fn domain(&self) -> ClockDomain {
        DOMAIN
    }
    fn poll(&mut self, clock: &RobotClock) -> CuResult<Option<ClockObservation>> {
        let raw = clock.raw_now();
        let elapsed = (raw - *self.anchor.get_or_insert(raw)).as_nanos();
        Ok(Some(ClockObservation {
            raw_local: raw,
            parent_ns: 1_800_000_000_000_000_000 + elapsed + elapsed / 50_000,
            uncertainty: CuDuration(100),
            domain: DOMAIN,
        }))
    }
    fn stop(&mut self) -> CuResult<()> {
        Ok(())
    }
}

pub fn run<A: CuApplication<MmapSectionStorage, UnifiedLoggerWrite>>(
    app: CuAppLifecycle<MmapSectionStorage, UnifiedLoggerWrite, A>,
    clock: &RobotClock,
) -> CuResult<()> {
    let mut app = app.start().map_err(|failure| failure.error)?;
    for _ in 0..100 {
        app.run_one_iteration()?;
        std::thread::sleep(Duration::from_millis(50));
    }
    app.stop().map_err(|failure| failure.error)?;
    println!("Reference quality: {:?}", clock.sync_status());
    Ok(())
}

pub fn log_path(name: &str) -> CuResult<std::path::PathBuf> {
    let directory = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("logs");
    std::fs::create_dir_all(&directory)
        .map_err(|error| CuError::new_with_cause("Cannot create example log directory", error))?;
    Ok(directory.join(format!("{name}.copper")))
}
