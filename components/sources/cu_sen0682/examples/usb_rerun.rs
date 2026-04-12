#[path = "common/usb_common.rs"]
mod usb_common;

use clap::Parser;
use cu_sensor_payloads::PointCloudSoa;
use cu29::prelude::*;
use cu29_logviz::{apply_tov, log_pointcloud};
use rerun::{Arrows3D, RecordingStream, RecordingStreamBuilder};
use std::path::PathBuf;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::thread;
use std::time::Duration;
use usb_common::{change_to_manifest_dir, override_serial_resource, resolve_usb_serial_port};

#[copper_runtime(config = "examples/usb_rerun.ron")]
struct UsbRerunApp {}

#[derive(Parser, Debug)]
#[command(name = "usb_rerun")]
struct Args {
    #[arg(long)]
    port: Option<PathBuf>,
    #[arg(long)]
    duration_s: Option<u64>,
}

#[derive(Reflect)]
#[reflect(from_reflect = false)]
struct Sen0682RerunSink {
    #[reflect(ignore)]
    rec: RecordingStream,
}

impl Freezable for Sen0682RerunSink {}

impl CuSinkTask for Sen0682RerunSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(PointCloudSoa<512>);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let rec = RecordingStreamBuilder::new("cu-sen0682-usb-rerun")
            .spawn()
            .map_err(|e| CuError::new_with_cause("Failed to spawn Rerun stream", e))?;

        rec.log_static("sen0682", &rerun::ViewCoordinates::RIGHT_HAND_Z_UP())
            .map_err(|e| CuError::new_with_cause("Failed to log world coordinates", e))?;
        log_axes(&rec, "sen0682", 0.15)?;

        Ok(Self { rec })
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        let Some(cloud) = input.payload() else {
            return Ok(());
        };

        apply_tov(&self.rec, &input.tov);
        log_pointcloud(&self.rec, "sen0682/pointcloud", cloud)
    }
}

fn log_axes(rec: &RecordingStream, path: &str, axis_len: f32) -> CuResult<()> {
    let axes = Arrows3D::from_vectors([
        [axis_len, 0.0, 0.0],
        [0.0, axis_len, 0.0],
        [0.0, 0.0, axis_len],
    ])
    .with_colors([
        rerun::Color::from([255, 0, 0]),
        rerun::Color::from([0, 255, 0]),
        rerun::Color::from([0, 0, 255]),
    ]);

    rec.log_static(format!("{path}/axes"), &axes)
        .map_err(|e| CuError::new_with_cause("Failed to log axes", e))
}

fn main() {
    if let Err(err) = run() {
        eprintln!("usb_rerun failed: {err}");
        std::process::exit(1);
    }
}

fn run() -> CuResult<()> {
    change_to_manifest_dir()?;

    let args = Args::parse();
    let port = resolve_usb_serial_port(args.port)?;

    let mut config = cu29::read_configuration("examples/usb_rerun.ron")?;
    override_serial_resource(&mut config, &port);

    let mut app = UsbRerunApp::builder().with_config(config).build()?;
    let clock = app.clock();
    app.start_all_tasks()?;

    let running = Arc::new(AtomicBool::new(true));
    let running_for_signal = Arc::clone(&running);
    ctrlc::set_handler(move || {
        running_for_signal.store(false, Ordering::Relaxed);
    })
    .map_err(|err| CuError::new_with_cause("failed to install Ctrl-C handler", err))?;

    match args.duration_s {
        Some(duration_s) => {
            println!(
                "streaming SEN0682 from {} into Rerun for {}s",
                port.display(),
                duration_s
            );
        }
        None => {
            println!(
                "streaming SEN0682 from {} into Rerun; press Ctrl-C to stop",
                port.display()
            );
        }
    }

    let deadline_ns = args
        .duration_s
        .map(|duration_s| {
            clock
                .now()
                .as_nanos()
                .saturating_add(CuDuration::from_secs(duration_s).as_nanos())
        })
        .unwrap_or(u64::MAX);

    let mut run_error: Option<CuError> = None;
    while running.load(Ordering::Relaxed) && clock.now().as_nanos() < deadline_ns {
        if let Err(err) = app.run_one_iteration() {
            run_error = Some(err);
            break;
        }

        thread::sleep(Duration::from_millis(5));
    }

    app.stop_all_tasks()?;

    if let Some(err) = run_error {
        return Err(err);
    }

    Ok(())
}
