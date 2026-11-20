//! Run the Copper graph while displaying its messages live in Rerun.

mod viewer;

use clap::Parser;
use cu29::prelude::*;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

#[copper_runtime(config = "copperconfig.ron")]
struct SoftArmApp {}

#[derive(Parser)]
#[command(about = "A live Copper soft-arm app with Rerun visualization and replayable logs")]
struct Args {
    /// Copper log base; each run replaces this log family.
    #[arg(long, default_value = concat!(env!("CARGO_MANIFEST_DIR"), "/logs/soft-arm.copper"))]
    log_base: PathBuf,
    /// Save Rerun data instead of opening a viewer (defaults to 360 frames).
    #[arg(long)]
    save: Option<PathBuf>,
    /// Stop after this many frames; otherwise run until the viewer closes or Ctrl-C.
    #[arg(long, value_parser = clap::value_parser!(u64).range(1..))]
    frames: Option<u64>,
}

fn record(args: &Args, rec: rerun::RecordingStream, stop: &AtomicBool) -> CuResult<()> {
    cu29::replay::remove_log_family(&args.log_base)?;
    let app = SoftArmApp::builder()
        .with_log_path(&args.log_base, Some(16 * 1024 * 1024))?
        .with_resources(move |_| cu_rerun_soft_arm::tasks::viewer_resources(rec))
        .build()?;
    let mut app = app.start()?;
    let mut remaining = args.frames.or_else(|| args.save.as_ref().map(|_| 360));
    let result = app.run_until(|| {
        if stop.load(Ordering::Relaxed) || remaining == Some(0) {
            return true;
        }
        if let Some(frames) = &mut remaining {
            *frames -= 1;
        }
        false
    });
    let stopped = app.stop();
    result?;
    stopped?;
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let stop = Arc::new(AtomicBool::new(false));
    let builder = viewer::recording_builder();
    if let Some(path) = args.save.as_ref() {
        let rec = builder.save(path)?;
        viewer::log_guide(&rec)?;
        record(&args, rec, &stop)?;
    } else {
        let (rec, receiver) = viewer::connect(builder)?;
        viewer::log_guide(&rec)?;
        let worker_stop = stop.clone();
        let worker = std::thread::spawn(move || {
            let result = record(&args, rec, &worker_stop);
            worker_stop.store(true, Ordering::Relaxed);
            result
        });
        let result = viewer::show(receiver, stop.clone());
        stop.store(true, Ordering::Relaxed);
        worker.join().map_err(|_| "Copper thread panicked")??;
        result?;
    }
    Ok(())
}
