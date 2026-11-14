//! Open the custom payload directly in Rerun; no Copper debugger or server is involved.

use clap::Parser;
use cu_rerun_soft_arm::SoftArm;
use rerun::blueprint::components::{LoopMode, PlayState};
use rerun::blueprint::{Blueprint, Horizontal, Spatial3DView, TextDocumentView, TimePanel};
use std::path::PathBuf;
use std::time::Duration;

#[derive(Parser)]
#[command(about = "Three tendon-driven sections rendered from nine length measurements")]
struct Args {
    /// Write an RRD recording instead of opening the embedded viewer.
    #[arg(long)]
    save: Option<PathBuf>,
}

fn log_gesture(rec: &rerun::RecordingStream) -> Result<(), Box<dyn std::error::Error>> {
    rec.log_static("world", &rerun::ViewCoordinates::RIGHT_HAND_Z_UP())?;
    rec.log_static("guide", &rerun::TextDocument::from_markdown(
        "# Soft arm\n\nPress **Space** to pause or play, or scrub the timeline.\n\nNine tendon lengths become three bending sections. Rings show local orientation; the arrows show the tip frame.\n\nPink, green, and blue curves are the three tendons. The thick curve is the backbone."
    ))?;
    for frame in 0..=360 {
        rec.set_time("motion", Duration::from_secs_f64(frame as f64 / 30.0));
        let arm = SoftArm::at_phase(std::f32::consts::TAU * frame as f32 / 360.0);
        // This is the entire logging API. Geometry belongs to the payload implementation.
        rec.log("world/soft_arm", &arm)?;
    }
    rec.flush_blocking()?;
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let blueprint = Blueprint::new(
        Horizontal::new([
            Spatial3DView::new("Soft arm").with_origin("world").into(),
            TextDocumentView::new("Guide").with_origin("guide").into(),
        ])
        .with_column_shares([3.0, 1.0]),
    )
    .with_time_panel(
        TimePanel::new()
            .with_timeline("motion")
            .with_play_state(PlayState::Playing)
            .with_loop_mode(LoopMode::All),
    );
    let builder = rerun::RecordingStreamBuilder::new("Copper · soft arm").with_blueprint(blueprint);
    if let Some(path) = args.save {
        let rec = builder.save(path)?;
        log_gesture(&rec)?;
    } else {
        let main_thread = rerun::MainThreadToken::i_promise_i_am_on_the_main_thread();
        let (rec, storage) = builder.memory()?;
        log_gesture(&rec)?;
        rerun::native_viewer::show(main_thread, storage.take())?;
    }
    Ok(())
}
