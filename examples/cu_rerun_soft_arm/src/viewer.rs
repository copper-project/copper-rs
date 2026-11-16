//! In-process transport to the native Rerun viewer; the Copper graph runs on a worker thread.

use rerun::blueprint::components::PlayState;
use rerun::blueprint::{Blueprint, Horizontal, Spatial3DView, TextDocumentView, TimePanel};
use rerun::external::{eframe, re_build_info, re_log_channel, re_viewer};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

pub fn recording_builder() -> rerun::RecordingStreamBuilder {
    let blueprint = Blueprint::new(
        Horizontal::new([
            Spatial3DView::new("Soft arm").with_origin("world").into(),
            TextDocumentView::new("Guide").with_origin("guide").into(),
        ])
        .with_column_shares([3.0, 1.0]),
    )
    .with_time_panel(
        TimePanel::new()
            .with_timeline("cu_time")
            .with_play_state(PlayState::Following),
    );
    rerun::RecordingStreamBuilder::new("Copper · soft arm").with_blueprint(blueprint)
}

pub fn log_guide(rec: &rerun::RecordingStream) -> rerun::RecordingStreamResult<()> {
    rec.log_static("world", &rerun::ViewCoordinates::RIGHT_HAND_Z_UP())?;
    // Fixed workspace bounds keep the initial camera framing useful as the arm bends.
    rec.log_static(
        "world/workspace",
        &rerun::Boxes3D::from_half_sizes([[0.9, 0.9, 0.7]])
            .with_centers([[0.0, 0.0, 0.6]])
            .with_colors([[65, 65, 65]])
            .with_radii([0.001]),
    )?;
    rec.log_static("guide", &rerun::TextDocument::from_markdown(
        "# Soft arm\n\nLive Copper messages at 30 Hz. Press **Space** to pause the view, or scrub the timeline.\n\nNine tendon lengths become three bending sections. Rings show local orientation; arrows show the tip frame.\n\nPink, green, and blue curves are the tendons. The thick curve is the backbone.\n\nClose the viewer to finish recording. Run `just debug` to inspect the Copper log in Timetraveler."
    ))
}

pub fn connect(
    builder: rerun::RecordingStreamBuilder,
) -> rerun::RecordingStreamResult<(rerun::RecordingStream, re_log_channel::LogReceiver)> {
    let (sender, receiver) = re_log_channel::log_channel(re_log_channel::LogSource::Sdk);
    let sink = rerun::sink::CallbackSink::new(move |messages| {
        for message in messages {
            let _ = sender.send(message.clone().into());
        }
    });
    let sinks: Vec<Box<dyn rerun::sink::LogSink>> = vec![Box::new(sink)];
    Ok((builder.set_sinks(sinks)?, receiver))
}

struct LiveViewer {
    app: re_viewer::App,
    stop: Arc<AtomicBool>,
}

impl eframe::App for LiveViewer {
    fn logic(&mut self, ctx: &eframe::egui::Context, frame: &mut eframe::Frame) {
        self.app.logic(ctx, frame);
        if self.stop.load(Ordering::Relaxed) {
            ctx.send_viewport_cmd(eframe::egui::ViewportCommand::Close);
        }
        ctx.request_repaint_after(std::time::Duration::from_millis(33));
    }

    fn ui(&mut self, ui: &mut eframe::egui::Ui, frame: &mut eframe::Frame) {
        self.app.ui(ui, frame);
    }

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        self.app.save(storage);
    }
}

pub fn show(
    receiver: re_log_channel::LogReceiver,
    stop: Arc<AtomicBool>,
) -> Result<(), Box<dyn std::error::Error>> {
    let main_thread = rerun::MainThreadToken::i_promise_i_am_on_the_main_thread();
    let runtime = tokio::runtime::Runtime::new()?;
    let runtime_handle = re_viewer::AsyncRuntimeHandle::new_native(runtime.handle().clone());
    re_viewer::run_native_app(
        main_thread,
        Box::new(move |cc| {
            let mut app = re_viewer::App::new(
                main_thread,
                // Viewer persistence uses Rerun's version, not Copper's prerelease syntax.
                re_build_info::BuildInfo {
                    crate_name: "rerun".into(),
                    version: re_build_info::CrateVersion::LOCAL,
                    features: "".into(),
                    rustc_version: "".into(),
                    llvm_version: "".into(),
                    git_hash: "".into(),
                    git_branch: "".into(),
                    is_in_rerun_workspace: false,
                    target_triple: "".into(),
                    datetime: "".into(),
                    is_debug_build: cfg!(debug_assertions),
                },
                re_viewer::AppEnvironment::Custom("Copper soft arm".into()),
                re_viewer::StartupOptions::default(),
                cc,
                None,
                runtime_handle,
            );
            app.add_log_receiver(receiver);
            Ok(Box::new(LiveViewer { app, stop }))
        }),
        None,
    )?;
    Ok(())
}
