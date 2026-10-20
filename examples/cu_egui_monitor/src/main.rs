//! An egui monitor example with a live Copper app and asynchronous SVG tiles.
mod graph;
mod monitor;
mod tasks;

use clap::Parser;
use cu29::prelude::*;
use eframe::egui;
use graph::GraphView;
use monitor::MonitorState;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::thread;
use std::time::Duration;

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

#[derive(Parser)]
#[command(about = "Live Copper monitor; --config previews another graph without running it")]
struct Args {
    /// Preview a single- or multi-Copper config instead of running the demo.
    #[arg(long)]
    config: Option<PathBuf>,
    /// Mission to render; omitted means all missions for a single-Copper graph.
    #[arg(long, requires = "config")]
    mission: Option<String>,
    /// Cargo features used to resolve conditional config fragments.
    #[arg(long, value_delimiter = ',', requires = "config")]
    features: Vec<String>,
}

struct RuntimeThread {
    state: Arc<MonitorState>,
    worker: Option<thread::JoinHandle<CuResult<()>>>,
}

impl RuntimeThread {
    fn start() -> CuResult<Self> {
        let (sender, receiver) = std::sync::mpsc::sync_channel(1);
        let worker = thread::Builder::new()
            .name("copper-demo".into())
            .spawn(move || {
                let mut app = match App::builder().build() {
                    Ok(app) => app,
                    Err(error) => {
                        let _ = sender.send(Err(error.clone()));
                        return Err(error);
                    }
                };
                // Clone the monitor's UI handle only during initialization.
                let state = app.inner_mut().copper_runtime_mut().monitor.state();
                let mut app = match app.start() {
                    Ok(app) => app,
                    Err(error) => {
                        let error = CuError::from(error);
                        let _ = sender.send(Err(error.clone()));
                        return Err(error);
                    }
                };
                if sender.send(Ok(Arc::clone(&state))).is_err() {
                    let _ = app.stop();
                    return Ok(());
                }
                let result = (|| {
                    while !state.stop_requested.load(Ordering::Relaxed) {
                        app.run_one_iteration()?;
                    }
                    Ok(())
                })();
                let stopped = app.stop().map_err(CuError::from);
                state.running.store(false, Ordering::Relaxed);
                result.and(stopped.map(|_| ()))
            })
            .map_err(|error| CuError::new_with_cause("Cannot start Copper demo", error))?;
        match receiver.recv() {
            Ok(Ok(state)) => Ok(Self {
                state,
                worker: Some(worker),
            }),
            Ok(Err(error)) => {
                let _ = worker.join();
                Err(error)
            }
            Err(error) => {
                let _ = worker.join();
                Err(CuError::new_with_cause(
                    "Copper demo initialization failed",
                    error,
                ))
            }
        }
    }
}

impl Drop for RuntimeThread {
    fn drop(&mut self) {
        self.state.stop_requested.store(true, Ordering::Relaxed);
        if let Some(worker) = self.worker.take() {
            match worker.join() {
                Ok(Ok(())) => {}
                Ok(Err(error)) => error!("Copper demo stopped with error: {}", error),
                Err(_) => error!("Copper demo thread panicked"),
            }
        }
    }
}

struct MonitorApp {
    graph: GraphView,
    runtime: Option<RuntimeThread>,
    title: String,
}

impl eframe::App for MonitorApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        if let Some(runtime) = &self.runtime {
            egui::Panel::left("monitor")
                .default_size(235.0)
                .show(ui, |ui| monitor_panel(ui, &runtime.state));
            ui.ctx().request_repaint_after(Duration::from_millis(100));
        }
        egui::CentralPanel::default().show(ui, |ui| {
            ui.heading(&self.title);
            self.graph.toolbar(ui);
            ui.separator();
            self.graph.show(ui);
        });
    }
}

fn monitor_panel(ui: &mut egui::Ui, state: &MonitorState) {
    ui.heading("Live Copper monitor");
    ui.label(format!("Mission: {}", state.metadata.mission_id()));
    ui.label(if state.running.load(Ordering::Relaxed) {
        "Running"
    } else {
        "Stopped"
    });
    ui.label(format!(
        "CopperLists: {}",
        state.cycles.load(Ordering::Relaxed)
    ));
    ui.label(format!("Errors: {}", state.errors.load(Ordering::Relaxed)));
    if ui.button("Stop runtime").clicked() {
        state.stop_requested.store(true, Ordering::Relaxed);
    }
    ui.small("Callbacks update preallocated atomics. Graph rendering runs on a worker.");
    ui.separator();
    ui.label("Latest message processing times");
    egui::ScrollArea::vertical()
        .id_salt("timings")
        .show(ui, |ui| {
            for (component, stats) in state
                .metadata
                .components()
                .iter()
                .zip(state.components.iter())
            {
                let ns = stats.duration_ns.load(Ordering::Relaxed);
                let timing = if ns == u64::MAX {
                    "—".into()
                } else {
                    format!("{:.2} µs", ns as f64 / 1000.0)
                };
                ui.horizontal(|ui| {
                    if stats.failed.load(Ordering::Relaxed) {
                        ui.colored_label(egui::Color32::LIGHT_RED, component.id());
                    } else {
                        ui.label(component.id());
                    }
                    ui.label(timing);
                });
            }
        });
}

fn preview_svg(args: &Args) -> CuResult<(String, String)> {
    let path = args
        .config
        .as_ref()
        .ok_or_else(|| CuError::from("Missing preview config"))?;
    let path_str = path
        .to_str()
        .ok_or_else(|| CuError::from("Config path is not UTF-8"))?;
    let features = args.features.iter().map(String::as_str).collect::<Vec<_>>();
    let svg = match cu29::config::read_multi_configuration_with_features(path_str, &features) {
        Ok(config) => cu29_graph_view::render_multi_config_svg(&config, args.mission.as_deref())?,
        Err(multi_error) => {
            let config = cu29::config::read_configuration_with_features(path_str, &features).map_err(|error| {
                CuError::from(format!("Cannot read graph as single or multi-Copper config. Single: {error}; multi: {multi_error}"))
            })?;
            cu29_graph_view::render_config_svg(&config, args.mission.as_deref())?
        }
    };
    Ok((svg, format!("Graph preview · {}", path.display())))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let (svg, title, runtime) = if args.config.is_some() {
        let (svg, title) = preview_svg(&args)?;
        (svg, title, None)
    } else {
        let config = CuConfig::deserialize_ron(include_str!("../copperconfig.ron"))?;
        (
            cu29_graph_view::render_config_svg(&config, None)?,
            "Copper task graph".into(),
            Some(RuntimeThread::start()?),
        )
    };
    eframe::run_native(
        "Copper egui monitor",
        eframe::NativeOptions {
            viewport: egui::ViewportBuilder::default().with_inner_size([1500.0, 950.0]),
            ..Default::default()
        },
        Box::new(move |cc| {
            cc.egui_ctx.set_visuals(egui::Visuals::dark());
            cc.egui_ctx
                .global_style_mut(|style| style.spacing.scroll = egui::style::ScrollStyle::solid());
            Ok(Box::new(MonitorApp {
                graph: GraphView::new(&svg, cc.egui_ctx.clone())?,
                runtime,
                title,
            }))
        }),
    )
    .map_err(|error| error.to_string().into())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn runtime_thread_observes_iterations_and_stops_on_drop() {
        // Copper's process-wide logger is initialized once: exercise one application
        // through its full thread lifecycle instead of building two apps in this test binary.
        let runtime = RuntimeThread::start().unwrap();
        let state = Arc::clone(&runtime.state);
        assert_eq!(state.metadata.component_count(), 3);
        assert!(state.running.load(Ordering::Relaxed));
        let deadline = std::time::Instant::now() + Duration::from_secs(2);
        while state.cycles.load(Ordering::Relaxed) < 3 && std::time::Instant::now() < deadline {
            thread::sleep(Duration::from_millis(5));
        }
        assert!(state.cycles.load(Ordering::Relaxed) >= 3);
        assert_eq!(state.errors.load(Ordering::Relaxed), 0);
        drop(runtime);
        assert!(!state.running.load(Ordering::Relaxed));
        assert!(
            state
                .components
                .iter()
                .any(|stats| stats.duration_ns.load(Ordering::Relaxed) != u64::MAX)
        );
    }

    #[test]
    fn flight_controller_deployment_renders_in_process() {
        let args = Args::try_parse_from([
            "monitor",
            "--config",
            concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../cu_flight_controller/flight_controller.ron"
            ),
            "--features",
            "sim",
            "--mission",
            "default",
        ])
        .unwrap();
        let (svg, _) = preview_svg(&args).unwrap();
        assert!(svg.contains("Subsystem: compute"));
        assert!(svg.contains("Subsystem: mcu"));
        assert!(svg.contains("stroke-dasharray"));
    }
}
