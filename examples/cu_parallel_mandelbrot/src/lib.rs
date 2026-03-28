use cu29::prelude::*;
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

pub mod payloads;
pub mod tasks;

pub use payloads::MandelbrotStripe;

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

const SLAB_SIZE: Option<usize> = Some(512 * 1024 * 1024);

fn benchmark_logger_path(name: &str) -> CuResult<PathBuf> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("logs")
        .join(name);
    if let Some(parent) = path.parent()
        && !parent.exists()
    {
        fs::create_dir_all(parent).map_err(|err| {
            CuError::new_with_cause("failed to create benchmark log directory", err)
        })?;
    }
    Ok(path)
}

pub fn load_benchmark_settings() -> CuResult<tasks::BenchmarkSettings> {
    let bundled = <log_only::App as CuApplication<
        memmap::MmapSectionStorage,
        UnifiedLoggerWrite,
    >>::get_original_config();
    let config = CuConfig::deserialize_ron(&bundled)
        .map_err(|err| CuError::from(format!("failed to deserialize bundled config: {err}")))?;
    let graph = config.get_graph(Some("log_only"))?;

    for (_, node) in graph.get_all_nodes() {
        if node.get_id() == "src" {
            let source_config = node
                .get_instance_config()
                .ok_or_else(|| CuError::from("source node is missing config"))?;
            return tasks::BenchmarkSettings::from_component_config(source_config);
        }
    }

    Err(CuError::from(
        "could not find `src` node in Mandelbrot benchmark config",
    ))
}

fn log_summary(
    mission: &str,
    logger_path: &Path,
    settings: tasks::BenchmarkSettings,
    elapsed: std::time::Duration,
) {
    let logger_path_display = logger_path.display().to_string();
    let frames_emitted = tasks::frames_emitted();
    let last_digest = tasks::last_frame_digest();
    let stripes = settings.total_stripes() as f64;
    let rows = settings.total_rows() as f64;
    let elapsed_s = elapsed.as_secs_f64();
    let cl_hz = if elapsed_s > 0.0 {
        stripes / elapsed_s
    } else {
        0.0
    };
    let row_hz = if elapsed_s > 0.0 {
        rows / elapsed_s
    } else {
        0.0
    };
    let frame_hz = if elapsed_s > 0.0 {
        frames_emitted as f64 / elapsed_s
    } else {
        0.0
    };

    info!(
        "parallel-mandelbrot summary: mission={} logger={} frames_emitted={} expected_frames={} stripes={} rows={} elapsed_s={:.3} cl_hz={:.2} row_hz={:.2} frame_hz={:.2} last_frame_digest=0x{:016x}",
        mission,
        logger_path_display,
        frames_emitted,
        settings.frames,
        settings.total_stripes(),
        settings.total_rows(),
        elapsed_s,
        cl_hz,
        row_hz,
        frame_hz,
        last_digest
    );
}

pub fn run_log_only() -> CuResult<()> {
    tasks::reset_benchmark_summary();
    let settings = load_benchmark_settings()?;
    let logger_path = benchmark_logger_path("parallel_mandelbrot_log_only.copper")?;
    let logger_path_display = logger_path.display().to_string();
    let mut app = log_only::App::builder()
        .with_log_path(&logger_path, SLAB_SIZE)?
        .build()
        .map_err(|err| CuError::from(format!("failed to build log_only mission: {err}")))?;

    info!(
        "parallel-mandelbrot: mission=log_only logger={}",
        logger_path_display
    );
    let started = Instant::now();
    app.run()?;
    log_summary("log_only", &logger_path, settings, started.elapsed());
    Ok(())
}

pub fn run_viewer_live() -> CuResult<()> {
    tasks::reset_benchmark_summary();
    let settings = load_benchmark_settings()?;
    let logger_path = benchmark_logger_path("parallel_mandelbrot_viewer.copper")?;
    let logger_path_display = logger_path.display().to_string();
    let mut app = viewer_live::App::builder()
        .with_log_path(&logger_path, SLAB_SIZE)?
        .build()
        .map_err(|err| CuError::from(format!("failed to build viewer_live mission: {err}")))?;

    info!(
        "parallel-mandelbrot: mission=viewer_live logger={}",
        logger_path_display
    );
    let started = Instant::now();
    app.run()?;
    log_summary("viewer_live", &logger_path, settings, started.elapsed());
    Ok(())
}
