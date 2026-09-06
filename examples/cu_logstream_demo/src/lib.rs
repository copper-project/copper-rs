#![cfg(feature = "demo")]

extern crate self as cu_logstream_demo;
pub mod tasks;
pub mod telemetry;

use cu29::prelude::*;

#[copper_runtime(config = "copperconfig.ron")]
struct Demo {}

pub mod twin {
    use cu29::prelude::*;
    #[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
    struct Replay {}
    pub type DataSet = default::CuStampedDataSet;
    pub type Twin = default::Replay;
}
pub type DataSet = twin::DataSet;
pub type List = CopperList<DataSet>;

pub const ITERATIONS: u64 = 256;
pub const TICK_NS: u64 = 10_000_000;
// Generated native keyframe sections currently reserve 10 MiB.
pub const SLAB_BYTES: usize = 16 * 1024 * 1024;

pub fn read_lists(path: &std::path::Path) -> CuResult<Vec<List>> {
    Ok(
        cu29_export::copperlists_reader::<DataSet>(UnifiedLoggerIOReader::new(
            UnifiedLoggerRead::new(path)
                .map_err(|error| CuError::new_with_cause("Open archive", error))?,
            UnifiedLogType::CopperList,
        ))
        .collect(),
    )
}

pub fn run_sender(
    remote: std::net::SocketAddr,
    path: &std::path::Path,
    iterations: u64,
    idle_ms: u64,
) -> CuResult<()> {
    let mut config = CuConfig::deserialize_ron(include_str!("../copperconfig.ron"))?;
    config.resources[0]
        .config
        .as_mut()
        .unwrap()
        .set("remote_addr", remote.to_string());
    let (clock, mock) = RobotClock::mock();
    let app = Demo::builder()
        .with_clock(clock)
        .with_instance_id(41)
        .with_config(config)
        .with_log_path(path, Some(SLAB_BYTES))?
        .build()?;
    let mut running = app.start()?;
    for id in 0..iterations {
        mock.set_value((id + 1) * TICK_NS);
        running.run_one_iteration()?;
        // Low-rate example driving, outside task execution. This is not link pacing.
        std::thread::sleep(std::time::Duration::from_nanos(TICK_NS));
    }
    std::thread::sleep(std::time::Duration::from_millis(idle_ms));
    drop(running.stop()?);
    Ok(())
}

/// Read native captures for offline reconstruction, with no debug side data.
pub fn read_captures(
    path: &std::path::Path,
) -> CuResult<Vec<cu29_logstream::capture::CapturedList<DataSet>>> {
    Ok(read_lists(path)?
        .into_iter()
        .map(cu29_logstream::capture::CapturedList::new)
        .collect())
}

pub fn read_keyframes(path: &std::path::Path) -> CuResult<Vec<cu29::curuntime::KeyFrame>> {
    Ok(cu29_export::keyframes_reader(UnifiedLoggerIOReader::new(
        UnifiedLoggerRead::new(path).map_err(|e| CuError::new_with_cause("Open keyframes", e))?,
        UnifiedLogType::FrozenTasks,
    ))
    .collect())
}
