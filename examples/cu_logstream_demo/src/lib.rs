#![cfg(feature = "demo")]

extern crate self as cu_logstream_demo;
pub mod tasks;

use cu29::prelude::*;

#[copper_runtime(config = "copperconfig.ron")]
struct Demo {}

pub type DataSet = default::CuStampedDataSet;
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
    drop(running.stop()?);
    Ok(())
}
