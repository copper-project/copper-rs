use cu_logmon::CuLogMon;
use cu29::prelude::*;
use log::{Level, LevelFilter, Metadata, Record};
use std::thread;
use std::time::Duration;

const MONITORED_COMPONENTS: &[MonitorComponentMetadata] = &[
    MonitorComponentMetadata::new("src", ComponentKind::Task, None),
    MonitorComponentMetadata::new("task", ComponentKind::Task, None),
    MonitorComponentMetadata::new("sink", ComponentKind::Task, None),
];

#[derive(Debug)]
struct NullStream;

impl WriteStream<CuLogEntry> for NullStream {
    fn log(&mut self, _obj: &CuLogEntry) -> CuResult<()> {
        Ok(())
    }
}

#[derive(Debug)]
struct StdoutLogger;

impl log::Log for StdoutLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Level::Info
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            println!("[{}] {}", record.level(), record.args());
        }
    }

    fn flush(&self) {}
}

fn build_metadata(ctx: &CuContext, work_micros: u64) -> CuMsgMetadata {
    let mut meta = CuMsgMetadata::default();
    let start = ctx.recent();
    let end = start + CuDuration::from_micros(work_micros);
    meta.process_time.start = OptionCuTime::from(start);
    meta.process_time.end = OptionCuTime::from(end);
    meta
}

fn main() -> CuResult<()> {
    // Set up logging so the example prints to stdout
    static LOGGER: StdoutLogger = StdoutLogger;
    log::set_logger(&LOGGER)
        .map(|()| log::set_max_level(LevelFilter::Info))
        .ok();
    let ctx = CuContext::new_with_clock();
    let _guard = LoggerRuntime::init(ctx.clock.clone(), NullStream, Some(StdoutLogger));
    info!("cu_logmon demo starting");

    let config = CuConfig::default();
    let metadata = CuMonitoringMetadata::new(
        DEFAULT_MISSION_ID.into(),
        MONITORED_COMPONENTS,
        &[],
        CopperListInfo::new(0, 0),
        MonitorTopology::default(),
        config
            .get_monitor_configs()
            .first()
            .and_then(|entry| entry.get_config().cloned()),
    )?;
    let runtime = CuMonitoringRuntime::unavailable();
    let mut monitor = CuLogMon::new(metadata, runtime)?;
    monitor.start(&ctx)?;

    let workloads: [u64; 3] = [500, 1500, 900]; // microseconds per task

    for _ in 0..30 {
        let metas: Vec<CuMsgMetadata> = workloads
            .iter()
            .map(|&micros| build_metadata(&ctx, micros))
            .collect();
        let refs: Vec<&CuMsgMetadata> = metas.iter().collect();
        monitor.process_copperlist(&ctx, &refs)?;
        thread::sleep(Duration::from_millis(100));
    }

    info!("cu_logmon demo finished");
    Ok(())
}
