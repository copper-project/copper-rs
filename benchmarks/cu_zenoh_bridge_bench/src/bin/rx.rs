use cu_zenoh_bridge_bench::{
    BenchEndpointRole, bench_endpoint_summary, load_bench_config, parse_run_options,
    prepare_bench_transport,
};
use cu29::prelude::*;

pub mod bridges {
    pub use cu_zenoh_bridge_bench::bridges::*;
}

pub mod messages {
    pub use cu_zenoh_bridge_bench::messages::*;
}

pub mod tasks {
    pub use cu_zenoh_bridge_bench::tasks::*;
}

#[copper_runtime(config = "multi_copper.ron", subsystem = "rx")]
struct RxApp {}

const SLAB_SIZE: Option<usize> = Some(16 * 1024 * 1024);

fn main() {
    if let Err(err) = drive() {
        eprintln!("zenoh-bench-rx failed: {err}");
        std::process::exit(1);
    }
}

fn drive() -> CuResult<()> {
    let options = parse_run_options("zenoh_bench_rx.copper")?;
    prepare_bench_transport(BenchEndpointRole::Rx, options.transport)?;
    let config = load_bench_config(
        &RxApp::original_config(),
        BenchEndpointRole::Rx,
        options.transport,
    )?;
    let summary = bench_endpoint_summary(&config, BenchEndpointRole::Rx, options.transport)?;
    let transport_details = summary.transport_details();
    println!(
        "RX starting: instance_id={} rate_target_hz={} route={} {} log={}",
        options.instance_id,
        summary
            .rate_target_hz
            .map(|rate| rate.to_string())
            .unwrap_or_else(|| "unbounded".to_string()),
        summary.route,
        transport_details,
        options.log_path.display(),
    );
    let mut app = RxApp::builder()
        .with_log_path(&options.log_path, SLAB_SIZE)?
        .with_instance_id(options.instance_id)
        .with_config(config)
        .build()?;
    app.start_all_tasks()?;
    println!(
        "RX ready: subscribed on route={} {}",
        summary.route,
        summary.transport_details(),
    );
    let run_result = app.run();
    let stop_result = app.stop_all_tasks();
    run_result?;
    stop_result?;
    Ok(())
}
