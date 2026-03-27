use cu_distributed_resim_demo::{
    DEFAULT_CONTROL_PERIOD_MS, LOG_SLAB_SIZE, parse_run_options, sleep_period,
};
use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;

pub mod bridges {
    pub use cu_distributed_resim_demo::bridges::*;
}

pub mod messages {
    pub use cu_distributed_resim_demo::messages::*;
}

pub mod tasks {
    pub use cu_distributed_resim_demo::tasks::*;
}

#[copper_runtime(config = "multi_copper.ron", subsystem = "control")]
struct ControlApp {}

fn main() {
    if let Err(err) = drive() {
        eprintln!("distributed-control failed: {err}");
        std::process::exit(1);
    }
}

fn drive() -> CuResult<()> {
    let options = parse_run_options("control.copper", DEFAULT_CONTROL_PERIOD_MS)?;
    let ctx = basic_copper_setup(&options.log_path, LOG_SLAB_SIZE, true, None)?
        .with_instance_id(options.instance_id);

    let mut app = ControlAppBuilder::new().with_context(&ctx).build()?;
    app.start_all_tasks()?;

    if let Some(iterations) = options.iterations {
        for _ in 0..iterations {
            app.run_one_iteration()?;
            sleep_period(options.period);
        }
        app.stop_all_tasks()?;
        return Ok(());
    }

    loop {
        app.run_one_iteration()?;
        sleep_period(options.period);
    }
}
