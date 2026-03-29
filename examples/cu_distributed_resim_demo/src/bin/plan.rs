use cu_distributed_resim_demo::{
    DEFAULT_PLAN_PERIOD_MS, LOG_SLAB_SIZE, parse_run_options, sleep_period,
};
use cu29::prelude::*;

pub mod bridges {
    pub use cu_distributed_resim_demo::bridges::*;
}

pub mod messages {
    pub use cu_distributed_resim_demo::messages::*;
}

pub mod tasks {
    pub use cu_distributed_resim_demo::tasks::*;
}

#[copper_runtime(config = "multi_copper.ron", subsystem = "plan")]
struct PlanApp {}

fn main() {
    if let Err(err) = drive() {
        eprintln!("distributed-plan failed: {err}");
        std::process::exit(1);
    }
}

fn drive() -> CuResult<()> {
    let options = parse_run_options("plan.copper", DEFAULT_PLAN_PERIOD_MS)?;
    let mut app = PlanApp::builder()
        .with_log_path(&options.log_path, LOG_SLAB_SIZE)?
        .with_instance_id(options.instance_id)
        .build()?;
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
