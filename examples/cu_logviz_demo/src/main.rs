pub mod tasks;

use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use std::path::PathBuf;

#[copper_runtime(config = "copperconfig.ron")]
struct LogvizDemoApp {}

fn main() {
    let log_path = std::env::var_os("LOGVIZ_DEMO_LOG")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("logs/logviz_demo.copper"));

    if let Some(parent) = log_path.parent() {
        std::fs::create_dir_all(parent).expect("Failed to create log directory");
    }

    let copper_ctx = basic_copper_setup(&log_path, Some(10 * 1024 * 1024), false, None)
        .expect("Failed to setup copper");
    let mut application = LogvizDemoAppBuilder::new()
        .with_context(&copper_ctx)
        .build()
        .expect("Failed to create application");
    application
        .start_all_tasks()
        .expect("Failed to start tasks");

    for _ in 0..120 {
        application
            .run_one_iteration()
            .expect("Failed to run iteration");
    }

    println!("Log written to: {}", log_path.display());
    println!(
        "View with: cargo run -p cu-logviz-demo --bin cu-logviz-demo-logviz --features logviz -- {} --spawn",
        log_path.display()
    );
}
