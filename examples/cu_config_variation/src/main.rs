use cu29::prelude::*;
use std::path::{Path, PathBuf};

#[copper_runtime(config = "copperconfig.ron")]
struct MyApp {}

fn run_once(app: &mut MyApp) -> CuResult<()> {
    app.start_all_tasks()?;
    app.run_one_iteration()?;
    app.stop_all_tasks()?;
    Ok(())
}

fn main() {
    let mut copperconfig: CuConfig = read_configuration("copperconfig.ron").unwrap();

    let logger_path = PathBuf::from("logs/cu_config_variation.copper");
    if let Some(parent) = Path::new(&logger_path).parent()
        && !parent.exists()
    {
        std::fs::create_dir_all(parent).expect("Failed to create logs directory");
    }
    let clock = RobotClock::default();

    // First run with the base configuration
    {
        let mut application = MyApp::builder()
            .with_clock(clock.clone())
            .with_log_path(&logger_path, None)
            .expect("Failed to setup logger.")
            .with_config(copperconfig.clone())
            .build()
            .expect("Failed to create application.");
        run_once(&mut application).expect("Failed to run application.");

        // everything will be teared down here
    }

    // restart with a variation of the configuration
    {
        let graph = copperconfig.get_graph_mut(None).unwrap();
        if let Some(node_id) = graph.get_node_id_by_name("dst") {
            let node = graph.get_node_mut(node_id).unwrap();
            node.set_param("pin", 42);
        }

        let mut application = MyApp::builder()
            .with_clock(clock.clone())
            .with_log_path(&logger_path, None)
            .expect("Failed to setup logger.")
            .with_config(copperconfig.clone())
            .build()
            .expect("Failed to create application.");
        run_once(&mut application).expect("Failed to run application.");
    }
}
