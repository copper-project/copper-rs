use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use std::path::PathBuf;

#[copper_runtime(config = "copperconfig.ron")]
struct MyApp {}

fn main() {
    let mut copperconfig: CuConfig = read_configuration("copperconfig.ron").unwrap();

    let tmp_dir = tempfile::TempDir::new().expect("could not create a tmp dir");
    let logger_path = tmp_dir.path().join("test.copper");

    let copper_ctx = basic_copper_setup(&PathBuf::from(logger_path), None, true, None)
        .expect("Failed to setup logger.");

    // First run with the base configuration
    {
        let mut application = MyAppBuilder::new()
            .with_context(&copper_ctx)
            .with_config(copperconfig.clone())
            .build()
            .expect("Failed to create application.");

        application
            .start_all_tasks()
            .expect("Failed to start application.");

        // loop here until you need to change the configuration
        application
            .run_one_iteration()
            .expect("Failed to run one iteration.");
        application
            .stop_all_tasks()
            .expect("Failed to stop application.");

        // everything will be teared down here
    }

    // restart with a variation of the configuration
    {
        let node_indices = copperconfig
            .get_graph(None)
            .unwrap()
            .node_indices()
            .collect::<Vec<_>>();
        node_indices.iter().for_each(|node_index| {
            let node = copperconfig
                .get_node_mut(node_index.index() as NodeId, None)
                .unwrap();
            if node.get_id() == "dst" {
                node.set_param("pin", 42);
            }
        });

        let mut application = MyAppBuilder::new()
            .with_context(&copper_ctx)
            .with_config(copperconfig.clone())
            .build()
            .expect("Failed to create application.");
        application
            .start_all_tasks()
            .expect("Failed to start application.");

        // loop here until you need to change the configuration
        application
            .run_one_iteration()
            .expect("Failed to run one iteration.");
        application
            .stop_all_tasks()
            .expect("Failed to stop application.");
    }
}
