pub mod tasks;

use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use std::path::PathBuf;
use std::thread::sleep;
use std::time::Duration;

const PREALLOCATED_STORAGE_SIZE: Option<usize> = Some(1024 * 1024 * 100);

#[copper_runtime(config = "copperconfig.ron")]
struct {{project-name | upper_camel_case}}Application {}

fn main() {
    let logger_path = "{{project-name | kebab_case}}.copper";
    let copper_ctx =
        basic_copper_setup(&PathBuf::from(logger_path), PREALLOCATED_STORAGE_SIZE, true, None).expect("Failed to setup logger.");
    debug!("Logger created at {}.", logger_path);
    let clock = copper_ctx.clock;
    debug!("Creating application... ");
    let mut application = {{project-name | upper_camel_case}}Builder::new()
            .with_context(&copper_ctx)
            .build()
            .expect("Failed to create application.");
    debug!("Running... starting clock: {}.", clock.now());

    application.run().expect("Failed to run application.");
    debug!("End of program: {}.", clock.now());
    sleep(Duration::from_secs(1));
}
