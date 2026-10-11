use cu29_derive::copper_runtime;

#[copper_runtime( //~ ERROR: `ignore_resources` is only supported when `sim_mode` is enabled
    config = "tests/config/ignore_resources_sim_mode_valid.ron",
    ignore_resources = true
)]
struct App {}

fn main() {}
