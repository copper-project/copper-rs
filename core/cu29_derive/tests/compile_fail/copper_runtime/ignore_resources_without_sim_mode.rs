use cu29_derive::copper_runtime;

#[copper_runtime(
    config = "config/ignore_resources_sim_mode_valid.ron",
    ignore_resources = true
)]
struct App {}

fn main() {}
