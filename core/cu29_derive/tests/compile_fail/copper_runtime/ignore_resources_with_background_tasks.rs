use cu29_derive::copper_runtime;

#[copper_runtime(
    config = "config/ignore_resources_background_invalid.ron",
    sim_mode = true,
    ignore_resources = true
)]
struct App {}

fn main() {}
