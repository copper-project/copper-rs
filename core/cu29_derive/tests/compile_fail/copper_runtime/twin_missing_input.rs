use cu29_derive::copper_runtime;

#[copper_runtime(config = "config/reconstruct_missing_input.ron", sim_mode = true)]
struct App;

fn main() {}
