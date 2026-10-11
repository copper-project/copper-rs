use cu29_derive::copper_runtime;

#[copper_runtime(config = "tests/config/reconstruct_missing_input.ron", sim_mode = true)] //~ ERROR: input 'Image' from 'camera' has logging.enabled: false
struct App;

fn main() {}
