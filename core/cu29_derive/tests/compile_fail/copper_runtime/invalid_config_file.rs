use cu29_derive::copper_runtime;

#[copper_runtime(config = "tests/config/invalid_config.ron")] //~ ERROR: Failed to parse configuration
struct MyApplicationStruct;

fn main() {}
