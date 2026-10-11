use cu29_derive::copper_runtime;

#[copper_runtime(config = "tests/config/invalid_logging_config.ron")] //~ ERROR: cannot be larger than slab size
struct MyApplicationStruct;

fn main() {}
