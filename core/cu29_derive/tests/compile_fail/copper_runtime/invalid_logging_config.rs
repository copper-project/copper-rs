use cu29_derive::copper_runtime;

#[copper_runtime(config = "config/invalid_logging_config.ron")]
struct MyApplicationStruct;

fn main() {}