use cu29_derive::copper_runtime;

const CONFIG_FILE: &str = "/path/to/config.ron";

#[copper_runtime(config = CONFIG_FILE)] //~ ERROR: expected string literal
struct MyApplicationStruct;

fn main() {}
