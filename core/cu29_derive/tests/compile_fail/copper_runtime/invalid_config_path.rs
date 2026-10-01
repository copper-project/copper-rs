use cu29_derive::copper_runtime;

#[copper_runtime(config = "path/to/config.ron")] //~ ERROR: The configuration file `path/to/config.ron` does not exist.
struct MyApplicationStruct;

fn main() {}
