use cu29_derive::copper_runtime;

#[copper_runtime(config = "config/constants_invalid_module.ron")]
struct App {}

fn main() {}
