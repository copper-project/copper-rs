use cu29_derive::copper_runtime;

#[copper_runtime(config = "tests/config/constants_invalid_module.ron")] //~ ERROR: Constant 'COUNT' module path '::diagnostics' must be relative
struct App {}

fn main() {}
