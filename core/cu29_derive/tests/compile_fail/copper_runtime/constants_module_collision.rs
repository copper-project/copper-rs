use cu29_derive::copper_runtime;

#[copper_runtime(config = "tests/config/constants_module_collision.ron")] //~ ERROR: Constant module 'robot::drive' conflicts with constant 'drive'
struct App {}

fn main() {}
