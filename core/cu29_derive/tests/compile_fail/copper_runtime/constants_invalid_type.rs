use cu29_derive::copper_runtime;

#[copper_runtime(config = "tests/config/constants_invalid_type.ron")] //~ ERROR: Constant 'BAD_TYPE' type 'crate::Pair<' is not a valid Rust type
struct App {}

fn main() {}
