use cu29_derive::copper_runtime;

#[copper_runtime(config = "tests/config/constants_invalid_unit.ron")] //~ ERROR: Constant 'BAD_LENGTH' unit 'degree' is not compatible with quantity 'length'
struct App;

fn main() {}
