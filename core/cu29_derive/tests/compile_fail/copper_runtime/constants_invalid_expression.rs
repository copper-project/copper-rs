use cu29_derive::copper_runtime;

#[copper_runtime(config = "tests/config/constants_invalid_expression.ron")] //~ ERROR: Constant 'BAD_EXPRESSION' expression is not a valid Rust expression
struct App {}

fn main() {}
