use cu29_derive::copper_runtime;

#[copper_runtime(config = "config/constants_invalid_expression.ron")]
struct App {}

fn main() {}
