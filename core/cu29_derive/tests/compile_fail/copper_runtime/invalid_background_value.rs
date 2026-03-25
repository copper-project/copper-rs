use cu29_derive::copper_runtime;

#[copper_runtime(config = "config/invalid_background_value.ron")]
struct InvalidBackgroundApp;

fn main() {}
