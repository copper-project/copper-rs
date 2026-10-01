use cu29_derive::gen_cumsgs;

const CONFIG_FILE: &str = "/path/to/config.ron";

gen_cumsgs!(CONFIG_FILE); //~ ERROR: expected string literal

fn main() {}
