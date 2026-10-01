use cu29_derive::gen_cumsgs;

gen_cumsgs!("tests/config/invalid_config.ron"); //~ ERROR: Failed to parse configuration

fn main() {}
