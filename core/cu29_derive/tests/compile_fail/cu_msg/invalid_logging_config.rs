use cu29_derive::gen_cumsgs;

gen_cumsgs!("tests/config/invalid_logging_config.ron"); //~ ERROR: cannot be larger than slab size

fn main() {}
