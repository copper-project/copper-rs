use cu29_derive::gen_cumsgs;

gen_cumsgs!("invalid/path/to/config.ron"); //~ ERROR: The configuration file `invalid/path/to/config.ron` does not exist.

fn main() {}
