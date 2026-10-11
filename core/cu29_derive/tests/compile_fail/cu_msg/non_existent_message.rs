use cu29_derive::gen_cumsgs;

gen_cumsgs!("tests/config/non_existent_message.ron"); //~ E0425

fn main() {}
