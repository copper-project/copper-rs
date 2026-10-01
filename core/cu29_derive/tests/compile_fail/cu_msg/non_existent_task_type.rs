use cu29_derive::gen_cumsgs;

struct MyMsg;

gen_cumsgs!("tests/config/non_existent_task_type.ron"); //~ E0425

fn main() {}
