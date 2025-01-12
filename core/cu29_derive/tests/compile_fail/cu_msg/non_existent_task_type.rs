use cu29_derive::gen_cumsgs;

struct MyMsg;

gen_cumsgs!("config/non_existent_task_type.ron");

fn main() {}