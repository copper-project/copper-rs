use cu29_derive::gen_cumsgs;

struct MyMsg;
struct FlippingSource;
struct FlippingSourceTwo;

gen_cumsgs!("config/non_existent_id.ron");

fn main() {}