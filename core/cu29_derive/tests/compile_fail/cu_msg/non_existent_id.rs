use cu29_derive::gen_cumsgs;

struct MyMsg;
struct FlippingSource;
struct FlippingSourceTwo;

gen_cumsgs!("tests/config/non_existent_id.ron"); //~ ERROR: Source node not found: unknown_src

fn main() {}
