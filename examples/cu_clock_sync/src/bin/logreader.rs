use cu29::prelude::*;
gen_cumsgs!("mock.ron");
fn main() {
    cu29_export::run_cli::<CuMsgs>().expect("Clock sync log export failed");
}
