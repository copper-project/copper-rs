use cu29::prelude::*;

gen_cumsgs!("copperconfig.ron");

fn main() -> CuResult<()> {
    cu29_export::run_cli::<CuMsgs>()
}
