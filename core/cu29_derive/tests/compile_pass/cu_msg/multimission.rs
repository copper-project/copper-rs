use cu29_derive::gen_cumsgs;

gen_cumsgs!("config/multimission_valid.ron");

fn main() {
    let _a: cumsgs::A::CuStampedDataSet = Default::default();
    let _b: cumsgs::B::CuStampedDataSet = Default::default();
}
