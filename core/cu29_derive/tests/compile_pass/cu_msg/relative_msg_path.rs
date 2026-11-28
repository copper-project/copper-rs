//! Resolve payloads at the invocation site even when the caller defines its own Result.

mod payloads {
    use bincode::{Decode, Encode};
    use cu29::prelude::*;

    #[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
    pub struct Count(pub u32);
}

mod simple {
    use super::payloads;

    type Result<T> = std::io::Result<T>;

    cu29_derive::gen_cumsgs!("tests/config/relative_msg_path_valid.ron");
}

mod missions {
    type Result<T> = std::io::Result<T>;

    cu29_derive::gen_cumsgs!("tests/config/relative_msg_path_missions_valid.ron");

    fn check(a: cumsgs::A::CuStampedDataSet, b: cumsgs::B::CuStampedDataSet) {
        let _: &cu29::prelude::CuMsg<super::payloads::Count> = a.get_src_output();
        let _: &cu29::prelude::CuMsg<super::payloads::Count> = b.get_src_output();
    }
}

fn main() {}
