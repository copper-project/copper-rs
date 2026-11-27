//! Message paths resolve from the module that invokes `gen_cumsgs!`, as they do
//! for `#[copper_runtime]`.

mod payloads {
    use bincode::{Decode, Encode};
    use cu29::prelude::*;

    #[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
    pub struct Count(pub u32);
}

mod simple {
    use super::payloads;

    cu29_derive::gen_cumsgs!("tests/config/relative_msg_path_valid.ron");
}

mod missions {
    use super::payloads;

    cu29_derive::gen_cumsgs!("tests/config/relative_msg_path_missions_valid.ron");
}

fn main() {}
