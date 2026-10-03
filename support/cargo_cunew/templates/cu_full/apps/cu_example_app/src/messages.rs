use cu29::bincode::{Decode, Encode};
use cu29::prelude::*;
use serde::{Deserialize, Serialize};

// Define a message type
#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
#[bincode(crate = "cu29::bincode")]
pub struct MyPayload {
    pub value: i32,
}
