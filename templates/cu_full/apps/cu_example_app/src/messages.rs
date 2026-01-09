use bincode::{Decode, Encode};
use cu29::prelude::*;

// Define a message type
#[derive(Default, Debug, Clone, Encode, Decode, Serialize)]
pub struct MyPayload {
    pub value: i32,
}
