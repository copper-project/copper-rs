use bincode::{Decode, Encode};
use cu29::prelude::*;

// Shared bridge message type
#[derive(Default, Debug, Clone, Encode, Decode, Serialize)]
pub struct SharedBridgePayload {
    pub value: i32,
}
