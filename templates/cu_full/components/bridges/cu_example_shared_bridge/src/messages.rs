use bincode::{Decode, Encode};
use cu29::prelude::*;
use serde::{Deserialize, Serialize};

// Shared bridge message type
#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize)]
pub struct SharedBridgePayload {
    pub value: i32,
}
