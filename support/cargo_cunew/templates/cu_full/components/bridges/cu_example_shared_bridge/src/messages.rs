use cu29::bincode::{Decode, Encode};
use serde::{Deserialize, Serialize};

// Shared bridge message type
#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize)]
#[bincode(crate = "cu29::bincode")]
pub struct SharedBridgePayload {
    pub value: i32,
}
