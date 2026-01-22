use bincode::{Decode, Encode};
use serde::{Deserialize, Serialize};

// Define a message type
#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize)]
pub struct MyPayload {
    pub value: i32,
}
