use bincode::{Decode, Encode};
use serde::Serialize;

// Define shared message types here.
#[derive(Default, Debug, Clone, Encode, Decode, Serialize)]
pub struct MyPayload {
    pub value: i32,
}
