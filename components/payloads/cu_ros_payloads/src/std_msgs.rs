use serde::{Deserialize, Serialize};

use crate::RosMsgAdapter;

// std_msgs/Int8 like struct
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Int8 {
    pub data: i8,
}

impl RosMsgAdapter for i8 {
    type Output = Int8;

    fn convert(&self) -> Self::Output {
        Int8 { data: *self }
    }

    fn namespace() -> &'static str {
        "std_msgs"
    }

    fn type_name() -> &'static str {
        "Int8"
    }
}
