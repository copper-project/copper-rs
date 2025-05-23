use serde::{Deserialize, Serialize};

use crate::RosMsgAdapter;

// std_msgs/Int8 like struct
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Int8 {
    pub data: i8,
}

impl RosMsgAdapter<'static> for i8 {
    type Output = Int8;

    fn namespace() -> &'static str {
        "std_msgs"
    }
}

impl From<&i8> for Int8 {
    fn from(value: &i8) -> Self {
        Self { data: *value }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_adapter_type_info() {
        let x: i8 = 42;
        let msg = x.convert();
        assert_eq!(msg.data, 42);
        assert_eq!(<i8 as RosMsgAdapter>::namespace(), "std_msgs");
        assert_eq!(<i8 as RosMsgAdapter>::type_name(), "Int8");
    }
}
