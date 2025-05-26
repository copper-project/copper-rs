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

    fn type_hash() -> &'static str {
        "RIHS01_26525065a403d972cb672f0777e333f0c799ad444ae5fcd79e43d1e73bd0f440"
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
        let (msg, _) = x.convert();
        assert_eq!(msg.data, 42);
        assert_eq!(<i8 as RosMsgAdapter>::namespace(), "std_msgs");
        assert_eq!(<i8 as RosMsgAdapter>::type_name(), "Int8");
    }

    #[test]
    #[cfg(feature = "humble")]
    fn test_humble() {
        let x: i8 = 42;
        let (_, hash) = x.convert();
        assert_eq!(hash, "TypeHashNotSupported");
    }

    #[test]
    #[cfg(not(feature = "humble"))]
    fn test_not_humble() {
        let x: i8 = 42;
        let (_, hash) = x.convert();
        assert_eq!(
            hash,
            "RIHS01_26525065a403d972cb672f0777e333f0c799ad444ae5fcd79e43d1e73bd0f440"
        );
    }
}
