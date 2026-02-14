use serde::{Deserialize, Serialize};

use crate::RosMsgAdapter;

// ROS 2 transports (notably Iron+ metadata paths used by the bridge stack) carry a type hash
// string alongside the message type. These RIHS01 hashes are the canonical ROS 2 interface hashes
// generated from IDL definitions, so each adapter must expose the exact hash for wire compatibility.
macro_rules! define_scalar_adapter {
    ($rust:ty, $msg:ident, $hash:literal) => {
        #[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
        pub struct $msg {
            pub data: $rust,
        }

        impl RosMsgAdapter<'static> for $rust {
            type Output = $msg;

            fn namespace() -> &'static str {
                "std_msgs"
            }

            // RIHS01 hash for this ROS 2 message type (std_msgs/*).
            fn type_hash() -> &'static str {
                $hash
            }
        }

        impl From<&$rust> for $msg {
            fn from(value: &$rust) -> Self {
                Self { data: *value }
            }
        }

        impl TryFrom<$msg> for $rust {
            type Error = &'static str;

            fn try_from(value: $msg) -> Result<Self, Self::Error> {
                Ok(value.data)
            }
        }
    };
}

define_scalar_adapter!(
    bool,
    Bool,
    "RIHS01_feb91e995ff9ebd09c0cb3d2aed18b11077585839fb5db80193b62d74528f6c9"
);
define_scalar_adapter!(
    i8,
    Int8,
    "RIHS01_26525065a403d972cb672f0777e333f0c799ad444ae5fcd79e43d1e73bd0f440"
);
define_scalar_adapter!(
    i16,
    Int16,
    "RIHS01_1dcc3464e47c288a55f943a389d337cdb06804de3f5cd7a266b0de718eee17e5"
);
define_scalar_adapter!(
    i32,
    Int32,
    "RIHS01_b6578ded3c58c626cfe8d1a6fb6e04f706f97e9f03d2727c9ff4e74b1cef0deb"
);
define_scalar_adapter!(
    i64,
    Int64,
    "RIHS01_8cd1048c2f186b6bd9a92472dc1ce51723c0833a221e2b7aecfff111774f4b49"
);
define_scalar_adapter!(
    u8,
    UInt8,
    "RIHS01_6138bd83d8c3569cb80a667db03cfc1629f529fee79d944c39c34e352e72f010"
);
define_scalar_adapter!(
    u16,
    UInt16,
    "RIHS01_08a406e4b022bc22e907f985d6a9e9dd1d4fbecae573549cf49350113e7757b1"
);
define_scalar_adapter!(
    u32,
    UInt32,
    "RIHS01_a5c874829b752bc5fa190024b0ad76f578cc278271e855c7d02a818b3516fb4a"
);
define_scalar_adapter!(
    u64,
    UInt64,
    "RIHS01_fbdc52018fc13755dce18024d1a671c856aa8b4aaf63adfb095b608f98e8c943"
);
define_scalar_adapter!(
    f32,
    Float32,
    "RIHS01_7170d3d8f841f7be3172ce5f4f59f3a4d7f63b0447e8b33327601ad64d83d6e2"
);
define_scalar_adapter!(
    f64,
    Float64,
    "RIHS01_705ba9c3d1a09df43737eb67095534de36fd426c0587779bda2bc51fe790182a"
);

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct StringMsg {
    pub data: String,
}

impl RosMsgAdapter<'static> for String {
    type Output = StringMsg;

    fn namespace() -> &'static str {
        "std_msgs"
    }

    fn type_name() -> &'static str {
        "String"
    }

    fn type_hash() -> &'static str {
        "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18"
    }
}

impl From<&String> for StringMsg {
    fn from(value: &String) -> Self {
        Self {
            data: value.clone(),
        }
    }
}

impl TryFrom<StringMsg> for String {
    type Error = &'static str;

    fn try_from(value: StringMsg) -> Result<Self, Self::Error> {
        Ok(value.data)
    }
}

#[cfg(test)]
mod tests {
    use crate::RosBridgeAdapter;

    fn roundtrip<T>(value: T)
    where
        T: RosBridgeAdapter + PartialEq + core::fmt::Debug,
    {
        let ros_value = value.to_ros_message();
        let bytes = cdr::serialize::<_, _, cdr::CdrBe>(&ros_value, cdr::Infinite)
            .expect("cdr encode should succeed");
        let decoded_ros: <T as RosBridgeAdapter>::RosMessage =
            cdr::deserialize(bytes.as_slice()).expect("cdr decode should succeed");
        let recovered = T::from_ros_message(decoded_ros).expect("adapter decode should succeed");
        assert_eq!(value, recovered);
    }

    #[test]
    fn std_msgs_scalar_roundtrip() {
        roundtrip::<bool>(true);
        roundtrip::<i8>(-42);
        roundtrip::<i16>(-32000);
        roundtrip::<i32>(-1_000_000);
        roundtrip::<i64>(-9_000_000_000);
        roundtrip::<u8>(250);
        roundtrip::<u16>(65_000);
        roundtrip::<u32>(3_000_000_000);
        roundtrip::<u64>(9_000_000_000);
        roundtrip::<f32>(123.5);
        roundtrip::<f64>(-9876.125);
        roundtrip::<String>("Copper ROS2".to_string());
    }
}
