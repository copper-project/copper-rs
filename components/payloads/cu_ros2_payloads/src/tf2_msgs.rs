//! `tf2_msgs` wire types.

use crate::RosMessage;
use crate::geometry_msgs::TransformStamped;
use serde::{Deserialize, Serialize};

/// `tf2_msgs/TFMessage`.
///
/// The message `/tf` and `/tf_static` actually carry, and the reason this type exists separately
/// from [`TransformStamped`]: `tf2_ros` subscribes to `/tf` as a `TFMessage` and nothing else, so
/// a graph publishing a bare `TransformStamped` on that topic publishes a type no transform
/// listener is subscribed to. Nothing errors — rmw keys include the type hash, so the two simply
/// never meet — and `rviz2` shows an empty tf tree with every frame "does not exist".
///
/// The wrapping is also what makes a tree possible at all: tf2 requires that all the transforms
/// making up one update arrive together, so a publisher sends the whole set in one message rather
/// than one message per edge.
///
/// `transforms` is a sequence, so it carries a `uint32` length prefix on the wire. An empty one is
/// legal and means "no update", not "clear the tree".
#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct TFMessage {
    pub transforms: Vec<TransformStamped>,
}

impl RosMessage for TFMessage {
    const NAMESPACE: &'static str = "tf2_msgs";
    const TYPE_NAME: &'static str = "TFMessage";
    const TYPE_HASH: &'static str =
        "RIHS01_e369d0f05a23ae52508854b66f6aa0437f3449d652e8cbf22d5abe85d020f087";
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builtin::{Header, Time};
    use crate::geometry_msgs::{Quaternion, Transform, Vector3};

    fn edge(parent: &str, child: &str, x: f64) -> TransformStamped {
        TransformStamped {
            header: Header {
                stamp: Time {
                    sec: 1_700_000_000,
                    nanosec: 250_000_000,
                },
                frame_id: parent.into(),
            },
            child_frame_id: child.into(),
            transform: Transform {
                translation: Vector3 { x, y: 0.0, z: 0.0 },
                rotation: Quaternion::default(),
            },
        }
    }

    fn roundtrip<T>(value: &T) -> (T, Vec<u8>)
    where
        T: Serialize + for<'de> Deserialize<'de>,
    {
        let bytes =
            cdr::serialize::<_, _, cdr::CdrLe>(value, cdr::Infinite).expect("cdr encode succeeds");
        let decoded = cdr::deserialize(bytes.as_slice()).expect("cdr decode succeeds");
        (decoded, bytes)
    }

    #[test]
    fn tf_message_roundtrips_a_chain_of_transforms() {
        // The shape a tf publisher actually sends: several edges of one tree in a single message.
        let value = TFMessage {
            transforms: vec![
                edge("odom", "base_link", 1.0),
                edge("base_link", "camera_link", 0.1),
            ],
        };

        let (decoded, bytes) = roundtrip(&value);
        assert_eq!(decoded, value);
        assert_eq!(decoded.transforms.len(), 2);
        // The sequence length prefix sits immediately after the 4-byte CDR encapsulation header.
        assert_eq!(bytes[4..8], 2u32.to_le_bytes());
    }

    #[test]
    fn empty_tf_message_roundtrips() {
        // Legal on the wire, and it must decode as "no update" rather than fail.
        let (decoded, bytes) = roundtrip(&TFMessage::default());
        assert_eq!(decoded, TFMessage::default());
        assert!(decoded.transforms.is_empty());
        assert_eq!(bytes.len(), 4 + 4);
    }
}
