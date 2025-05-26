use cu29::CuResult;
use cu_ros_payloads::RosMsgAdapter;
use zenoh::key_expr::KeyExpr;

use crate::{format_keyexpr, node::Node};

pub struct Topic<'a> {
    name: &'a str,
    type_name: String,
    type_hash: String,
}

impl<'a> Topic<'a> {
    pub fn new<'b, T>(name: &'a str) -> Self
    where
        T: RosMsgAdapter<'b>,
    {
        Self {
            name,
            type_name: get_dds_type_name::<T>(),
            type_hash: T::type_hash().into(),
        }
    }

    pub fn name(&self) -> &str {
        self.name
    }

    pub fn type_name(&self) -> &str {
        self.type_name.as_ref()
    }

    pub fn hash(&self) -> &str {
        self.type_hash.as_ref()
    }

    pub fn qos(&self) -> &str {
        // TODO implement QoS
        "::,:,:,:,,"
    }

    pub fn pubsub_keyexpr(&self, node: &Node) -> CuResult<KeyExpr<'static>> {
        format_keyexpr!(node.domain_id, self.name(), self.type_name(), self.hash())
    }
}

fn get_dds_type_name<'a, T>() -> String
where
    T: RosMsgAdapter<'a>,
{
    // DDS name encoding relates to https://github.com/ros2/rmw_fastrtps/blob/469624e3d483290d6f88fe4b89ee5feaa7694e61/rmw_fastrtps_cpp/src/type_support_common.hpp
    format!("{}::msg::dds_::{}_", T::namespace(), T::type_name())
}
