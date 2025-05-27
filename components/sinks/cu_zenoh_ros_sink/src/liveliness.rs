use std::fmt;

use cu29::CuResult;
use zenoh::key_expr::KeyExpr;

use crate::{format_liveliness_keyexpr, node::Node, topic::Topic};

const ADMIN_SPACE: &str = "@ros2_lv";

enum Entity {
    Node,
    Publisher,
}

impl fmt::Display for Entity {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Entity::Node => write!(f, "NN"),
            Entity::Publisher => write!(f, "MP"),
        }
    }
}

pub fn node_liveliness(node: &Node) -> CuResult<KeyExpr<'static>> {
    liveliness_base_keyexpr(node, Entity::Node, node.id)
}

pub fn publisher_liveliness(
    node: &Node,
    topic: &Topic,
    instance_id: u32,
) -> CuResult<KeyExpr<'static>> {
    let base_keyexpr = liveliness_base_keyexpr(node, Entity::Publisher, instance_id)?;
    let topic_keyexpr =
        format_liveliness_keyexpr!(topic.name(), topic.type_name(), topic.hash(), topic.qos())?;

    Ok(base_keyexpr / topic_keyexpr.as_ref())
}

fn liveliness_base_keyexpr(
    node: &Node,
    entity: Entity,
    entity_id: u32,
) -> CuResult<KeyExpr<'static>> {
    // Admin space / Domain ID / ZID /  Node ID / Entity ID / Entity type /  Enclave /
    // Namespace / Node name

    format_liveliness_keyexpr!(
        ADMIN_SPACE,
        node.domain_id,
        node.zid,
        node.id,
        entity_id,
        entity,
        "", // enclave (useless?)
        node.namespace,
        node.name
    )
}
