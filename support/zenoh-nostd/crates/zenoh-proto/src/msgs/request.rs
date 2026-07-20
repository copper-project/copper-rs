use core::time::Duration;

use crate::{exts::*, fields::*, msgs::*, *};

#[derive(ZEnum, Debug, PartialEq)]
pub enum RequestBody<'a> {
    Query(Query<'a>),
}

impl Default for RequestBody<'_> {
    fn default() -> Self {
        RequestBody::Query(Query::default())
    }
}

#[derive(ZStruct, Debug, PartialEq, Default)]
#[zenoh(header = "Z|M|N|ID:5=0x1c")]
pub struct Request<'a> {
    pub id: u32,

    #[zenoh(flatten, shift = 5)]
    pub wire_expr: WireExpr<'a>,

    #[zenoh(ext = 0x1, default = QoS::default())]
    pub qos: QoS,
    #[zenoh(ext = 0x2)]
    pub timestamp: Option<Timestamp>,
    #[zenoh(ext = 0x3, default = NodeId::default(), mandatory)]
    pub nodeid: NodeId,
    #[zenoh(ext = 0x4, default = QueryTarget::default(), mandatory)]
    pub target: QueryTarget,
    #[zenoh(ext = 0x5)]
    pub budget: Option<Budget>,
    #[zenoh(ext = 0x6)]
    pub timeout: Option<Duration>,

    pub payload: RequestBody<'a>,
}
