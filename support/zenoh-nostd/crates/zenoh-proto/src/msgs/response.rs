use crate::{exts::*, fields::*, msgs::*, *};

#[derive(ZEnum, Debug, PartialEq)]
pub enum ResponseBody<'a> {
    Err(Err<'a>),
    Reply(Reply<'a>),
}

impl Default for ResponseBody<'_> {
    fn default() -> Self {
        ResponseBody::Err(Err::default())
    }
}

#[derive(ZStruct, Debug, PartialEq, Default)]
#[zenoh(header = "Z|M|N|ID:5=0x1b")]
pub struct Response<'a> {
    pub rid: u32,

    #[zenoh(flatten, shift = 5)]
    pub wire_expr: WireExpr<'a>,

    #[zenoh(ext = 0x1, default = QoS::default())]
    pub qos: QoS,
    #[zenoh(ext = 0x2)]
    pub timestamp: Option<Timestamp>,
    #[zenoh(ext = 0x3)]
    pub respid: Option<EntityGlobalId>,

    pub payload: ResponseBody<'a>,
}

#[derive(ZStruct, Debug, PartialEq, Default)]
#[zenoh(header = "Z|_:2|ID:5=0x1a")]
pub struct ResponseFinal {
    pub rid: u32,

    #[zenoh(ext = 0x1, default = QoS::default())]
    pub qos: QoS,
    #[zenoh(ext = 0x2)]
    pub timestamp: Option<Timestamp>,
}
