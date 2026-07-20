use crate::{exts::*, fields::*, *};

#[derive(ZStruct, Clone, Copy, Debug, PartialEq, Default)]
#[zenoh(header = "Z|_|R|ID:5=0x05")]
pub struct FrameHeader {
    #[zenoh(header = R)]
    pub reliability: Reliability,
    pub sn: u32,

    #[zenoh(ext = 0x1, default = QoS::default())]
    pub qos: QoS,
}
