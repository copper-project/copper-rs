use crate::{exts::*, fields::*, *};

#[derive(ZStruct, Debug, PartialEq, Default)]
#[zenoh(header = "Z|E|_|ID:5=0x5")]
pub struct Err<'a> {
    #[zenoh(presence = header(E), default = Encoding::default())]
    pub encoding: Encoding<'a>,

    #[zenoh(ext = 0x1)]
    pub sinfo: Option<SourceInfo>,

    #[zenoh(size = prefixed)]
    pub payload: &'a [u8],
}
