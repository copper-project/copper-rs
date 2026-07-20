use crate::{exts::*, fields::*, *};

#[derive(ZStruct, Debug, PartialEq, Default)]
#[zenoh(header = "Z|P|C|ID:5=0x3")]
pub struct Query<'a> {
    #[zenoh(presence = header(C), default = ConsolidationMode::default())]
    pub consolidation: ConsolidationMode,
    #[zenoh(presence = header(P), size = prefixed, default = "")]
    pub parameters: &'a str,

    #[zenoh(ext = 0x1)]
    pub sinfo: Option<SourceInfo>,
    #[zenoh(ext = 0x3)]
    pub body: Option<Value<'a>>,
    #[zenoh(ext = 0x5)]
    pub attachment: Option<Attachment<'a>>,
}
