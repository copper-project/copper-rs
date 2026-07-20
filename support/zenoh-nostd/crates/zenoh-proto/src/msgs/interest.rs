use crate::{exts::*, fields::*, *};

#[repr(u8)]
#[derive(ZRU8, Debug, Clone, Copy, PartialEq, Default)]
pub enum InterestMode {
    #[default]
    Final = 0b00,
    Current = 0b01,
    Future = 0b10,
    CurrentFuture = 0b11,
}

#[derive(ZStruct, Debug, Default)]
#[zenoh(header = "A|M|N|R|T|Q|S|K")]
pub struct InterestInner<'a> {
    #[zenoh(header = FULL)]
    pub options: u8,

    #[zenoh(presence = header(R), flatten, shift = 5)]
    pub wire_expr: Option<WireExpr<'a>>,
}

#[derive(ZStruct, Debug, PartialEq, Default)]
#[zenoh(header = "Z|MODE:2|ID:5=0x19")]
pub struct Interest<'a> {
    pub id: u32,

    #[zenoh(header = MODE)]
    pub mode: InterestMode,

    pub inner: InterestInner<'a>,

    #[zenoh(ext = 0x1, default = QoS::default())]
    pub qos: QoS,
    #[zenoh(ext = 0x2)]
    pub timestamp: Option<Timestamp>,
    #[zenoh(ext = 0x3, default = NodeId::default(), mandatory)]
    pub nodeid: NodeId,
}

#[derive(ZStruct, Debug, PartialEq, Default)]
#[zenoh(header = "Z|MODE:2=0x0|ID:5=0x19")]
pub struct InterestFinal {
    pub id: u32,

    #[zenoh(ext = 0x1, default = QoS::default())]
    pub qos: QoS,
    #[zenoh(ext = 0x2)]
    pub timestamp: Option<Timestamp>,
    #[zenoh(ext = 0x3, default = NodeId::default(), mandatory)]
    pub nodeid: NodeId,
}

#[repr(transparent)]
#[derive(Clone, Copy, Debug)]
pub struct InterestOptions {
    pub options: u8,
}

impl PartialEq for InterestInner<'_> {
    fn eq(&self, other: &Self) -> bool {
        let options = InterestOptions::options(self.options);
        let other_options = InterestOptions::options(other.options);

        options == other_options && self.wire_expr == other.wire_expr
    }
}

impl PartialEq for InterestOptions {
    fn eq(&self, other: &Self) -> bool {
        self.keyexprs() == other.keyexprs()
            && self.subscribers() == other.subscribers()
            && self.queryables() == other.queryables()
            && self.tokens() == other.tokens()
            && self.aggregate() == other.aggregate()
    }
}

impl InterestOptions {
    pub const KEYEXPRS: InterestOptions = InterestOptions::options(1);
    pub const SUBSCRIBERS: InterestOptions = InterestOptions::options(1 << 1);
    pub const QUERYABLES: InterestOptions = InterestOptions::options(1 << 2);
    pub const TOKENS: InterestOptions = InterestOptions::options(1 << 3);

    pub const AGGREGATE: InterestOptions = InterestOptions::options(1 << 7);

    const fn options(options: u8) -> Self {
        Self { options }
    }

    pub const fn keyexprs(&self) -> bool {
        self.options & Self::KEYEXPRS.options != 0
    }

    pub const fn subscribers(&self) -> bool {
        self.options & Self::SUBSCRIBERS.options != 0
    }

    pub const fn queryables(&self) -> bool {
        self.options & Self::QUERYABLES.options != 0
    }

    pub const fn tokens(&self) -> bool {
        self.options & Self::TOKENS.options != 0
    }

    pub const fn aggregate(&self) -> bool {
        self.options & Self::AGGREGATE.options != 0
    }
}
