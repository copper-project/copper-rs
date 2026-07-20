use crate::*;

#[derive(ZStruct, Debug, PartialEq, Default)]
#[zenoh(header = "_:2|S|ID:5=0x03")]
pub struct Close {
    pub reason: u8,

    #[zenoh(header = S)]
    pub behaviour: CloseBehaviour,
}

#[repr(u8)]
#[derive(ZRU8, Default, Debug, Clone, Copy, PartialEq)]
pub enum CloseBehaviour {
    #[default]
    Link = 0,
    Session = 1,
}
