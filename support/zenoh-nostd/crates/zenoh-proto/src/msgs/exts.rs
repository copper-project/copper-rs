use core::time::Duration;

use crate::{fields::*, *};

#[derive(ZExt, Debug, PartialEq, Default)]
#[zenoh(header = "ID:4|_:4")]
pub struct EntityGlobalId {
    #[zenoh(size = header(ID))]
    pub zid: ZenohIdProto,

    pub eid: u32,
}

#[derive(ZExt, Debug, PartialEq, Default)]
pub struct SourceInfo {
    pub id: EntityGlobalId,
    pub sn: u32,
}

#[derive(ZExt, Debug, PartialEq, Default)]
pub struct Value<'a> {
    pub encoding: Encoding<'a>,

    #[zenoh(size = remain)]
    pub payload: &'a [u8],
}

#[derive(ZExt, Debug, PartialEq, Default, Clone)]
pub struct Attachment<'a> {
    #[zenoh(size = remain)]
    pub buffer: &'a [u8],
}

#[derive(ZExt, Clone, Copy, Debug, PartialEq)]
pub struct QoS {
    pub inner: u8,
}

impl Default for QoS {
    fn default() -> Self {
        Self::new(Priority::Data, CongestionControl::Drop, false)
    }
}

impl QoS {
    const D_FLAG: u8 = 0b00001000;
    const E_FLAG: u8 = 0b00010000;

    pub const fn declare() -> Self {
        Self::new(Priority::Data, CongestionControl::Block, false)
    }

    pub const fn new(
        priority: Priority,
        congestion_control: CongestionControl,
        is_express: bool,
    ) -> Self {
        let mut inner = priority as u8;
        if matches!(congestion_control, CongestionControl::Block) {
            inner |= Self::D_FLAG;
        }
        if is_express {
            inner |= Self::E_FLAG;
        }
        Self { inner }
    }
}

#[derive(ZExt, Debug, PartialEq, Default)]
pub struct NodeId {
    pub node_id: u16,
}
#[repr(u8)]
#[derive(ZRU8, Debug, Default, Clone, Copy, PartialEq)]
pub enum QueryTarget {
    #[default]
    BestMatching = 0,
    All = 1,
    AllComplete = 2,
}

#[derive(Debug, PartialEq, Default)]
pub struct QueryableInfo {
    pub complete: bool,
    pub distance: u16,
}

impl ZBodyLen for QueryableInfo {
    fn z_body_len(&self) -> usize {
        <u64 as ZLen>::z_len(&self.as_u64())
    }
}

impl ZBodyEncode for QueryableInfo {
    fn z_body_encode(
        &self,
        w: &mut impl crate::ZWriteable,
    ) -> core::result::Result<(), crate::CodecError> {
        <u64 as ZEncode>::z_encode(&self.as_u64(), w)
    }
}

impl<'a> ZBodyDecode<'a> for QueryableInfo {
    type Ctx = ();

    fn z_body_decode(
        r: &mut impl crate::ZReadable<'a>,
        _: (),
    ) -> core::result::Result<Self, crate::CodecError> {
        let value = <u64 as ZDecode>::z_decode(r)?;
        let complete = (value & 0b0000_0001) != 0;
        let distance = ((value >> 8) & 0xFFFF) as u16;
        Ok(QueryableInfo { complete, distance })
    }
}

crate::derive_zstruct_with_body!(QueryableInfo);

impl<'a> ZExt<'a> for QueryableInfo {
    const KIND: ZExtKind = ZExtKind::U64;
}

impl QueryableInfo {
    fn as_u64(&self) -> u64 {
        let mut flags: u8 = 0;
        if self.complete {
            flags |= 0b0000_0001;
        }
        (flags as u64) | ((self.distance as u64) << 8)
    }
}

#[derive(ZExt, Debug, PartialEq, Default)]
pub struct Budget {
    pub budget: u32,
}

impl ZHeader for Duration {
    fn z_header(&self) -> u8 {
        let header = 0u8;
        match self.as_millis() % 1_000 {
            0 => header | 0b0000_0001,
            _ => header,
        }
    }
}

impl ZBodyLen for Duration {
    fn z_body_len(&self) -> usize {
        let v = match self.as_millis() % 1_000 {
            0 => self.as_millis() / 1_000,
            _ => self.as_millis(),
        } as u64;

        <u64 as ZLen>::z_len(&v)
    }
}

impl ZLen for Duration {
    fn z_len(&self) -> usize {
        <u64 as ZLen>::z_len(&(self.as_millis() as u64))
    }
}

impl ZBodyEncode for Duration {
    fn z_body_encode(
        &self,
        w: &mut impl crate::ZWriteable,
    ) -> core::result::Result<(), crate::CodecError> {
        let v = match self.as_millis() % 1_000 {
            0 => self.as_millis() / 1_000,
            _ => self.as_millis(),
        } as u64;

        <u64 as ZEncode>::z_encode(&v, w)
    }
}

impl ZEncode for Duration {
    fn z_encode(
        &self,
        w: &mut impl crate::ZWriteable,
    ) -> core::result::Result<(), crate::CodecError> {
        <u64 as ZEncode>::z_encode(&(self.as_millis() as u64), w)
    }
}

impl<'a> ZBodyDecode<'a> for Duration {
    type Ctx = u8;

    fn z_body_decode(
        r: &mut impl crate::ZReadable<'a>,
        h: u8,
    ) -> core::result::Result<Self, crate::CodecError> {
        let is_seconds = (h & 0b0000_0001) != 0;
        let value = <u64 as ZDecode>::z_decode(r)?;
        if is_seconds {
            Ok(Duration::from_secs(value))
        } else {
            Ok(Duration::from_millis(value))
        }
    }
}

impl<'a> ZDecode<'a> for Duration {
    fn z_decode(
        r: &mut impl crate::ZReadable<'a>,
    ) -> core::result::Result<Self, crate::CodecError> {
        let value = <u64 as ZDecode>::z_decode(r)?;
        Ok(Duration::from_millis(value))
    }
}

impl<'a> ZExt<'a> for Duration {
    const KIND: ZExtKind = ZExtKind::U64;
}

#[derive(ZExt, Debug, PartialEq, Default)]
pub struct HasQoS {}

#[derive(ZExt, Debug, PartialEq, Default)]
pub struct QoSLink {
    pub qos: u64,
}

#[derive(ZExt, Debug, PartialEq, Default)]
pub struct Auth<'a> {
    #[zenoh(size = remain)]
    pub payload: &'a [u8],
}

#[derive(ZExt, Debug, PartialEq, Default)]
pub struct MultiLink<'a> {
    #[zenoh(size = remain)]
    pub payload: &'a [u8],
}

#[derive(ZExt, Debug, PartialEq, Default)]
pub struct MultiLinkSyn<'a> {
    #[zenoh(size = remain)]
    pub payload: &'a [u8],
}

#[derive(ZExt, Debug, PartialEq, Default)]
pub struct HasMultiLinkAck {}

#[derive(ZExt, Debug, PartialEq, Default)]
pub struct HasLowLatency {}

#[derive(ZExt, Debug, PartialEq, Default)]
pub struct HasCompression {}

#[derive(ZExt, Debug, PartialEq, Default)]
pub struct Patch {
    pub int: u8,
}

impl Patch {
    pub fn none() -> Self {
        Self { int: 0 }
    }

    pub fn current() -> Self {
        Self { int: 1 }
    }
}
