use crate::{ZExt, ZExtKind, ZStruct};

#[derive(ZExt, PartialEq, Debug)]
pub struct ZExtEmpty {}

#[derive(ZExt, PartialEq, Debug)]
pub struct ZExtCounter {
    pub counter: u64,
}

#[derive(ZExt, PartialEq, Debug)]
pub struct ZExtData<'a> {
    #[zenoh(size = prefixed)]
    pub bytes: &'a [u8],
}

#[derive(ZExt, PartialEq, Debug)]
pub struct ZExtInfo<'a> {
    pub id: u16,
    #[zenoh(size = remain)]
    pub name: &'a str,
}

#[derive(ZExt, PartialEq, Debug)]
#[zenoh(header = "Z|E|_:6")]
pub struct ZExtHeader<'a> {
    #[zenoh(presence = header(Z))]
    pub maybe_u8: Option<u8>,

    #[zenoh(presence = header(E), size = remain)]
    pub maybe_str: Option<&'a str>,
}

#[derive(ZStruct, PartialEq, Debug)]
#[zenoh(header = "Z|_:7")]
pub struct ZMsgOption<'a> {
    #[zenoh(size = prefixed)]
    pub title: &'a str,

    #[zenoh(ext = 0x1)]
    pub ext_data: Option<ZExtData<'a>>,
    #[zenoh(ext = 0x2)]
    pub ext_info: Option<ZExtInfo<'a>>,

    #[zenoh(size = remain)]
    pub payload: &'a [u8],
}

#[derive(ZStruct, PartialEq, Debug)]
#[zenoh(header = "Z|_:7")]
pub struct ZMsgMix<'a> {
    #[zenoh(size = prefixed)]
    pub topic: &'a str,

    #[zenoh(ext = 0x1)]
    pub ext_header: Option<ZExtHeader<'a>>,
    #[zenoh(ext = 0x2, default = Self::DEFAULT_EXT_EMPTY)]
    pub ext_empty: ZExtEmpty,

    pub value: u32,
}

impl ZMsgMix<'_> {
    const DEFAULT_EXT_EMPTY: ZExtEmpty = ZExtEmpty {};
}

#[derive(ZStruct, PartialEq, Debug)]
#[zenoh(header = "Z|_:7")]
pub struct ZMsgCounters {
    #[zenoh(ext = 0x1)]
    pub ext1: Option<ZExtCounter>,

    pub checksum: u16,
}

#[derive(ZStruct, PartialEq, Debug)]
#[zenoh(header = "Z|_:7")]
pub struct ZMsgComplex<'a> {
    #[zenoh(size = prefixed)]
    pub name: &'a str,

    #[zenoh(ext = 0x1)]
    pub ext_info: Option<ZExtInfo<'a>>,
    #[zenoh(ext = 0x2, default = Self::DEFAULT_EXT_DATA)]
    pub ext_data: ZExtData<'a>,
    #[zenoh(ext = 0x3, default = Self::DEFAULT_EXT_EMPTY)]
    pub ext_empty: ZExtEmpty,

    #[zenoh(size = remain)]
    pub trailing: &'a str,
}

impl ZMsgComplex<'_> {
    const DEFAULT_EXT_DATA: ZExtData<'static> = ZExtData { bytes: &[] };
    const DEFAULT_EXT_EMPTY: ZExtEmpty = ZExtEmpty {};
}

#[derive(ZStruct, PartialEq, Debug)]
#[zenoh(header = "Z|_:7")]
pub struct ZMsgComplexOption<'a> {
    #[zenoh(size = prefixed)]
    pub name: &'a str,

    #[zenoh(ext = 0x1)]
    pub ext_info: Option<ZExtInfo<'a>>,
    #[zenoh(ext = 0x2)]
    pub ext_data: Option<ZExtData<'a>>,
    #[zenoh(ext = 0x3)]
    pub ext_empty: Option<ZExtEmpty>,

    #[zenoh(size = remain)]
    pub trailing: &'a str,
}

macro_rules! roundtrip {
    ($ty:ty, $value:expr) => {{
        let mut data = [0u8; 256];

        let len = $crate::ZLen::z_len(&$value);
        $crate::ZEncode::z_encode(&$value, &mut &mut data[..]).unwrap();

        let decoded = <$ty as $crate::ZDecode>::z_decode(
            &mut crate::ZReadable::read_slice(&mut &data[..], len).unwrap(),
        )
        .unwrap();

        assert_eq!(decoded, $value);
    }};
}

#[test]
fn test_zext_kinds() {
    assert_eq!(ZExtEmpty::KIND, ZExtKind::Unit);
    assert_eq!(ZExtCounter::KIND, ZExtKind::U64);
    assert_eq!(ZExtData::KIND, ZExtKind::ZStruct);
    assert_eq!(ZExtInfo::KIND, ZExtKind::ZStruct);
    assert_eq!(ZExtHeader::KIND, ZExtKind::ZStruct);
}

#[test]
fn test_zmsg_simple() {
    let buf = [1, 2, 3, 4];
    let msg = ZMsgOption {
        title: "simple",
        ext_data: Some(ZExtData { bytes: &buf }),
        ext_info: Some(ZExtInfo {
            id: 99,
            name: "info",
        }),
        payload: &buf,
    };
    roundtrip!(ZMsgOption, msg);
}

#[test]
fn test_zmsg_header() {
    let msg = ZMsgMix {
        topic: "topic/1",
        ext_header: Some(ZExtHeader {
            maybe_u8: Some(7),
            maybe_str: Some("extra"),
        }),
        ext_empty: ZExtEmpty {},
        value: 12345,
    };
    roundtrip!(ZMsgMix, msg);
}

#[test]
fn test_zmsg_counters() {
    let msg = ZMsgCounters {
        ext1: Some(ZExtCounter { counter: 10 }),
        checksum: 55,
    };
    roundtrip!(ZMsgCounters, msg);
}

#[test]
fn test_zmsg_complex() {
    let data = [9, 9, 9];
    let msg = ZMsgComplex {
        name: "complex",
        ext_info: Some(ZExtInfo {
            id: 11,
            name: "ext",
        }),
        ext_data: ZExtData { bytes: &data },
        ext_empty: ZExtEmpty {},
        trailing: "end",
    };
    roundtrip!(ZMsgComplex, msg);
}

#[test]
fn test_zmsg_partial_exts() {
    let msg = ZMsgComplex {
        name: "partial",
        ext_info: None,
        ext_data: ZExtData { bytes: &[1, 2, 3] },
        ext_empty: ZExtEmpty {},
        trailing: "zzz",
    };

    roundtrip!(ZMsgComplex, msg);
    let msg = ZMsgComplexOption {
        name: "partial",
        ext_info: None,
        ext_data: None,
        ext_empty: None,
        trailing: "zzz",
    };
    roundtrip!(ZMsgComplexOption, msg);
}
