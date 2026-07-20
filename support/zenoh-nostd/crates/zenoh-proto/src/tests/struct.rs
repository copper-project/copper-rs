use core::fmt::Debug;

use crate::{ZStruct, tests::r#struct::nested::Inner};

#[derive(ZStruct, PartialEq, Debug)]
struct ZBasic {
    pub id: u8,
    pub value: u32,
    pub array: [u8; 4],
}

#[derive(ZStruct, PartialEq, Debug)]
struct ZWithLifetime<'a> {
    pub sn: u16,
    #[zenoh(size = prefixed)]
    pub data: &'a [u8],
}

#[derive(ZStruct, PartialEq, Debug)]
struct ZOptionPrefixed<'a> {
    #[zenoh(presence = prefixed)]
    pub maybe_u32: Option<u32>,

    #[zenoh(presence = prefixed, size = prefixed)]
    pub maybe_str: Option<&'a str>,
}

#[derive(ZStruct, PartialEq, Debug)]
#[zenoh(header = "Z|S:7")]
struct ZOptionHeaderRemain<'a> {
    #[zenoh(presence = header(Z), size = header(S))]
    pub maybe_slice: Option<&'a [u8]>,

    #[zenoh(size = remain)]
    pub trailing_data: &'a str,
}

mod nested {
    use super::*;
    #[derive(ZStruct, PartialEq, Debug)]
    pub struct Inner {
        pub a: u32,
        pub b: u8,
    }
}

#[derive(ZStruct, PartialEq, Debug)]
struct ZNested {
    pub field1: nested::Inner,
    pub field2: nested::Inner,
}

#[derive(ZStruct, PartialEq, Debug)]
struct ZNestedOption<'a> {
    #[zenoh(presence = prefixed)]
    pub maybe_inner: Option<nested::Inner>,

    #[zenoh(size = prefixed)]
    pub name: &'a str,
}

#[derive(ZStruct, PartialEq, Debug)]
#[zenoh(header = "U|S_P|S_S:2|SS_P|SS_S:3")]
struct ZHeaderComplex<'a> {
    #[zenoh(presence = header(U))]
    pub maybe_u8: Option<u8>,

    #[zenoh(presence = header(S_P), size = header(S_S))]
    pub maybe_slice: Option<&'a [u8]>,

    #[zenoh(presence = header(SS_P), size = header(SS_S), maybe_empty)]
    pub maybe_str: Option<&'a str>,

    pub payload: u64,
}

#[derive(ZStruct, PartialEq, Debug)]
struct ZArrays<'a> {
    pub fixed_array: [u8; 8],

    #[zenoh(size = prefixed)]
    pub slice_prefixed: &'a [u8],

    #[zenoh(size = remain)]
    pub no_size_str: &'a str,
}

mod deep {
    use super::*;
    #[derive(ZStruct, PartialEq, Debug)]
    pub struct Inner<'a> {
        pub seq: u32,
        #[zenoh(size = prefixed)]
        pub data: &'a [u8],
    }
}

#[derive(ZStruct, PartialEq, Debug)]
#[zenoh(header = "I|S_P|S_S:6")]
struct ZComplex<'a> {
    pub id: u32,
    pub qos: u8,

    #[zenoh(presence = header(I))]
    pub opt_int: Option<u16>,

    #[zenoh(presence = header(S_P), size = header(S_S))]
    pub opt_str: Option<&'a str>,

    #[zenoh(presence = prefixed)]
    pub opt_inner: Option<deep::Inner<'a>>,

    pub inner: deep::Inner<'a>,

    #[zenoh(size = remain)]
    pub trailing: &'a str,
}

#[derive(ZStruct, PartialEq, Debug)]
#[zenoh(header = "K|F2|I:1=0x1|V1:3|V2:2|")]
struct ZHeader<'a> {
    #[zenoh(header = V1)]
    pub vu8: u8,

    #[zenoh(header = V2)]
    pub vu8_2: u8,

    #[zenoh(presence = header(K), size = prefixed)]
    pub keyexpr: Option<&'a str>,

    #[zenoh(size = prefixed)]
    pub field1: deep::Inner<'a>,

    #[zenoh(presence = header(F2), size = remain)]
    pub field2: Option<ZComplex<'a>>,
}

const DEFAULT_INNER: nested::Inner = nested::Inner { a: 1, b: 2 };

#[derive(ZStruct, PartialEq, Debug)]
#[zenoh(header = "D|_:7")]
struct ZDefaultPresence {
    #[zenoh(presence = prefixed, default = 42)]
    pub maybe_u16: u16,

    #[zenoh(presence = header(D), default = DEFAULT_INNER)]
    pub maybe_u32: nested::Inner,
}

#[derive(ZStruct, PartialEq, Debug)]
#[zenoh(header = "_|V|_|_:1=0x3|_:4")]
struct ZInnerWithHeader {
    #[zenoh(header = V)]
    pub a: u8,
}

const DEFAULT_INNER_WITH_HEADER: ZInnerWithHeader = ZInnerWithHeader { a: 0 };

#[derive(ZStruct, PartialEq, Debug)]
#[zenoh(header = "D|_|_:1=0x2|_|_:4=0x4")]
struct ZFlattenHeader<'a> {
    #[zenoh(presence = prefixed, default = 42)]
    pub maybe_u16: u16,

    #[zenoh(flatten, presence = header(D), default = DEFAULT_INNER_WITH_HEADER)]
    pub maybe_inner: ZInnerWithHeader,

    #[zenoh(size = remain)]
    pub payload: &'a str,
}

macro_rules! roundtrip {
    ($ty:ty, $value:expr) => {{
        let mut data = [0u8; 256];

        let len = $crate::ZLen::z_len(&$value);
        $crate::ZEncode::z_encode(&$value, &mut &mut data[..]).unwrap();

        let decoded = <$ty as $crate::ZDecode>::z_decode(&mut &data[..len]).unwrap();

        assert_eq!(decoded, $value);
    }};
}

#[test]
fn test_zbasic() {
    let s = ZBasic {
        id: 42,
        value: 123456,
        array: [1, 2, 3, 4],
    };
    roundtrip!(ZBasic, s);
}

#[test]
fn test_zwitlifetime() {
    let buf = [10, 20, 30];
    let s = ZWithLifetime { sn: 11, data: &buf };
    roundtrip!(ZWithLifetime, s);
}

#[test]
fn test_zoption_prefixed() {
    let s = ZOptionPrefixed {
        maybe_u32: Some(99),
        maybe_str: Some("hello"),
    };
    roundtrip!(ZOptionPrefixed, s);

    let s2 = ZOptionPrefixed {
        maybe_u32: None,
        maybe_str: None,
    };
    roundtrip!(ZOptionPrefixed, s2);
}

#[test]
fn test_zoption_header_remain() {
    let buf = [1, 2, 3];
    let s = ZOptionHeaderRemain {
        maybe_slice: Some(&buf),
        trailing_data: "xyz",
    };
    roundtrip!(ZOptionHeaderRemain, s);
}

#[test]
fn test_znested() {
    let s = ZNested {
        field1: nested::Inner { a: 1, b: 2 },
        field2: nested::Inner { a: 3, b: 4 },
    };
    roundtrip!(ZNested, s);
}

#[test]
fn test_znested_option() {
    let s = ZNestedOption {
        maybe_inner: Some(nested::Inner { a: 42, b: 7 }),
        name: "nested",
    };
    roundtrip!(ZNestedOption, s);
}

#[test]
fn test_zheader_complex() {
    let buf = [5, 6, 7];
    let s = ZHeaderComplex {
        maybe_u8: Some(1),
        maybe_slice: Some(&buf),
        maybe_str: Some("hi"),
        payload: 123456789,
    };
    roundtrip!(ZHeaderComplex, s);
}

#[test]
fn test_zarrays() {
    let fixed = [9; 8];
    let s = ZArrays {
        fixed_array: fixed,
        slice_prefixed: &[1, 2, 3],
        no_size_str: "str",
    };
    roundtrip!(ZArrays, s);
}

#[test]
fn test_zcomplex() {
    let buf = [1, 2, 3, 4];
    let opt_inner = deep::Inner {
        seq: 42,
        data: &buf,
    };

    let inner = deep::Inner {
        seq: 99,
        data: &buf,
    };
    let s = ZComplex {
        id: 1,
        qos: 2,
        opt_int: Some(123),
        opt_str: Some("hello"),
        opt_inner: Some(opt_inner),
        inner,
        trailing: "world",
    };
    roundtrip!(ZComplex, s);
}

#[test]
fn test_zheader() {
    let buf = [1, 2, 3, 4];
    let opt_inner = deep::Inner {
        seq: 42,
        data: &buf,
    };

    let inner = deep::Inner {
        seq: 99,
        data: &buf,
    };
    let s = ZComplex {
        id: 1,
        qos: 2,
        opt_int: Some(123),
        opt_str: Some("hello"),
        opt_inner: Some(opt_inner),
        inner,
        trailing: "world",
    };
    let header = ZHeader {
        vu8: 0b0000_0101,
        vu8_2: 0b0000_0011,
        keyexpr: Some("key.expr"),
        field1: deep::Inner { seq: 7, data: &buf },
        field2: Some(s),
    };

    roundtrip!(ZHeader, header);
}

#[test]
fn test_default_presence() {
    let s = ZDefaultPresence {
        maybe_u16: 42,
        maybe_u32: DEFAULT_INNER,
    };

    roundtrip!(ZDefaultPresence, s);

    let s = ZDefaultPresence {
        maybe_u16: 100,
        maybe_u32: DEFAULT_INNER,
    };

    roundtrip!(ZDefaultPresence, s);

    let s = ZDefaultPresence {
        maybe_u16: 42,
        maybe_u32: Inner { a: 10, b: 20 },
    };

    roundtrip!(ZDefaultPresence, s);

    let s = ZDefaultPresence {
        maybe_u16: 100,
        maybe_u32: Inner { a: 10, b: 20 },
    };

    roundtrip!(ZDefaultPresence, s);
}

#[test]
fn test_flatten_inner_with_header() {
    let s1 = ZInnerWithHeader { a: 1 };
    roundtrip!(ZInnerWithHeader, s1);
    let h1 = <ZInnerWithHeader as crate::ZHeader>::z_header(&s1);

    let s2 = ZFlattenHeader {
        maybe_u16: 100,
        maybe_inner: s1,
        payload: "data",
    };
    roundtrip!(ZFlattenHeader, s2);
    let h2 = <ZFlattenHeader as crate::ZHeader>::z_header(&s2);

    assert_eq!(h1 & h2, h1);
}
