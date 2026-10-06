//! Type-level descriptions work independently of the codec feature.
extern crate cu_bincode as bincode;

use bincode::{Encode, ValueDecode, ValueDecodeSpec};

struct Custom(u16);

impl Encode for Custom {
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        self.0.encode(encoder)
    }
}

impl ValueDecode for Custom {
    const DECODE: &'static ValueDecodeSpec = <u16 as ValueDecode>::DECODE;
}

#[derive(Encode)]
#[bincode(describe)]
struct Record<T> {
    value: T,
}

#[derive(Encode)]
#[bincode(describe)]
enum Choice {
    Empty,
    Value(Record<Custom>),
}

#[test]
fn describes_nested_custom_encoders_without_feature_forwarding() {
    let ValueDecodeSpec::Enum { variants, .. } = Choice::DECODE else {
        panic!("expected enum recipe")
    };
    assert_eq!(variants.len(), 2);
    let child = (variants[1].fields[0].value.describe)();
    let ValueDecodeSpec::Record { fields, .. } = child.spec else {
        panic!("expected record recipe")
    };
    assert!(matches!(
        (fields[0].value.describe)().spec,
        ValueDecodeSpec::Scalar(bincode::value_decode::Scalar::U16)
    ));
    assert_eq!(
        bincode::encode_to_vec(Choice::Empty, bincode::config::standard()).unwrap(),
        [0]
    );
    assert_eq!(
        bincode::encode_to_vec(
            Choice::Value(Record { value: Custom(7) }),
            bincode::config::standard()
        )
        .unwrap(),
        [1, 7]
    );
}

#[test]
fn shared_description_kinds_retain_their_binary_tags() {
    use bincode::value_decode::RecordShape;
    use bincode::value_decode::Scalar;

    fn check_kind<T>(kind: T, tag: u32, config: impl bincode::config::Config)
    where
        T: Encode + bincode::Decode<()> + ValueDecode + core::fmt::Debug + PartialEq,
    {
        let bytes = bincode::encode_to_vec(&kind, config).unwrap();
        assert_eq!(bytes, bincode::encode_to_vec(tag, config).unwrap());
        let (decoded, used): (T, _) = bincode::decode_from_slice(&bytes, config).unwrap();
        assert_eq!(decoded, kind);
        assert_eq!(used, bytes.len());
        let ValueDecodeSpec::Enum { variants, .. } = T::DECODE else {
            panic!("expected enum recipe");
        };
        assert!(variants.iter().any(|variant| variant.tag == tag));
    }

    let config = bincode::config::standard();
    for (kind, tag) in [
        (Scalar::Bool, 0),
        (Scalar::U8, 1),
        (Scalar::U16, 2),
        (Scalar::U32, 3),
        (Scalar::U64, 4),
        (Scalar::U128, 5),
        (Scalar::I8, 6),
        (Scalar::I16, 7),
        (Scalar::I32, 8),
        (Scalar::I64, 9),
        (Scalar::I128, 10),
        (Scalar::F32, 11),
        (Scalar::F64, 12),
        (Scalar::Char, 13),
    ] {
        check_kind(kind, tag, config);
        check_kind(
            kind,
            tag,
            config.with_big_endian().with_fixed_int_encoding(),
        );
    }
    for (kind, tag) in [
        (RecordShape::Unit, 0),
        (RecordShape::Tuple, 1),
        (RecordShape::Newtype, 2),
        (RecordShape::Struct, 3),
    ] {
        check_kind(kind, tag, config);
        check_kind(
            kind,
            tag,
            config.with_big_endian().with_fixed_int_encoding(),
        );
    }
    let bytes = bincode::encode_to_vec(99u32, config).unwrap();
    assert!(bincode::decode_from_slice::<Scalar, _>(&bytes, config).is_err());
    assert!(bincode::decode_from_slice::<RecordShape, _>(&bytes, config).is_err());
}
