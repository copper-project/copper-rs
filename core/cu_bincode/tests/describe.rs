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
