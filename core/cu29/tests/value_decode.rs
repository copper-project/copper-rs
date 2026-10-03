//! User API agreement using payloads satisfying the native Copper message contract.
#![cfg(feature = "self-describing-logs")]
use cu29::bincode::Decode;
use cu29::bincode::Encode;
use cu29::prelude::*;
use cu29::units::si::f32::Length;
use cu29::units::si::length::centimeter;
use std::collections::BTreeMap;

#[derive(Clone, Debug, Default, Serialize, Deserialize, Encode, Decode, Reflect)]
#[bincode(decode_context = "()")]
#[reflect(from_reflect = false)]
struct SensorSample {
    ticks: u32,
    distance: Length,
    timestamp: CuTime,
    samples: CuArrayVec<i16, 4>,
    valid: bool,
}

#[test]
fn test_native_copper_payload_user_api() {
    fn assert_payload<T: CuMsgPayload + ValueDecode>() {}
    assert_payload::<SensorSample>();
    let mut sample = SensorSample {
        ticks: 42,
        distance: Length::new::<centimeter>(125.0),
        timestamp: CuTime::from_nanos(12345),
        samples: CuArrayVec::default(),
        valid: true,
    };
    sample.samples.0.extend([-2, 0, 300]);
    let mut bytes = [0; 128];
    let config = cu29::bincode::config::standard();
    let len = cu29::bincode::encode_into_slice(&sample, &mut bytes, config).unwrap();
    let description = ValueDecodeDescription::from_type::<SensorSample>().unwrap();
    let description_bytes = cu29::bincode::encode_to_vec(description, config).unwrap();
    let (offline_description, _): (ValueDecodeDescription, _) =
        cu29::bincode::decode_from_slice(&description_bytes, config).unwrap();
    let (value, used) = offline_description
        .decode(&bytes[..len], config, ValueDecodeLimits::default())
        .unwrap();
    let expected = Value::Map(BTreeMap::from([
        (Value::String("ticks".into()), Value::U32(42)),
        (Value::String("distance".into()), Value::F32(1.25)),
        (Value::String("timestamp".into()), Value::U64(12345)),
        (
            Value::String("samples".into()),
            Value::Seq(vec![Value::I16(-2), Value::I16(0), Value::I16(300)]),
        ),
        (Value::String("valid".into()), Value::Bool(true)),
    ]));
    assert_eq!(value, expected);
    assert!(offline_description.schemas.iter().any(|schema| {
        schema.type_path.ends_with("CuTime")
            && schema.quantity.as_ref().is_some_and(|quantity| {
                quantity.quantity == "time" && quantity.storage_unit == "ns"
            })
    }));
    let (native, native_used): (SensorSample, _) =
        cu29::bincode::decode_from_slice(&bytes[..len], config).unwrap();
    assert_eq!(native.samples.0.as_slice(), sample.samples.0.as_slice());
    assert_eq!(native.distance, sample.distance);
    assert_eq!(used, native_used);
    assert_eq!(used, len);
}

#[test]
fn test_fixed_capacity_counts_match_the_native_encoders() {
    let config = cu29::bincode::config::standard().with_fixed_int_encoding();
    let mut array = CuArray::<u8, 4>::new();
    array.fill_from_iter([1, 2, 3]);
    let vector = CuArrayVec::<u8, 4>([1, 2, 3].into_iter().collect());
    let array_bytes = cu29::bincode::encode_to_vec(&array, config).unwrap();
    let vector_bytes = cu29::bincode::encode_to_vec(&vector, config).unwrap();
    assert_eq!(array_bytes.len(), 4 + 3);
    assert_eq!(vector_bytes.len(), 8 + 3);
    let expected = Value::Seq(vec![Value::U8(1), Value::U8(2), Value::U8(3)]);
    let description = ValueDecodeDescription::from_type::<CuArray<u8, 4>>().unwrap();
    assert_eq!(
        description
            .decode(&array_bytes, config, ValueDecodeLimits::default())
            .unwrap(),
        (expected.clone(), array_bytes.len())
    );
    let description = ValueDecodeDescription::from_type::<CuArrayVec<u8, 4>>().unwrap();
    assert_eq!(
        description
            .decode(&vector_bytes, config, ValueDecodeLimits::default())
            .unwrap(),
        (expected, vector_bytes.len())
    );
    let description = ValueDecodeDescription::from_type::<CuArray<u8, 2>>().unwrap();
    assert!(
        description
            .decode(&array_bytes, config, ValueDecodeLimits::default())
            .is_err()
    );
}
