//! Agreement tests using native Encode/Decode and portable descriptions.
use super::decode::*;
use crate::Value;
use alloc::boxed::Box;
use alloc::collections::BTreeMap;
use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;
use bevy_reflect::Reflect;
use bincode::Decode;
use bincode::Encode;
use bincode::ValueDecode;
use bincode::ValueDecodeSpec;
use bincode::config::Config;
use cu29_units::si::f32::Length;
use cu29_units::si::f32::Mass;
use cu29_units::si::f32::Velocity;
use cu29_units::si::length::centimeter;
use cu29_units::si::mass::gram;
use cu29_units::si::velocity::kilometer_per_hour;

fn map(fields: &[(&str, Value)]) -> Value {
    Value::Map(
        fields
            .iter()
            .map(|(name, value)| (Value::String((*name).into()), value.clone()))
            .collect(),
    )
}

fn round_trip<T, C>(value: &T, expected: Value, config: C) -> ValueDecodeDescription
where
    T: ValueDecode
        + bevy_reflect::GetTypeRegistration
        + Encode
        + Decode<()>
        + PartialEq
        + core::fmt::Debug,
    C: Config,
{
    let mut bytes = [0; 4096];
    let len = bincode::encode_into_slice(value, &mut bytes, config).unwrap();
    let bytes = &bytes[..len];
    let (native, native_used): (T, usize) = bincode::decode_from_slice(bytes, config).unwrap();
    assert_eq!(&native, value);
    let description = ValueDecodeDescription::from_type::<T>().unwrap();
    // The offline reader receives serialized data, rather than function pointers or Rust types.
    let description_bytes =
        bincode::encode_to_vec(description, bincode::config::standard()).unwrap();
    let (description, used): (ValueDecodeDescription, usize) =
        bincode::decode_from_slice(&description_bytes, bincode::config::standard()).unwrap();
    assert_eq!(used, description_bytes.len());
    let (tree, used) = description
        .decode(bytes, config, ValueDecodeLimits::default())
        .unwrap();
    assert_eq!(tree, expected);
    assert_eq!(used, native_used);
    assert_eq!(used, len);
    description
}

#[derive(Clone, Debug, Default, PartialEq, Encode, Decode, Reflect)]
struct WheelSample {
    ticks: u32,
    distance: Length,
    speed: Velocity,
    mass: Mass,
    valid: bool,
}

#[test]
fn test_native_quantities_and_shared_scalar_operations() {
    let sample = WheelSample {
        ticks: 42,
        distance: Length::new::<centimeter>(125.0),
        speed: Velocity::new::<kilometer_per_hour>(36.0),
        mass: Mass::new::<gram>(1000.0),
        valid: true,
    };
    let description = round_trip(
        &sample,
        map(&[
            ("ticks", Value::U32(42)),
            ("distance", Value::F32(1.25)),
            ("speed", Value::F32(10.0)),
            ("mass", Value::F32(1.0)),
            ("valid", Value::Bool(true)),
        ]),
        bincode::config::standard(),
    );
    let quantities: Vec<_> = description
        .schemas
        .iter()
        .filter_map(|schema| schema.quantity.as_ref())
        .collect();
    assert_eq!(quantities.len(), 3);
    assert!(
        quantities
            .iter()
            .any(|q| q.quantity == "length" && q.storage_unit == "m")
    );
    assert!(
        quantities
            .iter()
            .any(|q| q.quantity == "velocity" && q.storage_unit == "m s^-1")
    );
    assert!(
        quantities
            .iter()
            .any(|q| q.quantity == "mass" && q.storage_unit == "kg")
    );
    let quantity_bindings: Vec<_> = description
        .bindings
        .iter()
        .filter(|binding| description.schemas[binding.schema].quantity.is_some())
        .collect();
    assert!(
        quantity_bindings
            .iter()
            .all(|binding| binding.operation == quantity_bindings[0].operation)
    );
    assert_ne!(quantity_bindings[0].schema, quantity_bindings[1].schema);
    assert!(
        description.schemas[description.bindings[description.root].schema]
            .type_path
            .ends_with("WheelSample")
    );
}

#[test]
fn test_all_native_scalars_under_every_codec_configuration() {
    macro_rules! scalars {
        ($config:expr) => {{
            let config = $config;
            round_trip(&true, Value::Bool(true), config);
            round_trip(&u8::MAX, Value::U8(u8::MAX), config);
            round_trip(&u16::MAX, Value::U16(u16::MAX), config);
            round_trip(&u32::MAX, Value::U32(u32::MAX), config);
            round_trip(&u64::MAX, Value::U64(u64::MAX), config);
            round_trip(&u128::MAX, Value::U128(u128::MAX), config);
            round_trip(&i8::MIN, Value::I8(i8::MIN), config);
            round_trip(&i16::MIN, Value::I16(i16::MIN), config);
            round_trip(&i32::MIN, Value::I32(i32::MIN), config);
            round_trip(&i64::MIN, Value::I64(i64::MIN), config);
            round_trip(&i128::MIN, Value::I128(i128::MIN), config);
            round_trip(&-1.25f32, Value::F32(-1.25), config);
            round_trip(&f64::INFINITY, Value::F64(f64::INFINITY), config);
            round_trip(&'🦀', Value::Char('🦀'), config);
            round_trip(&(), Value::Unit, config);
            for value in [
                0,
                250,
                251,
                65535,
                65536,
                u32::MAX as u64,
                u32::MAX as u64 + 1,
            ] {
                round_trip(&value, Value::U64(value), config);
            }
        }};
    }
    scalars!(bincode::config::standard());
    scalars!(bincode::config::standard().with_big_endian());
    scalars!(bincode::config::standard().with_fixed_int_encoding());
    scalars!(
        bincode::config::standard()
            .with_big_endian()
            .with_fixed_int_encoding()
    );
}

#[derive(Clone, Debug, PartialEq, Encode, Decode, Reflect)]
struct Envelope<T: Reflect> {
    sequence: u64,
    payload: T,
}
#[derive(Clone, Debug, PartialEq, Encode, Decode, Reflect)]
enum Event {
    Idle,
    Reading(i16),
    Window(u32, bool),
    Fault { code: u8, detail: String },
}

#[test]
fn test_nested_generics_collections_and_every_enum_branch() {
    let variants = [
        (Event::Idle, Value::String("Idle".into())),
        (Event::Reading(-42), map(&[("Reading", Value::I16(-42))])),
        (
            Event::Window(300, true),
            map(&[(
                "Window",
                Value::Seq(vec![Value::U32(300), Value::Bool(true)]),
            )]),
        ),
        (
            Event::Fault {
                code: 7,
                detail: "Café 🦀".into(),
            },
            map(&[(
                "Fault",
                map(&[
                    ("code", Value::U8(7)),
                    ("detail", Value::String("Café 🦀".into())),
                ]),
            )]),
        ),
    ];
    for (event, tree) in variants {
        round_trip(
            &Envelope {
                sequence: u64::MAX,
                payload: vec![Some(event), None],
            },
            map(&[
                ("sequence", Value::U64(u64::MAX)),
                (
                    "payload",
                    Value::Seq(vec![
                        Value::Option(Some(Box::new(tree))),
                        Value::Option(None),
                    ]),
                ),
            ]),
            bincode::config::standard(),
        );
    }
    let values = BTreeMap::from([(String::from("samples"), vec![1u16, 300])]);
    round_trip(
        &values,
        map(&[("samples", Value::Seq(vec![Value::U16(1), Value::U16(300)]))]),
        bincode::config::standard(),
    );
    round_trip(
        &(false, [1f32, 2.0, 3.0], Option::<u8>::None),
        Value::Seq(vec![
            Value::Bool(false),
            Value::Seq(vec![Value::F32(1.0), Value::F32(2.0), Value::F32(3.0)]),
            Value::Option(None),
        ]),
        bincode::config::standard(),
    );
    round_trip(&[0u8; 0], Value::Seq(vec![]), bincode::config::standard());
    round_trip(
        &Vec::<u16>::new(),
        Value::Seq(vec![]),
        bincode::config::standard(),
    );
}

#[derive(Clone, Debug, Default, PartialEq, Reflect)]
#[reflect(opaque)]
struct Orientation([f32; 4]);
impl Encode for Orientation {
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        self.0.encode(encoder)
    }
}
impl<C> Decode<C> for Orientation {
    fn decode<D: bincode::de::Decoder<Context = C>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        Ok(Self(<[f32; 4]>::decode(decoder)?))
    }
}
bincode::impl_borrow_decode!(Orientation);
impl ValueDecode for Orientation {
    const DECODE: &'static ValueDecodeSpec = <[f32; 4] as ValueDecode>::DECODE;
}
#[derive(Clone, Debug, PartialEq, Encode, Decode, Reflect)]
struct Attitude {
    orientation: Orientation,
    valid: bool,
}

#[test]
fn test_manual_operation_delegation_and_consecutive_native_values() {
    let sample = Attitude {
        orientation: Orientation([0.0, 0.5, -0.5, 1.0]),
        valid: true,
    };
    let expected = map(&[
        (
            "orientation",
            Value::Seq(vec![
                Value::F32(0.0),
                Value::F32(0.5),
                Value::F32(-0.5),
                Value::F32(1.0),
            ]),
        ),
        ("valid", Value::Bool(true)),
    ]);
    let description = round_trip(&sample, expected.clone(), bincode::config::standard());
    assert!(
        description
            .schemas
            .iter()
            .any(|schema| schema.type_path.ends_with("Orientation"))
    );
    let bytes =
        bincode::encode_to_vec((&sample, &sample, 1234u32), bincode::config::standard()).unwrap();
    let (first, used) = description
        .decode(
            &bytes,
            bincode::config::standard(),
            ValueDecodeLimits::default(),
        )
        .unwrap();
    assert_eq!(first, expected);
    assert_eq!(used, 17);
    let (second, second_used) = description
        .decode(
            &bytes[used..],
            bincode::config::standard(),
            ValueDecodeLimits::default(),
        )
        .unwrap();
    assert_eq!(second, expected);
    assert_eq!(second_used, 17);
    assert_eq!(
        bincode::decode_from_slice::<u32, _>(
            &bytes[used + second_used..],
            bincode::config::standard()
        )
        .unwrap()
        .0,
        1234
    );
}

#[derive(Clone, Debug, PartialEq, Encode, Decode, Reflect)]
struct Skipped {
    first: u16,
    #[bincode(skip)]
    #[reflect(ignore)]
    runtime: bool,
    last: u32,
}
#[derive(Encode, Reflect)]
struct HiddenEncoded {
    #[reflect(ignore)]
    secret: u32,
}

#[test]
fn test_skip_attributes_and_reflection_identity_checks() {
    let description = round_trip(
        &Skipped {
            first: 300,
            runtime: false,
            last: 42,
        },
        map(&[("first", Value::U16(300)), ("last", Value::U32(42))]),
        bincode::config::standard(),
    );
    assert_eq!(
        description.schemas[description.bindings[description.root].schema]
            .fields
            .len(),
        2
    );
    let error = ValueDecodeDescription::from_type::<HiddenEncoded>()
        .unwrap_err()
        .to_string();
    assert!(error.contains("HiddenEncoded"));
    assert!(error.contains("secret"));
}

#[test]
fn test_invalid_truncated_data_operations_and_limits() {
    let sample = Attitude {
        orientation: Orientation([1.0, 2.0, 3.0, 4.0]),
        valid: true,
    };
    let description = ValueDecodeDescription::from_type::<Attitude>().unwrap();
    let bytes = bincode::encode_to_vec(sample, bincode::config::standard()).unwrap();
    for len in 0..bytes.len() {
        assert!(
            description
                .decode(
                    &bytes[..len],
                    bincode::config::standard(),
                    ValueDecodeLimits::default()
                )
                .is_err()
        );
    }
    let mut invalid = bytes.clone();
    *invalid.last_mut().unwrap() = 2;
    assert!(
        description
            .decode(
                &invalid,
                bincode::config::standard(),
                ValueDecodeLimits::default()
            )
            .is_err()
    );
    let mut invalid = description.clone();
    invalid.root = usize::MAX;
    assert!(
        invalid
            .decode(
                &bytes,
                bincode::config::standard(),
                ValueDecodeLimits::default()
            )
            .is_err()
    );
    assert!(
        description
            .decode(
                &bytes,
                bincode::config::standard(),
                ValueDecodeLimits {
                    max_depth: 1,
                    ..Default::default()
                }
            )
            .is_err()
    );
    assert!(
        description
            .decode(
                &bytes,
                bincode::config::standard(),
                ValueDecodeLimits {
                    max_values: 1,
                    ..Default::default()
                }
            )
            .is_err()
    );
    let option = ValueDecodeDescription::from_type::<Option<u8>>().unwrap();
    assert!(
        option
            .decode(
                &[2],
                bincode::config::standard(),
                ValueDecodeLimits::default()
            )
            .is_err()
    );
    let event = ValueDecodeDescription::from_type::<Event>().unwrap();
    assert!(
        event
            .decode(
                &[99],
                bincode::config::standard(),
                ValueDecodeLimits::default()
            )
            .is_err()
    );
    let sequence = ValueDecodeDescription::from_type::<Vec<()>>().unwrap();
    let large_count = bincode::encode_to_vec(1_000_001u64, bincode::config::standard()).unwrap();
    assert!(
        sequence
            .decode(
                &large_count,
                bincode::config::standard(),
                ValueDecodeLimits::default()
            )
            .is_err()
    );
    let string = ValueDecodeDescription::from_type::<String>().unwrap();
    assert!(
        string
            .decode(
                &[2, 0xff, 0xff],
                bincode::config::standard(),
                ValueDecodeLimits::default()
            )
            .is_err()
    );
}

#[test]
fn test_recursive_operation_and_execution_limits() {
    #[derive(Clone, Debug, PartialEq, Encode, Decode, Reflect)]
    #[reflect(no_field_bounds)]
    struct Node {
        value: u32,
        children: Vec<Node>,
    }
    let node = Node {
        value: 1,
        children: vec![Node {
            value: 2,
            children: vec![],
        }],
    };
    let expected = map(&[
        ("value", Value::U32(1)),
        (
            "children",
            Value::Seq(vec![map(&[
                ("value", Value::U32(2)),
                ("children", Value::Seq(vec![])),
            ])]),
        ),
    ]);
    round_trip(&node, expected, bincode::config::standard());
}

#[test]
fn test_tuple_declaration_indices_and_ambiguous_reflection() {
    #[derive(Debug, PartialEq, Encode, Decode, Reflect)]
    struct TupleSkipped(#[bincode(skip)] u8, u16);
    let description = round_trip(
        &TupleSkipped(0, 300),
        Value::Seq(vec![Value::U16(300)]),
        bincode::config::standard(),
    );
    let schema = &description.schemas[description.bindings[description.root].schema];
    assert_eq!(schema.fields[0].index, 1);
    #[derive(Encode, Reflect)]
    struct TupleHidden(
        #[bincode(skip)]
        #[reflect(ignore)]
        u8,
        u8,
    );
    assert_eq!(TupleHidden(1, 2).0, 1);
    let error = ValueDecodeDescription::from_type::<TupleHidden>()
        .unwrap_err()
        .to_string();
    assert!(error.contains("TupleHidden tuple field 1"));
}

#[test]
fn test_explicit_bytes_operation_and_manual_type_identity() {
    #[derive(Clone, Debug, PartialEq, Decode, Reflect)]
    #[reflect(opaque)]
    struct OpaqueBytes(Vec<u8>);
    impl Encode for OpaqueBytes {
        fn encode<E: bincode::enc::Encoder>(
            &self,
            encoder: &mut E,
        ) -> Result<(), bincode::error::EncodeError> {
            self.0.encode(encoder)
        }
    }
    impl ValueDecode for OpaqueBytes {
        const DECODE: &'static ValueDecodeSpec = &ValueDecodeSpec::Bytes;
    }
    round_trip(
        &OpaqueBytes(vec![0, 1, 255]),
        Value::Bytes(vec![0, 1, 255]),
        bincode::config::standard(),
    );
}

#[test]
fn test_enum_tags_use_declaration_order_and_skipped_fields() {
    #[derive(Debug, PartialEq, Encode, Decode, Reflect)]
    #[repr(u8)]
    enum WireEvent {
        Ready = 42,
        Point {
            value: u16,
            #[bincode(skip)]
            runtime: bool,
        } = 99,
        Pair(u16, #[bincode(skip)] bool) = 7,
    }
    round_trip(
        &WireEvent::Ready,
        Value::String("Ready".into()),
        bincode::config::standard(),
    );
    round_trip(
        &WireEvent::Point {
            value: 300,
            runtime: false,
        },
        map(&[("Point", map(&[("value", Value::U16(300))]))]),
        bincode::config::standard(),
    );
    round_trip(
        &WireEvent::Pair(300, false),
        map(&[("Pair", Value::Seq(vec![Value::U16(300)]))]),
        bincode::config::standard(),
    );
}

#[test]
fn test_derived_newtype_shape() {
    #[derive(Debug, PartialEq, Encode, Decode, Reflect)]
    struct Newtype(u32);
    round_trip(
        &Newtype(300),
        Value::Newtype(Box::new(Value::U32(300))),
        bincode::config::standard(),
    );
}

#[test]
fn test_quantity_registrations_cover_both_widths_and_named_dimensionless_units() {
    use core::any::TypeId;
    let registrations = cu29_units::value_decode_quantities();
    let identities: alloc::collections::BTreeSet<_> = registrations
        .iter()
        .map(|registration| registration.type_id)
        .collect();
    assert_eq!(identities.len(), registrations.len());
    assert!(registrations.len() > 200);
    for (identity, unit) in [
        (TypeId::of::<cu29_units::si::f32::Length>(), "m"),
        (TypeId::of::<cu29_units::si::f64::Length>(), "m"),
        (TypeId::of::<cu29_units::si::f32::Mass>(), "kg"),
        (TypeId::of::<cu29_units::si::f64::Mass>(), "kg"),
        (TypeId::of::<cu29_units::si::f32::Angle>(), "rad"),
        (TypeId::of::<cu29_units::si::f64::SolidAngle>(), "sr"),
        (TypeId::of::<cu29_units::si::f32::Information>(), "bit"),
        (
            TypeId::of::<cu29_units::si::f64::InformationRate>(),
            "bit s^-1",
        ),
    ] {
        assert_eq!(
            registrations
                .iter()
                .find(|registration| registration.type_id == identity)
                .unwrap()
                .storage_unit,
            unit
        );
    }
}

#[test]
fn test_shared_output_budget_is_not_reset_between_bindings() {
    let description = ValueDecodeDescription::from_type::<String>().unwrap();
    let bytes = bincode::encode_to_vec("x".repeat(1_000_000), bincode::config::standard()).unwrap();
    let limits = ValueDecodeLimits::default();
    let mut budget = ValueDecodeBudget::new(limits);
    description
        .decode_at_with_budget(
            description.root,
            &bytes,
            bincode::config::standard(),
            limits,
            &mut budget,
        )
        .unwrap();
    for _ in 0..32 {
        match description.decode_at_with_budget(
            description.root,
            &bytes,
            bincode::config::standard(),
            limits,
            &mut budget,
        ) {
            Ok(_) => {}
            Err(error) => {
                assert!(error.to_string().contains("output byte limit"));
                return;
            }
        }
    }
    panic!("Shared output budget was reset between bindings");
}

#[test]
fn test_repeated_field_names_are_charged_to_output_budget() {
    #[derive(Encode, Reflect)]
    struct Named {
        value: (),
    }
    let mut description = ValueDecodeDescription::from_type::<Vec<Named>>().unwrap();
    let field = description
        .schemas
        .iter_mut()
        .flat_map(|schema| &mut schema.fields)
        .find(|field| field.name.as_deref() == Some("value"))
        .unwrap();
    field.name = Some("x".repeat(1024));
    let samples = (0..20_000).map(|_| Named { value: () }).collect::<Vec<_>>();
    let bytes = bincode::encode_to_vec(samples, bincode::config::standard()).unwrap();
    let error = description
        .decode(
            &bytes,
            bincode::config::standard(),
            ValueDecodeLimits::default(),
        )
        .unwrap_err();
    assert!(error.to_string().contains("output byte limit"));
}

#[test]
fn test_unused_wire_operations_are_validated() {
    let mut description = ValueDecodeDescription::from_type::<u32>().unwrap();
    description.operations.push(ValueDecodeOp::Record {
        shape: ValueDecodeShape::Newtype,
        fields: Vec::new(),
    });
    assert!(description.validate().is_err());
    description.operations.pop();
    description.operations.push(ValueDecodeOp::Enum {
        tag: ValueDecodeScalar::U8,
        branches: vec![ValueDecodeBranch {
            tag: 256,
            shape: ValueDecodeShape::Unit,
            fields: Vec::new(),
        }],
    });
    assert!(description.validate().is_err());
}

#[test]
fn test_opaque_encoding_recipes_describe_enum_and_record_fields() {
    #[derive(Debug, Clone, PartialEq, Encode, Decode, Reflect)]
    #[reflect(opaque)]
    enum Update {
        NoChange,
        Set(u16),
        Clear,
    }
    for (sample, expected) in [
        (Update::NoChange, Value::String("NoChange".into())),
        (Update::Set(300), map(&[("Set", Value::U16(300))])),
        (Update::Clear, Value::String("Clear".into())),
    ] {
        round_trip(&sample, expected, bincode::config::standard());
    }

    #[derive(Debug, Clone, PartialEq, Encode, Decode, Reflect)]
    #[reflect(opaque)]
    struct Sample {
        channel: u16,
    }
    round_trip(
        &Sample { channel: 300 },
        map(&[("channel", Value::U16(300))]),
        bincode::config::standard(),
    );
}
