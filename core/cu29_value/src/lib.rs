#![doc(html_root_url = "https://docs.rs/serde-value/0.7.0/")]

use cu29_clock::{CuDuration, CuTime};
use ordered_float::OrderedFloat;
use serde::Deserialize;
use std::cmp::Ordering;
use std::collections::BTreeMap;
use std::fmt::{Display, Formatter};
use std::hash::{Hash, Hasher};

mod bdec;
mod benc;
mod de;
mod ser;

pub use de::*;
pub use ser::*;

#[derive(Clone, Debug)]
pub enum Value {
    Bool(bool),

    U8(u8),
    U16(u16),
    U32(u32),
    U64(u64),

    I8(i8),
    I16(i16),
    I32(i32),
    I64(i64),

    F32(f32),
    F64(f64),

    Char(char),
    String(String),
    Unit,
    Option(Option<Box<Value>>),
    Newtype(Box<Value>),
    Seq(Vec<Value>),
    Map(BTreeMap<Value, Value>),
    Bytes(Vec<u8>),

    CuTime(CuTime),
}

impl Display for Value {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Value::Bool(v) => write!(f, "{v}"),
            Value::U8(v) => write!(f, "{v}"),
            Value::U16(v) => write!(f, "{v}"),
            Value::U32(v) => write!(f, "{v}"),
            Value::U64(v) => write!(f, "{v}"),
            Value::I8(v) => write!(f, "{v}"),
            Value::I16(v) => write!(f, "{v}"),
            Value::I32(v) => write!(f, "{v}"),
            Value::I64(v) => write!(f, "{v}"),
            Value::F32(v) => write!(f, "{v}"),
            Value::F64(v) => write!(f, "{v}"),
            Value::Char(v) => write!(f, "{v}"),
            Value::String(v) => write!(f, "{v}"),
            Value::Unit => write!(f, "()"),
            Value::Option(v) => match v {
                Some(v) => write!(f, "Some({v})"),
                None => write!(f, "None"),
            },
            Value::Newtype(v) => write!(f, "{v}"),
            Value::Seq(v) => {
                write!(f, "[")?;
                for (i, v) in v.iter().enumerate() {
                    if i > 0 {
                        write!(f, ", ")?;
                    }
                    write!(f, "{v}")?;
                }
                write!(f, "]")
            }
            Value::Map(v) => {
                write!(f, "{{")?;
                for (i, (k, v)) in v.iter().enumerate() {
                    if i > 0 {
                        write!(f, ", ")?;
                    }
                    write!(f, "{k}: {v}")?;
                }
                write!(f, "}}")
            }
            Value::Bytes(v) => {
                write!(f, "[")?;
                for (i, b) in v.iter().enumerate() {
                    if i > 0 {
                        write!(f, " ")?;
                    }
                    write!(f, "{b:02x}")?;
                }
                write!(f, "]")
            }
            Value::CuTime(v) => write!(f, "{v}"),
        }
    }
}

impl Hash for Value {
    fn hash<H>(&self, hasher: &mut H)
    where
        H: Hasher,
    {
        self.discriminant().hash(hasher);
        match *self {
            Value::Bool(v) => v.hash(hasher),
            Value::U8(v) => v.hash(hasher),
            Value::U16(v) => v.hash(hasher),
            Value::U32(v) => v.hash(hasher),
            Value::U64(v) => v.hash(hasher),
            Value::I8(v) => v.hash(hasher),
            Value::I16(v) => v.hash(hasher),
            Value::I32(v) => v.hash(hasher),
            Value::I64(v) => v.hash(hasher),
            Value::F32(v) => OrderedFloat(v).hash(hasher),
            Value::F64(v) => OrderedFloat(v).hash(hasher),
            Value::Char(v) => v.hash(hasher),
            Value::String(ref v) => v.hash(hasher),
            Value::Unit => 0_u8.hash(hasher),
            Value::Option(ref v) => v.hash(hasher),
            Value::Newtype(ref v) => v.hash(hasher),
            Value::Seq(ref v) => v.hash(hasher),
            Value::Map(ref v) => v.hash(hasher),
            Value::Bytes(ref v) => v.hash(hasher),
            Value::CuTime(v) => {
                let CuDuration(nanos) = v;
                nanos.hash(hasher)
            }
        }
    }
}

impl PartialEq for Value {
    fn eq(&self, rhs: &Self) -> bool {
        match (self, rhs) {
            (&Value::Bool(v0), &Value::Bool(v1)) if v0 == v1 => true,
            (&Value::U8(v0), &Value::U8(v1)) if v0 == v1 => true,
            (&Value::U16(v0), &Value::U16(v1)) if v0 == v1 => true,
            (&Value::U32(v0), &Value::U32(v1)) if v0 == v1 => true,
            (&Value::U64(v0), &Value::U64(v1)) if v0 == v1 => true,
            (&Value::I8(v0), &Value::I8(v1)) if v0 == v1 => true,
            (&Value::I16(v0), &Value::I16(v1)) if v0 == v1 => true,
            (&Value::I32(v0), &Value::I32(v1)) if v0 == v1 => true,
            (&Value::I64(v0), &Value::I64(v1)) if v0 == v1 => true,
            (&Value::F32(v0), &Value::F32(v1)) if OrderedFloat(v0) == OrderedFloat(v1) => true,
            (&Value::F64(v0), &Value::F64(v1)) if OrderedFloat(v0) == OrderedFloat(v1) => true,
            (&Value::Char(v0), &Value::Char(v1)) if v0 == v1 => true,
            (Value::String(v0), Value::String(v1)) if v0 == v1 => true,
            (&Value::Unit, &Value::Unit) => true,
            (Value::Option(v0), Value::Option(v1)) if v0 == v1 => true,
            (Value::Newtype(v0), Value::Newtype(v1)) if v0 == v1 => true,
            (Value::Seq(v0), Value::Seq(v1)) if v0 == v1 => true,
            (Value::Map(v0), Value::Map(v1)) if v0 == v1 => true,
            (Value::Bytes(v0), Value::Bytes(v1)) if v0 == v1 => true,
            (&Value::CuTime(v0), &Value::CuTime(v1)) if v0 == v1 => true,
            _ => false,
        }
    }
}

impl Ord for Value {
    fn cmp(&self, rhs: &Self) -> Ordering {
        match (self, rhs) {
            (&Value::Bool(v0), Value::Bool(v1)) => v0.cmp(v1),
            (&Value::U8(v0), Value::U8(v1)) => v0.cmp(v1),
            (&Value::U16(v0), Value::U16(v1)) => v0.cmp(v1),
            (&Value::U32(v0), Value::U32(v1)) => v0.cmp(v1),
            (&Value::U64(v0), Value::U64(v1)) => v0.cmp(v1),
            (&Value::I8(v0), Value::I8(v1)) => v0.cmp(v1),
            (&Value::I16(v0), Value::I16(v1)) => v0.cmp(v1),
            (&Value::I32(v0), Value::I32(v1)) => v0.cmp(v1),
            (&Value::I64(v0), Value::I64(v1)) => v0.cmp(v1),
            (&Value::F32(v0), &Value::F32(v1)) => OrderedFloat(v0).cmp(&OrderedFloat(v1)),
            (&Value::F64(v0), &Value::F64(v1)) => OrderedFloat(v0).cmp(&OrderedFloat(v1)),
            (&Value::Char(v0), Value::Char(v1)) => v0.cmp(v1),
            (Value::String(v0), Value::String(v1)) => v0.cmp(v1),
            (&Value::Unit, &Value::Unit) => Ordering::Equal,
            (Value::Option(v0), Value::Option(v1)) => v0.cmp(v1),
            (Value::Newtype(v0), Value::Newtype(v1)) => v0.cmp(v1),
            (Value::Seq(v0), Value::Seq(v1)) => v0.cmp(v1),
            (Value::Map(v0), Value::Map(v1)) => v0.cmp(v1),
            (Value::Bytes(v0), Value::Bytes(v1)) => v0.cmp(v1),
            (&Value::CuTime(v0), &Value::CuTime(v1)) => v0.cmp(&v1),
            (v0, v1) => v0.discriminant().cmp(&v1.discriminant()),
        }
    }
}

impl Value {
    fn discriminant(&self) -> usize {
        match *self {
            Value::Bool(..) => 0,
            Value::U8(..) => 1,
            Value::U16(..) => 2,
            Value::U32(..) => 3,
            Value::U64(..) => 4,
            Value::I8(..) => 5,
            Value::I16(..) => 6,
            Value::I32(..) => 7,
            Value::I64(..) => 8,
            Value::F32(..) => 9,
            Value::F64(..) => 10,
            Value::Char(..) => 11,
            Value::String(..) => 12,
            Value::Unit => 13,
            Value::Option(..) => 14,
            Value::Newtype(..) => 15,
            Value::Seq(..) => 16,
            Value::Map(..) => 17,
            Value::Bytes(..) => 18,
            Value::CuTime(..) => 32,
        }
    }

    fn unexpected(&self) -> serde::de::Unexpected {
        match *self {
            Value::Bool(b) => serde::de::Unexpected::Bool(b),
            Value::U8(n) => serde::de::Unexpected::Unsigned(n as u64),
            Value::U16(n) => serde::de::Unexpected::Unsigned(n as u64),
            Value::U32(n) => serde::de::Unexpected::Unsigned(n as u64),
            Value::U64(n) => serde::de::Unexpected::Unsigned(n),
            Value::I8(n) => serde::de::Unexpected::Signed(n as i64),
            Value::I16(n) => serde::de::Unexpected::Signed(n as i64),
            Value::I32(n) => serde::de::Unexpected::Signed(n as i64),
            Value::I64(n) => serde::de::Unexpected::Signed(n),
            Value::F32(n) => serde::de::Unexpected::Float(n as f64),
            Value::F64(n) => serde::de::Unexpected::Float(n),
            Value::Char(c) => serde::de::Unexpected::Char(c),
            Value::String(ref s) => serde::de::Unexpected::Str(s),
            Value::Unit => serde::de::Unexpected::Unit,
            Value::Option(_) => serde::de::Unexpected::Option,
            Value::Newtype(_) => serde::de::Unexpected::NewtypeStruct,
            Value::Seq(_) => serde::de::Unexpected::Seq,
            Value::Map(_) => serde::de::Unexpected::Map,
            Value::Bytes(ref b) => serde::de::Unexpected::Bytes(b),
            Value::CuTime(n) => {
                let CuDuration(nanos) = n;
                serde::de::Unexpected::Unsigned(nanos)
            }
        }
    }

    pub fn deserialize_into<'de, T: Deserialize<'de>>(self) -> Result<T, DeserializerError> {
        T::deserialize(self)
    }
}

impl Eq for Value {}
impl PartialOrd for Value {
    fn partial_cmp(&self, rhs: &Self) -> Option<Ordering> {
        Some(self.cmp(rhs))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bincode::{config::standard, decode_from_slice, encode_to_vec};
    use cu29_clock::{CuDuration, CuTime, RobotClock};
    use serde_derive::{Deserialize, Serialize};
    use std::collections::{hash_map::DefaultHasher, BTreeMap};
    use std::hash::{Hash, Hasher};
    use std::time::Duration;

    #[test]
    fn de_smoke_test() {
        // some convoluted Value
        let value = Value::Option(Some(Box::new(Value::Seq(vec![
            Value::U16(8),
            Value::Char('a'),
            Value::F32(1.0),
            Value::String("hello".into()),
            Value::Map(
                vec![
                    (Value::Bool(false), Value::Unit),
                    (
                        Value::Bool(true),
                        Value::Newtype(Box::new(Value::Bytes(b"hi".as_ref().into()))),
                    ),
                ]
                .into_iter()
                .collect(),
            ),
        ]))));

        // assert that the value remains unchanged through deserialization
        let value_de = Value::deserialize(value.clone()).unwrap();
        assert_eq!(value_de, value);
    }

    #[test]
    fn ser_smoke_test() {
        #[derive(Serialize)]
        struct Foo {
            a: u32,
            b: String,
            c: Vec<bool>,
        }

        let foo = Foo {
            a: 15,
            b: "hello".into(),
            c: vec![true, false],
        };

        let expected = Value::Map(
            vec![
                (Value::String("a".into()), Value::U32(15)),
                (Value::String("b".into()), Value::String("hello".into())),
                (
                    Value::String("c".into()),
                    Value::Seq(vec![Value::Bool(true), Value::Bool(false)]),
                ),
            ]
            .into_iter()
            .collect(),
        );

        let value = to_value(&foo).unwrap();
        assert_eq!(expected, value);
    }

    #[test]
    fn deserialize_into_enum() {
        #[derive(Deserialize, Debug, PartialEq, Eq)]
        enum Foo {
            Bar,
            Baz(u8),
        }

        let value = Value::String("Bar".into());
        assert_eq!(Foo::deserialize(value).unwrap(), Foo::Bar);

        let value = Value::Map(
            vec![(Value::String("Baz".into()), Value::U8(1))]
                .into_iter()
                .collect(),
        );
        assert_eq!(Foo::deserialize(value).unwrap(), Foo::Baz(1));
    }

    #[test]
    fn serialize_from_enum() {
        #[derive(Serialize)]
        enum Foo {
            Bar,
            Baz(u8),
            Qux { quux: u8 },
            Corge(u8, u8),
        }

        let bar = Foo::Bar;
        assert_eq!(to_value(&bar).unwrap(), Value::String("Bar".into()));

        let baz = Foo::Baz(1);
        assert_eq!(
            to_value(&baz).unwrap(),
            Value::Map(
                vec![(Value::String("Baz".into()), Value::U8(1))]
                    .into_iter()
                    .collect(),
            )
        );

        let qux = Foo::Qux { quux: 2 };
        assert_eq!(
            to_value(&qux).unwrap(),
            Value::Map(
                vec![(
                    Value::String("Qux".into()),
                    Value::Map(
                        vec![(Value::String("quux".into()), Value::U8(2))]
                            .into_iter()
                            .collect()
                    )
                )]
                .into_iter()
                .collect()
            )
        );

        let corge = Foo::Corge(3, 4);
        assert_eq!(
            to_value(&corge).unwrap(),
            Value::Map(
                vec![(
                    Value::String("Corge".into()),
                    Value::Seq(vec![Value::U8(3), Value::U8(4)])
                )]
                .into_iter()
                .collect()
            )
        );
    }

    #[test]
    fn deserialize_inside_deserialize_impl() {
        #[derive(Debug, PartialEq, Eq)]
        enum Event {
            Added(u32),
            Error(u8),
        }

        impl<'de> serde::Deserialize<'de> for Event {
            fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
            where
                D: serde::Deserializer<'de>,
            {
                #[derive(Deserialize)]
                struct RawEvent {
                    kind: String,
                    object: Value,
                }

                let raw_event = RawEvent::deserialize(deserializer)?;

                // Cannot directly use Value as Deserializer, since error type needs to be
                // generic D::Error rather than specific serde_value::DeserializerError
                let object_deserializer = ValueDeserializer::new(raw_event.object);

                Ok(match &*raw_event.kind {
                    "ADDED" => Event::Added(<_>::deserialize(object_deserializer)?),
                    "ERROR" => Event::Error(<_>::deserialize(object_deserializer)?),
                    kind => {
                        return Err(serde::de::Error::unknown_variant(kind, &["ADDED", "ERROR"]))
                    }
                })
            }
        }

        let input = Value::Map(
            vec![
                (
                    Value::String("kind".to_owned()),
                    Value::String("ADDED".to_owned()),
                ),
                (Value::String("object".to_owned()), Value::U32(5)),
            ]
            .into_iter()
            .collect(),
        );
        let event = Event::deserialize(input).expect("could not deserialize ADDED event");
        assert_eq!(event, Event::Added(5));

        let input = Value::Map(
            vec![
                (
                    Value::String("kind".to_owned()),
                    Value::String("ERROR".to_owned()),
                ),
                (Value::String("object".to_owned()), Value::U8(5)),
            ]
            .into_iter()
            .collect(),
        );
        let event = Event::deserialize(input).expect("could not deserialize ERROR event");
        assert_eq!(event, Event::Error(5));

        let input = Value::Map(
            vec![
                (
                    Value::String("kind".to_owned()),
                    Value::String("ADDED".to_owned()),
                ),
                (Value::String("object".to_owned()), Value::Unit),
            ]
            .into_iter()
            .collect(),
        );
        let _ =
            Event::deserialize(input).expect_err("expected deserializing bad ADDED event to fail");
    }

    #[test]
    fn deserialize_newtype() {
        #[derive(Debug, Deserialize, PartialEq)]
        struct Foo(i32);

        let input = Value::I32(5);
        let foo = Foo::deserialize(input).unwrap();
        assert_eq!(foo, Foo(5));
    }

    #[test]
    fn deserialize_newtype2() {
        #[derive(Debug, Deserialize, PartialEq)]
        struct Foo(i32);

        #[derive(Debug, Deserialize, PartialEq)]
        struct Bar {
            foo: Foo,
        }

        let input = Value::Map(
            vec![(Value::String("foo".to_owned()), Value::I32(5))]
                .into_iter()
                .collect(),
        );
        let bar = Bar::deserialize(input).unwrap();
        assert_eq!(bar, Bar { foo: Foo(5) });
    }

    #[test]
    fn clock_ser_deser() {
        let (clock, mock) = RobotClock::mock();
        mock.increment(Duration::from_nanos(42));
        let c = clock.now();

        let input = Value::CuTime(c);
        let foo = CuTime::deserialize(input).unwrap();
        assert_eq!(foo, CuTime::from(Duration::from_nanos(42)));
    }
    #[test]
    fn value_encode_decode() {
        fn check_value(value: Value) {
            let v = bincode::encode_to_vec(&value, standard()).expect("encode failed");
            let (v2, s) = bincode::decode_from_slice::<Value, _>(v.as_slice(), standard())
                .expect("decode failed");
            assert_eq!(s, v.len());
            assert_eq!(&v2, &value);
        }

        check_value(Value::Bool(true));
        check_value(Value::U8(42));
        check_value(Value::U16(42));
        check_value(Value::U32(42));
        check_value(Value::U64(42));
        check_value(Value::I8(42));
        check_value(Value::I16(42));
        check_value(Value::I32(42));
        check_value(Value::I64(42));
        check_value(Value::F32(42.42));
        check_value(Value::F64(42.42));
        check_value(Value::Char('4'));
        check_value(Value::String("42".into()));
        check_value(Value::Unit);
        check_value(Value::Option(Some(Box::new(Value::U32(42)))));
        check_value(Value::Newtype(Box::new(Value::U32(42))));
        check_value(Value::Seq(vec![Value::Bool(true), Value::U32(42)]));
        check_value(Value::Map(BTreeMap::from([
            (Value::Bool(true), Value::U32(42)),
            (Value::String("42".into()), Value::I32(42)),
        ])));
        check_value(Value::Bytes(vec![0x4, 0x2]));
        check_value(Value::CuTime(CuTime::from(Duration::from_nanos(42))));
    }

    #[test]
    fn test_cutime_tovalue() {
        let c = CuTime::from(Duration::from_nanos(42));
        let v = to_value(c).expect("to_value failed");
        assert_eq!(v, Value::CuTime(c));
    }
    /// Test basic value creation and type checking
    #[test]
    fn test_value_creation_and_types() {
        // Create various value types
        let bool_val = Value::Bool(true);
        let i32_val = Value::I32(42);
        let str_val = Value::String("test".to_string());
        let unit_val = Value::Unit;
        let option_val = Value::Option(Some(Box::new(Value::U8(5))));

        // Verify type discrimination
        assert!(matches!(bool_val, Value::Bool(true)));
        assert!(matches!(i32_val, Value::I32(42)));
        assert!(matches!(str_val, Value::String(ref s) if s == "test"));
        assert!(matches!(unit_val, Value::Unit));
        assert!(matches!(option_val, Value::Option(Some(_))));

        // Check discriminant values
        assert_eq!(bool_val.discriminant(), 0);
        assert_eq!(i32_val.discriminant(), 7);
        assert_eq!(str_val.discriminant(), 12);
    }

    /// Test numeric boundary values and special floating points
    #[test]
    fn test_numeric_boundaries_and_special_values() {
        // Integer boundaries
        let min_i8 = Value::I8(i8::MIN);
        let max_i8 = Value::I8(i8::MAX);
        let min_i64 = Value::I64(i64::MIN);
        let max_u64 = Value::U64(u64::MAX);

        // Special floating points
        let nan = Value::F64(f64::NAN);
        let pos_inf = Value::F64(f64::INFINITY);
        let neg_inf = Value::F64(f64::NEG_INFINITY);
        let zero = Value::F64(0.0);
        let neg_zero = Value::F64(-0.0);

        // Make sure these encode and decode correctly
        for val in [
            min_i8,
            max_i8,
            min_i64,
            max_u64,
            nan.clone(),
            pos_inf.clone(),
            neg_inf.clone(),
            zero.clone(),
            neg_zero.clone(),
        ] {
            let encoded = encode_to_vec(&val, standard()).unwrap();
            let (decoded, _): (Value, _) = decode_from_slice(&encoded, standard()).unwrap();

            // Special case for NaN since NaN != NaN in normal floating point comparisons
            if matches!(val, Value::F64(f) if f.is_nan()) {
                assert!(matches!(decoded, Value::F64(f) if f.is_nan()));
            } else {
                assert_eq!(val, decoded);
            }
        }

        // Test ordering behavior with special values
        assert!(pos_inf > zero);
        assert!(neg_inf < zero);

        // NaN should be equal to itself when wrapped in Value (due to OrderedFloat)
        let nan2 = Value::F64(f64::NAN);
        assert_eq!(nan, nan2); // This works because Value uses OrderedFloat

        // Verify zero and negative zero are treated as equal
        assert_eq!(zero, neg_zero);
    }

    /// Test handling of containers (maps, sequences)
    #[test]
    fn test_container_types() {
        // Empty containers
        let empty_seq = Value::Seq(vec![]);
        let empty_map = Value::Map(BTreeMap::new());

        // Simple containers
        let simple_seq = Value::Seq(vec![Value::I32(1), Value::I32(2), Value::I32(3)]);
        let mut simple_map = BTreeMap::new();
        simple_map.insert(Value::String("key".to_string()), Value::Bool(true));
        let simple_map_val = Value::Map(simple_map);

        // Deeply nested containers
        let mut nested_map = BTreeMap::new();
        nested_map.insert(
            Value::String("outer".to_string()),
            Value::Seq(vec![
                Value::Option(Some(Box::new(Value::Map({
                    let mut m = BTreeMap::new();
                    m.insert(Value::I32(1), Value::String("nested".to_string()));
                    m
                })))),
                Value::Bool(false),
            ]),
        );
        let nested_val = Value::Map(nested_map);

        // Encode and decode all container types
        for val in [empty_seq, empty_map, simple_seq, simple_map_val, nested_val] {
            let encoded = encode_to_vec(&val, standard()).unwrap();
            let (decoded, _): (Value, _) = decode_from_slice(&encoded, standard()).unwrap();
            assert_eq!(val, decoded);
        }
    }

    /// Test handling of large values
    #[test]
    fn test_large_values() {
        // Large sequence
        let large_seq = Value::Seq((0..10000).map(Value::I32).collect());

        // Large string
        let large_string = Value::String("x".repeat(100000));

        // Large bytes
        let large_bytes = Value::Bytes((0..10000).map(|i| (i % 256) as u8).collect());

        // Large nested structure
        let mut large_map = BTreeMap::new();
        for i in 0..1000 {
            large_map.insert(
                Value::I32(i),
                Value::Seq((0..10).map(|j| Value::I32(i * j)).collect()),
            );
        }
        let large_nested = Value::Map(large_map);

        // Test round-trip for large values
        for val in [large_seq, large_string, large_bytes, large_nested] {
            let encoded = encode_to_vec(&val, standard()).unwrap();
            let (decoded, _): (Value, _) = decode_from_slice(&encoded, standard()).unwrap();
            assert_eq!(val, decoded);
        }
    }

    /// Test value comparison across different types
    #[test]
    fn test_value_comparison() {
        // Same type comparisons
        assert!(Value::I32(1) < Value::I32(2));
        assert!(Value::String("a".to_string()) < Value::String("b".to_string()));
        assert!(Value::Bool(false) < Value::Bool(true));

        // Different type comparisons (based on discriminant)
        assert!(Value::Bool(true) < Value::I32(0)); // Bool(0) < I32(7) by discriminant
        assert!(Value::I32(100) < Value::String("a".to_string())); // I32(7) < String(12)

        // Container comparisons
        assert!(
            Value::Seq(vec![Value::I32(1), Value::I32(2)])
                < Value::Seq(vec![Value::I32(1), Value::I32(3)])
        );

        let mut map1 = BTreeMap::new();
        map1.insert(Value::String("key".to_string()), Value::I32(1));

        let mut map2 = BTreeMap::new();
        map2.insert(Value::String("key".to_string()), Value::I32(2));

        assert!(Value::Map(map1) < Value::Map(map2));

        // Test equality with NaN handling
        let nan1 = Value::F64(f64::NAN);
        let nan2 = Value::F64(f64::NAN);
        assert_eq!(nan1, nan2); // OrderedFloat makes NaN == NaN
    }

    /// Test hash consistency for various value types
    #[test]
    fn test_value_hashing() {
        let values = [
            Value::Bool(true),
            Value::I32(42),
            Value::String("hash me".to_string()),
            Value::F64(3.1),
            Value::Char('🦀'),
            Value::Option(Some(Box::new(Value::U8(5)))),
            Value::Unit,
        ];

        for val in values {
            // Hash the same value twice, should be consistent
            let mut hasher1 = DefaultHasher::new();
            let mut hasher2 = DefaultHasher::new();
            val.hash(&mut hasher1);
            val.hash(&mut hasher2);
            assert_eq!(hasher1.finish(), hasher2.finish());

            // Clone and hash, should be the same
            let val_clone = val.clone();
            let mut hasher3 = DefaultHasher::new();
            val_clone.hash(&mut hasher3);
            assert_eq!(hasher1.finish(), hasher3.finish());
        }

        // Special case: NaN should have consistent hash
        let nan1 = Value::F64(f64::NAN);
        let nan2 = Value::F64(f64::NAN);

        let mut hasher1 = DefaultHasher::new();
        let mut hasher2 = DefaultHasher::new();
        nan1.hash(&mut hasher1);
        nan2.hash(&mut hasher2);
        assert_eq!(hasher1.finish(), hasher2.finish());
    }

    /// Test serialization/deserialization of custom data structures
    #[test]
    fn test_struct_serde() {
        #[derive(Serialize, Deserialize, Debug, PartialEq)]
        struct Person {
            name: String,
            age: u32,
            addresses: Vec<Address>,
        }

        #[derive(Serialize, Deserialize, Debug, PartialEq)]
        struct Address {
            street: String,
            city: String,
            zip: u32,
        }

        let person = Person {
            name: "Alice".to_string(),
            age: 30,
            addresses: vec![
                Address {
                    street: "123 Main St".to_string(),
                    city: "Anytown".to_string(),
                    zip: 12345,
                },
                Address {
                    street: "456 Oak Ave".to_string(),
                    city: "Somewhere".to_string(),
                    zip: 67890,
                },
            ],
        };

        // Convert to Value
        let value = to_value(&person).unwrap();

        // Check structure
        assert!(matches!(value, Value::Map(_)));

        // Convert back to original type
        let person2 = value.deserialize_into::<Person>().unwrap();
        assert_eq!(person, person2);
    }

    /// Test enum serialization/deserialization
    #[test]
    fn test_enum_serde() {
        #[derive(Serialize, Deserialize, Debug, PartialEq)]
        enum MyEnum {
            Unit,
            NewType(i32),
            Tuple(String, bool),
            Struct { x: f64, y: f64 },
        }

        // Test all variants
        let variants = vec![
            MyEnum::Unit,
            MyEnum::NewType(42),
            MyEnum::Tuple("hello".to_string(), true),
            MyEnum::Struct { x: 1.0, y: 2.0 },
        ];

        for variant in variants {
            let value = to_value(&variant).unwrap();
            let roundtrip = value.deserialize_into::<MyEnum>().unwrap();
            assert_eq!(variant, roundtrip);
        }
    }

    /// Test custom CuTime type handling
    #[test]
    fn test_cutime_handling() {
        // Test round-trip for CuTime values
        let times = vec![
            CuTime::from(CuDuration(0)),
            CuTime::from(CuDuration(1)),
            CuTime::from(CuDuration(u64::MAX / 2)),
            // Exclude MAX as it might be reserved for special use
        ];

        for time in times {
            // Direct Value creation
            let time_value = Value::CuTime(time);

            // Serialize/deserialize as Value
            let encoded = encode_to_vec(&time_value, standard()).unwrap();
            let (decoded, _): (Value, _) = decode_from_slice(&encoded, standard()).unwrap();
            assert_eq!(time_value, decoded);

            // Convert through to_value
            let via_to_value = to_value(time).unwrap();
            assert_eq!(via_to_value, time_value);

            // Deserialize back to CuTime
            let time_roundtrip = via_to_value.deserialize_into::<CuTime>().unwrap();
            assert_eq!(time, time_roundtrip);
        }
    }

    /// Test error handling in deserialization
    #[test]
    fn test_error_handling() {
        // Type mismatch
        let bool_val = Value::Bool(true);
        let result = bool_val.clone().deserialize_into::<i32>();
        assert!(result.is_err());

        // Missing fields
        let empty_map = Value::Map(BTreeMap::new());

        #[derive(Deserialize)]
        struct RequiredFields {
            _required: String,
        }

        let result = empty_map.deserialize_into::<RequiredFields>();
        assert!(result.is_err());

        // Invalid enum variant
        let invalid_variant = Value::String("NonExistentVariant".to_string());

        #[derive(Deserialize)]
        enum TestEnum {
            A,
            B,
            C,
        }

        let result = invalid_variant.deserialize_into::<TestEnum>();
        assert!(result.is_err());

        // Check we get appropriate error types
        match bool_val.deserialize_into::<String>() {
            Err(DeserializerError::InvalidType(..)) => (), // Expected
            other => panic!("Expected InvalidType error, got: {other:?}"),
        }
    }

    /// Test unicode handling in strings and chars
    #[test]
    fn test_unicode_handling() {
        let strings = vec![
            "".to_string(),             // Empty
            "ASCII only".to_string(),   // ASCII
            "Café 🍰".to_string(),      // Mixed ASCII and Unicode
            "日本語".to_string(),       // CJK characters
            "👨‍👩‍👧‍👦 Family".to_string(),    // Complex emoji with ZWJ sequences
            "ᛁᚲ ᚲᚨᚾ ᚱᚢᚾᛖᛋ".to_string(), // Ancient runes
        ];

        for s in strings {
            let string_val = Value::String(s.clone());

            // Test round-trip
            let encoded = encode_to_vec(&string_val, standard()).unwrap();
            let (decoded, _): (Value, _) = decode_from_slice(&encoded, standard()).unwrap();

            if let Value::String(decoded_s) = decoded {
                assert_eq!(s, decoded_s);
            } else {
                panic!("Expected String value");
            }
        }

        // Test various Unicode characters
        let chars = vec!['a', 'é', '日', '🦀'];

        for c in chars {
            let char_val = Value::Char(c);

            // Test round-trip
            let encoded = encode_to_vec(&char_val, standard()).unwrap();
            let (decoded, _): (Value, _) = decode_from_slice(&encoded, standard()).unwrap();

            if let Value::Char(decoded_c) = decoded {
                assert_eq!(c, decoded_c);
            } else {
                panic!("Expected Char value");
            }
        }
    }

    /// Test ValueDeserializer directly
    #[test]
    fn test_value_deserializer() {
        let original = vec![1, 2, 3];
        let value = to_value(&original).unwrap();

        // Create a deserializer
        let deserializer: de::ValueDeserializer<DeserializerError> = ValueDeserializer::new(value);

        // Use it to deserialize
        let result: Vec<i32> = serde::Deserialize::deserialize(deserializer).unwrap();

        assert_eq!(original, result);
    }

    /// Test serialization/deserialization with custom types and Option
    #[test]
    fn test_option_handling() {
        // Some values
        let some_i32 = Some(42);
        let some_string = Some("test".to_string());

        // None values of different types
        let none_i32: Option<i32> = None;
        let none_string: Option<String> = None;

        // Convert to Value
        let some_i32_value = to_value(some_i32).unwrap();
        let some_string_value = to_value(&some_string).unwrap();
        let none_i32_value = to_value(none_i32).unwrap();
        let none_string_value = to_value(&none_string).unwrap();

        // Check structure
        assert!(matches!(some_i32_value, Value::Option(Some(_))));
        assert!(matches!(some_string_value, Value::Option(Some(_))));
        assert!(matches!(none_i32_value, Value::Option(None)));
        assert!(matches!(none_string_value, Value::Option(None)));

        // Round-trip
        let some_i32_rt: Option<i32> = some_i32_value.deserialize_into().unwrap();
        let some_string_rt: Option<String> = some_string_value.deserialize_into().unwrap();
        let none_i32_rt: Option<i32> = none_i32_value.deserialize_into().unwrap();
        let none_string_rt: Option<String> = none_string_value.deserialize_into().unwrap();

        assert_eq!(some_i32, some_i32_rt);
        assert_eq!(some_string, some_string_rt);
        assert_eq!(none_i32, none_i32_rt);
        assert_eq!(none_string, none_string_rt);
    }

    /// Test deeply nested option values
    #[test]
    fn test_nested_options() {
        // Create deeply nested Option structure
        let nested_option: Option<Option<Option<i32>>> = Some(Some(Some(42)));

        // Convert to Value
        let value = to_value(nested_option).unwrap();

        // Verify structure
        let mut current = &value;
        for _ in 0..3 {
            assert!(matches!(current, Value::Option(Some(_))));
            if let Value::Option(Some(inner)) = current {
                current = inner;
            } else {
                panic!("Expected Some");
            }
        }
        assert!(matches!(current, Value::I32(42)));

        // Round-trip test
        let result: Option<Option<Option<i32>>> = value.deserialize_into().unwrap();
        assert_eq!(nested_option, result);
    }

    /// Test conversion behaviors between numeric types
    #[test]
    fn test_numeric_conversions() {
        // Create values of different numeric types
        let i8_val = Value::I8(42);
        let i16_val = Value::I16(42);
        let i32_val = Value::I32(42);
        let i64_val = Value::I64(42);
        let u8_val = Value::U8(42);
        let u16_val = Value::U16(42);
        let u32_val = Value::U32(42);
        let u64_val = Value::U64(42);
        let u64_val_large = Value::U64(u64::MAX);
        let f32_val = Value::F32(42.0);
        let f64_val = Value::F64(42.0);

        // Test valid conversions
        // Note: Some of these might depend on implementation details
        assert!(i8_val.deserialize_into::<i16>().is_ok());
        assert!(i16_val.deserialize_into::<i32>().is_ok());
        assert!(i32_val.clone().deserialize_into::<i64>().is_ok());
        assert!(u8_val.deserialize_into::<u16>().is_ok());
        assert!(u16_val.deserialize_into::<u32>().is_ok());
        assert!(u32_val.deserialize_into::<u64>().is_ok());
        assert!(u64_val.clone().deserialize_into::<f64>().is_ok());
        assert!(i32_val.deserialize_into::<f32>().is_ok());
        assert!(f32_val.deserialize_into::<f64>().is_ok());
        assert!(i64_val.clone().deserialize_into::<f64>().is_ok());
        assert!(u64_val.deserialize_into::<i8>().is_ok());

        // Test conversions that shouldn't work
        assert!(u64_val_large.deserialize_into::<i8>().is_err());
        assert!(f64_val.deserialize_into::<u32>().is_err());
        assert!(i64_val.deserialize_into::<bool>().is_err());
    }

    /// Test Display implementation
    #[test]
    fn test_display_implementation() {
        // Sample different value types and check their string representation
        let values = [
            (Value::Bool(true), "true"),
            (Value::I32(42), "42"),
            (Value::String("test".to_string()), "test"),
            (Value::Unit, "()"),
            (
                Value::CuTime(CuTime::from(CuDuration(1_000_000_000))),
                "1.000 s",
            ),
        ];

        for (val, expected) in values {
            assert_eq!(val.to_string(), expected);
        }

        // More complex values
        let seq = Value::Seq(vec![Value::I32(1), Value::I32(2), Value::I32(3)]);
        assert_eq!(seq.to_string(), "[1, 2, 3]");

        let mut map = BTreeMap::new();
        map.insert(Value::String("key".to_string()), Value::Bool(true));
        let map_val = Value::Map(map);
        assert_eq!(map_val.to_string(), "{key: true}");
    }
    #[test]
    fn test_numeric_overflow_detection() {
        // Test overflow detection
        let large_i64 = Value::I64(i64::MAX);
        assert!(large_i64.deserialize_into::<i32>().is_err());

        // Test underflow detection
        let negative = Value::I64(-1);
        assert!(negative.deserialize_into::<u64>().is_err());

        // Edge case: exactly at boundary
        let max_i32 = Value::I64(i32::MAX as i64);
        assert!(max_i32.deserialize_into::<i32>().is_ok());

        // Edge case: one beyond boundary
        let beyond_max_i32 = Value::I64((i32::MAX as i64) + 1);
        assert!(beyond_max_i32.deserialize_into::<i32>().is_err());
    }

    #[test]
    fn test_float_precision_handling() {
        // Integer -> float -> integer round trip
        let original = i64::MAX;
        let as_value = Value::I64(original);
        let as_f64: f64 = as_value.deserialize_into().unwrap();
        let round_trip = Value::F64(as_f64).deserialize_into::<i64>();

        // the round_trip should return Error since f64 cannot represent all i64 values
        assert!(round_trip.is_err());

        // Fractional values to integers
        let half = Value::F64(0.5);
        let half_as_i32 = half.deserialize_into::<i32>();
        assert!(half_as_i32.is_err());
    }

    #[test]
    fn test_float_special_values() {
        // NaN to integer
        let nan = Value::F64(f64::NAN);
        assert!(nan.deserialize_into::<i32>().is_err());

        // Infinity to integer
        let infinity = Value::F64(f64::INFINITY);
        assert!(infinity.deserialize_into::<i64>().is_err());

        // Huge values within float range but beyond integer range
        let huge = Value::F64(1e20);
        assert!(huge.deserialize_into::<i64>().is_err());
    }
}
