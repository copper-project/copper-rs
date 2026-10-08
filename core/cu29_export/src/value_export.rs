//! Plain offline value serialization; schemas retain native scalar widths.

use cu29::prelude::Value;
use serde::Serialize;
use serde::Serializer;
use serde::ser::{SerializeMap, SerializeSeq};

pub(crate) struct PlainValue<'a>(pub &'a Value);

impl Serialize for PlainValue<'_> {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        match self.0 {
            Value::Bool(v) => serializer.serialize_bool(*v),
            Value::U8(v) => serializer.serialize_u8(*v),
            Value::U16(v) => serializer.serialize_u16(*v),
            Value::U32(v) => serializer.serialize_u32(*v),
            Value::U64(v) => serializer.serialize_u64(*v),
            Value::U128(v) => serializer.serialize_u128(*v),
            Value::I8(v) => serializer.serialize_i8(*v),
            Value::I16(v) => serializer.serialize_i16(*v),
            Value::I32(v) => serializer.serialize_i32(*v),
            Value::I64(v) => serializer.serialize_i64(*v),
            Value::I128(v) => serializer.serialize_i128(*v),
            Value::F32(v) if v.is_finite() => serializer.serialize_f32(*v),
            Value::F64(v) if v.is_finite() => serializer.serialize_f64(*v),
            Value::F32(v) => nonfinite(f64::from(*v), serializer),
            Value::F64(v) => nonfinite(*v, serializer),
            Value::Char(v) => serializer.serialize_char(*v),
            Value::String(v) => serializer.serialize_str(v),
            Value::Bytes(v) => serializer.serialize_bytes(v),
            Value::CuTime(v) => serializer.serialize_u64(v.as_nanos()),
            Value::Unit | Value::Option(None) => serializer.serialize_unit(),
            Value::Option(Some(v)) | Value::Newtype(v) => PlainValue(v).serialize(serializer),
            Value::Seq(values) => {
                let mut seq = serializer.serialize_seq(Some(values.len()))?;
                for value in values {
                    seq.serialize_element(&PlainValue(value))?;
                }
                seq.end()
            }
            Value::Map(values) if values.keys().all(|key| matches!(key, Value::String(_))) => {
                let mut map = serializer.serialize_map(Some(values.len()))?;
                for (key, value) in values {
                    if let Value::String(key) = key {
                        map.serialize_entry(key, &PlainValue(value))?;
                    }
                }
                map.end()
            }
            Value::Map(values) => {
                struct Pairs<'a>(&'a std::collections::BTreeMap<Value, Value>);
                impl Serialize for Pairs<'_> {
                    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
                        let mut seq = serializer.serialize_seq(Some(self.0.len()))?;
                        for (key, value) in self.0 {
                            seq.serialize_element(&(PlainValue(key), PlainValue(value)))?;
                        }
                        seq.end()
                    }
                }
                let mut map = serializer.serialize_map(Some(1))?;
                map.serialize_entry("$map", &Pairs(values))?;
                map.end()
            }
        }
    }
}
fn nonfinite<S: Serializer>(value: f64, serializer: S) -> Result<S::Ok, S::Error> {
    let mut map = serializer.serialize_map(Some(1))?;
    map.serialize_entry(
        "$float",
        if value.is_nan() {
            "NaN"
        } else if value.is_sign_positive() {
            "+Inf"
        } else {
            "-Inf"
        },
    )?;
    map.end()
}
pub(crate) fn serialize_payload<S: Serializer>(
    payload: &Option<Value>,
    serializer: S,
) -> Result<S::Ok, S::Error> {
    payload.as_ref().map(PlainValue).serialize(serializer)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_plain_values_keep_wide_numbers_and_special_values() {
        assert_eq!(
            serde_json::to_string(&PlainValue(&Value::U128(u128::MAX))).unwrap(),
            u128::MAX.to_string()
        );
        assert_eq!(
            serde_json::to_string(&PlainValue(&Value::F32(f32::NAN))).unwrap(),
            "{\"$float\":\"NaN\"}"
        );
        assert_eq!(
            serde_json::to_string(&PlainValue(&Value::Bytes(vec![0, 255]))).unwrap(),
            "[0,255]"
        );
        let map = Value::Map(std::collections::BTreeMap::from([(
            Value::U32(7),
            Value::Bool(true),
        )]));
        assert_eq!(
            serde_json::to_string(&PlainValue(&map)).unwrap(),
            "{\"$map\":[[7,true]]}"
        );
    }
}
