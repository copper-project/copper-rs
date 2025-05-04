use crate::Value;
use bincode::BorrowDecode;
use std::collections::BTreeMap;

use bincode::de::Decoder;
use bincode::de::{BorrowDecoder, Decode};
use bincode::error::DecodeError;
use cu29_clock::CuTime;

// TODO: Unharcode all those enum types values
impl<Context> Decode<Context> for Value {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        match u8::decode(decoder)? {
            0 => Ok(Value::Bool(bool::decode(decoder)?)),
            1 => Ok(Value::U8(u8::decode(decoder)?)),
            2 => Ok(Value::U16(u16::decode(decoder)?)),
            3 => Ok(Value::U32(u32::decode(decoder)?)),
            4 => Ok(Value::U64(u64::decode(decoder)?)),
            5 => Ok(Value::I8(i8::decode(decoder)?)),
            6 => Ok(Value::I16(i16::decode(decoder)?)),
            7 => Ok(Value::I32(i32::decode(decoder)?)),
            8 => Ok(Value::I64(i64::decode(decoder)?)),
            9 => Ok(Value::F32(f32::decode(decoder)?)),
            10 => Ok(Value::F64(f64::decode(decoder)?)),
            11 => Ok(Value::Char(char::decode(decoder)?)),
            12 => Ok(Value::String(String::decode(decoder)?)),
            13 => Ok(Value::Unit),
            14 => Ok(Value::Option(Option::<Box<Value>>::decode(decoder)?)),
            15 => Ok(Value::Newtype(Box::<Value>::decode(decoder)?)),
            16 => Ok(Value::Seq(Vec::<Value>::decode(decoder)?)),
            17 => Ok(Value::Map(BTreeMap::<Value, Value>::decode(decoder)?)),
            18 => Ok(Value::Bytes(Vec::<u8>::decode(decoder)?)),
            32 => Ok(Value::CuTime(CuTime::decode(decoder)?)),
            r => Err(DecodeError::OtherString(format!(
                "Unknown Value variant: {r}"
            ))),
        }
    }
}

impl<'de, Context> BorrowDecode<'de, Context> for Value {
    fn borrow_decode<D: BorrowDecoder<'de>>(decoder: &mut D) -> Result<Self, DecodeError> {
        match u8::borrow_decode(decoder)? {
            0 => Ok(Value::U8(u8::borrow_decode(decoder)?)),
            1 => Ok(Value::U16(u16::borrow_decode(decoder)?)),
            2 => Ok(Value::U32(u32::borrow_decode(decoder)?)),
            3 => Ok(Value::U64(u64::borrow_decode(decoder)?)),
            4 => Ok(Value::I8(i8::borrow_decode(decoder)?)),
            5 => Ok(Value::I16(i16::borrow_decode(decoder)?)),
            6 => Ok(Value::I32(i32::borrow_decode(decoder)?)),
            7 => Ok(Value::I64(i64::borrow_decode(decoder)?)),
            8 => Ok(Value::F32(f32::borrow_decode(decoder)?)),
            9 => Ok(Value::F64(f64::borrow_decode(decoder)?)),
            10 => Ok(Value::Bool(bool::borrow_decode(decoder)?)),
            11 => Ok(Value::Char(char::borrow_decode(decoder)?)),
            12 => Ok(Value::String(String::borrow_decode(decoder)?)),
            13 => Ok(Value::Bytes(Vec::<u8>::borrow_decode(decoder)?)),
            14 => Ok(Value::Unit),
            15 => Ok(Value::Seq(Vec::<Value>::borrow_decode(decoder)?)),
            16 => Ok(Value::Map(BTreeMap::<Value, Value>::borrow_decode(
                decoder,
            )?)),
            17 => Ok(Value::Option(Option::<Box<Value>>::borrow_decode(decoder)?)),
            18 => Ok(Value::Newtype(Box::<Value>::borrow_decode(decoder)?)),
            r => Err(DecodeError::OtherString(format!(
                "Unknown Value variant: {r}"
            ))),
        }
    }
}
