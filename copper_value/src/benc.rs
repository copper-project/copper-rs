use crate::Value;

use bincode::enc::Encode;
use bincode::enc::Encoder;
use bincode::error::EncodeError;

impl Encode for Value {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        let discriminant = self.discriminant() as u8;
        discriminant.encode(encoder)?;
        match self {
            Value::U8(v) => v.to_owned().encode(encoder),
            Value::U16(v) => v.to_owned().encode(encoder),
            Value::U32(v) => v.to_owned().encode(encoder),
            Value::U64(v) => v.to_owned().encode(encoder),
            Value::I8(v) => v.to_owned().encode(encoder),
            Value::I16(v) => v.to_owned().encode(encoder),
            Value::I32(v) => v.to_owned().encode(encoder),
            Value::I64(v) => v.to_owned().encode(encoder),
            Value::F32(v) => v.to_owned().encode(encoder),
            Value::F64(v) => v.to_owned().encode(encoder),
            Value::Bool(v) => v.to_owned().encode(encoder),
            Value::Char(v) => v.to_owned().encode(encoder),
            Value::String(v) => v.to_owned().encode(encoder),
            Value::Bytes(v) => v.to_owned().encode(encoder),
            Value::Unit => ().encode(encoder),
            Value::Seq(v) => v.to_owned().encode(encoder),
            Value::Map(v) => v.to_owned().encode(encoder),
            Value::Option(v) => v.to_owned().encode(encoder),
            Value::CuTime(v) => v.to_owned().encode(encoder),
            Value::Newtype(v) => v.to_owned().encode(encoder),
        }
    }
}
