//! Borrowed component parameters emitted by the runtime generator.
extern crate alloc;
use serde::de::{self, DeserializeSeed, IntoDeserializer, Visitor};
use serde::{Deserializer, forward_to_deserialize_any};

#[doc(hidden)]
#[derive(Clone, Copy, Debug)]
pub enum StaticValue {
    Bool(bool),
    Char(char),
    String(&'static str),
    Bytes(&'static [u8]),
    I8(i8),
    I16(i16),
    I32(i32),
    I64(i64),
    U8(u8),
    U16(u16),
    U32(u32),
    U64(u64),
    F32(f32),
    F64(f64),
    Unit,
    Option(Option<&'static Self>),
    Seq(&'static [Self]),
    Map(&'static [(Self, Self)]),
}

type Error = de::value::Error;

impl<'de> IntoDeserializer<'de, Error> for StaticValue {
    type Deserializer = Self;
    fn into_deserializer(self) -> Self {
        self
    }
}

impl<'de> Deserializer<'de> for StaticValue {
    type Error = Error;
    fn deserialize_any<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Error> {
        match self {
            Self::Bool(v) => visitor.visit_bool(v),
            Self::Char(v) => visitor.visit_char(v),
            Self::String(v) => visitor.visit_borrowed_str(v),
            Self::Bytes(v) => visitor.visit_borrowed_bytes(v),
            Self::I8(v) => visitor.visit_i8(v),
            Self::I16(v) => visitor.visit_i16(v),
            Self::I32(v) => visitor.visit_i32(v),
            Self::I64(v) => visitor.visit_i64(v),
            Self::U8(v) => visitor.visit_u8(v),
            Self::U16(v) => visitor.visit_u16(v),
            Self::U32(v) => visitor.visit_u32(v),
            Self::U64(v) => visitor.visit_u64(v),
            Self::F32(v) => visitor.visit_f32(v),
            Self::F64(v) => visitor.visit_f64(v),
            Self::Unit => visitor.visit_unit(),
            Self::Option(None) => visitor.visit_none(),
            Self::Option(Some(v)) => visitor.visit_some(*v),
            Self::Seq(v) => {
                de::value::SeqDeserializer::new(v.iter().copied()).deserialize_any(visitor)
            }
            Self::Map(v) => {
                de::value::MapDeserializer::new(v.iter().copied()).deserialize_any(visitor)
            }
        }
    }
    fn deserialize_option<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Error> {
        match self {
            Self::Option(_) | Self::Unit => self.deserialize_any(visitor),
            _ => visitor.visit_some(self),
        }
    }
    fn deserialize_enum<V: Visitor<'de>>(
        self,
        _: &'static str,
        _: &'static [&'static str],
        visitor: V,
    ) -> Result<V::Value, Error> {
        match self {
            Self::String(v) => visitor.visit_enum(v.into_deserializer()),
            Self::Map([(key, value)]) => visitor.visit_enum(Variant(*key, *value)),
            _ => Err(de::Error::custom(
                "expected enum name or one-entry variant map",
            )),
        }
    }
    fn deserialize_newtype_struct<V: Visitor<'de>>(
        self,
        _: &'static str,
        v: V,
    ) -> Result<V::Value, Error> {
        v.visit_newtype_struct(self)
    }
    forward_to_deserialize_any! { bool i8 i16 i32 i64 i128 u8 u16 u32 u64 u128 f32 f64 char str string bytes byte_buf unit unit_struct seq tuple tuple_struct map struct identifier ignored_any }
}

struct Variant(StaticValue, StaticValue);
impl<'de> de::EnumAccess<'de> for Variant {
    type Error = Error;
    type Variant = StaticValue;
    fn variant_seed<V: DeserializeSeed<'de>>(
        self,
        seed: V,
    ) -> Result<(V::Value, StaticValue), Error> {
        Ok((seed.deserialize(self.0)?, self.1))
    }
}
impl<'de> de::VariantAccess<'de> for StaticValue {
    type Error = Error;
    fn unit_variant(self) -> Result<(), Error> {
        serde::Deserialize::deserialize(self)
    }
    fn newtype_variant_seed<T: DeserializeSeed<'de>>(self, seed: T) -> Result<T::Value, Error> {
        seed.deserialize(self)
    }
    fn tuple_variant<V: Visitor<'de>>(self, _: usize, visitor: V) -> Result<V::Value, Error> {
        self.deserialize_any(visitor)
    }
    fn struct_variant<V: Visitor<'de>>(
        self,
        _: &'static [&'static str],
        visitor: V,
    ) -> Result<V::Value, Error> {
        self.deserialize_any(visitor)
    }
}

impl StaticValue {
    pub fn decode<T: serde::de::DeserializeOwned>(self) -> cu29_traits::CuResult<T> {
        T::deserialize(self)
            .map_err(|e| cu29_traits::CuError::from(alloc::string::ToString::to_string(&e)))
    }
}
