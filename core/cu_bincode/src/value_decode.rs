//! Static recipes for reading native Encode bytes without the native Rust type.
//! Recipes allocate nothing and are evaluated only by offline readers. Enable
//! `self-describing` to generate companion implementations alongside Encode, or
//! use `#[bincode(describe)]` on individual Encode types for unconditional support.

use core::any::{TypeId, type_name};
pub use cu29_value_types::{QuantityMetadata, TimeStorageUnit, ValueMetadata};

/// Describes the encoded representation, independently of reflection and memory layout.
/// A handwritten encoder can delegate to the type it actually writes:
/// ```
/// use cu_bincode::{ValueDecode, ValueDecodeSpec};
/// struct Orientation([f32; 4]);
/// impl ValueDecode for Orientation {
///     const DECODE: &'static ValueDecodeSpec = <[f32; 4] as ValueDecode>::DECODE;
/// }
/// ```
pub trait ValueDecode: 'static + Sized {
    /// Complete wire recipe for this encoding.
    const DECODE: &'static ValueDecodeSpec;

    /// Static logical metadata retained alongside the wire recipe.
    ///
    /// Metadata is borrowed from the image and never changes payload encoding.
    /// Copper owns the typed metadata vocabulary.
    const METADATA: &'static [ValueMetadata] = &[];

    /// Typed lazy reference for generated field bindings.
    #[doc(hidden)]
    const __DECODE_REF: ValueDecodeRef = ValueDecodeRef {
        describe: describe::<Self>,
    };
}

/// Lazy typed reference, allowing recursive descriptions without recursive constants.
#[derive(Clone, Copy, Debug)]
pub struct ValueDecodeRef {
    /// Obtain the original type identity and its recipe during packaging.
    pub describe: fn() -> ValueDecodeType,
}

impl ValueDecodeRef {
    /// Bind a recipe to its original Rust type, including delegated representations.
    pub const fn of<T: ValueDecode>() -> Self {
        Self {
            describe: describe::<T>,
        }
    }
}

fn describe<T: ValueDecode>() -> ValueDecodeType {
    ValueDecodeType {
        type_id: TypeId::of::<T>(),
        type_name: type_name::<T>(),
        spec: T::DECODE,
        metadata: T::METADATA,
    }
}

/// Type identity is retained even when two types share an identical recipe.
#[derive(Clone, Copy, Debug)]
pub struct ValueDecodeType {
    /// Native identity used to check reflection bindings at packaging time.
    pub type_id: TypeId,
    /// Original Rust type name.
    pub type_name: &'static str,
    /// Encoded representation.
    pub spec: &'static ValueDecodeSpec,
    /// Borrowed logical metadata emitted with this type's description.
    pub metadata: &'static [ValueMetadata],
}

pub use cu29_value_types::Scalar;

/// Identity-based selector for a reflected field, preserving the declaration index for tuples.
#[derive(Clone, Copy, Debug)]
pub enum FieldSelector {
    /// Named field identity.
    Named(&'static str),
    /// Tuple field identity. The full arity detects reflection that omits positions.
    Index {
        /// Original declaration index.
        index: usize,
        /// Number of declared fields before wire skips.
        declared_fields: usize,
    },
}

/// One field in wire order; skipped fields are absent.
#[derive(Clone, Copy, Debug)]
pub struct ValueDecodeField {
    /// Logical field selector.
    pub selector: FieldSelector,
    /// Original declaration index, including fields omitted from encoding.
    pub declaration_index: usize,
    /// Typed child recipe.
    pub value: ValueDecodeRef,
}

pub use cu29_value_types::RecordShape;

/// One encoded enum branch. Tags are declaration indices, regardless of Rust discriminants.
#[derive(Clone, Copy, Debug)]
pub struct ValueDecodeVariant {
    /// Wire tag.
    pub tag: u32,
    /// Logical variant name.
    pub name: &'static str,
    /// Shape of the variant's fields.
    pub shape: RecordShape,
    /// Fields in wire order.
    pub fields: &'static [ValueDecodeField],
}

/// Wire operations supported by the first self-description API.
#[derive(Clone, Copy, Debug)]
pub enum ValueDecodeSpec {
    /// No bytes.
    Unit,
    /// One native scalar.
    Scalar(Scalar),
    /// UTF-8 bytes prefixed by a codec-encoded u64 length.
    String,
    /// Raw bytes prefixed by a codec-encoded u64 length.
    Bytes,
    /// Fields concatenated in wire order.
    Record {
        /// Logical shape.
        shape: RecordShape,
        /// Encoded fields in wire order.
        fields: &'static [ValueDecodeField],
    },
    /// Fixed repeat, without a count prefix.
    Array {
        /// Child representation.
        element: ValueDecodeRef,
        /// Fixed count.
        len: usize,
    },
    /// Count-prefixed repeat; capacity is checked by the offline decoder.
    Sequence {
        /// Child representation.
        element: ValueDecodeRef,
        /// Count encoding.
        count: Scalar,
        /// Native capacity, when bounded.
        capacity: Option<usize>,
    },
    /// Count-prefixed key/value pairs.
    Map {
        /// Key representation.
        key: ValueDecodeRef,
        /// Value representation.
        value: ValueDecodeRef,
    },
    /// One-byte presence tag followed by an optional child.
    Option(ValueDecodeRef),
    /// Transparent indirection retaining the child's original type binding.
    Delegate(ValueDecodeRef),
    /// Integer tag followed by the selected branch.
    Enum {
        /// Tag encoding.
        tag: Scalar,
        /// Supported branches.
        variants: &'static [ValueDecodeVariant],
    },
}

macro_rules! scalars {
    ($($ty:ty => $scalar:ident),* $(,)?) => {$(
        impl ValueDecode for $ty { const DECODE: &'static ValueDecodeSpec = &ValueDecodeSpec::Scalar(Scalar::$scalar); }
    )*};
}
scalars!(bool=>Bool, u8=>U8, u16=>U16, u32=>U32, u64=>U64, u128=>U128,
    i8=>I8, i16=>I16, i32=>I32, i64=>I64, i128=>I128, f32=>F32, f64=>F64, char=>Char,
    usize=>U64, isize=>I64);
impl ValueDecode for () {
    const DECODE: &'static ValueDecodeSpec = &ValueDecodeSpec::Unit;
}
impl<T: 'static> ValueDecode for core::marker::PhantomData<T> {
    const DECODE: &'static ValueDecodeSpec = &ValueDecodeSpec::Unit;
}
impl<T: ValueDecode, const N: usize> ValueDecode for [T; N] {
    const DECODE: &'static ValueDecodeSpec = &ValueDecodeSpec::Array {
        element: ValueDecodeRef::of::<T>(),
        len: N,
    };
}
impl<T: ValueDecode> ValueDecode for Option<T> {
    const DECODE: &'static ValueDecodeSpec = &ValueDecodeSpec::Option(ValueDecodeRef::of::<T>());
}
impl<T: ValueDecode> ValueDecode for &'static T {
    const DECODE: &'static ValueDecodeSpec = &ValueDecodeSpec::Delegate(ValueDecodeRef::of::<T>());
}
impl ValueDecode for &'static str {
    const DECODE: &'static ValueDecodeSpec = &ValueDecodeSpec::String;
}
impl<T: ValueDecode> ValueDecode for &'static [T] {
    const DECODE: &'static ValueDecodeSpec = &ValueDecodeSpec::Sequence {
        element: ValueDecodeRef::of::<T>(),
        count: Scalar::U64,
        capacity: None,
    };
}

#[cfg(feature = "alloc")]
mod allocated {
    use super::*;
    use alloc::{
        boxed::Box,
        collections::{BTreeMap, BTreeSet, VecDeque},
        rc::Rc,
        string::String,
        sync::Arc,
        vec::Vec,
    };
    impl ValueDecode for String {
        const DECODE: &'static ValueDecodeSpec = &ValueDecodeSpec::String;
    }
    macro_rules! sequence { ($($container:ident),*) => {$(
        impl<T: ValueDecode> ValueDecode for $container<T> {
            const DECODE: &'static ValueDecodeSpec = &ValueDecodeSpec::Sequence { element: ValueDecodeRef::of::<T>(), count: Scalar::U64, capacity: None };
        }
    )*}; }
    sequence!(Vec, VecDeque, BTreeSet);
    impl<K: ValueDecode, V: ValueDecode> ValueDecode for BTreeMap<K, V> {
        const DECODE: &'static ValueDecodeSpec = &ValueDecodeSpec::Map {
            key: ValueDecodeRef::of::<K>(),
            value: ValueDecodeRef::of::<V>(),
        };
    }
    macro_rules! delegate { ($($container:ident),*) => {$(
        impl<T: ValueDecode> ValueDecode for $container<T> { const DECODE: &'static ValueDecodeSpec = &ValueDecodeSpec::Delegate(ValueDecodeRef::of::<T>()); }
    )*}; }
    delegate!(Box, Rc, Arc);
}

macro_rules! tuple_len { ($($ty:ident),+) => { [$(stringify!($ty)),+].len() }; }

macro_rules! tuples {
    ($($ty:ident:$index:tt),+) => {
        impl<$($ty: ValueDecode),+> ValueDecode for ($($ty,)+) {
            const DECODE: &'static ValueDecodeSpec = &ValueDecodeSpec::Record { shape: RecordShape::Tuple, fields: { const LEN: usize = tuple_len!($($ty),+); &[$(
                ValueDecodeField { selector: FieldSelector::Index { index: $index, declared_fields: LEN }, declaration_index: $index, value: ValueDecodeRef::of::<$ty>() }
            ),+] } };
        }
    };
}
tuples!(A:0);
tuples!(A:0,B:1);
tuples!(A:0,B:1,C:2);
tuples!(A:0,B:1,C:2,D:3);
tuples!(A:0,B:1,C:2,D:3,E:4);
tuples!(A:0,B:1,C:2,D:3,E:4,F:5);
tuples!(A:0,B:1,C:2,D:3,E:4,F:5,G:6);
tuples!(A:0,B:1,C:2,D:3,E:4,F:5,G:6,H:7);
tuples!(A:0,B:1,C:2,D:3,E:4,F:5,G:6,H:7,I:8);
tuples!(A:0,B:1,C:2,D:3,E:4,F:5,G:6,H:7,I:8,J:9);
tuples!(A:0,B:1,C:2,D:3,E:4,F:5,G:6,H:7,I:8,J:9,K:10);
tuples!(A:0,B:1,C:2,D:3,E:4,F:5,G:6,H:7,I:8,J:9,K:10,L:11);

#[cfg(feature = "std")]
mod standard {
    use super::*;
    use std::collections::{HashMap, HashSet};
    impl<K: ValueDecode, V: ValueDecode, S: 'static> ValueDecode for HashMap<K, V, S> {
        const DECODE: &'static ValueDecodeSpec = &ValueDecodeSpec::Map {
            key: ValueDecodeRef::of::<K>(),
            value: ValueDecodeRef::of::<V>(),
        };
    }
    impl<T: ValueDecode, S: 'static> ValueDecode for HashSet<T, S> {
        const DECODE: &'static ValueDecodeSpec = &ValueDecodeSpec::Sequence {
            element: ValueDecodeRef::of::<T>(),
            count: Scalar::U64,
            capacity: None,
        };
    }
}

// The codec owns serialization for the shared description vocabulary.
macro_rules! encode_description_kind {
    ($ty:ident, {$($variant:ident = $id:literal,)+}) => {
        impl crate::Encode for $ty {
            fn encode<E: crate::enc::Encoder>(&self, encoder: &mut E) -> Result<(), crate::error::EncodeError> {
                (*self as u32).encode(encoder)
            }
        }
        impl<Context> crate::Decode<Context> for $ty {
            fn decode<D: crate::de::Decoder<Context = Context>>(decoder: &mut D) -> Result<Self, crate::error::DecodeError> {
                match <u32 as crate::Decode<Context>>::decode(decoder)? {
                    $($id => Ok(Self::$variant),)+
                    found => Err(crate::error::DecodeError::UnexpectedVariant {
                        type_name: core::any::type_name::<Self>(),
                        allowed: &crate::error::AllowedEnumVariants::Allowed(&[$($id,)+]),
                        found,
                    }),
                }
            }
        }
        crate::impl_borrow_decode!($ty);
        impl ValueDecode for $ty {
            const DECODE: &'static ValueDecodeSpec = &ValueDecodeSpec::Enum {
                tag: Scalar::U32,
                variants: &[$(ValueDecodeVariant {
                    tag: $id,
                    name: stringify!($variant),
                    shape: RecordShape::Unit,
                    fields: &[],
                },)+],
            };
        }
    };
}
encode_description_kind!(Scalar, {
    Bool = 0,
    U8 = 1,
    U16 = 2,
    U32 = 3,
    U64 = 4,
    U128 = 5,
    I8 = 6,
    I16 = 7,
    I32 = 8,
    I64 = 9,
    I128 = 10,
    F32 = 11,
    F64 = 12,
    Char = 13,
});
encode_description_kind!(RecordShape, {
    Unit = 0,
    Tuple = 1,
    Newtype = 2,
    Struct = 3,
});
