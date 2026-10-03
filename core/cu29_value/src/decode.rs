//! Offline decoding of native bincode bytes from a portable description.
//!
//! Build a description once with [`ValueDecodeDescription::from_type`], then serialize
//! it alongside the data. [`ValueDecodeDescription::decode`] needs only that description
//! and the producer's bincode configuration; it never reconstructs the native payload.

use crate::Value;
use alloc::boxed::Box;
use alloc::collections::BTreeMap;
use alloc::format;
use alloc::string::String;
use alloc::string::ToString;
use alloc::vec::Vec;
use bevy_reflect::GetTypeRegistration;
use bevy_reflect::TypeInfo;
use bevy_reflect::TypeRegistry;
use bevy_reflect::enums::VariantInfo;
use bincode::Decode;
use bincode::Encode;
use bincode::ValueDecode;
use bincode::ValueDecodeSpec;
use bincode::config::Config;
use bincode::de::Decoder;
use bincode::de::read::Reader;
use bincode::error::DecodeError;
use bincode::value_decode::FieldSelector;
use bincode::value_decode::RecordShape;
use bincode::value_decode::Scalar;
use bincode::value_decode::ValueDecodeField;
use bincode::value_decode::ValueDecodeRef;
use core::any::TypeId;
use cu29_clock::CuDuration;
use cu29_clock::CuTime;
use serde::{Deserialize, Serialize};

/// A portable description of native encoded bytes. This API is experimental.
///
/// Wire operations are shared independently of schemas, preserving the identity
/// and coherent storage unit of quantities that share a scalar representation.
/// Versioned transport is supplied by `ValueDecodeCatalog` when the
/// `decode-catalog` feature is enabled.
#[derive(Clone, Debug, Encode, Decode, Serialize, Deserialize)]
pub struct ValueDecodeDescription {
    /// Binding for the described payload.
    pub root: usize,
    /// Typed associations between wire operations and schemas.
    pub bindings: Vec<ValueDecodeBinding>,
    /// Deduplicated wire operations.
    pub operations: Vec<ValueDecodeOp>,
    /// Original type and field information.
    pub schemas: Vec<ValueDecodeSchema>,
}

/// A wire operation bound to its original logical schema.
#[derive(Clone, Debug, Encode, Decode, Serialize, Deserialize)]
pub struct ValueDecodeBinding {
    /// Index into the operation table.
    pub operation: usize,
    /// Index into the schema table.
    pub schema: usize,
}

/// Original type identity and names, retained separately from wire operations.
#[derive(Clone, Debug, Encode, Decode, Serialize, Deserialize)]
pub struct ValueDecodeSchema {
    /// Reflected type path, or the native name for a standard wire type.
    pub type_path: String,
    /// Quantity identity and coherent storage unit, when registered.
    pub quantity: Option<ValueDecodeQuantity>,
    /// Encoded fields in wire order.
    pub fields: Vec<ValueDecodeSchemaField>,
    /// All supported enum branches.
    pub variants: Vec<ValueDecodeSchemaVariant>,
}

/// Canonical storage metadata; independent of debugger display preferences.
#[derive(Clone, Debug, PartialEq, Eq, Encode, Decode, Serialize, Deserialize)]
pub struct ValueDecodeQuantity {
    /// Quantity identity such as `length` or `mass`.
    pub quantity: String,
    /// Storage unit such as `m`, `kg`, or `ns` for Copper clock values.
    pub storage_unit: String,
}

/// A logical field bound by name or original declaration index.
#[derive(Clone, Debug, Encode, Decode, Serialize, Deserialize)]
pub struct ValueDecodeSchemaField {
    /// Named fields use their reflected name; tuple fields have no name.
    pub name: Option<String>,
    /// Original declaration index, including fields omitted from encoding.
    pub index: usize,
    /// Child schema index.
    pub schema: usize,
}

/// A logical enum branch matched to its encoded tag.
#[derive(Clone, Debug, Encode, Decode, Serialize, Deserialize)]
pub struct ValueDecodeSchemaVariant {
    /// Original variant name.
    pub name: String,
    /// Encoded fields in wire order.
    pub fields: Vec<ValueDecodeSchemaField>,
}

/// Portable native scalar encoding. Widths are preserved in the value tree.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Encode, Decode, Serialize, Deserialize)]
pub enum ValueDecodeScalar {
    /// Native `bool` encoding.
    Bool,
    /// Native `u8` encoding.
    U8,
    /// Native `u16` encoding.
    U16,
    /// Native `u32` encoding.
    U32,
    /// Native `u64` encoding.
    U64,
    /// Native `u128` encoding.
    U128,
    /// Native `i8` encoding.
    I8,
    /// Native `i16` encoding.
    I16,
    /// Native `i32` encoding.
    I32,
    /// Native `i64` encoding.
    I64,
    /// Native `i128` encoding.
    I128,
    /// Native `f32` encoding.
    F32,
    /// Native `f64` encoding.
    F64,
    /// Native `char` encoding.
    Char,
}

/// Aggregate shape, independent of the native Rust memory layout.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Encode, Decode, Serialize, Deserialize)]
pub enum ValueDecodeShape {
    /// A unit record.
    Unit,
    /// An ordered tuple record.
    Tuple,
    /// A one-field tuple struct or variant.
    Newtype,
    /// A record with named fields.
    Struct,
}

/// One portable branch of a tagged encoding.
#[derive(Clone, Debug, PartialEq, Eq, Encode, Decode, Serialize, Deserialize)]
pub struct ValueDecodeBranch {
    /// Wire tag.
    pub tag: u32,
    /// Field representation.
    pub shape: ValueDecodeShape,
    /// Child binding indices in wire order.
    pub fields: Vec<usize>,
}

/// Portable wire operations. Codec integer settings and endianness are supplied
/// to [`ValueDecodeDescription::decode`] exactly as used by the producer.
#[derive(Clone, Debug, PartialEq, Eq, Encode, Decode, Serialize, Deserialize)]
pub enum ValueDecodeOp {
    /// No bytes.
    Unit,
    /// One scalar using the producer codec settings.
    Scalar(ValueDecodeScalar),
    /// Count-prefixed UTF-8 bytes.
    String,
    /// Count-prefixed raw bytes.
    Bytes,
    /// Fields concatenated in wire order.
    Record {
        /// Logical representation.
        shape: ValueDecodeShape,
        /// Child bindings in wire order.
        fields: Vec<usize>,
    },
    /// A fixed repeat without a count prefix.
    Array {
        /// Child binding.
        element: usize,
        /// Fixed element count.
        len: usize,
    },
    /// A count-prefixed repeat.
    Sequence {
        /// Child binding.
        element: usize,
        /// Count encoding.
        count: ValueDecodeScalar,
        /// Maximum native capacity, if bounded.
        capacity: Option<usize>,
    },
    /// Count-prefixed key/value pairs.
    Map {
        /// Key binding.
        key: usize,
        /// Value binding.
        value: usize,
    },
    /// A one-byte presence tag and optional child binding.
    Option(usize),
    /// A transparent child binding.
    Delegate(usize),
    /// An integer tag selecting one branch.
    Enum {
        /// Tag encoding.
        tag: ValueDecodeScalar,
        /// Supported branches in schema order.
        branches: Vec<ValueDecodeBranch>,
    },
}

/// Limits on offline execution and value-tree allocation.
#[derive(Clone, Copy, Debug)]
pub struct ValueDecodeLimits {
    /// Maximum nesting, counting the root as one level.
    pub max_depth: usize,
    /// Total values, including map keys and container nodes.
    pub max_values: usize,
    /// Maximum items in a container, or bytes in a string/byte buffer.
    pub max_collection_len: usize,
}

impl Default for ValueDecodeLimits {
    fn default() -> Self {
        Self {
            max_depth: 128,
            max_values: 1_000_000,
            max_collection_len: 1_000_000,
        }
    }
}

/// Internal allocation budget shared across payload slots in an offline CopperList.
#[doc(hidden)]
pub struct ValueDecodeBudget {
    values: usize,
    bytes: usize,
}
impl ValueDecodeBudget {
    pub fn new(limits: ValueDecodeLimits) -> Self {
        Self {
            values: limits.max_values,
            bytes: 16 * 1024 * 1024,
        }
    }
    fn spend_values(&mut self, count: usize) -> Result<(), DecodeError> {
        spend(&mut self.values, count)?;
        let bytes = count
            .checked_mul(2 * core::mem::size_of::<Value>())
            .ok_or(DecodeError::Other("ValueDecode output size overflow"))?;
        self.spend_bytes(bytes)
    }
    fn spend_bytes(&mut self, count: usize) -> Result<(), DecodeError> {
        self.bytes = self
            .bytes
            .checked_sub(count)
            .ok_or(DecodeError::Other("ValueDecode output byte limit exceeded"))?;
        Ok(())
    }
}

impl ValueDecodeDescription {
    /// Validate all graph references and aggregate shapes, including unused branches.
    /// Empty graphs represent catalogs containing only uncaptured slots.
    pub fn validate(&self) -> Result<(), DecodeError> {
        if self.bindings.is_empty() {
            return if self.root == 0 && self.operations.is_empty() && self.schemas.is_empty() {
                Ok(())
            } else {
                Err(DecodeError::Other("invalid empty ValueDecode graph"))
            };
        }
        let binding = |id: usize| {
            self.bindings
                .get(id)
                .ok_or(DecodeError::Other("invalid ValueDecode binding reference"))
        };
        binding(self.root)?;
        let count = |scalar| match scalar {
            ValueDecodeScalar::U8
            | ValueDecodeScalar::U16
            | ValueDecodeScalar::U32
            | ValueDecodeScalar::U64 => Ok(()),
            _ => Err(DecodeError::Other(
                "ValueDecode count/tag must be an unsigned integer",
            )),
        };
        let record = |shape, fields: &[usize], schema: &[ValueDecodeSchemaField]| {
            if fields.len() != schema.len()
                || (shape == ValueDecodeShape::Unit && !fields.is_empty())
                || (shape == ValueDecodeShape::Newtype && fields.len() != 1)
            {
                return Err(DecodeError::Other(
                    "ValueDecode record/schema shape mismatch",
                ));
            }
            let mut names = alloc::collections::BTreeSet::new();
            for (id, field) in fields.iter().zip(schema) {
                if binding(*id)?.schema != field.schema {
                    return Err(DecodeError::Other("ValueDecode child/schema mismatch"));
                }
                if shape == ValueDecodeShape::Struct {
                    let name = field
                        .name
                        .as_ref()
                        .ok_or(DecodeError::Other("missing ValueDecode field name"))?;
                    if !names.insert(name) {
                        return Err(DecodeError::Other("duplicate ValueDecode field name"));
                    }
                }
            }
            Ok(())
        };
        for schema in &self.schemas {
            for field in schema
                .fields
                .iter()
                .chain(schema.variants.iter().flat_map(|variant| &variant.fields))
            {
                if field.schema >= self.schemas.len() {
                    return Err(DecodeError::Other("invalid ValueDecode schema reference"));
                }
            }
        }
        for operation in &self.operations {
            match operation {
                ValueDecodeOp::Record { shape, fields } => {
                    check_shape(*shape, fields.len())?;
                    for id in fields {
                        binding(*id)?;
                    }
                }
                ValueDecodeOp::Array { element, .. }
                | ValueDecodeOp::Option(element)
                | ValueDecodeOp::Delegate(element) => {
                    binding(*element)?;
                }
                ValueDecodeOp::Sequence {
                    element,
                    count: scalar,
                    ..
                } => {
                    binding(*element)?;
                    count(*scalar)?;
                }
                ValueDecodeOp::Map { key, value } => {
                    binding(*key)?;
                    binding(*value)?;
                }
                ValueDecodeOp::Enum { tag, branches } => {
                    count(*tag)?;
                    let mut tags = alloc::collections::BTreeSet::new();
                    for branch in branches {
                        check_shape(branch.shape, branch.fields.len())?;
                        let maximum = match tag {
                            ValueDecodeScalar::U8 => u32::from(u8::MAX),
                            ValueDecodeScalar::U16 => u32::from(u16::MAX),
                            _ => u32::MAX,
                        };
                        if branch.tag > maximum {
                            return Err(DecodeError::Other(
                                "ValueDecode enum tag exceeds its wire width",
                            ));
                        }
                        if !tags.insert(branch.tag) {
                            return Err(DecodeError::Other("duplicate ValueDecode enum tag"));
                        }
                        for id in &branch.fields {
                            binding(*id)?;
                        }
                    }
                }
                _ => {}
            }
        }
        for bound in &self.bindings {
            let operation = self
                .operations
                .get(bound.operation)
                .ok_or(DecodeError::Other(
                    "invalid ValueDecode operation reference",
                ))?;
            let schema = self
                .schemas
                .get(bound.schema)
                .ok_or(DecodeError::Other("invalid ValueDecode schema reference"))?;
            match operation {
                ValueDecodeOp::Record { shape, fields } => record(*shape, fields, &schema.fields)?,
                ValueDecodeOp::Enum { branches, .. } => {
                    if branches.len() != schema.variants.len() {
                        return Err(DecodeError::Other("ValueDecode enum/schema mismatch"));
                    }
                    let mut names = alloc::collections::BTreeSet::new();
                    for (branch, variant) in branches.iter().zip(&schema.variants) {
                        if !names.insert(&variant.name) {
                            return Err(DecodeError::Other("duplicate ValueDecode variant name"));
                        }
                        record(branch.shape, &branch.fields, &variant.fields)?;
                    }
                }
                _ => {}
            }
        }
        Ok(())
    }

    /// Bind generated wire operations to reflection and Copper standard quantities.
    /// Encoded fields hidden from reflection produce an error naming that field.
    ///
    /// ```
    /// use cu29_value::decode::{ValueDecodeDescription, ValueDecodeLimits};
    /// use bincode::{Encode, Decode};
    /// use bevy_reflect::Reflect;
    /// #[derive(Encode, Decode, Reflect)]
    /// struct Sample { ticks: u32, valid: bool }
    /// let description = ValueDecodeDescription::from_type::<Sample>().unwrap();
    /// let bytes = bincode::encode_to_vec(Sample { ticks: 42, valid: true }, bincode::config::standard()).unwrap();
    /// let (tree, consumed) = description.decode(&bytes, bincode::config::standard(), ValueDecodeLimits::default()).unwrap();
    /// assert_eq!(consumed, bytes.len());
    /// assert!(matches!(tree, cu29_value::Value::Map(_)));
    /// ```
    pub fn from_type<T: ValueDecode + GetTypeRegistration>() -> Result<Self, DecodeError> {
        let mut registry = TypeRegistry::default();
        registry.register::<T>();
        Self::from_registry::<T>(&registry)
    }

    /// Build using an existing registry. Bindings are checked by native type identity,
    /// field name and tuple declaration index, rather than reflected field position.
    pub fn from_registry<T: ValueDecode>(registry: &TypeRegistry) -> Result<Self, DecodeError> {
        let (description, _) = Self::from_registry_roots(registry, &[ValueDecodeRef::of::<T>()])?;
        Ok(description)
    }

    pub(crate) fn from_registry_roots(
        registry: &TypeRegistry,
        roots: &[ValueDecodeRef],
    ) -> Result<(Self, Vec<usize>), DecodeError> {
        let mut quantities = cu29_units::value_decode_quantities();
        for type_id in [TypeId::of::<CuTime>(), TypeId::of::<CuDuration>()] {
            quantities.push(cu29_units::ValueDecodeQuantity {
                type_id,
                quantity: "time",
                storage_unit: String::from("ns"),
            });
        }
        let mut builder = DescriptionBuilder {
            description: Self {
                root: 0,
                bindings: Vec::new(),
                operations: Vec::new(),
                schemas: Vec::new(),
            },
            types: BTreeMap::new(),
            registry,
            quantities,
        };
        let roots = roots
            .iter()
            .map(|root| builder.bind(*root, 0))
            .collect::<Result<Vec<_>, _>>()?;
        builder.description.root = roots.first().copied().unwrap_or(0);
        Ok((builder.description, roots))
    }

    /// Read one native value, returning the exact number of consumed bytes.
    /// Slice the input at that offset to read a consecutive value. Invalid operations,
    /// invalid tags, truncated bytes and exceeded limits return an error.
    pub fn decode<C: Config>(
        &self,
        bytes: &[u8],
        config: C,
        limits: ValueDecodeLimits,
    ) -> Result<(Value, usize), DecodeError> {
        self.decode_at(self.root, bytes, config, limits)
    }

    /// Decode one binding from a shared catalog without cloning its graph.
    pub fn decode_at<C: Config>(
        &self,
        binding: usize,
        bytes: &[u8],
        config: C,
        limits: ValueDecodeLimits,
    ) -> Result<(Value, usize), DecodeError> {
        let mut budget = ValueDecodeBudget::new(limits);
        self.decode_at_with_budget(binding, bytes, config, limits, &mut budget)
    }

    /// Shared allocation budget for offline CopperList readers.
    #[doc(hidden)]
    pub fn decode_at_with_budget<C: Config>(
        &self,
        binding: usize,
        bytes: &[u8],
        config: C,
        limits: ValueDecodeLimits,
        budget: &mut ValueDecodeBudget,
    ) -> Result<(Value, usize), DecodeError> {
        let reader = ValueReader { bytes, offset: 0 };
        let mut decoder = bincode::de::DecoderImpl::new(reader, config, ());
        let value = self.decode_binding(binding, &mut decoder, limits, budget, 1)?;
        Ok((value, decoder.reader().offset))
    }

    fn decode_binding<D: Decoder<Context = ()>>(
        &self,
        id: usize,
        decoder: &mut D,
        limits: ValueDecodeLimits,
        budget: &mut ValueDecodeBudget,
        depth: usize,
    ) -> Result<Value, DecodeError> {
        if depth > limits.max_depth {
            return Err(DecodeError::Other("ValueDecode nesting limit exceeded"));
        }
        budget.spend_values(1)?;
        let binding = self
            .bindings
            .get(id)
            .ok_or(DecodeError::Other("invalid ValueDecode binding index"))?;
        let schema = self
            .schemas
            .get(binding.schema)
            .ok_or(DecodeError::Other("invalid ValueDecode schema index"))?;
        let operation = self
            .operations
            .get(binding.operation)
            .ok_or(DecodeError::Other("invalid ValueDecode operation index"))?;
        let child = |id, decoder: &mut D, budget: &mut ValueDecodeBudget| {
            self.decode_binding(id, decoder, limits, budget, depth + 1)
        };
        Ok(match operation {
            ValueDecodeOp::Unit => Value::Unit,
            ValueDecodeOp::Scalar(scalar) => decode_scalar(*scalar, decoder)?,
            ValueDecodeOp::String | ValueDecodeOp::Bytes => {
                let len = checked_len(u64::decode(decoder)?, limits.max_collection_len)?;
                budget.spend_bytes(len)?;
                decoder.claim_bytes_read(len)?;
                // Read before constructing the tree; truncation is reported by the bounded reader.
                let mut bytes = alloc::vec![0; len];
                decoder.reader().read(&mut bytes)?;
                if matches!(operation, ValueDecodeOp::String) {
                    Value::String(String::from_utf8(bytes).map_err(|error| DecodeError::Utf8 {
                        inner: error.utf8_error(),
                    })?)
                } else {
                    Value::Bytes(bytes)
                }
            }
            ValueDecodeOp::Record { shape, fields } => self.decode_record(
                ValueDecodeRecord {
                    shape: *shape,
                    fields,
                    schema: &schema.fields,
                },
                decoder,
                limits,
                budget,
                depth,
            )?,
            ValueDecodeOp::Array { element, len } => {
                check_count(*len, limits, budget.values)?;
                let mut values = Vec::new();
                for _ in 0..*len {
                    values.push(child(*element, decoder, budget)?);
                }
                Value::Seq(values)
            }
            ValueDecodeOp::Sequence {
                element,
                count,
                capacity,
            } => {
                let len = decode_count(*count, decoder)?;
                let len = checked_len(
                    len,
                    capacity
                        .unwrap_or(usize::MAX)
                        .min(limits.max_collection_len),
                )?;
                check_count(len, limits, budget.values)?;
                let mut values = Vec::new();
                for _ in 0..len {
                    values.push(child(*element, decoder, budget)?);
                }
                Value::Seq(values)
            }
            ValueDecodeOp::Map { key, value } => {
                let len = checked_len(u64::decode(decoder)?, limits.max_collection_len)?;
                check_count(
                    len.checked_mul(2)
                        .ok_or(DecodeError::Other("ValueDecode map size overflow"))?,
                    ValueDecodeLimits {
                        max_collection_len: usize::MAX,
                        ..limits
                    },
                    budget.values,
                )?;
                let mut values = BTreeMap::new();
                for _ in 0..len {
                    values.insert(
                        child(*key, decoder, budget)?,
                        child(*value, decoder, budget)?,
                    );
                }
                Value::Map(values)
            }
            ValueDecodeOp::Delegate(element) => child(*element, decoder, budget)?,
            ValueDecodeOp::Option(element) => match u8::decode(decoder)? {
                0 => Value::Option(None),
                1 => Value::Option(Some(Box::new(child(*element, decoder, budget)?))),
                _ => return Err(DecodeError::Other("invalid ValueDecode option tag")),
            },
            ValueDecodeOp::Enum { tag, branches } => {
                let tag = decode_count(*tag, decoder)?;
                let (index, branch) = branches
                    .iter()
                    .enumerate()
                    .find(|(_, branch)| u64::from(branch.tag) == tag)
                    .ok_or(DecodeError::Other("invalid ValueDecode enum tag"))?;
                let variant = schema
                    .variants
                    .get(index)
                    .ok_or(DecodeError::Other("missing ValueDecode variant schema"))?;
                budget.spend_bytes(variant.name.len())?;
                // Serde-compatible externally tagged enum representation.
                if branch.shape == ValueDecodeShape::Unit {
                    if !branch.fields.is_empty() {
                        return Err(DecodeError::Other("unit variant has encoded fields"));
                    }
                    Value::String(variant.name.clone())
                } else {
                    budget.spend_values(1)?;
                    if branch.shape != ValueDecodeShape::Newtype {
                        budget.spend_values(1)?;
                    }
                    let value = self.decode_record(
                        ValueDecodeRecord {
                            shape: branch.shape,
                            fields: &branch.fields,
                            schema: &variant.fields,
                        },
                        decoder,
                        limits,
                        budget,
                        depth,
                    )?;
                    let value = match value {
                        Value::Newtype(value) if branch.shape == ValueDecodeShape::Newtype => {
                            *value
                        }
                        value => value,
                    };
                    Value::Map(BTreeMap::from([(
                        Value::String(variant.name.clone()),
                        value,
                    )]))
                }
            }
        })
    }

    fn decode_record<D: Decoder<Context = ()>>(
        &self,
        record: ValueDecodeRecord<'_>,
        decoder: &mut D,
        limits: ValueDecodeLimits,
        budget: &mut ValueDecodeBudget,
        depth: usize,
    ) -> Result<Value, DecodeError> {
        let ValueDecodeRecord {
            shape,
            fields,
            schema,
        } = record;
        if fields.len() != schema.len() {
            return Err(DecodeError::Other(
                "ValueDecode record/schema field mismatch",
            ));
        }
        check_count(fields.len(), limits, budget.values)?;
        match shape {
            ValueDecodeShape::Unit if fields.is_empty() => Ok(Value::Unit),
            ValueDecodeShape::Unit => Err(DecodeError::Other("unit record has encoded fields")),
            ValueDecodeShape::Newtype if fields.len() == 1 => Ok(Value::Newtype(Box::new(
                self.decode_binding(fields[0], decoder, limits, budget, depth + 1)?,
            ))),
            ValueDecodeShape::Newtype => Err(DecodeError::Other(
                "newtype record must have one encoded field",
            )),
            ValueDecodeShape::Tuple => {
                let mut values = Vec::new();
                for id in fields {
                    values.push(self.decode_binding(*id, decoder, limits, budget, depth + 1)?);
                }
                Ok(Value::Seq(values))
            }
            ValueDecodeShape::Struct => {
                let mut values = BTreeMap::new();
                for (id, field) in fields.iter().zip(schema) {
                    budget.spend_values(1)?;
                    let name = field
                        .name
                        .as_ref()
                        .ok_or(DecodeError::Other("missing ValueDecode field name"))?;
                    budget.spend_bytes(name.len())?;
                    let value = self.decode_binding(*id, decoder, limits, budget, depth + 1)?;
                    if values.insert(Value::String(name.clone()), value).is_some() {
                        return Err(DecodeError::Other("duplicate ValueDecode field name"));
                    }
                }
                Ok(Value::Map(values))
            }
        }
    }
}

fn check_shape(shape: ValueDecodeShape, count: usize) -> Result<(), DecodeError> {
    if (shape == ValueDecodeShape::Unit && count != 0)
        || (shape == ValueDecodeShape::Newtype && count != 1)
    {
        Err(DecodeError::Other("invalid ValueDecode record shape"))
    } else {
        Ok(())
    }
}

fn spend(remaining: &mut usize, amount: usize) -> Result<(), DecodeError> {
    *remaining = remaining
        .checked_sub(amount)
        .ok_or(DecodeError::Other("ValueDecode value limit exceeded"))?;
    Ok(())
}
fn check_count(
    count: usize,
    limits: ValueDecodeLimits,
    remaining: usize,
) -> Result<(), DecodeError> {
    if count > limits.max_collection_len || count > remaining {
        return Err(DecodeError::Other(
            "ValueDecode collection/value limit exceeded",
        ));
    }
    Ok(())
}
fn checked_len(len: u64, maximum: usize) -> Result<usize, DecodeError> {
    let len =
        usize::try_from(len).map_err(|_| DecodeError::Other("ValueDecode length overflow"))?;
    if len > maximum {
        return Err(DecodeError::Other("ValueDecode collection limit exceeded"));
    }
    Ok(len)
}
fn decode_count<D: Decoder<Context = ()>>(
    scalar: ValueDecodeScalar,
    decoder: &mut D,
) -> Result<u64, DecodeError> {
    match scalar {
        ValueDecodeScalar::U8 => Ok(u64::from(u8::decode(decoder)?)),
        ValueDecodeScalar::U16 => Ok(u64::from(u16::decode(decoder)?)),
        ValueDecodeScalar::U32 => Ok(u64::from(u32::decode(decoder)?)),
        ValueDecodeScalar::U64 => u64::decode(decoder),
        _ => Err(DecodeError::Other(
            "ValueDecode count/tag must be an unsigned integer",
        )),
    }
}
fn decode_scalar<D: Decoder<Context = ()>>(
    scalar: ValueDecodeScalar,
    decoder: &mut D,
) -> Result<Value, DecodeError> {
    macro_rules! decode { ($($variant:ident => $ty:ty),*) => { match scalar { $(ValueDecodeScalar::$variant => Ok(Value::$variant(<$ty>::decode(decoder)?))),* } }; }
    decode!(Bool=>bool,U8=>u8,U16=>u16,U32=>u32,U64=>u64,U128=>u128,I8=>i8,I16=>i16,I32=>i32,I64=>i64,I128=>i128,F32=>f32,F64=>f64,Char=>char)
}

struct ValueDecodeRecord<'a> {
    shape: ValueDecodeShape,
    fields: &'a [usize],
    schema: &'a [ValueDecodeSchemaField],
}

struct ValueReader<'a> {
    bytes: &'a [u8],
    offset: usize,
}
impl Reader for ValueReader<'_> {
    fn read(&mut self, bytes: &mut [u8]) -> Result<(), DecodeError> {
        let end = self
            .offset
            .checked_add(bytes.len())
            .ok_or(DecodeError::Other("ValueDecode cursor overflow"))?;
        let input = self
            .bytes
            .get(self.offset..end)
            .ok_or(DecodeError::UnexpectedEnd {
                additional: end.saturating_sub(self.bytes.len()),
            })?;
        bytes.copy_from_slice(input);
        self.offset = end;
        Ok(())
    }
}

struct DescriptionBuilder<'a> {
    description: ValueDecodeDescription,
    types: BTreeMap<TypeId, usize>,
    registry: &'a TypeRegistry,
    quantities: Vec<cu29_units::ValueDecodeQuantity>,
}

impl DescriptionBuilder<'_> {
    fn bind(&mut self, reference: ValueDecodeRef, depth: usize) -> Result<usize, DecodeError> {
        let ty = (reference.describe)();
        if let Some(id) = self.types.get(&ty.type_id) {
            return Ok(*id);
        }
        if depth >= 128 {
            return Err(DecodeError::Other(
                "ValueDecode description nesting limit exceeded",
            ));
        }
        let id = self.description.bindings.len();
        self.types.insert(ty.type_id, id);
        self.description.bindings.push(ValueDecodeBinding {
            operation: 0,
            schema: id,
        });
        let info = self.registry.get_type_info(ty.type_id);
        let quantity = self
            .quantities
            .iter()
            .find(|quantity| quantity.type_id == ty.type_id)
            .map(|quantity| ValueDecodeQuantity {
                quantity: quantity.quantity.to_string(),
                storage_unit: quantity.storage_unit.clone(),
            });
        self.description.schemas.push(ValueDecodeSchema {
            type_path: info.map_or(ty.type_name, TypeInfo::type_path).to_string(),
            quantity,
            fields: Vec::new(),
            variants: Vec::new(),
        });
        let operation = match ty.spec {
            ValueDecodeSpec::Unit => ValueDecodeOp::Unit,
            ValueDecodeSpec::Scalar(scalar) => ValueDecodeOp::Scalar(scalar_kind(*scalar)),
            ValueDecodeSpec::String => ValueDecodeOp::String,
            ValueDecodeSpec::Bytes => ValueDecodeOp::Bytes,
            ValueDecodeSpec::Record { shape, fields } => {
                let (children, fields) =
                    self.bind_fields(fields, info, None, ty.type_name, depth)?;
                self.description.schemas[id].fields = fields;
                ValueDecodeOp::Record {
                    shape: record_shape(*shape),
                    fields: children,
                }
            }
            ValueDecodeSpec::Array { element, len } => ValueDecodeOp::Array {
                element: self.bind(*element, depth + 1)?,
                len: *len,
            },
            ValueDecodeSpec::Sequence {
                element,
                count,
                capacity,
            } => ValueDecodeOp::Sequence {
                element: self.bind(*element, depth + 1)?,
                count: scalar_kind(*count),
                capacity: *capacity,
            },
            ValueDecodeSpec::Map { key, value } => ValueDecodeOp::Map {
                key: self.bind(*key, depth + 1)?,
                value: self.bind(*value, depth + 1)?,
            },
            ValueDecodeSpec::Delegate(element) => {
                ValueDecodeOp::Delegate(self.bind(*element, depth + 1)?)
            }
            ValueDecodeSpec::Option(element) => {
                ValueDecodeOp::Option(self.bind(*element, depth + 1)?)
            }
            ValueDecodeSpec::Enum { tag, variants } => {
                let mut branches = Vec::new();
                for variant in *variants {
                    let reflected = match info {
                        Some(TypeInfo::Enum(info)) => info.variant(variant.name),
                        _ => None,
                    };
                    if reflected.is_none() && !matches!(info, Some(TypeInfo::Opaque(_))) {
                        return Err(DecodeError::OtherString(format!(
                            "ValueDecode: {0} variant {1} is missing from reflection",
                            ty.type_name, variant.name
                        )));
                    }
                    let (children, fields) =
                        self.bind_fields(variant.fields, info, reflected, ty.type_name, depth)?;
                    self.description.schemas[id]
                        .variants
                        .push(ValueDecodeSchemaVariant {
                            name: variant.name.to_string(),
                            fields,
                        });
                    branches.push(ValueDecodeBranch {
                        tag: variant.tag,
                        shape: record_shape(variant.shape),
                        fields: children,
                    });
                }
                ValueDecodeOp::Enum {
                    tag: scalar_kind(*tag),
                    branches,
                }
            }
        };
        let operation_id = self
            .description
            .operations
            .iter()
            .position(|candidate| candidate == &operation)
            .unwrap_or_else(|| {
                let id = self.description.operations.len();
                self.description.operations.push(operation);
                id
            });
        self.description.bindings[id].operation = operation_id;
        Ok(id)
    }

    fn bind_fields(
        &mut self,
        fields: &[ValueDecodeField],
        info: Option<&TypeInfo>,
        variant: Option<&VariantInfo>,
        parent: &str,
        depth: usize,
    ) -> Result<(Vec<usize>, Vec<ValueDecodeSchemaField>), DecodeError> {
        let mut children = Vec::new();
        let mut schemas = Vec::new();
        // Opaque reflection intentionally has no field/variant view. Its static
        // encoding recipe supplies the declaration names, positions and types.
        let opaque = matches!(info, Some(TypeInfo::Opaque(_)));
        for field in fields {
            let ty = (field.value.describe)();
            let (name, reflected) = match field.selector {
                FieldSelector::Named(name) => {
                    let reflected = match variant {
                        Some(VariantInfo::Struct(info)) => {
                            info.field(name).map(|field| field.type_id())
                        }
                        Some(_) => None,
                        None => match info {
                            Some(TypeInfo::Struct(info)) => {
                                info.field(name).map(|field| field.type_id())
                            }
                            _ => None,
                        },
                    };
                    (Some(name.to_string()), reflected)
                }
                FieldSelector::Index {
                    index,
                    declared_fields,
                } => {
                    let reflected_len = match variant {
                        Some(VariantInfo::Tuple(info)) => Some(info.field_len()),
                        Some(_) => None,
                        None => match info {
                            Some(TypeInfo::TupleStruct(info)) => Some(info.field_len()),
                            Some(TypeInfo::Tuple(info)) => Some(info.field_len()),
                            _ => None,
                        },
                    };
                    if !opaque && reflected_len != Some(declared_fields) {
                        return Err(DecodeError::OtherString(format!(
                            "ValueDecode: {parent} tuple field {index} cannot be bound because reflection omits declaration positions"
                        )));
                    }
                    // Tuple positions are safe only after verifying the full declaration arity.
                    let reflected = match variant {
                        Some(VariantInfo::Tuple(info)) => info
                            .iter()
                            .find(|field| field.index() == index)
                            .map(|field| field.type_id()),
                        Some(_) => None,
                        None => match info {
                            Some(TypeInfo::TupleStruct(info)) => info
                                .iter()
                                .find(|field| field.index() == index)
                                .map(|field| field.type_id()),
                            Some(TypeInfo::Tuple(info)) => info
                                .iter()
                                .find(|field| field.index() == index)
                                .map(|field| field.type_id()),
                            _ => None,
                        },
                    };
                    (None, reflected)
                }
            };
            if !opaque && reflected != Some(ty.type_id) {
                return Err(DecodeError::OtherString(format!(
                    "ValueDecode: {parent} field {:?} ({}) is missing from reflection or has a different type",
                    field.selector, ty.type_name
                )));
            }
            let child = self.bind(field.value, depth + 1)?;
            children.push(child);
            schemas.push(ValueDecodeSchemaField {
                name,
                index: field.declaration_index,
                schema: self.description.bindings[child].schema,
            });
        }
        Ok((children, schemas))
    }
}

fn scalar_kind(scalar: Scalar) -> ValueDecodeScalar {
    macro_rules! kinds { ($($kind:ident),*) => { match scalar { $(Scalar::$kind => ValueDecodeScalar::$kind),* } }; }
    kinds!(
        Bool, U8, U16, U32, U64, U128, I8, I16, I32, I64, I128, F32, F64, Char
    )
}
fn record_shape(shape: RecordShape) -> ValueDecodeShape {
    match shape {
        RecordShape::Unit => ValueDecodeShape::Unit,
        RecordShape::Tuple => ValueDecodeShape::Tuple,
        RecordShape::Newtype => ValueDecodeShape::Newtype,
        RecordShape::Struct => ValueDecodeShape::Struct,
    }
}
