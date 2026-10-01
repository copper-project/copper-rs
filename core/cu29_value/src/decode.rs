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

/// A portable description of native encoded bytes. This API is experimental.
///
/// Wire operations are shared independently of schemas, preserving the identity
/// and coherent storage unit of quantities that share a scalar representation.
/// The description's serialization is a prototype for build packaging, not yet
/// a versioned on-disk catalogue format.
#[derive(Clone, Debug, Encode, Decode)]
pub struct ValueDecodeDescription {
    /// Binding for the described payload.
    pub root: usize,
    /// Typed associations between wire operations and schemas.
    pub bindings: Vec<ValueDecodeBinding>,
    /// Deduplicated wire operations.
    pub recipes: Vec<ValueDecodeRecipe>,
    /// Original type and field information.
    pub schemas: Vec<ValueDecodeSchema>,
}

/// A wire recipe bound to its original logical schema.
#[derive(Clone, Debug, Encode, Decode)]
pub struct ValueDecodeBinding {
    /// Index into the recipe table.
    pub recipe: usize,
    /// Index into the schema table.
    pub schema: usize,
}

/// Original type identity and names, retained separately from wire operations.
#[derive(Clone, Debug, Encode, Decode)]
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
#[derive(Clone, Debug, PartialEq, Eq, Encode, Decode)]
pub struct ValueDecodeQuantity {
    /// Quantity identity such as `length` or `mass`.
    pub quantity: String,
    /// Storage unit such as `m`, `kg`, or `ns` for Copper clock values.
    pub storage_unit: String,
}

/// A logical field bound by name or original declaration index.
#[derive(Clone, Debug, Encode, Decode)]
pub struct ValueDecodeSchemaField {
    /// Named fields use their reflected name; tuple fields have no name.
    pub name: Option<String>,
    /// Original declaration index, including fields omitted from encoding.
    pub index: usize,
    /// Child schema index.
    pub schema: usize,
}

/// A logical enum branch matched to its encoded tag.
#[derive(Clone, Debug, Encode, Decode)]
pub struct ValueDecodeSchemaVariant {
    /// Original variant name.
    pub name: String,
    /// Encoded fields in wire order.
    pub fields: Vec<ValueDecodeSchemaField>,
}

/// Portable native scalar encoding. Widths are preserved in the value tree.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Encode, Decode)]
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
#[derive(Clone, Copy, Debug, PartialEq, Eq, Encode, Decode)]
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
#[derive(Clone, Debug, PartialEq, Eq, Encode, Decode)]
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
#[derive(Clone, Debug, PartialEq, Eq, Encode, Decode)]
pub enum ValueDecodeRecipe {
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

impl ValueDecodeDescription {
    /// Bind generated wire recipes to reflection and Copper standard quantities.
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
                recipes: Vec::new(),
                schemas: Vec::new(),
            },
            types: BTreeMap::new(),
            registry,
            quantities,
        };
        builder.description.root = builder.bind(ValueDecodeRef::of::<T>(), 0)?;
        Ok(builder.description)
    }

    /// Read one native value, returning the exact number of consumed bytes.
    /// Slice the input at that offset to read a consecutive value. Invalid recipes,
    /// invalid tags, truncated bytes and exceeded limits return an error.
    pub fn decode<C: Config>(
        &self,
        bytes: &[u8],
        config: C,
        limits: ValueDecodeLimits,
    ) -> Result<(Value, usize), DecodeError> {
        let reader = ValueReader { bytes, offset: 0 };
        let mut decoder = bincode::de::DecoderImpl::new(reader, config, ());
        let mut remaining = limits.max_values;
        let value = self.decode_binding(self.root, &mut decoder, limits, &mut remaining, 1)?;
        Ok((value, decoder.reader().offset))
    }

    fn decode_binding<D: Decoder<Context = ()>>(
        &self,
        id: usize,
        decoder: &mut D,
        limits: ValueDecodeLimits,
        remaining: &mut usize,
        depth: usize,
    ) -> Result<Value, DecodeError> {
        if depth > limits.max_depth {
            return Err(DecodeError::Other("ValueDecode nesting limit exceeded"));
        }
        spend(remaining, 1)?;
        let binding = self
            .bindings
            .get(id)
            .ok_or(DecodeError::Other("invalid ValueDecode binding index"))?;
        let schema = self
            .schemas
            .get(binding.schema)
            .ok_or(DecodeError::Other("invalid ValueDecode schema index"))?;
        let recipe = self
            .recipes
            .get(binding.recipe)
            .ok_or(DecodeError::Other("invalid ValueDecode recipe index"))?;
        let child = |id, decoder: &mut D, remaining: &mut usize| {
            self.decode_binding(id, decoder, limits, remaining, depth + 1)
        };
        Ok(match recipe {
            ValueDecodeRecipe::Unit => Value::Unit,
            ValueDecodeRecipe::Scalar(scalar) => decode_scalar(*scalar, decoder)?,
            ValueDecodeRecipe::String | ValueDecodeRecipe::Bytes => {
                let len = checked_len(u64::decode(decoder)?, limits.max_collection_len)?;
                decoder.claim_bytes_read(len)?;
                // Read before constructing the tree; truncation is reported by the bounded reader.
                let mut bytes = alloc::vec![0; len];
                decoder.reader().read(&mut bytes)?;
                if matches!(recipe, ValueDecodeRecipe::String) {
                    Value::String(String::from_utf8(bytes).map_err(|error| DecodeError::Utf8 {
                        inner: error.utf8_error(),
                    })?)
                } else {
                    Value::Bytes(bytes)
                }
            }
            ValueDecodeRecipe::Record { shape, fields } => self.decode_record(
                ValueDecodeRecord {
                    shape: *shape,
                    fields,
                    schema: &schema.fields,
                },
                decoder,
                limits,
                remaining,
                depth,
            )?,
            ValueDecodeRecipe::Array { element, len } => {
                check_count(*len, limits, *remaining)?;
                let mut values = Vec::new();
                for _ in 0..*len {
                    values.push(child(*element, decoder, remaining)?);
                }
                Value::Seq(values)
            }
            ValueDecodeRecipe::Sequence {
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
                check_count(len, limits, *remaining)?;
                let mut values = Vec::new();
                for _ in 0..len {
                    values.push(child(*element, decoder, remaining)?);
                }
                Value::Seq(values)
            }
            ValueDecodeRecipe::Map { key, value } => {
                let len = checked_len(u64::decode(decoder)?, limits.max_collection_len)?;
                check_count(
                    len.checked_mul(2)
                        .ok_or(DecodeError::Other("ValueDecode map size overflow"))?,
                    ValueDecodeLimits {
                        max_collection_len: usize::MAX,
                        ..limits
                    },
                    *remaining,
                )?;
                let mut values = BTreeMap::new();
                for _ in 0..len {
                    values.insert(
                        child(*key, decoder, remaining)?,
                        child(*value, decoder, remaining)?,
                    );
                }
                Value::Map(values)
            }
            ValueDecodeRecipe::Delegate(element) => child(*element, decoder, remaining)?,
            ValueDecodeRecipe::Option(element) => match u8::decode(decoder)? {
                0 => Value::Option(None),
                1 => Value::Option(Some(Box::new(child(*element, decoder, remaining)?))),
                _ => return Err(DecodeError::Other("invalid ValueDecode option tag")),
            },
            ValueDecodeRecipe::Enum { tag, branches } => {
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
                // Serde-compatible externally tagged enum representation.
                if branch.shape == ValueDecodeShape::Unit {
                    if !branch.fields.is_empty() {
                        return Err(DecodeError::Other("unit variant has encoded fields"));
                    }
                    Value::String(variant.name.clone())
                } else {
                    spend(remaining, 1)?;
                    if branch.shape != ValueDecodeShape::Newtype {
                        spend(remaining, 1)?;
                    }
                    let value = self.decode_record(
                        ValueDecodeRecord {
                            shape: branch.shape,
                            fields: &branch.fields,
                            schema: &variant.fields,
                        },
                        decoder,
                        limits,
                        remaining,
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
        remaining: &mut usize,
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
        check_count(fields.len(), limits, *remaining)?;
        match shape {
            ValueDecodeShape::Unit if fields.is_empty() => Ok(Value::Unit),
            ValueDecodeShape::Unit => Err(DecodeError::Other("unit record has encoded fields")),
            ValueDecodeShape::Newtype if fields.len() == 1 => Ok(Value::Newtype(Box::new(
                self.decode_binding(fields[0], decoder, limits, remaining, depth + 1)?,
            ))),
            ValueDecodeShape::Newtype => Err(DecodeError::Other(
                "newtype record must have one encoded field",
            )),
            ValueDecodeShape::Tuple => {
                let mut values = Vec::new();
                for id in fields {
                    values.push(self.decode_binding(*id, decoder, limits, remaining, depth + 1)?);
                }
                Ok(Value::Seq(values))
            }
            ValueDecodeShape::Struct => {
                let mut values = BTreeMap::new();
                for (id, field) in fields.iter().zip(schema) {
                    spend(remaining, 1)?;
                    let name = field
                        .name
                        .as_ref()
                        .ok_or(DecodeError::Other("missing ValueDecode field name"))?;
                    let value = self.decode_binding(*id, decoder, limits, remaining, depth + 1)?;
                    if values.insert(Value::String(name.clone()), value).is_some() {
                        return Err(DecodeError::Other("duplicate ValueDecode field name"));
                    }
                }
                Ok(Value::Map(values))
            }
        }
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
            recipe: 0,
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
        let recipe = match ty.spec {
            ValueDecodeSpec::Unit => ValueDecodeRecipe::Unit,
            ValueDecodeSpec::Scalar(scalar) => ValueDecodeRecipe::Scalar(scalar_kind(*scalar)),
            ValueDecodeSpec::String => ValueDecodeRecipe::String,
            ValueDecodeSpec::Bytes => ValueDecodeRecipe::Bytes,
            ValueDecodeSpec::Record { shape, fields } => {
                let (children, fields) =
                    self.bind_fields(fields, info, None, ty.type_name, depth)?;
                self.description.schemas[id].fields = fields;
                ValueDecodeRecipe::Record {
                    shape: record_shape(*shape),
                    fields: children,
                }
            }
            ValueDecodeSpec::Array { element, len } => ValueDecodeRecipe::Array {
                element: self.bind(*element, depth + 1)?,
                len: *len,
            },
            ValueDecodeSpec::Sequence {
                element,
                count,
                capacity,
            } => ValueDecodeRecipe::Sequence {
                element: self.bind(*element, depth + 1)?,
                count: scalar_kind(*count),
                capacity: *capacity,
            },
            ValueDecodeSpec::Map { key, value } => ValueDecodeRecipe::Map {
                key: self.bind(*key, depth + 1)?,
                value: self.bind(*value, depth + 1)?,
            },
            ValueDecodeSpec::Delegate(element) => {
                ValueDecodeRecipe::Delegate(self.bind(*element, depth + 1)?)
            }
            ValueDecodeSpec::Option(element) => {
                ValueDecodeRecipe::Option(self.bind(*element, depth + 1)?)
            }
            ValueDecodeSpec::Enum { tag, variants } => {
                let mut branches = Vec::new();
                for variant in *variants {
                    let reflected = match info {
                        Some(TypeInfo::Enum(info)) => info.variant(variant.name),
                        _ => None,
                    };
                    if reflected.is_none() {
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
                ValueDecodeRecipe::Enum {
                    tag: scalar_kind(*tag),
                    branches,
                }
            }
        };
        let recipe_id = self
            .description
            .recipes
            .iter()
            .position(|candidate| candidate == &recipe)
            .unwrap_or_else(|| {
                let id = self.description.recipes.len();
                self.description.recipes.push(recipe);
                id
            });
        self.description.bindings[id].recipe = recipe_id;
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
                    if reflected_len != Some(declared_fields) {
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
            if reflected != Some(ty.type_id) {
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
